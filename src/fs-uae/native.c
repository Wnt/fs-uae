/* Kernel-hive host-native mode.
 *
 * WHY. The lab's streaming daemon captures a station's frames one of two ways:
 * off an X window (an Xvfb per station, XTEST for input) or straight out of a
 * shared-memory framebuffer the emulator publishes itself (SH_CAPTURE=shm).
 * MAME's drawshm module and VICE's shmfb already speak the second contract; the
 * X detour costs a software rasterise, a texture upload, a blit through llvmpipe
 * and a read-back of the very same pixels. FS-UAE is the last emulator in the
 * museum still paying it.
 *
 * ROUTE (the "second frontend"). FS-UAE's own author already separated the UAE
 * core from the presentation layer with a C callback struct: the core hands a
 * finished RGB32 frame to g_libamiga_callbacks.render (src/od-fs/video.cpp:249)
 * before any GL exists, and amiga_set_render_function() is the public way to
 * claim it. So this file is not a hole carved in a monolith -- it is a second
 * consumer of an existing frontend API. We deliberately do NOT try to build a
 * "null video driver" inside libfsemu: there isn't one (the dummy symbols in
 * libfsemu/src/ml/ are empty-translation-unit padding) and SDL_VIDEODRIVER=dummy
 * still creates a GL context. Instead, under the knob, main() skips
 * FS_EMU_INIT_VIDEO, we register our render function OVER the frontend's after
 * fs_uae_init_video(), and fsuae_native_run() calls the emulation main function
 * directly rather than fs_emu_run()'s emulation-thread + GL main-loop pair.
 * That keeps every line of FS-UAE's configuration, model and input wiring --
 * the part that makes a golden restorable -- exactly as it is.
 *
 * THE KNOB. FSUAE_NATIVE_SHM names the file to publish into. Unset, every
 * function here returns immediately and the binary takes the stock path; the
 * four live Xvfb stations keep running off this same source.
 * FSUAE_NATIVE_SHM_TRACE=1 logs the mapping and a periodic publish counter to
 * stderr.
 *
 * THE FRAME still goes nowhere in this commit -- the render callback only
 * counts frames and reports the crop rectangle under the trace knob. The next
 * commit gives it the shared-memory publisher.
 *
 * FRAME PACING: dropping libfsemu drops fs_emu_wait_for_frame(). Without a
 * replacement the guest free-runs at whatever speed the host can manage.
 * fsuae_native_wait_for_frame() is a monotonic deadline pacer fed by the
 * chipset refresh rate the core reports on rd->refresh_rate, and it resnaps
 * (rather than bursting to catch up) whenever it falls more than a few frames
 * behind -- the same discipline the audio FIFO uses.
 */

#include "native.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <uae/uae.h>

static int g_enabled = -1;
static const char *g_path = NULL;
static int g_trace = 0;

static uint64_t g_frames = 0;

int fsuae_native_enabled(void)
{
    if (g_enabled < 0) {
        const char *p = getenv("FSUAE_NATIVE_SHM");
        g_enabled = (p && *p) ? 1 : 0;
        g_path = g_enabled ? p : NULL;
        const char *t = getenv("FSUAE_NATIVE_SHM_TRACE");
        g_trace = (t && *t && strcmp(t, "0") != 0) ? 1 : 0;
    }
    return g_enabled;
}

const char *fsuae_native_shm_path(void)
{
    fsuae_native_enabled();
    return g_path;
}

/* --- the render buffer ---------------------------------------------------
 *
 * The core does not own its pixel buffer: the frontend hands one over with
 * amiga_set_render_buffer() and may be asked to grow it (RTG). Stock FS-UAE
 * satisfies that from libfsemu's rotating video buffers -- which is exactly the
 * machinery we are not initialising, and leaving it out is what segfaulted the
 * first headless run inside target_graphics_buffer_update() (a memset through a
 * NULL g_renderdata.pixels, before the first frame). So host-native mode brings
 * its own: one plain heap allocation, generously sized for the 752x572 chipset
 * raster, grown on demand. One buffer, no rotation -- there is no second
 * consumer to hand a finished frame to, because our render callback publishes
 * synchronously. */

static unsigned char *g_pix = NULL;
static int g_pix_w = 0;
static int g_pix_h = 0;

static void *native_grow(int width, int height)
{
    if (width > g_pix_w || height > g_pix_h) {
        if (width > g_pix_w) g_pix_w = width;
        if (height > g_pix_h) g_pix_h = height;
        unsigned char *p = realloc(g_pix, (size_t) g_pix_w * g_pix_h * 4);
        if (p == NULL) {
            fprintf(stderr, "[fsuae-shm] out of memory growing to %dx%d\n",
                    g_pix_w, g_pix_h);
            return g_pix;
        }
        g_pix = p;
    }
    return g_pix;
}

/* The crop rectangle the core wants shown, normalized to hires/laced pixels and
 * clamped into the allocated buffer. Mirrors src/fs-uae/video.c:466-475; RTG
 * (Picasso96) frames already arrive at their true size with both shifts zero. */
static void native_crop(RenderData *rd, int *cx, int *cy, int *cw, int *ch)
{
    int hshift = (rd->flags & AMIGA_VIDEO_LOW_RESOLUTION) ? 1 : 0;
    int vshift = (!(rd->flags & AMIGA_VIDEO_LINE_DOUBLING)) ? 1 : 0;

    int x = rd->limit_x << hshift;
    int w = rd->limit_w << hshift;
    int y = rd->limit_y << vshift;
    int h = rd->limit_h << vshift;

    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (w <= 0 || x + w > rd->width) w = rd->width - x;
    if (h <= 0 || y + h > rd->height) h = rd->height - y;
    if (w < 0) w = 0;
    if (h < 0) h = 0;

    *cx = x;
    *cy = y;
    *cw = w;
    *ch = h;
}

static double g_render_hz = 0.0;

/* Commit 1 consumes the frame without publishing it: this proves the core runs
 * headless and hands over a real, correctly-cropped RGB32 frame, and the next
 * commit replaces the body with the IFB1 shm publisher. */
static void native_render(RenderData *rd)
{
    if (rd == NULL || rd->pixels == NULL) {
        return;
    }
    if (rd->refresh_rate > 1.0 && rd->refresh_rate < 200.0) {
        g_render_hz = rd->refresh_rate;
    }
    int cx, cy, cw, ch;
    native_crop(rd, &cx, &cy, &cw, &ch);
    if (cw <= 0 || ch <= 0) {
        return;
    }
    g_frames++;
    if (g_trace && (g_frames % 300) == 0) {
        fprintf(stderr, "[fsuae-shm] frame %llu crop %dx%d+%d+%d bpp %d\n",
                (unsigned long long) g_frames, cw, ch, cx, cy, rd->bpp);
    }
}

/* The core also calls a "display" callback after render; with no window there is
 * nothing to present, and the stock one drives libfsemu's buffer rotation. */
static void native_display(void)
{
}

void fsuae_native_init_video(void)
{
    if (!fsuae_native_enabled()) {
        return;
    }
    /* 1024x1024 covers the 752x572 chipset raster and the smaller RTG modes
     * without a single realloc; anything larger arrives through native_grow. */
    g_pix_w = 1024;
    g_pix_h = 1024;
    g_pix = calloc((size_t) g_pix_w * g_pix_h, 4);
    if (g_pix == NULL) {
        fprintf(stderr, "[fsuae-shm] cannot allocate render buffer\n");
        abort();
    }
    amiga_set_render_buffer(g_pix, g_pix_w * g_pix_h * 4, 1, native_grow);

    amiga_set_render_function(native_render);
    amiga_set_display_function(native_display);
    fprintf(stderr, "[fsuae-shm] host-native video: publishing to %s\n", g_path);
}

/* --- frame pacing -------------------------------------------------------- */

static double g_hz = 0.0;
static int64_t g_next_ns = 0;

static int64_t native_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t) ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

void fsuae_native_wait_for_frame(void)
{
    if (!fsuae_native_enabled()) {
        return;
    }
    /* The chipset refresh rate is whatever the core is currently running at --
     * PAL 50, NTSC ~60, and -1 while warp mode is on. Re-read it every frame so
     * a mode switch repaces us instead of desynchronising. */
    double hz = g_render_hz;
    if (!(hz > 1.0) || hz > 200.0) {
        hz = 50.0;
    }
    if (hz != g_hz) {
        g_hz = hz;
        g_next_ns = 0;
    }
    int64_t period = (int64_t) (1000000000.0 / g_hz);
    int64_t now = native_now_ns();
    if (g_next_ns == 0) {
        g_next_ns = now + period;
        return;
    }
    int64_t delta = g_next_ns - now;
    if (delta > 0) {
        struct timespec ts;
        ts.tv_sec = delta / 1000000000LL;
        ts.tv_nsec = delta % 1000000000LL;
        while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
            /* keep sleeping the remainder */
        }
        g_next_ns += period;
    } else if (-delta > period * 5) {
        /* More than five frames behind: resnap the schedule rather than burst
         * through a backlog of frames nobody will ever see. */
        g_next_ns = now + period;
    } else {
        g_next_ns += period;
    }
}

int fsuae_native_run(void (*main_function)(void))
{
    fprintf(stderr, "[fsuae-shm] host-native run: no window, no GL, no X\n");
    main_function();
    return 0;
}
