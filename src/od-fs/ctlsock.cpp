/*
 * kernel-hive: unix control socket for host-native input injection (mamectl/1).
 *
 * WHY THIS WIRE CONTRACT
 * ----------------------
 * The streamhost daemon already has two socket input backends: `mame_sock.rs`
 * (mamectl/1) and `vice_sock.rs` (vicectl). This module speaks *mamectl/1*
 * verbatim so that ZERO daemon code changes: a station sets
 *   SH_INPUT_BACKEND=mamesock   SH_MAMECTL_SOCK=<path>   SH_MAMESOCK_KEYMAP=<file>
 * and the existing MameSockSink drives FS-UAE. The one machine-specific piece,
 * `KEY <0|1> <port> <field>`, is resolved daemon-side through the station's
 * keymap file, so we declare port `amiga` and carry the Amiga RAW KEYCODE in
 * the field. vicectl was rejected: it carries X11 keysyms and expects the
 * emulator to own a symbolic keymap, which FS-UAE's keymap machinery only
 * exposes through the SDL frontend we are removing.
 *
 * THREADING
 * ---------
 * The reader thread ENQUEUES ONLY. Every uae_mousehack_helper /
 * setmousebuttonstate / inputdevice_do_keyboard / uae_reset / save_state call
 * happens on the UAE core thread in fsuae_ctlsock_poll(), which handle_events()
 * calls once per frame (od-fs/input.cpp). Acks are emitted from that drain, so
 * an ack means the verb was APPLIED to the core, not merely parsed.
 *
 * SINGLE INJECTOR (BINDING)
 * -------------------------
 * A station launched with FSUAE_NATIVE_CTL_SOCK set must NOT also have the
 * Xvfb/XTEST injection path armed. Two injectors fight over the same mousehack
 * accumulator and button mask; the daemon's restate-before-edge invariant is
 * silently broken and drags land in the wrong place. Same rule as MAME_CTL_SOCK.
 */

#include "sysconfig.h"
#include "sysdeps.h"

#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "options.h"
#include "inputdevice.h"
#include "uae.h"
#include "savestate.h"
#include "custom.h"
#include "xwin.h"
#include "ctlsock.h"
extern "C" int amiga_send_input_event(int input_event, int state);

#include <deque>
#include <string>

#define CTL_PROTO "mamectl/1"

/* uae_mousehack_helper() is declared in od-fs/target.h (C++ linkage). */

/* Published crop rect, refreshed every frame by od-fs/video.cpp. */
extern int g_fsuae_ctl_crop_x, g_fsuae_ctl_crop_y;
extern int g_fsuae_ctl_crop_w, g_fsuae_ctl_crop_h;

static int g_enabled = -1;
static int g_started = 0;
static int g_listen_fd = -1;
static int g_client_fd = -1;          /* written by the reader thread only */
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static std::deque<std::string> g_queue;   /* guarded by g_lock */
static unsigned g_generation = 0;         /* guarded by g_lock; bumped on connect */

/* Deferred LOADST ack: restore_state() only *arms* the restore. */
static int g_restore_pending = 0;
static unsigned long long g_restore_seq = 0;
static unsigned g_restore_gen = 0;

/* Mirror of the button mask, for STAT. */
static int g_buttons = 0;
static int g_last_x = -1, g_last_y = -1;
static unsigned long long g_applied = 0;

int fsuae_ctlsock_enabled(void)
{
    if (g_enabled < 0) {
        const char *p = getenv("FSUAE_NATIVE_CTL_SOCK");
        g_enabled = (p && *p) ? 1 : 0;
    }
    return g_enabled;
}

static int trace_on(void)
{
    static int t = -1;
    if (t < 0) {
        const char *p = getenv("FSUAE_NATIVE_CTL_TRACE");
        t = (p && *p && strcmp(p, "0") != 0) ? 1 : 0;
    }
    return t;
}

/* Write on the client fd. Called from BOTH threads; fd writes are small and
 * ordered under g_lock by every caller. */
static void ctl_write_locked(unsigned gen, const char *s)
{
    if (g_client_fd < 0 || gen != g_generation)
        return;
    size_t n = strlen(s);
    ssize_t w = write(g_client_fd, s, n);
    if (w < 0 && errno != EINTR && errno != EAGAIN) {
        /* peer gone; the reader thread will notice on its next read */
    }
    if (trace_on())
        write_log("[fsuae-ctl] tx %s", s);
}

static void ack(unsigned gen, unsigned long long seq, const char *what)
{
    char buf[160];
    snprintf(buf, sizeof(buf), "%llu %s\n", seq, what);
    pthread_mutex_lock(&g_lock);
    ctl_write_locked(gen, buf);
    pthread_mutex_unlock(&g_lock);
}

/* ------------------------------------------------------------------ drain */

static void apply_movea(int x, int y)
{
    /*
     * MOVEA arrives in the PUBLISHED FRAME's pixel space -- the crop rect the
     * capture plane advertises, which is what the visitor's browser clamps to.
     * mousehack_helper() works in native buffer pixels, so add the crop origin
     * back. getgfxoffset() is forced to identity under the knob (video.cpp),
     * because its fs_emu_video_offset_ and scale_ inputs are libfsemu RENDERER
     * state -- correct under the SDL frontend only by accident of it also being
     * the thing that sets them, and garbage headless.
     */
    g_last_x = x;
    g_last_y = y;
    uae_mousehack_helper(x + g_fsuae_ctl_crop_x, y + g_fsuae_ctl_crop_y);
}

static int parse_field_keycode(const char *field, int *out)
{
    char *end = NULL;
    long v;
    while (*field == ' ')
        field++;
    v = strtol(field, &end, 0);
    if (end == field || v < 0 || v > 0xff)
        return 0;
    *out = (int) v;
    return 1;
}

/* Returns 0 on success, else an error string in *err. */
static const char *apply_line(unsigned gen, unsigned long long seq, char *verb)
{
    char *arg = strchr(verb, ' ');
    if (arg) {
        *arg++ = 0;
        while (*arg == ' ')
            arg++;
    } else {
        arg = (char *) "";
    }

    if (!strcmp(verb, "MOVEA")) {
        int x, y;
        if (sscanf(arg, "%d %d", &x, &y) != 2)
            return "bad MOVEA args";
        apply_movea(x, y);
        return NULL;
    }
    if (!strncmp(verb, "DOWN", 4) || !strncmp(verb, "UP", 2)) {
        int down = (verb[0] == 'D');
        const char *nptr = down ? verb + 4 : verb + 2;
        int btn = atoi(nptr);
        if (btn < 1 || btn > 3)
            return "bad button";
        /* mamectl: 1=left 2=right 3=middle. UAE: 0=left 1=right 2=middle. */
        static const int uaebtn[4] = { 0, 0, 1, 2 };
        static const int joyev[4] = { 0, INPUTEVENT_JOY1_FIRE_BUTTON,
                                      INPUTEVENT_JOY1_2ND_BUTTON,
                                      INPUTEVENT_JOY1_3RD_BUTTON };
        if (g_last_x >= 0)
            apply_movea(g_last_x, g_last_y);   /* restate before every edge */
        setmousebuttonstate(0, uaebtn[btn], down);
        /* setmousebuttonstate() alone moves only the *device* button state of
         * mouse index 0, which headless is not bound to a joyport; the port
         * bits the guest reads come from the input EVENT. Send both so the
         * edge lands whether or not a host mouse device was enumerated. */
        amiga_send_input_event(joyev[btn], down);
        if (down)
            g_buttons |= 1 << btn;
        else
            g_buttons &= ~(1 << btn);
        return NULL;
    }
    if (!strcmp(verb, "KEY")) {
        int state = 0;
        char port[32];
        char field[96];
        port[0] = field[0] = 0;
        if (sscanf(arg, "%d %31s %95[^\n]", &state, port, field) < 3)
            return "bad KEY args";
        if (strcmp(port, "amiga") != 0)
            return "unknown port (want 'amiga')";
        int raw;
        if (!parse_field_keycode(field, &raw))
            return "bad raw keycode";
        inputdevice_do_keyboard(raw, state ? 1 : 0);
        return NULL;
    }
    if (!strcmp(verb, "RESET")) {
        uae_reset(1, 1);
        return NULL;
    }
    if (!strcmp(verb, "SAVEST")) {
        if (!*arg)
            return "SAVEST needs a path";
        /* save_state() is synchronous; returning means the file is written. */
        if (!save_state(arg, "kernel-hive ctlsock"))
            return "save_state failed";
        return NULL;
    }
    if (!strcmp(verb, "LOADST")) {
        if (!*arg)
            return "LOADST needs a path";
        if (g_restore_pending)
            return "restore already in flight";
        restore_state(arg);
        g_restore_pending = 1;
        g_restore_seq = seq;
        g_restore_gen = gen;
        return NULL;    /* ack deferred to completion, see drain below */
    }
    if (!strcmp(verb, "STAT")) {
        char buf[192];
        snprintf(buf, sizeof(buf),
                 "%llu OK screen=%dx%d crop=%d,%d buttons=%d applied=%llu\n",
                 seq, g_fsuae_ctl_crop_w, g_fsuae_ctl_crop_h,
                 g_fsuae_ctl_crop_x, g_fsuae_ctl_crop_y, g_buttons, g_applied);
        pthread_mutex_lock(&g_lock);
        ctl_write_locked(gen, buf);
        pthread_mutex_unlock(&g_lock);
        return (const char *) -1;   /* already acked */
    }
    return "unknown verb";
}

/* ---------------------------------------------------------------- listener */

static void send_hello(unsigned gen)
{
    char buf[192];
    snprintf(buf, sizeof(buf),
             "HELLO " CTL_PROTO " fs-uae caps=natkbd,savest,reset screen=%dx%d\n",
             g_fsuae_ctl_crop_w > 0 ? g_fsuae_ctl_crop_w : 1,
             g_fsuae_ctl_crop_h > 0 ? g_fsuae_ctl_crop_h : 1);
    ctl_write_locked(gen, buf);
}

static void *reader_thread(void *ignored)
{
    (void) ignored;
    for (;;) {
        int fd = accept(g_listen_fd, NULL, NULL);
        if (fd < 0) {
            if (errno == EINTR)
                continue;
            write_log("[fsuae-ctl] accept: %s\n", strerror(errno));
            break;
        }
        pthread_mutex_lock(&g_lock);
        g_client_fd = fd;
        g_generation++;
        unsigned gen = g_generation;
        g_queue.clear();
        send_hello(gen);
        pthread_mutex_unlock(&g_lock);
        write_log("[fsuae-ctl] client connected (gen %u)\n", gen);

        std::string buf;
        char chunk[1024];
        for (;;) {
            ssize_t n = read(fd, chunk, sizeof(chunk));
            if (n <= 0) {
                if (n < 0 && errno == EINTR)
                    continue;
                break;
            }
            buf.append(chunk, (size_t) n);
            size_t nl;
            while ((nl = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                if (!line.empty() && line[line.size() - 1] == '\r')
                    line.erase(line.size() - 1);
                if (line.empty())
                    continue;
                pthread_mutex_lock(&g_lock);
                if (g_queue.size() < 4096)
                    g_queue.push_back(line);
                pthread_mutex_unlock(&g_lock);
            }
        }
        pthread_mutex_lock(&g_lock);
        g_client_fd = -1;
        g_generation++;
        g_queue.clear();
        pthread_mutex_unlock(&g_lock);
        close(fd);
        write_log("[fsuae-ctl] client disconnected\n");
    }
    return NULL;
}

static void ctlsock_start(void)
{
    const char *path = getenv("FSUAE_NATIVE_CTL_SOCK");
    struct sockaddr_un addr;

    g_started = 1;
    if (!path || !*path)
        return;
    if (strlen(path) >= sizeof(addr.sun_path)) {
        write_log("[fsuae-ctl] FATAL: socket path too long: %s\n", path);
        abort();
    }
    unlink(path);
    g_listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (g_listen_fd < 0) {
        write_log("[fsuae-ctl] FATAL: socket: %s\n", strerror(errno));
        abort();
    }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, path);
    if (bind(g_listen_fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        /* Loud failure, no fallback: a station that asked for the ctlsock and
         * did not get it must not silently run uncontrollable. */
        write_log("[fsuae-ctl] FATAL: bind %s: %s\n", path, strerror(errno));
        abort();
    }
    chmod(path, 0666);
    if (listen(g_listen_fd, 1) < 0) {
        write_log("[fsuae-ctl] FATAL: listen: %s\n", strerror(errno));
        abort();
    }
    pthread_t t;
    if (pthread_create(&t, NULL, reader_thread, NULL) != 0) {
        write_log("[fsuae-ctl] FATAL: pthread_create\n");
        abort();
    }
    pthread_detach(t);
    write_log("[fsuae-ctl] listening on %s (" CTL_PROTO ")\n", path);
}

void fsuae_ctlsock_poll(void)
{
    if (!fsuae_ctlsock_enabled())
        return;
    if (!g_started)
        ctlsock_start();

    for (;;) {
        std::string line;
        unsigned gen;
        pthread_mutex_lock(&g_lock);
        if (g_queue.empty()) {
            pthread_mutex_unlock(&g_lock);
            break;
        }
        line = g_queue.front();
        g_queue.pop_front();
        gen = g_generation;
        pthread_mutex_unlock(&g_lock);

        if (trace_on())
            write_log("[fsuae-ctl] rx %s\n", line.c_str());

        /* Wire framing: "<seq> <VERB> [args]". */
        char *dup = strdup(line.c_str());
        char *sp = strchr(dup, ' ');
        unsigned long long seq = strtoull(dup, NULL, 10);
        if (!sp) {
            ack(gen, seq, "ERR no verb");
            free(dup);
            continue;
        }
        *sp++ = 0;
        const char *err = apply_line(gen, seq, sp);
        if (err == NULL) {
            g_applied++;
            ack(gen, seq, "OK");
        } else if (err != (const char *) -1) {
            char buf[160];
            snprintf(buf, sizeof(buf), "ERR %s", err);
            ack(gen, seq, buf);
        }
        free(dup);
    }

    if (g_restore_pending && savestate_state == 0) {
        g_restore_pending = 0;
        ack(g_restore_gen, g_restore_seq, "OK");
    }
}
