/* Kernel-hive host-native mode: run the UAE core with no window, no GL and no
 * X server, publishing frames into a shared-memory framebuffer instead.
 * Everything here is inert unless FSUAE_NATIVE_SHM is set in the environment. */

#ifndef FS_UAE_NATIVE_H
#define FS_UAE_NATIVE_H

/* 1 when FSUAE_NATIVE_SHM names a path. Decided once, on first call. */
int fsuae_native_enabled(void);

/* Path from FSUAE_NATIVE_SHM, or NULL. */
const char *fsuae_native_shm_path(void);

/* Install the shm render/display callbacks over the libfsemu ones. Call after
 * fs_uae_init_video(); no-op unless enabled. */
void fsuae_native_init_video(void);

/* Frame limiter replacing fs_emu_wait_for_frame(): sleeps until this frame's
 * deadline on a monotonic clock derived from the chipset refresh rate. */
void fsuae_native_wait_for_frame(void);

/* Run the emulation main function on this thread instead of fs_emu_run()'s
 * emulation thread + video main loop. */
int fsuae_native_run(void (*main_function)(void));

#endif /* FS_UAE_NATIVE_H */
