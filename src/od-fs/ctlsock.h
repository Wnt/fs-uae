/*
 * kernel-hive: unix control socket for host-native input injection.
 *
 * Armed only when FSUAE_NATIVE_CTL_SOCK is set to a filesystem path; with the
 * knob unset every entry point below is a no-op and the binary behaves exactly
 * as stock FS-UAE.
 */
#ifndef FSUAE_OD_FS_CTLSOCK_H
#define FSUAE_OD_FS_CTLSOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* True when FSUAE_NATIVE_CTL_SOCK names a path. Cheap, cached. */
int fsuae_ctlsock_enabled(void);

/*
 * Called once per emulated frame from handle_events() -- i.e. ON THE UAE CORE
 * THREAD. Lazily starts the listener on first call, then applies every command
 * the reader thread enqueued since the last frame and writes their acks.
 */
void fsuae_ctlsock_poll(void);

#ifdef __cplusplus
}
#endif

#endif
