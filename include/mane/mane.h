/*
 * MaNE: run QEMU inside another process.
 *
 * MaNE (a MAME fork) links QEMU as a library and runs one machine on a
 * thread of its own. This file is the whole interface.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef MANE_MANE_H
#define MANE_MANE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Run QEMU with the given command line until the guest shuts down or
 * mane_qemu_request_stop() is called. Runs qemu_init(), qemu_main_loop()
 * and qemu_cleanup() on the calling thread, then releases the BQL and the
 * replay mutex. Returns the exit status of the main loop.
 *
 * Call it at most once per process. Bad options still call exit().
 */
int mane_qemu_run(int argc, char **argv);

/* Ask the running QEMU to shut down. Safe from any thread. */
void mane_qemu_request_stop(void);

/* True once mane_qemu_run() has started. */
extern bool mane_embedded;

#ifdef __cplusplus
}
#endif

#endif /* MANE_MANE_H */
