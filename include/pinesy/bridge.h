/*
 * The Pinesy bridge: run QEMU inside another process.
 *
 * Pinesy (a MAME fork) links QEMU as a library and runs one machine on a
 * thread of its own. This file is the whole interface.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef PINESY_BRIDGE_H
#define PINESY_BRIDGE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Run QEMU with the given command line until the guest shuts down or
 * pinesy_bridge_request_stop() is called. Runs qemu_init(), qemu_main_loop()
 * and qemu_cleanup() on the calling thread, then releases the BQL and the
 * replay mutex. Returns the exit status of the main loop.
 *
 * Call it at most once per process. Bad options still call exit().
 */
int pinesy_bridge_run(int argc, char **argv);

/* Ask the running QEMU to shut down. Safe from any thread. */
void pinesy_bridge_request_stop(void);

/* True once pinesy_bridge_run() has started. */
extern bool pinesy_bridge_embedded;

#ifdef __cplusplus
}
#endif

#endif /* PINESY_BRIDGE_H */
