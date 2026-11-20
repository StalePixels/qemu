/*
 * The Pinesy bridge: run QEMU inside another process.
 * See include/pinesy/bridge.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "system/replay.h"
#include "system/runstate.h"
#include "system/system.h"
#include "pinesy/bridge.h"

/* QEMU's main loop runs (pinesy_bridge_continue() can schedule) */
static bool main_loop_active;

/* the Pi waits (pinesy_bridge_waiting()) */
static bool vm_waiting;

/* The run states in which the Pi waits for a GDB client or the host. */
static bool pinesy_bridge_state_waits(RunState state)
{
    return state == RUN_STATE_PRELAUNCH || state == RUN_STATE_DEBUG ||
           state == RUN_STATE_PAUSED;
}

static void pinesy_bridge_vm_state(void *opaque, bool running, RunState state)
{
    qatomic_set(&vm_waiting, !running && pinesy_bridge_state_waits(state));
}

int pinesy_bridge_run(int argc, char **argv)
{
    int status;

    pinesy_bridge_embedded = true;

    /*
     * Only the process's first thread is an RCU reader by default (rcu_init()
     * in util/rcu.c). This thread runs QEMU's main loop, so register it too.
     */
    rcu_register_thread();

    /* qemu_init() returns holding the BQL and the replay mutex. */
    qemu_init(argc, argv);
    qemu_add_vm_change_state_handler(pinesy_bridge_vm_state, NULL);
    qatomic_set(&vm_waiting, pinesy_bridge_state_waits(runstate_get()));
    qatomic_set(&main_loop_active, true);
    status = qemu_main_loop();
    qatomic_set(&main_loop_active, false);
    qatomic_set(&vm_waiting, false);
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    rcu_unregister_thread();

    return status;
}

void pinesy_bridge_request_stop(void)
{
    qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_UI);
}

bool pinesy_bridge_waiting(void)
{
    return qatomic_read(&vm_waiting);
}

/* QEMU's main loop thread, with the BQL */
static void pinesy_bridge_continue_bh(void *opaque)
{
    if (pinesy_bridge_state_waits(runstate_get())) {
        vm_start();
    }
}

bool pinesy_bridge_continue(void)
{
    if (!qatomic_read(&main_loop_active)) {
        return false;
    }
    aio_bh_schedule_oneshot(qemu_get_aio_context(), pinesy_bridge_continue_bh,
                            NULL);
    return qatomic_read(&vm_waiting);
}
