/*
 * MaNE: run QEMU inside another process. See include/mane/mane.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "system/replay.h"
#include "system/runstate.h"
#include "system/system.h"
#include "mane/mane.h"

int mane_qemu_run(int argc, char **argv)
{
    int status;

    mane_embedded = true;

    /*
     * Only the process's first thread is an RCU reader by default (rcu_init()
     * in util/rcu.c). This thread runs QEMU's main loop, so register it too.
     */
    rcu_register_thread();

    /* qemu_init() returns holding the BQL and the replay mutex. */
    qemu_init(argc, argv);
    status = qemu_main_loop();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    rcu_unregister_thread();

    return status;
}

void mane_qemu_request_stop(void)
{
    qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_UI);
}
