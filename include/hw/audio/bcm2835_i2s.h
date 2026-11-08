/*
 * BCM2835 PCM / I2S audio interface
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef BCM2835_I2S_H
#define BCM2835_I2S_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_BCM2835_I2S "bcm2835-i2s"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835I2SState, BCM2835_I2S)

#define BCM2835_I2S_FIFO_DEPTH 64

/*
 * Outputs: sysbus IRQ 0 is the PCM interrupt; named GPIOs "dreq-tx" and
 * "dreq-rx" are the DMA requests. Input clock "clk" is the PCM clock
 * (CPRMAN "pcm-out"), the bit clock when the PCM block is master.
 */
struct BCM2835I2SState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dreq_tx;
    qemu_irq dreq_rx;
    Clock *clk;
    QEMUTimer frame_timer;
    QEMUBH *dreq_bh;
    QEMUBH *host_bh;

    uint32_t cs;
    uint32_t mode;
    uint32_t rxc;
    uint32_t txc;
    uint32_t dreq;
    uint32_t inten;
    uint32_t intstc;
    uint32_t gray;

    uint32_t tx_fifo[BCM2835_I2S_FIFO_DEPTH];
    uint32_t tx_head;
    uint32_t tx_count;
    uint32_t rx_fifo[BCM2835_I2S_FIFO_DEPTH];
    uint32_t rx_head;
    uint32_t rx_count;

    /* Levels last driven on the DREQ outputs */
    bool dreq_tx_level;
    bool dreq_rx_level;

    /* Master mode: frames counted from clock_start (virtual ns) */
    bool clock_running;
    int64_t clock_start;
    uint64_t frames_done;

    /* Slave mode: frames the host has clocked and QEMU not yet run */
    uint32_t host_frames;
};

#endif /* BCM2835_I2S_H */
