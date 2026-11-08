/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2835_DMA_H
#define BCM2835_DMA_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

typedef struct {
    uint32_t cs;
    uint32_t conblk_ad;
    uint32_t ti;
    uint32_t source_ad;
    uint32_t dest_ad;
    uint32_t txfr_len;
    uint32_t stride;
    uint32_t nextconbk;
    uint32_t debug;

    /* A control block is loaded and not yet finished */
    bool cb_loaded;
    /* Row length of the loaded control block in 2D mode */
    uint32_t row_len;

    qemu_irq irq;
} BCM2835DMAChan;

#define TYPE_BCM2835_DMA "bcm2835-dma"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835DMAState, BCM2835_DMA)

#define BCM2835_DMA_NCHANS 16

/* Peripheral DREQ lines (TI PERMAP); named GPIO inputs "dreq" */
#define BCM2835_DMA_NDREQS 32
#define BCM2835_DMA_DREQ_PCM_TX 2
#define BCM2835_DMA_DREQ_PCM_RX 3
#define BCM2835_DMA_DREQ_SPI_TX 6
#define BCM2835_DMA_DREQ_SPI_RX 7

struct BCM2835DMAState {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/

    MemoryRegion iomem0, iomem15;
    MemoryRegion *dma_mr;
    AddressSpace dma_as;

    BCM2835DMAChan chan[BCM2835_DMA_NCHANS];
    uint32_t int_status;
    uint32_t enable;

    /*
     * DREQ lines driven by a device model ("dreq-paced" property). The
     * other lines read as always asserted, so peripherals that do not
     * drive their DREQ get each control block at once.
     */
    uint32_t dreq_paced;
    uint32_t dreq_level;
    /* Channels inside bcm2835_dma_update(), to stop re-entry */
    uint32_t running;
};

#endif
