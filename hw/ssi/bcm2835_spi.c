/*
 * BCM2835 SPI Master Controller
 *
 * Copyright (c) 2024 Rayhan Faizel <rayhan.faizel@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Additions for Pinesy: DMA mode (DREQ lines, 32-bit FIFO access, DLEN),
 * chip select 0 from GPIO 8 or the native CS field, and a host on chip
 * select 0 through the Pinesy bridge (include/pinesy/bridge.h).
 *
 * Not modelled: LoSSI mode, 3-wire bidirectional mode (REN), CS polarity
 * (CSPOL, CSPOL0-2), ADCS, LTOH and the panic thresholds. Bytes take no
 * virtual time: a byte moves as soon as the TX FIFO has it and the RX
 * FIFO has room, or, for the bridge host, when the host answers.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/log.h"
#include "qemu/fifo8.h"
#include "qemu/main-loop.h"
#include "hw/ssi/bcm2835_spi.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "pinesy/bridge.h"
#include "trace.h"

/* DC register thresholds, in bytes */
#define DC_TDREQ(dc)            extract32(dc, 0, 8)
#define DC_RDREQ(dc)            extract32(dc, 16, 8)

static PinesySPIHandlers bcm2835_spi_handlers;
static void *bcm2835_spi_opaque;
/* The device on the bridge: the first one realized (SPI0) */
static BCM2835SPIState *bcm2835_spi_dev;

static bool bcm2835_spi_dma(BCM2835SPIState *s)
{
    return s->cs & BCM2835_SPI_CS_DMAEN;
}

static bool bcm2835_spi_on_bridge(BCM2835SPIState *s)
{
    return s == bcm2835_spi_dev && bcm2835_spi_handlers.transfer;
}

/*
 * Chip select 0 is active when the guest drives GPIO 8 low as an output
 * (Linux "cs-gpios"), or when a transfer is active on native chip select 0.
 */
static bool bcm2835_spi_cs0(BCM2835SPIState *s)
{
    return !s->cs0_gpio ||
           ((s->cs & BCM2835_SPI_CS_TA) &&
            (s->cs & BCM2835_SPI_CS_CS) == 0);
}

/* The transfer has nothing more to send: DONE */
static bool bcm2835_spi_done(BCM2835SPIState *s)
{
    if (!(s->cs & BCM2835_SPI_CS_TA) || s->host_pending) {
        return false;
    }
    if (bcm2835_spi_dma(s)) {
        /* bytes after DLEN stay in the TX FIFO until it is cleared */
        return s->dlen == 0;
    }
    return fifo8_is_empty(&s->tx_fifo);
}

static void bcm2835_spi_update_flags(BCM2835SPIState *s)
{
    s->cs &= ~(BCM2835_SPI_CS_RXF | BCM2835_SPI_CS_RXR | BCM2835_SPI_CS_TXD |
               BCM2835_SPI_CS_RXD | BCM2835_SPI_CS_DONE);
    if (!fifo8_is_empty(&s->rx_fifo)) {
        s->cs |= BCM2835_SPI_CS_RXD;
    }
    if (fifo8_is_full(&s->rx_fifo)) {
        s->cs |= BCM2835_SPI_CS_RXF;
    }
    /* RXR: RX FIFO 3/4 full or more, while TA is set */
    if ((s->cs & BCM2835_SPI_CS_TA) &&
        fifo8_num_used(&s->rx_fifo) >= FIFO_SIZE_3_4) {
        s->cs |= BCM2835_SPI_CS_RXR;
    }
    if (!fifo8_is_full(&s->tx_fifo)) {
        s->cs |= BCM2835_SPI_CS_TXD;
    }
    if (bcm2835_spi_done(s)) {
        s->cs |= BCM2835_SPI_CS_DONE;
    }
}

static void bcm2835_spi_set_dreq(BCM2835SPIState *s, qemu_irq line,
                                 bool *level, bool want)
{
    if (*level != want) {
        *level = want;
        qemu_set_irq(line, want);
    }
}

/*
 * Bring the flags, the interrupt and the DMA requests up to date. A raised
 * DREQ runs the DMA at once, which then reads or writes this device's FIFO.
 * During a register access that would re-enter this device, so from a
 * register access (now = false) a rising DREQ waits for dreq_bh.
 *
 * TX DREQ: TX FIFO level at or below DC TDREQ. As on the hardware, it stays
 * up after DLEN has run out, so a cyclic TX channel fills the FIFO to the
 * threshold. RX DREQ: RX FIFO level above DC RDREQ, or the transfer done
 * and the RX FIFO not empty.
 */
static void bcm2835_spi_update(BCM2835SPIState *s, bool now)
{
    bool want;
    int irq = 0;

    bcm2835_spi_update_flags(s);

    if ((s->cs & BCM2835_SPI_CS_INTD) && (s->cs & BCM2835_SPI_CS_DONE)) {
        irq = 1;
    }
    if ((s->cs & BCM2835_SPI_CS_INTR) && (s->cs & BCM2835_SPI_CS_RXR)) {
        irq = 1;
    }
    qemu_set_irq(s->irq, irq);

    want = bcm2835_spi_dma(s) &&
           fifo8_num_used(&s->tx_fifo) <= DC_TDREQ(s->dc);
    if (!want || now) {
        bcm2835_spi_set_dreq(s, s->dreq_tx, &s->dreq_tx_level, want);
    } else if (!s->dreq_tx_level) {
        qemu_bh_schedule(s->dreq_bh);
    }

    /* the DMA can have changed the FIFOs; look again */
    want = bcm2835_spi_dma(s) &&
           (fifo8_num_used(&s->rx_fifo) > DC_RDREQ(s->dc) ||
            (bcm2835_spi_done(s) && !fifo8_is_empty(&s->rx_fifo)));
    if (!want || now) {
        bcm2835_spi_set_dreq(s, s->dreq_rx, &s->dreq_rx_level, want);
    } else if (!s->dreq_rx_level) {
        qemu_bh_schedule(s->dreq_bh);
    }
}

static void bcm2835_spi_dreq_bh(void *opaque)
{
    bcm2835_spi_update(opaque, true);
}

/* Tell the bridge host when chip select 0 changes */
static void bcm2835_spi_update_cs0(BCM2835SPIState *s)
{
    bool sel = bcm2835_spi_cs0(s);

    if (sel != s->cs0_selected) {
        s->cs0_selected = sel;
        trace_bcm2835_spi_select(sel);
        if (bcm2835_spi_on_bridge(s) && bcm2835_spi_handlers.select) {
            bcm2835_spi_handlers.select(bcm2835_spi_opaque, sel);
        }
    }
}

/* The host's answer to host_pending bytes is in host_reply */
static void bcm2835_spi_host_done(BCM2835SPIState *s, uint32_t len)
{
    uint32_t n = s->host_pending;
    uint32_t i;

    if (len != n) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: host answered %u bytes for %u\n",
                      __func__, len, n);
    }
    trace_bcm2835_spi_host_reply(n, s->host_discard);
    s->host_pending = 0;
    if (s->host_discard) {
        s->host_discard = false;
        return;
    }
    for (i = 0; i < n; i++) {
        fifo8_push(&s->rx_fifo, i < len ? s->host_reply[i] : 0xff);
    }
}

/* Bytes the shifter may move now */
static uint32_t bcm2835_spi_can_shift(BCM2835SPIState *s)
{
    uint32_t n;

    if (!(s->cs & BCM2835_SPI_CS_TA) || s->host_pending) {
        return 0;
    }
    n = MIN(fifo8_num_used(&s->tx_fifo), fifo8_num_free(&s->rx_fifo));
    if (bcm2835_spi_dma(s)) {
        n = MIN(n, s->dlen);
    }
    return n;
}

/*
 * Move bytes from the TX FIFO to the RX FIFO. With chip select 0 on the
 * bridge host, the bytes the shifter can move go to the host in one call;
 * the shifter is then busy until the host answers, which can be inside
 * that call or later through host_bh. Otherwise each byte goes over the
 * SSI bus at once.
 */
static void bcm2835_spi_shift(BCM2835SPIState *s)
{
    uint8_t mosi[FIFO_SIZE];
    uint32_t n, i, len;

    while ((n = bcm2835_spi_can_shift(s)) != 0) {
        for (i = 0; i < n; i++) {
            mosi[i] = fifo8_pop(&s->tx_fifo);
        }
        if (bcm2835_spi_dma(s)) {
            s->dlen -= n;
        }
        if (!bcm2835_spi_on_bridge(s) || !s->cs0_selected) {
            for (i = 0; i < n; i++) {
                fifo8_push(&s->rx_fifo, ssi_transfer(s->bus, mosi[i]));
            }
            continue;
        }
        s->host_pending = n;
        trace_bcm2835_spi_host_transfer(n);
        bcm2835_spi_handlers.transfer(bcm2835_spi_opaque, mosi, n);
        len = qatomic_xchg(&s->host_ready, 0);
        if (!len) {
            return;
        }
        bcm2835_spi_host_done(s, len - 1);
    }
}

/* The bridge host has answered from another thread */
static void bcm2835_spi_host_bh(void *opaque)
{
    BCM2835SPIState *s = opaque;
    uint32_t len = qatomic_xchg(&s->host_ready, 0);

    if (!len || !s->host_pending) {
        return;
    }
    bcm2835_spi_host_done(s, len - 1);
    bcm2835_spi_shift(s);
    bcm2835_spi_update(s, true);
}

void pinesy_bridge_set_spi(const PinesySPIHandlers *handlers, void *opaque)
{
    bcm2835_spi_handlers = *handlers;
    bcm2835_spi_opaque = opaque;
}

void pinesy_bridge_spi_reply(const uint8_t *miso, uint32_t len)
{
    BCM2835SPIState *s = qatomic_load_acquire(&bcm2835_spi_dev);

    if (!s) {
        return;
    }
    len = MIN(len, FIFO_SIZE);
    memcpy(s->host_reply, miso, len);
    /* len + 1, so that an answer of 0 bytes is still an answer */
    qatomic_store_release(&s->host_ready, len + 1);
    qemu_bh_schedule(s->host_bh);
}

/* Chip select 0 GPIO pin (GPIO 8) changed level */
static void bcm2835_spi_cs0_gpio(void *opaque, int line, int level)
{
    BCM2835SPIState *s = opaque;

    s->cs0_gpio = level;
    bcm2835_spi_update_cs0(s);
    bcm2835_spi_shift(s);
    bcm2835_spi_update(s, false);
}

static void bcm2835_spi_write_cs(BCM2835SPIState *s, uint32_t value)
{
    uint32_t old = s->cs;

    s->cs = (value & ~RO_MASK) | (s->cs & RO_MASK);

    /* Clear RX FIFO */
    if (s->cs & BCM2835_SPI_CLEAR_RX) {
        fifo8_reset(&s->rx_fifo);
    }

    /* Clear TX FIFO */
    if (s->cs & BCM2835_SPI_CLEAR_TX) {
        fifo8_reset(&s->tx_fifo);
    }
    s->cs &= ~(BCM2835_SPI_CLEAR_RX | BCM2835_SPI_CLEAR_TX);

    /* A transfer stopped while the host owes bytes: drop its answer */
    if ((old & BCM2835_SPI_CS_TA) && !(s->cs & BCM2835_SPI_CS_TA) &&
        s->host_pending) {
        s->host_discard = true;
    }

    if (s->cs & BCM2835_SPI_CS_LEN) {
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "LoSSI not supported\n", __func__);
    }

    bcm2835_spi_update_cs0(s);
    bcm2835_spi_shift(s);
}

static uint64_t bcm2835_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    BCM2835SPIState *s = opaque;
    uint32_t readval = 0;
    uint32_t i, n;

    switch (addr) {
    case BCM2835_SPI_CS:
        readval = s->cs & 0xffffffff;
        break;
    case BCM2835_SPI_FIFO:
        /* DMA mode reads up to four bytes, the first in bits 7:0 */
        n = bcm2835_spi_dma(s) ? 4 : 1;
        for (i = 0; i < n && !fifo8_is_empty(&s->rx_fifo); i++) {
            readval |= fifo8_pop(&s->rx_fifo) << (8 * i);
        }
        bcm2835_spi_shift(s);
        bcm2835_spi_update(s, false);
        break;
    case BCM2835_SPI_CLK:
        readval = s->clk & 0xffff;
        break;
    case BCM2835_SPI_DLEN:
        readval = s->dlen & 0xffff;
        break;
    case BCM2835_SPI_LTOH:
        readval = s->ltoh & 0xf;
        break;
    case BCM2835_SPI_DC:
        readval = s->dc & 0xffffffff;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
    return readval;
}

static void bcm2835_spi_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    BCM2835SPIState *s = opaque;
    uint32_t i, n;

    switch (addr) {
    case BCM2835_SPI_CS:
        bcm2835_spi_write_cs(s, value);
        bcm2835_spi_update(s, false);
        break;
    case BCM2835_SPI_FIFO:
        if (bcm2835_spi_dma(s) && !(s->cs & BCM2835_SPI_CS_TA)) {
            /* DMA mode, TA clear: DLEN in bits 31:16, CS 7:0 in bits 7:0 */
            s->dlen = value >> 16;
            bcm2835_spi_write_cs(s, (s->cs & ~0xff) | (value & 0xff));
        } else if (s->cs & BCM2835_SPI_CS_TA) {
            /* DMA mode writes four bytes, the first from bits 7:0 */
            n = bcm2835_spi_dma(s) ? 4 : 1;
            for (i = 0; i < n && !fifo8_is_full(&s->tx_fifo); i++) {
                fifo8_push(&s->tx_fifo, value >> (8 * i));
            }
            bcm2835_spi_shift(s);
        }
        bcm2835_spi_update(s, false);
        break;
    case BCM2835_SPI_CLK:
        s->clk = value & 0xffff;
        break;
    case BCM2835_SPI_DLEN:
        s->dlen = value & 0xffff;
        bcm2835_spi_update(s, false);
        break;
    case BCM2835_SPI_LTOH:
        s->ltoh = value & 0xf;
        break;
    case BCM2835_SPI_DC:
        s->dc = value & 0xffffffff;
        bcm2835_spi_update(s, false);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
}

static const MemoryRegionOps bcm2835_spi_ops = {
    .read = bcm2835_spi_read,
    .write = bcm2835_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void bcm2835_spi_init(Object *obj)
{
    BCM2835SPIState *s = BCM2835_SPI(obj);

    qdev_init_gpio_out_named(DEVICE(s), &s->dreq_tx, "dreq-tx", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->dreq_rx, "dreq-rx", 1);
    qdev_init_gpio_in_named(DEVICE(s), bcm2835_spi_cs0_gpio, "cs0-gpio", 1);
}

static void bcm2835_spi_realize(DeviceState *dev, Error **errp)
{
    BCM2835SPIState *s = BCM2835_SPI(dev);
    s->bus = ssi_create_bus(dev, "spi");

    memory_region_init_io(&s->iomem, OBJECT(dev), &bcm2835_spi_ops, s,
                          TYPE_BCM2835_SPI, 0x18);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    fifo8_create(&s->tx_fifo, FIFO_SIZE);
    fifo8_create(&s->rx_fifo, FIFO_SIZE);

    /*
     * Not guarded against re-entry: both run the DMA, which then writes
     * or reads this device's FIFO.
     */
    s->dreq_bh = qemu_bh_new(bcm2835_spi_dreq_bh, s);
    s->host_bh = qemu_bh_new(bcm2835_spi_host_bh, s);
    if (!bcm2835_spi_dev) {
        qatomic_store_release(&bcm2835_spi_dev, s);
    }
}

static void bcm2835_spi_reset(DeviceState *dev)
{
    BCM2835SPIState *s = BCM2835_SPI(dev);

    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);

    /* Reset values according to BCM2835 Peripheral Documentation */
    s->cs = BCM2835_SPI_CS_TXD | BCM2835_SPI_CS_REN;
    s->clk = 0;
    s->dlen = 0;
    s->ltoh = 0x1;
    s->dc = 0x30201020;

    /* An answer still owed by the host is dropped when it comes */
    s->host_discard = s->host_pending != 0;
    s->cs0_gpio = true;
    bcm2835_spi_update_cs0(s);
    s->dreq_tx_level = false;
    s->dreq_rx_level = false;
    qemu_set_irq(s->dreq_tx, 0);
    qemu_set_irq(s->dreq_rx, 0);
    qemu_set_irq(s->irq, 0);
}

static const VMStateDescription vmstate_bcm2835_spi = {
    .name = TYPE_BCM2835_SPI,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_FIFO8(tx_fifo, BCM2835SPIState),
        VMSTATE_FIFO8(rx_fifo, BCM2835SPIState),
        VMSTATE_UINT32(cs, BCM2835SPIState),
        VMSTATE_UINT32(clk, BCM2835SPIState),
        VMSTATE_UINT32(dlen, BCM2835SPIState),
        VMSTATE_UINT32(ltoh, BCM2835SPIState),
        VMSTATE_UINT32(dc, BCM2835SPIState),
        VMSTATE_BOOL_V(dreq_tx_level, BCM2835SPIState, 2),
        VMSTATE_BOOL_V(dreq_rx_level, BCM2835SPIState, 2),
        VMSTATE_BOOL_V(cs0_gpio, BCM2835SPIState, 2),
        VMSTATE_BOOL_V(cs0_selected, BCM2835SPIState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, bcm2835_spi_reset);
    dc->realize = bcm2835_spi_realize;
    dc->vmsd = &vmstate_bcm2835_spi;
}

static const TypeInfo bcm2835_spi_info = {
    .name = TYPE_BCM2835_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835SPIState),
    .class_init = bcm2835_spi_class_init,
    .instance_init = bcm2835_spi_init,
};

static void bcm2835_spi_register_types(void)
{
    type_register_static(&bcm2835_spi_info);
}

type_init(bcm2835_spi_register_types)
