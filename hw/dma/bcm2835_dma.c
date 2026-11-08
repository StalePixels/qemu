/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/dma/bcm2835_dma.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "trace.h"

/* DMA CS Control and Status bits */
#define BCM2708_DMA_ACTIVE      (1 << 0)
#define BCM2708_DMA_END         (1 << 1) /* GE */
#define BCM2708_DMA_INT         (1 << 2)
#define BCM2708_DMA_ISPAUSED    (1 << 4)  /* Pause requested or not active */
#define BCM2708_DMA_ISHELD      (1 << 5)  /* Is held by DREQ flow control */
#define BCM2708_DMA_ERR         (1 << 8)
#define BCM2708_DMA_ABORT       (1 << 30) /* stop current CB, go to next, WO */
#define BCM2708_DMA_RESET       (1 << 31) /* WO, self clearing */

/* DMA control block "info" field bits */
#define BCM2708_DMA_INT_EN      (1 << 0)
#define BCM2708_DMA_TDMODE      (1 << 1)
#define BCM2708_DMA_WAIT_RESP   (1 << 3)
#define BCM2708_DMA_D_INC       (1 << 4)
#define BCM2708_DMA_D_WIDTH     (1 << 5)
#define BCM2708_DMA_D_DREQ      (1 << 6)
#define BCM2708_DMA_D_IGNORE    (1 << 7)
#define BCM2708_DMA_S_INC       (1 << 8)
#define BCM2708_DMA_S_WIDTH     (1 << 9)
#define BCM2708_DMA_S_DREQ      (1 << 10)
#define BCM2708_DMA_S_IGNORE    (1 << 11)

/* Register offsets */
#define BCM2708_DMA_CS          0x00 /* Control and Status */
#define BCM2708_DMA_ADDR        0x04 /* Control block address */
/* the current control block appears in the following registers - read only */
#define BCM2708_DMA_INFO        0x08
#define BCM2708_DMA_SOURCE_AD   0x0c
#define BCM2708_DMA_DEST_AD     0x10
#define BCM2708_DMA_TXFR_LEN    0x14
#define BCM2708_DMA_STRIDE      0x18
#define BCM2708_DMA_NEXTCB      0x1C
#define BCM2708_DMA_DEBUG       0x20

#define BCM2708_DMA_INT_STATUS  0xfe0 /* Interrupt status of each channel */
#define BCM2708_DMA_ENABLE      0xff0 /* Global enable bits for each channel */

#define BCM2708_DMA_CS_RW_MASK  0x30ff0001 /* All RW bits in DMA_CS */

/* DMA CS bit: state of the selected DREQ line, read only */
#define BCM2708_DMA_DREQ        (1 << 3)

/* Control block "info" field: peripheral that paces the transfer */
#define BCM2708_DMA_PERMAP(ti)  extract32(ti, 16, 5)

/*
 * True when the channel may move its next word: the control block uses no
 * DREQ, or uses a line that is not paced, or the line is asserted. PERMAP 0
 * is a permanently asserted line.
 */
static bool bcm2835_dma_dreq_ok(BCM2835DMAState *s, BCM2835DMAChan *ch)
{
    unsigned permap = BCM2708_DMA_PERMAP(ch->ti);

    if (!(ch->ti & (BCM2708_DMA_S_DREQ | BCM2708_DMA_D_DREQ))) {
        return true;
    }
    if (!(s->dreq_paced & (1u << permap))) {
        return true;
    }
    return s->dreq_level & (1u << permap);
}

static void bcm2835_dma_load_cb(BCM2835DMAState *s, BCM2835DMAChan *ch)
{
    ch->ti = ldl_le_phys(&s->dma_as, ch->conblk_ad);
    ch->source_ad = ldl_le_phys(&s->dma_as, ch->conblk_ad + 4);
    ch->dest_ad = ldl_le_phys(&s->dma_as, ch->conblk_ad + 8);
    ch->txfr_len = ldl_le_phys(&s->dma_as, ch->conblk_ad + 12);
    ch->stride = ldl_le_phys(&s->dma_as, ch->conblk_ad + 16);
    ch->nextconbk = ldl_le_phys(&s->dma_as, ch->conblk_ad + 20);
    ch->row_len = ch->txfr_len & 0xffff;
    ch->cb_loaded = true;
}

/*
 * Run channel c until it runs out of control blocks or is held by its DREQ
 * line. TXFR_LEN counts down as words move, so a held channel resumes where
 * it stopped. In 2D mode its top half counts the rows still to go after the
 * current one.
 */
static void bcm2835_dma_update(BCM2835DMAState *s, unsigned c)
{
    BCM2835DMAChan *ch = &s->chan[c];
    uint32_t data, xlen, ylen;
    int16_t dst_stride, src_stride;
    bool td;

    if (s->running & (1u << c)) {
        return;
    }
    s->running |= 1u << c;

    while ((s->enable & (1 << c)) && (ch->cs & BCM2708_DMA_ACTIVE)) {
        if (!ch->cb_loaded) {
            if (ch->conblk_ad == 0) {
                ch->cs &= ~BCM2708_DMA_ACTIVE;
                break;
            }
            bcm2835_dma_load_cb(s, ch);
            trace_bcm2835_dma_cb(c, ch->conblk_ad, ch->ti, ch->source_ad,
                                 ch->dest_ad, ch->txfr_len);
        }

        td = ch->ti & BCM2708_DMA_TDMODE;
        if (td) {
            ylen = (ch->txfr_len >> 16) & 0x3fff;
            xlen = ch->txfr_len & 0xffff;
            dst_stride = ch->stride >> 16;
            src_stride = ch->stride & 0xffff;
        } else {
            ylen = 0;
            xlen = ch->txfr_len;
            dst_stride = 0;
            src_stride = 0;
        }

        for (;;) {
            while (xlen != 0) {
                if (!bcm2835_dma_dreq_ok(s, ch)) {
                    if (!(ch->cs & BCM2708_DMA_ISHELD)) {
                        trace_bcm2835_dma_held(c, BCM2708_DMA_PERMAP(ch->ti));
                    }
                    ch->cs |= BCM2708_DMA_ISHELD;
                    goto out;
                }
                ch->cs &= ~BCM2708_DMA_ISHELD;

                if (ch->ti & BCM2708_DMA_S_IGNORE) {
                    /* Ignore reads */
                    data = 0;
                } else {
                    data = ldl_le_phys(&s->dma_as, ch->source_ad);
                }
                if (ch->ti & BCM2708_DMA_S_INC) {
                    ch->source_ad += 4;
                }

                if (ch->ti & BCM2708_DMA_D_IGNORE) {
                    /* Ignore writes */
                } else {
                    stl_le_phys(&s->dma_as, ch->dest_ad, data);
                }
                if (ch->ti & BCM2708_DMA_D_INC) {
                    ch->dest_ad += 4;
                }

                /* update remaining transfer length */
                xlen = xlen > 4 ? xlen - 4 : 0;
                ch->txfr_len = td ? (ylen << 16) | xlen : xlen;

                /* the write above can have stopped or reset the channel */
                if (!ch->cb_loaded || !(ch->cs & BCM2708_DMA_ACTIVE)) {
                    goto out;
                }
            }
            if (ylen == 0) {
                break;
            }
            ylen--;
            ch->source_ad += src_stride;
            ch->dest_ad += dst_stride;
            xlen = ch->row_len;
            ch->txfr_len = (ylen << 16) | xlen;
        }

        ch->cs |= BCM2708_DMA_END;
        if (ch->ti & BCM2708_DMA_INT_EN) {
            ch->cs |= BCM2708_DMA_INT;
            s->int_status |= (1 << c);
            qemu_set_irq(ch->irq, 1);
        }

        /* Process next CB */
        ch->conblk_ad = ch->nextconbk;
        ch->cb_loaded = false;
    }

    if (!(ch->cs & BCM2708_DMA_ACTIVE)) {
        ch->cs |= BCM2708_DMA_ISPAUSED;
    }
out:
    s->running &= ~(1u << c);
}

/* A peripheral DREQ line changed: run the channels it holds */
static void bcm2835_dma_dreq(void *opaque, int line, int level)
{
    BCM2835DMAState *s = opaque;
    unsigned c;

    if (level) {
        s->dreq_level |= 1u << line;
    } else {
        s->dreq_level &= ~(1u << line);
    }
    if (!level || !(s->dreq_paced & (1u << line))) {
        return;
    }
    for (c = 0; c < BCM2835_DMA_NCHANS; c++) {
        BCM2835DMAChan *ch = &s->chan[c];

        if ((ch->cs & BCM2708_DMA_ACTIVE) && ch->cb_loaded &&
            BCM2708_DMA_PERMAP(ch->ti) == line) {
            bcm2835_dma_update(s, c);
        }
    }
}

static void bcm2835_dma_chan_reset(BCM2835DMAChan *ch)
{
    ch->cs = 0;
    ch->conblk_ad = 0;
    ch->cb_loaded = false;
}

static uint64_t bcm2835_dma_read(BCM2835DMAState *s, hwaddr offset,
                                 unsigned size, unsigned c)
{
    BCM2835DMAChan *ch;
    uint32_t res = 0;

    assert(size == 4);
    assert(c < BCM2835_DMA_NCHANS);

    ch = &s->chan[c];

    switch (offset) {
    case BCM2708_DMA_CS:
        res = ch->cs & ~BCM2708_DMA_DREQ;
        if (!(s->dreq_paced & (1u << BCM2708_DMA_PERMAP(ch->ti))) ||
            (s->dreq_level & (1u << BCM2708_DMA_PERMAP(ch->ti)))) {
            res |= BCM2708_DMA_DREQ;
        }
        break;
    case BCM2708_DMA_ADDR:
        res = ch->conblk_ad;
        break;
    case BCM2708_DMA_INFO:
        res = ch->ti;
        break;
    case BCM2708_DMA_SOURCE_AD:
        res = ch->source_ad;
        break;
    case BCM2708_DMA_DEST_AD:
        res = ch->dest_ad;
        break;
    case BCM2708_DMA_TXFR_LEN:
        res = ch->txfr_len;
        break;
    case BCM2708_DMA_STRIDE:
        res = ch->stride;
        break;
    case BCM2708_DMA_NEXTCB:
        res = ch->nextconbk;
        break;
    case BCM2708_DMA_DEBUG:
        res = ch->debug;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, offset);
        break;
    }
    return res;
}

static void bcm2835_dma_write(BCM2835DMAState *s, hwaddr offset,
                              uint64_t value, unsigned size, unsigned c)
{
    BCM2835DMAChan *ch;
    uint32_t oldcs;

    assert(size == 4);
    assert(c < BCM2835_DMA_NCHANS);

    ch = &s->chan[c];

    switch (offset) {
    case BCM2708_DMA_CS:
        oldcs = ch->cs;
        if (value & BCM2708_DMA_RESET) {
            bcm2835_dma_chan_reset(ch);
        }
        if ((value & BCM2708_DMA_ABORT) && ch->cb_loaded) {
            /* drop the current control block and go to the next one */
            ch->cb_loaded = false;
            ch->conblk_ad = ch->nextconbk;
            ch->cs &= ~BCM2708_DMA_ISHELD;
        }
        if (value & BCM2708_DMA_END) {
            ch->cs &= ~BCM2708_DMA_END;
        }
        if (value & BCM2708_DMA_INT) {
            ch->cs &= ~BCM2708_DMA_INT;
            s->int_status &= ~(1 << c);
            qemu_set_irq(ch->irq, 0);
        }
        ch->cs &= ~BCM2708_DMA_CS_RW_MASK;
        ch->cs |= (value & BCM2708_DMA_CS_RW_MASK);
        if (ch->cs & BCM2708_DMA_ACTIVE) {
            if (!(oldcs & BCM2708_DMA_ACTIVE)) {
                ch->cs &= ~BCM2708_DMA_ISPAUSED;
            }
            bcm2835_dma_update(s, c);
        }
        break;
    case BCM2708_DMA_ADDR:
        ch->conblk_ad = value;
        ch->cb_loaded = false;
        break;
    case BCM2708_DMA_NEXTCB:
        ch->nextconbk = value;
        break;
    case BCM2708_DMA_DEBUG:
        ch->debug = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, offset);
        break;
    }
}

static uint64_t bcm2835_dma0_read(void *opaque, hwaddr offset, unsigned size)
{
    BCM2835DMAState *s = opaque;

    if (offset < 0xf00) {
        return bcm2835_dma_read(s, (offset & 0xff), size, (offset >> 8) & 0xf);
    } else {
        switch (offset) {
        case BCM2708_DMA_INT_STATUS:
            return s->int_status;
        case BCM2708_DMA_ENABLE:
            return s->enable;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                          __func__, offset);
            return 0;
        }
    }
}

static uint64_t bcm2835_dma15_read(void *opaque, hwaddr offset, unsigned size)
{
    return bcm2835_dma_read(opaque, (offset & 0xff), size, 15);
}

static void bcm2835_dma0_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    BCM2835DMAState *s = opaque;

    if (offset < 0xf00) {
        bcm2835_dma_write(s, (offset & 0xff), value, size, (offset >> 8) & 0xf);
    } else {
        switch (offset) {
        case BCM2708_DMA_INT_STATUS:
            break;
        case BCM2708_DMA_ENABLE:
            s->enable = (value & 0xffff);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                          __func__, offset);
        }
    }

}

static void bcm2835_dma15_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    bcm2835_dma_write(opaque, (offset & 0xff), value, size, 15);
}

static const MemoryRegionOps bcm2835_dma0_ops = {
    .read = bcm2835_dma0_read,
    .write = bcm2835_dma0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps bcm2835_dma15_ops = {
    .read = bcm2835_dma15_read,
    .write = bcm2835_dma15_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const VMStateDescription vmstate_bcm2835_dma_chan = {
    .name = TYPE_BCM2835_DMA "-chan",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cs, BCM2835DMAChan),
        VMSTATE_UINT32(conblk_ad, BCM2835DMAChan),
        VMSTATE_UINT32(ti, BCM2835DMAChan),
        VMSTATE_UINT32(source_ad, BCM2835DMAChan),
        VMSTATE_UINT32(dest_ad, BCM2835DMAChan),
        VMSTATE_UINT32(txfr_len, BCM2835DMAChan),
        VMSTATE_UINT32(stride, BCM2835DMAChan),
        VMSTATE_UINT32(nextconbk, BCM2835DMAChan),
        VMSTATE_UINT32(debug, BCM2835DMAChan),
        VMSTATE_BOOL_V(cb_loaded, BCM2835DMAChan, 2),
        VMSTATE_UINT32_V(row_len, BCM2835DMAChan, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_bcm2835_dma = {
    .name = TYPE_BCM2835_DMA,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(chan, BCM2835DMAState, BCM2835_DMA_NCHANS, 1,
                             vmstate_bcm2835_dma_chan, BCM2835DMAChan),
        VMSTATE_UINT32(int_status, BCM2835DMAState),
        VMSTATE_UINT32(enable, BCM2835DMAState),
        VMSTATE_UINT32_V(dreq_level, BCM2835DMAState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_dma_init(Object *obj)
{
    BCM2835DMAState *s = BCM2835_DMA(obj);
    int n;

    /* DMA channels 0-14 occupy a contiguous block of IO memory, along
     * with the global enable and interrupt status bits. Channel 15
     * has the same register map, but is mapped at a discontiguous
     * address in a separate IO block.
     */
    memory_region_init_io(&s->iomem0, OBJECT(s), &bcm2835_dma0_ops, s,
                          TYPE_BCM2835_DMA, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem0);

    memory_region_init_io(&s->iomem15, OBJECT(s), &bcm2835_dma15_ops, s,
                          TYPE_BCM2835_DMA "-chan15", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem15);

    for (n = 0; n < 16; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->chan[n].irq);
    }

    qdev_init_gpio_in_named(DEVICE(s), bcm2835_dma_dreq, "dreq",
                            BCM2835_DMA_NDREQS);
}

static void bcm2835_dma_reset(DeviceState *dev)
{
    BCM2835DMAState *s = BCM2835_DMA(dev);
    int n;

    s->enable = 0xffff;
    s->int_status = 0;
    for (n = 0; n < BCM2835_DMA_NCHANS; n++) {
        bcm2835_dma_chan_reset(&s->chan[n]);
    }
}

static void bcm2835_dma_realize(DeviceState *dev, Error **errp)
{
    BCM2835DMAState *s = BCM2835_DMA(dev);
    Object *obj;

    obj = object_property_get_link(OBJECT(dev), "dma-mr", &error_abort);
    s->dma_mr = MEMORY_REGION(obj);
    address_space_init(&s->dma_as, s->dma_mr, TYPE_BCM2835_DMA "-memory");

    bcm2835_dma_reset(dev);
}

static const Property bcm2835_dma_properties[] = {
    DEFINE_PROP_UINT32("dreq-paced", BCM2835DMAState, dreq_paced, 0),
};

static void bcm2835_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_dma_realize;
    device_class_set_legacy_reset(dc, bcm2835_dma_reset);
    dc->vmsd = &vmstate_bcm2835_dma;
    device_class_set_props(dc, bcm2835_dma_properties);
}

static const TypeInfo bcm2835_dma_info = {
    .name          = TYPE_BCM2835_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835DMAState),
    .class_init    = bcm2835_dma_class_init,
    .instance_init = bcm2835_dma_init,
};

static void bcm2835_dma_register_types(void)
{
    type_register_static(&bcm2835_dma_info);
}

type_init(bcm2835_dma_register_types)
