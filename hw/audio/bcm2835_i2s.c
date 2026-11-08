/*
 * BCM2835 PCM / I2S audio interface
 *
 * The PCM block of the BCM2835 ARM Peripherals datasheet, chapter 8: a TX
 * and an RX FIFO of 64 words, two channels per frame, DMA requests and an
 * interrupt. The serial side is the Pinesy bridge (include/pinesy/bridge.h):
 * each frame goes to or comes from the host one sample per channel. The
 * frames are made by a QEMU timer when the PCM block is clock and frame
 * master, and by the host otherwise.
 *
 * Not modelled: PDM input (MODE_A PDME, PDMN) and GRAY mode reception,
 * which are stored only; frame sync length, frame sync and clock polarity
 * and channel positions, which change only the timing within a frame;
 * RAM standby (CS_A STBY), which is stored only. CS_A SYNC echoes the
 * written value at once, and TXCLR and RXCLR clear at once, where the
 * hardware waits 2 PCM clocks.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "hw/audio/bcm2835_i2s.h"
#include "hw/core/clock.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "migration/vmstate.h"
#include "pinesy/bridge.h"
#include "trace.h"

#define PCM_CS_A        0x00
#define PCM_FIFO_A      0x04
#define PCM_MODE_A      0x08
#define PCM_RXC_A       0x0c
#define PCM_TXC_A       0x10
#define PCM_DREQ_A      0x14
#define PCM_INTEN_A     0x18
#define PCM_INTSTC_A    0x1c
#define PCM_GRAY        0x20

/* CS_A */
#define CS_EN           (1u << 0)
#define CS_RXON         (1u << 1)
#define CS_TXON         (1u << 2)
#define CS_TXCLR        (1u << 3)
#define CS_RXCLR        (1u << 4)
#define CS_TXTHR(v)     extract32(v, 5, 2)
#define CS_RXTHR(v)     extract32(v, 7, 2)
#define CS_DMAEN        (1u << 9)
#define CS_TXSYNC       (1u << 13)
#define CS_RXSYNC       (1u << 14)
#define CS_TXERR        (1u << 15)
#define CS_RXERR        (1u << 16)
#define CS_TXW          (1u << 17)
#define CS_RXR          (1u << 18)
#define CS_TXD          (1u << 19)
#define CS_RXD          (1u << 20)
#define CS_TXE          (1u << 21)
#define CS_RXF          (1u << 22)
#define CS_RXSEX        (1u << 23)
#define CS_SYNC         (1u << 24)
#define CS_STBY         (1u << 25)
/* bits kept as written: EN RXON TXON TXTHR RXTHR DMAEN RXSEX SYNC STBY */
#define CS_RW_MASK      0x038003e7u

/* MODE_A */
#define MODE_FLEN(v)    extract32(v, 10, 10)
#define MODE_FSM        (1u << 21)
#define MODE_CLKM       (1u << 23)
#define MODE_FTXP       (1u << 24)
#define MODE_FRXP       (1u << 25)
#define MODE_PDME       (1u << 26)
#define MODE_CLK_DIS    (1u << 28)
#define MODE_MASK       0x1fffffffu

/* RXC_A and TXC_A: channel 1 in the top half, channel 2 in the bottom */
#define XC_CH(v, ch)    extract32(v, (ch) ? 0 : 16, 16)
#define XC_CHWEX        (1u << 15)
#define XC_CHEN         (1u << 14)
#define XC_CHWID(c)     extract32(c, 0, 4)

/* DREQ_A */
#define DREQ_TX(v)      extract32(v, 8, 7)
#define DREQ_RX(v)      extract32(v, 0, 7)
#define DREQ_MASK       0x7f7f7f7fu
#define DREQ_RESET      0x10303020u

/* INTEN_A and INTSTC_A */
#define INT_TXW         (1u << 0)
#define INT_RXR         (1u << 1)
#define INT_TXERR       (1u << 2)
#define INT_RXERR       (1u << 3)
#define INT_MASK        0xfu

/* GRAY */
#define GRAY_EN         (1u << 0)
#define GRAY_CLR        (1u << 1)

/* Master mode: how often the frame timer runs, in virtual ns */
#define FRAME_TICK_NS   (1000 * 1000)

/* The host side, set through the bridge; one PCM block per process */
static PinesyI2SHandlers bcm2835_i2s_handlers;
static void *bcm2835_i2s_opaque;
static BCM2835I2SState *bcm2835_i2s_dev;

/* The frame layout of one direction, from TXC_A or RXC_A and MODE_A */
typedef struct {
    bool chan_on[2];
    unsigned width[2];
    bool packed;
} BCM2835I2SFrame;

static bool bcm2835_i2s_master(BCM2835I2SState *s)
{
    return !(s->mode & (MODE_CLKM | MODE_FSM));
}

static void bcm2835_i2s_frame_desc(uint32_t xc, bool packed_bit,
                                   BCM2835I2SFrame *f)
{
    unsigned ch;

    for (ch = 0; ch < 2; ch++) {
        uint32_t c = XC_CH(xc, ch);

        f->chan_on[ch] = c & XC_CHEN;
        f->width[ch] = XC_CHWID(c) + 8 + ((c & XC_CHWEX) ? 16 : 0);
        if (f->width[ch] > 32) {
            f->width[ch] = 32;
        }
    }
    /* packing needs both channels; each then has at most 16 bits */
    f->packed = packed_bit && f->chan_on[0] && f->chan_on[1];
    if (f->packed) {
        f->width[0] = MIN(f->width[0], 16);
        f->width[1] = MIN(f->width[1], 16);
    }
}

static unsigned bcm2835_i2s_words_per_frame(const BCM2835I2SFrame *f)
{
    if (f->packed) {
        return 1;
    }
    return f->chan_on[0] + f->chan_on[1];
}

/* Bit clocks per frame */
static uint32_t bcm2835_i2s_frame_bits(BCM2835I2SState *s)
{
    return MODE_FLEN(s->mode) + 1;
}

/* Master mode: frames per second, or 0 when there is no bit clock */
static double bcm2835_i2s_frame_rate(BCM2835I2SState *s)
{
    uint64_t period = clock_get(s->clk);

    if ((s->mode & MODE_CLK_DIS) || !period) {
        return 0;
    }
    return (double)CLOCK_PERIOD_1SEC / period / bcm2835_i2s_frame_bits(s);
}

static void bcm2835_i2s_format(BCM2835I2SState *s, const BCM2835I2SFrame *f,
                               PinesyI2SFormat *fmt)
{
    unsigned ch;

    fmt->frame_rate = bcm2835_i2s_master(s) ? bcm2835_i2s_frame_rate(s) : 0;
    fmt->frame_bits = bcm2835_i2s_frame_bits(s);
    for (ch = 0; ch < 2; ch++) {
        fmt->width[ch] = f->chan_on[ch] ? f->width[ch] : 0;
    }
}

/*
 * TXW: the datasheet gives 00 = TX FIFO empty, 01 and 10 = less than full,
 * 11 = full but for one sample.
 */
static bool bcm2835_i2s_txw(BCM2835I2SState *s)
{
    switch (CS_TXTHR(s->cs)) {
    case 0:
        return s->tx_count == 0;
    case 3:
        return s->tx_count < BCM2835_I2S_FIFO_DEPTH - 1;
    default:
        return s->tx_count < BCM2835_I2S_FIFO_DEPTH;
    }
}

/*
 * RXR: the datasheet gives 00 = a single sample in the RX FIFO, 01 and 10
 * = at least full, 11 = full.
 */
static bool bcm2835_i2s_rxr(BCM2835I2SState *s)
{
    if (CS_RXTHR(s->cs) == 0) {
        return s->rx_count > 0;
    }
    return s->rx_count == BCM2835_I2S_FIFO_DEPTH;
}

static uint32_t bcm2835_i2s_cs_read(BCM2835I2SState *s)
{
    BCM2835I2SFrame tx, rx;
    uint32_t cs = s->cs & (CS_RW_MASK | CS_TXERR | CS_RXERR);

    bcm2835_i2s_frame_desc(s->txc, s->mode & MODE_FTXP, &tx);
    bcm2835_i2s_frame_desc(s->rxc, s->mode & MODE_FRXP, &rx);

    if (s->rx_count == BCM2835_I2S_FIFO_DEPTH) {
        cs |= CS_RXF;
    }
    if (s->tx_count == 0) {
        cs |= CS_TXE;
    }
    if (s->rx_count > 0) {
        cs |= CS_RXD;
    }
    if (s->tx_count < BCM2835_I2S_FIFO_DEPTH) {
        cs |= CS_TXD;
    }
    if (bcm2835_i2s_rxr(s)) {
        cs |= CS_RXR;
    }
    if (bcm2835_i2s_txw(s)) {
        cs |= CS_TXW;
    }
    /* frames start whole here, so in sync = a whole number of frames */
    if (bcm2835_i2s_words_per_frame(&tx) &&
        s->tx_count % bcm2835_i2s_words_per_frame(&tx) == 0) {
        cs |= CS_TXSYNC;
    }
    if (bcm2835_i2s_words_per_frame(&rx) &&
        s->rx_count % bcm2835_i2s_words_per_frame(&rx) == 0) {
        cs |= CS_RXSYNC;
    }
    return cs;
}

static void bcm2835_i2s_set_dreq(BCM2835I2SState *s, qemu_irq line,
                                 bool *level, bool want)
{
    if (*level != want) {
        *level = want;
        trace_bcm2835_i2s_dreq(line == s->dreq_tx ? "tx" : "rx", want);
        qemu_set_irq(line, want);
    }
}

/*
 * Bring the interrupt and the DMA requests up to date. A raised DREQ runs
 * the DMA at once, which can write the TX FIFO or read the RX FIFO. A
 * register access must not reach this device again that way, so from a
 * register access (now = false) a rising DREQ waits for dreq_bh.
 */
static void bcm2835_i2s_update(BCM2835I2SState *s, bool now)
{
    bool tx_want, rx_want;

    if (bcm2835_i2s_txw(s)) {
        s->intstc |= INT_TXW;
    }
    if (bcm2835_i2s_rxr(s)) {
        s->intstc |= INT_RXR;
    }
    qemu_set_irq(s->irq, !!(s->intstc & s->inten));

    tx_want = (s->cs & CS_DMAEN) && s->tx_count < DREQ_TX(s->dreq);
    if (!tx_want || now) {
        bcm2835_i2s_set_dreq(s, s->dreq_tx, &s->dreq_tx_level, tx_want);
    } else if (!s->dreq_tx_level) {
        qemu_bh_schedule(s->dreq_bh);
    }

    /* the DMA can have changed the FIFOs; look again */
    rx_want = (s->cs & CS_DMAEN) && s->rx_count > DREQ_RX(s->dreq);
    if (!rx_want || now) {
        bcm2835_i2s_set_dreq(s, s->dreq_rx, &s->dreq_rx_level, rx_want);
    } else if (!s->dreq_rx_level) {
        qemu_bh_schedule(s->dreq_bh);
    }
}

static void bcm2835_i2s_dreq_bh(void *opaque)
{
    bcm2835_i2s_update(opaque, true);
}

static bool bcm2835_i2s_tx_pop(BCM2835I2SState *s, uint32_t *word)
{
    if (s->tx_count == 0) {
        *word = 0;
        if (!(s->cs & CS_TXERR)) {
            trace_bcm2835_i2s_underrun();
        }
        s->cs |= CS_TXERR;
        s->intstc |= INT_TXERR;
        return false;
    }
    *word = s->tx_fifo[s->tx_head];
    s->tx_head = (s->tx_head + 1) % BCM2835_I2S_FIFO_DEPTH;
    s->tx_count--;
    return true;
}

static void bcm2835_i2s_rx_push(BCM2835I2SState *s, uint32_t word)
{
    if (s->rx_count == BCM2835_I2S_FIFO_DEPTH) {
        if (!(s->cs & CS_RXERR)) {
            trace_bcm2835_i2s_overrun();
        }
        s->cs |= CS_RXERR;
        s->intstc |= INT_RXERR;
        return;
    }
    s->rx_fifo[(s->rx_head + s->rx_count) % BCM2835_I2S_FIFO_DEPTH] = word;
    s->rx_count++;
}

static uint32_t bcm2835_i2s_rx_pop(BCM2835I2SState *s)
{
    uint32_t word;

    if (s->rx_count == 0) {
        return 0;
    }
    word = s->rx_fifo[s->rx_head];
    s->rx_head = (s->rx_head + 1) % BCM2835_I2S_FIFO_DEPTH;
    s->rx_count--;
    return word;
}

/* Keep the top w bits of an MSB-aligned sample */
static uint32_t bcm2835_i2s_top_bits(uint32_t v, unsigned w)
{
    return w ? v & ~(UINT32_MAX >> w) : 0;
}

static void bcm2835_i2s_tx_frame(BCM2835I2SState *s)
{
    BCM2835I2SFrame f;
    PinesyI2SFormat fmt;
    int32_t sample[2] = { 0, 0 };
    uint32_t word;
    unsigned ch;

    bcm2835_i2s_frame_desc(s->txc, s->mode & MODE_FTXP, &f);
    if (f.packed) {
        /* channel 1 in the low half; each half sent MSB first */
        bcm2835_i2s_tx_pop(s, &word);
        sample[0] = bcm2835_i2s_top_bits(word << 16, f.width[0]);
        sample[1] = bcm2835_i2s_top_bits(word & 0xffff0000u, f.width[1]);
    } else {
        /* one right-justified word per enabled channel */
        for (ch = 0; ch < 2; ch++) {
            if (f.chan_on[ch]) {
                bcm2835_i2s_tx_pop(s, &word);
                sample[ch] = word << (32 - f.width[ch]);
            }
        }
    }

    trace_bcm2835_i2s_tx(sample[0], sample[1], s->tx_count);
    if (bcm2835_i2s_handlers.tx) {
        bcm2835_i2s_format(s, &f, &fmt);
        bcm2835_i2s_handlers.tx(bcm2835_i2s_opaque, sample, &fmt);
    }
}

static void bcm2835_i2s_rx_frame(BCM2835I2SState *s)
{
    BCM2835I2SFrame f;
    PinesyI2SFormat fmt;
    int32_t sample[2] = { 0, 0 };
    uint32_t v;
    unsigned ch;

    bcm2835_i2s_frame_desc(s->rxc, s->mode & MODE_FRXP, &f);
    if (bcm2835_i2s_handlers.rx) {
        bcm2835_i2s_format(s, &f, &fmt);
        bcm2835_i2s_handlers.rx(bcm2835_i2s_opaque, sample, &fmt);
    }
    trace_bcm2835_i2s_rx(sample[0], sample[1], s->rx_count);

    if (f.packed) {
        /* first channel in the low half, each padded to 16 bits */
        v = bcm2835_i2s_top_bits(sample[0], f.width[0]) >> 16;
        v |= bcm2835_i2s_top_bits(sample[1], f.width[1]) & 0xffff0000u;
        bcm2835_i2s_rx_push(s, v);
        return;
    }
    for (ch = 0; ch < 2; ch++) {
        if (!f.chan_on[ch]) {
            continue;
        }
        v = (uint32_t)sample[ch] >> (32 - f.width[ch]);
        if (s->cs & CS_RXSEX) {
            v = sextract32(v, 0, f.width[ch]);
        }
        bcm2835_i2s_rx_push(s, v);
    }
}

/* One frame on the serial side */
static void bcm2835_i2s_frame(BCM2835I2SState *s)
{
    if (!(s->cs & CS_EN)) {
        return;
    }
    if (s->cs & CS_TXON) {
        bcm2835_i2s_tx_frame(s);
    }
    if (s->cs & CS_RXON) {
        bcm2835_i2s_rx_frame(s);
    }
    bcm2835_i2s_update(s, true);
}

static bool bcm2835_i2s_clock_wanted(BCM2835I2SState *s)
{
    return bcm2835_i2s_master(s) && (s->cs & CS_EN) &&
           (s->cs & (CS_TXON | CS_RXON)) && bcm2835_i2s_frame_rate(s) > 0;
}

/* Master mode: frames due by now since clock_start */
static uint64_t bcm2835_i2s_frames_due(BCM2835I2SState *s, int64_t now)
{
    return clock_ns_to_ticks(s->clk, now - s->clock_start) /
           bcm2835_i2s_frame_bits(s);
}

/* Master mode: run the frames that are due */
static void bcm2835_i2s_run_frames(BCM2835I2SState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t due = bcm2835_i2s_frames_due(s, now);
    uint64_t max = (uint64_t)bcm2835_i2s_frame_rate(s) + 1;

    if (due <= s->frames_done) {
        return;
    }
    /* more than a second behind: drop the excess rather than stall */
    if (due - s->frames_done > max) {
        s->frames_done = due - max;
    }
    while (s->frames_done < due && s->clock_running) {
        s->frames_done++;
        bcm2835_i2s_frame(s);
    }
}

/*
 * Start, stop or restart the master frame clock after a change to the
 * registers or the PCM clock. A restart counts frames from now.
 */
static void bcm2835_i2s_clock_update(BCM2835I2SState *s, bool restart)
{
    if (!bcm2835_i2s_clock_wanted(s)) {
        if (s->clock_running) {
            trace_bcm2835_i2s_clock_stop();
        }
        s->clock_running = false;
        timer_del(&s->frame_timer);
        return;
    }
    if (!s->clock_running || restart) {
        s->clock_running = true;
        s->clock_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->frames_done = 0;
        trace_bcm2835_i2s_clock_start(clock_get_hz(s->clk),
                                      bcm2835_i2s_frame_bits(s));
    }
    if (!timer_pending(&s->frame_timer)) {
        timer_mod(&s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_TICK_NS);
    }
}

static void bcm2835_i2s_frame_tick(void *opaque)
{
    BCM2835I2SState *s = opaque;

    if (s->clock_running) {
        bcm2835_i2s_run_frames(s);
    }
    if (s->clock_running) {
        timer_mod(&s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_TICK_NS);
    }
}

static void bcm2835_i2s_clk_update(void *opaque, ClockEvent event)
{
    BCM2835I2SState *s = opaque;

    trace_bcm2835_i2s_clk(clock_get_hz(s->clk));
    bcm2835_i2s_clock_update(s, true);
}

/* Slave mode: run the frames the host has clocked */
static void bcm2835_i2s_host_bh(void *opaque)
{
    BCM2835I2SState *s = opaque;
    uint32_t n = qatomic_xchg(&s->host_frames, 0);

    trace_bcm2835_i2s_host_clock(n, bcm2835_i2s_master(s));
    if (bcm2835_i2s_master(s)) {
        return;
    }
    while (n--) {
        bcm2835_i2s_frame(s);
    }
}

void pinesy_bridge_set_i2s(const PinesyI2SHandlers *handlers, void *opaque)
{
    bcm2835_i2s_handlers = *handlers;
    bcm2835_i2s_opaque = opaque;
}

void pinesy_bridge_i2s_clock(uint32_t frames)
{
    BCM2835I2SState *s = qatomic_load_acquire(&bcm2835_i2s_dev);

    if (s && frames) {
        qatomic_add(&s->host_frames, frames);
        qemu_bh_schedule(s->host_bh);
    }
}

static void bcm2835_i2s_cs_write(BCM2835I2SState *s, uint32_t value)
{
    uint32_t old = s->cs;

    s->cs &= ~(value & (CS_TXERR | CS_RXERR));
    s->cs = (s->cs & ~CS_RW_MASK) | (value & CS_RW_MASK);

    if (value & CS_TXCLR) {
        s->tx_head = 0;
        s->tx_count = 0;
    }
    if (value & CS_RXCLR) {
        s->rx_head = 0;
        s->rx_count = 0;
    }
    if ((old ^ s->cs) & (CS_EN | CS_TXON | CS_RXON)) {
        bcm2835_i2s_clock_update(s, false);
    }
}

static uint64_t bcm2835_i2s_read(void *opaque, hwaddr offset, unsigned size)
{
    BCM2835I2SState *s = opaque;
    uint32_t value = 0;

    switch (offset) {
    case PCM_CS_A:
        value = bcm2835_i2s_cs_read(s);
        break;
    case PCM_FIFO_A:
        value = bcm2835_i2s_rx_pop(s);
        bcm2835_i2s_update(s, false);
        break;
    case PCM_MODE_A:
        value = s->mode;
        break;
    case PCM_RXC_A:
        value = s->rxc;
        break;
    case PCM_TXC_A:
        value = s->txc;
        break;
    case PCM_DREQ_A:
        value = s->dreq;
        break;
    case PCM_INTEN_A:
        value = s->inten;
        break;
    case PCM_INTSTC_A:
        value = s->intstc;
        break;
    case PCM_GRAY:
        /* FLUSH completes at once with no bits: FLUSHED and RXLEVEL 0 */
        value = (s->gray & (GRAY_EN | GRAY_CLR)) | (s->rx_count << 16);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        break;
    }
    trace_bcm2835_i2s_read(offset, value);
    return value;
}

static void bcm2835_i2s_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BCM2835I2SState *s = opaque;

    trace_bcm2835_i2s_write(offset, value);
    switch (offset) {
    case PCM_CS_A:
        bcm2835_i2s_cs_write(s, value);
        break;
    case PCM_FIFO_A:
        if (s->tx_count < BCM2835_I2S_FIFO_DEPTH) {
            s->tx_fifo[(s->tx_head + s->tx_count) % BCM2835_I2S_FIFO_DEPTH] =
                value;
            s->tx_count++;
        }
        break;
    case PCM_MODE_A:
        s->mode = value & MODE_MASK;
        if (s->mode & MODE_PDME) {
            qemu_log_mask(LOG_UNIMP, "%s: PDM input mode\n", __func__);
        }
        bcm2835_i2s_clock_update(s, true);
        break;
    case PCM_RXC_A:
        s->rxc = value;
        break;
    case PCM_TXC_A:
        s->txc = value;
        break;
    case PCM_DREQ_A:
        s->dreq = value & DREQ_MASK;
        break;
    case PCM_INTEN_A:
        s->inten = value & INT_MASK;
        break;
    case PCM_INTSTC_A:
        s->intstc &= ~(value & INT_MASK);
        break;
    case PCM_GRAY:
        s->gray = value & (GRAY_EN | GRAY_CLR);
        if (s->gray & GRAY_EN) {
            qemu_log_mask(LOG_UNIMP, "%s: GRAY mode\n", __func__);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }
    bcm2835_i2s_update(s, false);
}

static const MemoryRegionOps bcm2835_i2s_ops = {
    .read = bcm2835_i2s_read,
    .write = bcm2835_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static int bcm2835_i2s_post_load(void *opaque, int version_id)
{
    BCM2835I2SState *s = opaque;

    if (s->tx_head >= BCM2835_I2S_FIFO_DEPTH ||
        s->tx_count > BCM2835_I2S_FIFO_DEPTH ||
        s->rx_head >= BCM2835_I2S_FIFO_DEPTH ||
        s->rx_count > BCM2835_I2S_FIFO_DEPTH) {
        return -EINVAL;
    }
    return 0;
}

static const VMStateDescription vmstate_bcm2835_i2s = {
    .name = TYPE_BCM2835_I2S,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = bcm2835_i2s_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cs, BCM2835I2SState),
        VMSTATE_UINT32(mode, BCM2835I2SState),
        VMSTATE_UINT32(rxc, BCM2835I2SState),
        VMSTATE_UINT32(txc, BCM2835I2SState),
        VMSTATE_UINT32(dreq, BCM2835I2SState),
        VMSTATE_UINT32(inten, BCM2835I2SState),
        VMSTATE_UINT32(intstc, BCM2835I2SState),
        VMSTATE_UINT32(gray, BCM2835I2SState),
        VMSTATE_UINT32_ARRAY(tx_fifo, BCM2835I2SState,
                             BCM2835_I2S_FIFO_DEPTH),
        VMSTATE_UINT32(tx_head, BCM2835I2SState),
        VMSTATE_UINT32(tx_count, BCM2835I2SState),
        VMSTATE_UINT32_ARRAY(rx_fifo, BCM2835I2SState,
                             BCM2835_I2S_FIFO_DEPTH),
        VMSTATE_UINT32(rx_head, BCM2835I2SState),
        VMSTATE_UINT32(rx_count, BCM2835I2SState),
        VMSTATE_BOOL(dreq_tx_level, BCM2835I2SState),
        VMSTATE_BOOL(dreq_rx_level, BCM2835I2SState),
        VMSTATE_BOOL(clock_running, BCM2835I2SState),
        VMSTATE_INT64(clock_start, BCM2835I2SState),
        VMSTATE_UINT64(frames_done, BCM2835I2SState),
        VMSTATE_TIMER(frame_timer, BCM2835I2SState),
        VMSTATE_CLOCK(clk, BCM2835I2SState),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_i2s_reset(DeviceState *dev)
{
    BCM2835I2SState *s = BCM2835_I2S(dev);

    s->cs = 0;
    s->mode = 0;
    s->rxc = 0;
    s->txc = 0;
    s->dreq = DREQ_RESET;
    s->inten = 0;
    s->intstc = 0;
    s->gray = 0;
    s->tx_head = 0;
    s->tx_count = 0;
    s->rx_head = 0;
    s->rx_count = 0;
    s->clock_running = false;
    s->frames_done = 0;
    timer_del(&s->frame_timer);
    s->dreq_tx_level = false;
    s->dreq_rx_level = false;
    qemu_set_irq(s->dreq_tx, 0);
    qemu_set_irq(s->dreq_rx, 0);
    qemu_set_irq(s->irq, 0);
}

static void bcm2835_i2s_init(Object *obj)
{
    BCM2835I2SState *s = BCM2835_I2S(obj);

    memory_region_init_io(&s->iomem, obj, &bcm2835_i2s_ops, s,
                          TYPE_BCM2835_I2S, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->dreq_tx, "dreq-tx", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->dreq_rx, "dreq-rx", 1);
    s->clk = qdev_init_clock_in(DEVICE(s), "clk", bcm2835_i2s_clk_update, s,
                                ClockUpdate);
    timer_init_ns(&s->frame_timer, QEMU_CLOCK_VIRTUAL,
                  bcm2835_i2s_frame_tick, s);
}

static void bcm2835_i2s_realize(DeviceState *dev, Error **errp)
{
    BCM2835I2SState *s = BCM2835_I2S(dev);

    /*
     * Not guarded against re-entry: both run the DMA, which then writes
     * or reads this device's FIFO.
     */
    s->dreq_bh = qemu_bh_new(bcm2835_i2s_dreq_bh, s);
    s->host_bh = qemu_bh_new(bcm2835_i2s_host_bh, s);
    qatomic_store_release(&bcm2835_i2s_dev, s);
}

static void bcm2835_i2s_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_i2s_realize;
    device_class_set_legacy_reset(dc, bcm2835_i2s_reset);
    dc->vmsd = &vmstate_bcm2835_i2s;
}

static const TypeInfo bcm2835_i2s_info = {
    .name          = TYPE_BCM2835_I2S,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835I2SState),
    .class_init    = bcm2835_i2s_class_init,
    .instance_init = bcm2835_i2s_init,
};

static void bcm2835_i2s_register_types(void)
{
    type_register_static(&bcm2835_i2s_info);
}

type_init(bcm2835_i2s_register_types)
