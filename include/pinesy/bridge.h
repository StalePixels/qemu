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
#include <stdint.h>

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

/*
 * I2S audio: the Pi's PCM block (BCM2835 PCM/I2S, hw/audio/bcm2835_i2s.c).
 *
 * TX is PCM_DOUT, the Pi sending to the host. RX is PCM_DIN, the host
 * sending to the Pi. The host sees the PCM block one frame at a time.
 *
 * Clocking. MODE_A decides who makes the frames:
 *   - Pi is master (CLKM = 0 and FSM = 0, Linux "bitclock-master" and
 *     "frame-master" on the Pi's side). QEMU makes the frames from its own
 *     virtual clock at the rate Linux set: the CPRMAN PCM clock divided by
 *     the frame length (MODE_A FLEN + 1). pinesy_bridge_i2s_clock() is
 *     ignored.
 *   - Pi is slave (CLKM = 1 or FSM = 1). Frames happen only when the host
 *     calls pinesy_bridge_i2s_clock(). QEMU never makes a frame itself.
 *
 * Each frame, while the PCM block is enabled (CS_A EN):
 *   - TXON set: QEMU takes the frame's words from the TX FIFO and calls tx.
 *   - RXON set: QEMU calls rx, then puts the frame's words in the RX FIFO.
 * The FIFOs are fed and drained by the guest (DMA paced by DREQ, or the
 * CPU), as on the hardware.
 *
 * Sample format. A frame is two channels: sample[0] is channel 1 and
 * sample[1] is channel 2 of TXC_A (tx) or RXC_A (rx). With Linux's I2S
 * format, channel 1 is left and channel 2 is right. Each sample is a
 * signed 32-bit value, MSB-aligned: a channel of width w bits fills bits
 * 31 to 32 - w, and the bits below are 0. A host that wants 16 bits takes
 * sample >> 16, whatever w is. format->width[] gives w for each channel;
 * 0 means the channel is disabled, its sample is 0 (tx) or ignored (rx).
 *
 * Underrun and overrun, as the hardware does:
 *   - TX FIFO empty when a channel needs a word: that channel's sample is
 *     0, CS_A TXERR is set, and tx is still called. A two-channel frame
 *     that gets only one word keeps channel 1 first, so later frames can
 *     have the channels swapped, as on a real Pi.
 *   - RX FIFO full when a word arrives: the word is dropped and CS_A RXERR
 *     is set.
 *   - The host cannot underrun or overrun: rx must fill both samples every
 *     time (with 0 when it has nothing), and tx must take what it gets.
 *
 * Threads. tx and rx run on QEMU's main loop thread (the thread that
 * called pinesy_bridge_run()), with QEMU's big lock held, from a QEMU timer
 * (master) or a bottom half (slave). Not on the host's own threads: guard
 * anything they share with the host. They must return quickly, and must
 * not call any QEMU function except pinesy_bridge_i2s_clock(). Several
 * frames can arrive in one burst: in master mode QEMU makes the frames
 * that fell due since the last burst, about every millisecond of virtual
 * time; in slave mode, the frames of each pinesy_bridge_i2s_clock() call.
 */
typedef struct PinesyI2SFormat {
    /*
     * Frames per second when the Pi is master; 0 when the host clocks the
     * frames.
     */
    double frame_rate;
    /* Bit clocks per frame (MODE_A FLEN + 1). */
    uint32_t frame_bits;
    /* Width in bits of channel 1 and channel 2; 0 = channel disabled. */
    uint8_t width[2];
} PinesyI2SFormat;

typedef struct PinesyI2SHandlers {
    /* One frame the Pi sent. Either handler can be NULL. */
    void (*tx)(void *opaque, const int32_t sample[2],
               const PinesyI2SFormat *format);
    /* Fill in the frame the Pi receives now. */
    void (*rx)(void *opaque, int32_t sample[2],
               const PinesyI2SFormat *format);
} PinesyI2SHandlers;

/*
 * Set the I2S handlers and their opaque argument. QEMU copies *handlers.
 * Call it before pinesy_bridge_run(), on the thread that then calls it.
 */
void pinesy_bridge_set_i2s(const PinesyI2SHandlers *handlers, void *opaque);

/*
 * Slave mode: the host's I2S clock has made this many more frames. Safe
 * from any thread, and takes no lock: it adds to a count and schedules a
 * QEMU bottom half, which runs the frames on QEMU's main loop thread, with
 * the big lock, soon after. Frames given while the Pi is master or before
 * the PCM block exists are dropped. Call it only between the start and the
 * end of pinesy_bridge_run().
 */
void pinesy_bridge_i2s_clock(uint32_t frames);

/*
 * SPI: a slave on chip select 0 of the Pi's SPI0 master (BCM2835 SPI,
 * hw/ssi/bcm2835_spi.c), /dev/spidev0.0 in Linux.
 *
 * The Pi is the master. For every byte it sends on MOSI the host gives
 * the byte it receives on MISO. Only bytes sent while chip select 0 is
 * active go to the host. SPI mode (CPOL, CPHA) and clock rate are not
 * passed on; bytes go MSB first.
 *
 * Chip select 0 is active when the guest drives GPIO 8 (CE0) low as a GPIO
 * output, which is what Linux does ("cs-gpios" in the device tree), or
 * while a transfer is active (CS TA) with the native CS field at 0. CS
 * polarity bits are ignored. select is called on every change, with
 * selected = true when chip select 0 becomes active.
 *
 * Transfers. When the controller has bytes to send (CS TA set, bytes in
 * the TX FIFO, room for as many in the RX FIFO, and in DMA mode, DLEN not
 * yet reached), it gives them all to transfer in one call: from 1 to 64
 * bytes, in the order sent. The host answers with exactly one
 * pinesy_bridge_spi_reply() call, with as many bytes, MISO byte i being
 * the byte received while MOSI byte i is sent. Until that answer arrives
 * the controller sends nothing more: TA stays set, DONE stays clear, the
 * bytes are not in the RX FIFO, and the DMA requests stay as the FIFO
 * levels make them. The guest sees a slower slave, never wrong data.
 * Each MISO byte may depend on all MOSI bytes before it, as on a real
 * slave, because the host has them all when it answers.
 *
 * How long the guest waits. The host can answer inside transfer (the
 * fastest way: the bytes go into the RX FIFO before the guest's register
 * access returns), or later from any thread: the answer then schedules a
 * QEMU bottom half, which puts the bytes in the RX FIFO on the main loop
 * thread, usually within tens of microseconds. There is no limit in QEMU:
 * the controller waits for ever. The guest's own limits apply, in its
 * virtual time, which runs with the host's clock. Linux 6.12 fails a
 * transfer with ETIMEDOUT after (16000 * length / speed_hz + 200) ms,
 * speed_hz the speed asked for: a host must answer a whole transfer of
 * fewer than 400 bytes at 2 MHz within 200 ms. Linux sends a transfer in
 * one batch per FIFO's worth (up to 64 bytes, in DMA mode about 36), so
 * it takes about length / 36 answers.
 *
 * A transfer stopped before the answer (the guest clears TA, or the
 * machine resets) still needs its answer; QEMU drops it. Chip select can
 * change while an answer is owed; the host still answers.
 *
 * No handlers set (transfer NULL): chip select 0 behaves as before, the
 * bytes go to QEMU's own SSI bus, which has no device and reads 0.
 * Chip selects 1 and 2 always do.
 *
 * Threads. select and transfer run on a QEMU thread with QEMU's big lock
 * held: the vCPU thread during a guest register access, or the main loop
 * thread (the thread that called pinesy_bridge_run()) from a bottom half
 * or a DMA transfer. Not on the host's own threads: guard anything they
 * share with the host. They must return quickly, must not wait for the
 * host's other threads, and must not call any QEMU function except
 * pinesy_bridge_spi_reply().
 */
typedef struct PinesySPIHandlers {
    /* Chip select 0 became active (true) or inactive (false). Can be NULL. */
    void (*select)(void *opaque, bool selected);
    /* len bytes (1 to 64) the Pi sends; answer with pinesy_bridge_spi_reply */
    void (*transfer)(void *opaque, const uint8_t *mosi, uint32_t len);
} PinesySPIHandlers;

/*
 * Set the SPI handlers and their opaque argument. QEMU copies *handlers.
 * Call it before pinesy_bridge_run(), on the thread that then calls it.
 */
void pinesy_bridge_set_spi(const PinesySPIHandlers *handlers, void *opaque);

/*
 * Answer the last transfer call: len MISO bytes, len equal to that call's
 * len. QEMU copies the bytes before it returns. Safe from any thread,
 * including from inside transfer, and takes no lock. Call it once per
 * transfer call, and only between the start and the end of
 * pinesy_bridge_run(). A shorter answer is padded with 0xff and logged as
 * a guest error.
 */
void pinesy_bridge_spi_reply(const uint8_t *miso, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* PINESY_BRIDGE_H */
