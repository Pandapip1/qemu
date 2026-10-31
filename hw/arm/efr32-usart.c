/*
 * EFR32 Series 2 USART, asynchronous mode.
 *
 * Register layout and field encodings are the public EFR32BG22 device
 * header's (hal_silabs .../EFR32BG22/Include/efr32bg22_usart.h, USART_TypeDef
 * and the _USART_* field macros); the asynchronous baud formula is emlib's
 * USART_BaudrateCalc() (em_usart.c): br = 256 * fPCLK / (ovs * (256 + DIV)).
 *
 * The line side is a QEMU character device.  Each frame occupies the wire
 * for its real duration in virtual time (start bit, data bits, parity and
 * stop bits at the programmed baud rate), so the guest sees TXBL, TXC and
 * RXDATAV with silicon's pacing, and whatever owns the host side (a tty,
 * /dev/vhci, a socket) sees bytes at the line rate.
 *
 * Modelled: EN, CTRL (OVS, TXBIL, LOOPBK), FRAME, CMD (RX/TX enable and
 * disable, CLEARRX/CLEARTX), STATUS, CLKDIV, the RX/TX data registers
 * including the DOUBLE and peek forms, IF/IEN with the SET/CLR/TGL aliases,
 * and the RX and TX interrupt lines.  Not modelled: synchronous secondary
 * mode, AUTOCS, IrDA, I2S, the USART timer (TIMECMPn), multi-processor and
 * 9-bit frames, parity/framing errors (the chardev carries bytes, not line
 * levels) and PRS triggers; enabling any of them is reported as
 * unimplemented.
 *
 * Synchronous main mode (CTRL.SYNC with CMD.MASTEREN, RM SS21.3.3): the
 * line side is the USART's SSI bus ("ssi"), to which SPI devices attach as
 * they would to MOSI/MISO/SCLK: a board model, or a host spidev through a
 * passthrough device.  Chip selects are GPIO pins driven by software, as
 * the stock and Zephyr drivers do.  Each frame takes its data bits at the
 * synchronous bit rate br = fPCLK / (2 x (1 + CLKDIV / 256)) (RM Figure
 * 21.17); at its end the byte the device returned enters the receiver as
 * the last sample edge sets RXDATAV (RM SS21.3.3.2).  The bus carries
 * MSB-first bytes: with CTRL.MSBF clear the USART reverses the bit order on
 * both directions, which is what the wire order means to the device.
 * CLKPOL/CLKPHA have no effect on the bytes exchanged with a device that
 * samples in the same SPI mode; a mode mismatch is not modelled.
 *
 * Flow control: while the receiver is disabled the line is held (the host
 * side's data waits in its chardev, as a host waits on a deasserted RTS).
 * While enabled, frames arrive back-to-back at the line rate and a frame
 * that completes with the RX buffer full is lost with RXOF, as on silicon.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/fifo8.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "sysemu/sysemu.h"
#include "hw/arm/efr32-usart.h"
#include "hw/ssi/ssi.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32UsartState, EFR32_USART)

/* USART_TypeDef word offsets. */
enum {
    USART_IPVERSION = 0x00,
    USART_EN = 0x04,
    USART_CTRL = 0x08,
    USART_FRAME = 0x0c,
    USART_TRIGCTRL = 0x10,
    USART_CMD = 0x14,
    USART_STATUS = 0x18,
    USART_CLKDIV = 0x1c,
    USART_RXDATAX = 0x20,
    USART_RXDATA = 0x24,
    USART_RXDOUBLEX = 0x28,
    USART_RXDOUBLE = 0x2c,
    USART_RXDATAXP = 0x30,
    USART_RXDOUBLEXP = 0x34,
    USART_TXDATAX = 0x38,
    USART_TXDATA = 0x3c,
    USART_TXDOUBLEX = 0x40,
    USART_TXDOUBLE = 0x44,
    USART_IF = 0x48,
    USART_IEN = 0x4c,
    USART_IRCTRL = 0x50,
    USART_I2SCTRL = 0x54,
    USART_TIMING = 0x58,
    USART_CTRLX = 0x5c,
    USART_TIMECMP0 = 0x60,
    USART_TIMECMP1 = 0x64,
    USART_TIMECMP2 = 0x68,
    USART_REGISTER_END = 0x6c,
};

/* Series 2 register aliases: bits 13:12 of the offset. */
#define USART_ALIAS_SHIFT 12
#define USART_ALIAS_MASK 0x3000u
enum { USART_ALIAS_NONE, USART_ALIAS_SET, USART_ALIAS_CLR, USART_ALIAS_TGL };

#define USART_EN_EN (1u << 0)

#define USART_CTRL_SYNC (1u << 0)
#define USART_CTRL_LOOPBK (1u << 1)
#define USART_CTRL_OVS_SHIFT 5
#define USART_CTRL_OVS_MASK 0x60u
#define USART_CTRL_TXBIL (1u << 12)
#define USART_CTRL_MSBF (1u << 10)
#define USART_CTRL_AUTOCS (1u << 16)
/* Synchronous mode: two PCLK cycles per bit at CLKDIV = 0 (RM Fig. 21.17). */
#define USART_SYNC_PCLK_PER_BIT 2u

#define USART_FRAME_RESETVALUE 0x00001005u
#define USART_FRAME_DATABITS_MASK 0xfu
#define USART_FRAME_DATABITS_EIGHT 5u
/* _USART_FRAME_DATABITS_FOUR is 1: the field is the data width less 3. */
#define USART_FRAME_DATABITS_BIAS 3u
#define USART_FRAME_PARITY_SHIFT 8
#define USART_FRAME_PARITY_MASK 0x300u
#define USART_FRAME_PARITY_NONE 0u
#define USART_FRAME_PARITY_EVEN 2u
#define USART_FRAME_PARITY_ODD 3u
#define USART_FRAME_STOPBITS_SHIFT 12
#define USART_FRAME_STOPBITS_MASK 0x3000u
/* _USART_FRAME_STOPBITS_HALF is 0: the field is the stop length in half
 * bits, less one. */
#define USART_FRAME_STOPBITS_HALF_BIAS 1u

#define USART_CMD_RXEN (1u << 0)
#define USART_CMD_RXDIS (1u << 1)
#define USART_CMD_TXEN (1u << 2)
#define USART_CMD_TXDIS (1u << 3)
#define USART_CMD_CLEARTX (1u << 10)
#define USART_CMD_CLEARRX (1u << 11)
#define USART_CMD_MASTEREN (1u << 4)
#define USART_CMD_MASTERDIS (1u << 5)

#define USART_STATUS_RXENS (1u << 0)
#define USART_STATUS_TXENS (1u << 1)
#define USART_STATUS_MASTER (1u << 2)
#define USART_STATUS_TXC (1u << 5)
#define USART_STATUS_TXBL (1u << 6)
#define USART_STATUS_RXDATAV (1u << 7)
#define USART_STATUS_RXFULL (1u << 8)
#define USART_STATUS_TXIDLE (1u << 13)

#define USART_CLKDIV_DIV_MASK 0x7ffff8u
/* emlib: br = 256 * fPCLK / (ovs * (256 + CLKDIV.DIV)). */
#define USART_CLKDIV_UNITY 256u

#define USART_IF_TXC (1u << 0)
#define USART_IF_TXBL (1u << 1)
#define USART_IF_RXDATAV (1u << 2)
#define USART_IF_RXFULL (1u << 3)
#define USART_IF_RXOF (1u << 4)
#define USART_IF_RXUF (1u << 5)
#define USART_IF_TXOF (1u << 6)
#define USART_IF_TXUF (1u << 7)
#define USART_IF_PERR (1u << 8)
#define USART_IF_FERR (1u << 9)
#define USART_IF_MPAF (1u << 10)
#define USART_IF_SSM (1u << 11)
#define USART_IF_CCF (1u << 12)
#define USART_IF_TXIDLE (1u << 13)
#define USART_IF_TCMP0 (1u << 14)
#define USART_IF_TCMP1 (1u << 15)
#define USART_IF_TCMP2 (1u << 16)
#define USART_IF_MASK 0x1ffffu
/* Flags that follow their buffer condition rather than latching. */
#define USART_IF_LEVEL (USART_IF_TXBL | USART_IF_RXDATAV | USART_IF_RXFULL)
/*
 * Which flags drive the RX and which the TX interrupt line.  The device
 * header names the two lines (USART0_RX_IRQn, USART0_TX_IRQn) but not the
 * split; this one follows the flag names and the receive/transmit timer
 * comparators the Zephyr async driver uses (TCMP1 for RX timeout, TCMP2
 * for TX).  Zephyr routes both lines to one handler, so the split is not
 * observable to it.  Unverified on silicon.
 */
#define USART_IF_RX_LINE (USART_IF_RXDATAV | USART_IF_RXFULL | USART_IF_RXOF | \
                          USART_IF_RXUF | USART_IF_PERR | USART_IF_FERR |     \
                          USART_IF_MPAF | USART_IF_TCMP1)
#define USART_IF_TX_LINE (USART_IF_MASK & ~USART_IF_RX_LINE)

#define USART_RXDATAX_RXDATA_MASK 0x1ffu
#define USART_TXDATAX_TXDATAX_MASK 0x1ffu
/* RXDOUBLE/TXDOUBLE: the second frame in bits 15:8 (16:24 for the X form). */
#define USART_DOUBLE_SHIFT 8
#define USART_DOUBLEX_SHIFT 16

/*
 * Buffer depths in frames, excluding the shift registers: the DOUBLE data
 * registers move two frames at once and STATUS.RXFULL/TXBIL HALFFULL are
 * defined against a two-frame buffer.
 */
#define USART_BUFFER_FRAMES 2

/* Register words kept as written (offset / 4). */
#define USART_REGISTER_WORDS (USART_REGISTER_END / 4)

struct Efr32UsartState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    qemu_irq rx_irq;
    qemu_irq tx_irq;
    Clock *pclk;
    CharBackend chr;
    IcountBusModel *bus_model;
    hwaddr base;
    uint32_t index;

    uint32_t regs[USART_REGISTER_WORDS];
    uint32_t if_latched;
    bool rx_enabled;
    bool tx_enabled;
    bool tx_complete;
    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
    /* Synchronous main mode: CMD.MASTEREN, and the SPI bus it drives. */
    bool master;
    SSIBus *ssi;
    /* The frame on the wire towards the host, and when it ends. */
    bool tx_shifting;
    uint8_t tx_shift;
    QEMUTimer *tx_timer;
    /* A frame from the host is on the wire into the receiver. */
    bool rx_shifting;
    uint8_t rx_shift;
    QEMUTimer *rx_timer;
};

static uint32_t usart_reg(Efr32UsartState *s, hwaddr offset)
{
    return s->regs[offset / 4];
}

/* Duration of one frame on the wire at the programmed format and rate. */
static int64_t usart_frame_ns(Efr32UsartState *s)
{
    static const unsigned oversample[] = {16, 8, 6, 4};
    uint32_t ctrl = usart_reg(s, USART_CTRL);
    uint32_t frame = usart_reg(s, USART_FRAME);
    uint64_t pclk_hz = clock_get_hz(s->pclk);
    unsigned ovs = oversample[(ctrl & USART_CTRL_OVS_MASK) >>
                              USART_CTRL_OVS_SHIFT];
    unsigned data_bits = (frame & USART_FRAME_DATABITS_MASK) +
                         USART_FRAME_DATABITS_BIAS;
    unsigned parity_bits = ((frame & USART_FRAME_PARITY_MASK) >>
                            USART_FRAME_PARITY_SHIFT) !=
                           USART_FRAME_PARITY_NONE;
    unsigned stop_half_bits = ((frame & USART_FRAME_STOPBITS_MASK) >>
                               USART_FRAME_STOPBITS_SHIFT) +
                              USART_FRAME_STOPBITS_HALF_BIAS;
    /* One start bit, counted in half bits like the stop bits. */
    unsigned half_bits = 2 * (1 + data_bits + parity_bits) + stop_half_bits;
    uint64_t divisor = USART_CLKDIV_UNITY +
                       (usart_reg(s, USART_CLKDIV) & USART_CLKDIV_DIV_MASK);

    if (!pclk_hz) {
        return -1;
    }
    if (ctrl & USART_CTRL_SYNC) {
        /* data_bits * 2 * (256 + DIV) / (256 * fPCLK) seconds. */
        return muldiv64((uint64_t)data_bits * USART_SYNC_PCLK_PER_BIT *
                            divisor,
                        NANOSECONDS_PER_SECOND,
                        USART_CLKDIV_UNITY * pclk_hz);
    }
    /* half_bits / 2 * ovs * (256 + DIV) / (256 * fPCLK) seconds. */
    return muldiv64((uint64_t)half_bits * ovs * divisor, NANOSECONDS_PER_SECOND,
                    2 * USART_CLKDIV_UNITY * pclk_hz);
}

static bool usart_tx_buffer_level(Efr32UsartState *s)
{
    uint32_t used = fifo8_num_used(&s->tx_fifo);

    return usart_reg(s, USART_CTRL) & USART_CTRL_TXBIL ?
           used <= USART_BUFFER_FRAMES / 2 : used == 0;
}

static bool usart_tx_idle(Efr32UsartState *s)
{
    return !s->tx_shifting && fifo8_is_empty(&s->tx_fifo);
}

static uint32_t usart_status(Efr32UsartState *s)
{
    return (s->rx_enabled ? USART_STATUS_RXENS : 0) |
           (s->tx_enabled ? USART_STATUS_TXENS : 0) |
           (s->master ? USART_STATUS_MASTER : 0) |
           (s->tx_complete ? USART_STATUS_TXC : 0) |
           (usart_tx_buffer_level(s) ? USART_STATUS_TXBL : 0) |
           (!fifo8_is_empty(&s->rx_fifo) ? USART_STATUS_RXDATAV : 0) |
           (fifo8_is_full(&s->rx_fifo) ? USART_STATUS_RXFULL : 0) |
           (usart_tx_idle(s) ? USART_STATUS_TXIDLE : 0);
}

static uint32_t usart_if(Efr32UsartState *s)
{
    uint32_t status = usart_status(s);

    return s->if_latched |
           (status & USART_STATUS_TXBL ? USART_IF_TXBL : 0) |
           (status & USART_STATUS_RXDATAV ? USART_IF_RXDATAV : 0) |
           (status & USART_STATUS_RXFULL ? USART_IF_RXFULL : 0);
}

static void usart_update_irq(Efr32UsartState *s)
{
    uint32_t pending = usart_if(s) & usart_reg(s, USART_IEN);

    qemu_set_irq(s->rx_irq, !!(pending & USART_IF_RX_LINE));
    qemu_set_irq(s->tx_irq, !!(pending & USART_IF_TX_LINE));
}

/* Tell the host side the line format, for a backend that has one (a tty). */
static void usart_update_line_params(Efr32UsartState *s)
{
    static const char parity_names[] = {'N', 'N', 'E', 'O'};
    uint32_t frame = usart_reg(s, USART_FRAME);
    int64_t frame_ns = usart_frame_ns(s);
    uint64_t pclk_hz = clock_get_hz(s->pclk);
    static const unsigned oversample[] = {16, 8, 6, 4};
    unsigned ovs = oversample[(usart_reg(s, USART_CTRL) & USART_CTRL_OVS_MASK)
                              >> USART_CTRL_OVS_SHIFT];
    QEMUSerialSetParams params;

    if (frame_ns <= 0) {
        return;
    }
    params.speed = muldiv64(USART_CLKDIV_UNITY, pclk_hz,
                            (uint64_t)ovs * (USART_CLKDIV_UNITY +
                             (usart_reg(s, USART_CLKDIV) &
                              USART_CLKDIV_DIV_MASK)));
    params.parity = parity_names[(frame & USART_FRAME_PARITY_MASK) >>
                                 USART_FRAME_PARITY_SHIFT];
    params.data_bits = (frame & USART_FRAME_DATABITS_MASK) +
                       USART_FRAME_DATABITS_BIAS;
    /* Half and one-and-a-half stop bits have no termios form: round up. */
    params.stop_bits = (frame & USART_FRAME_STOPBITS_MASK) >>
                       USART_FRAME_STOPBITS_SHIFT > 1 ? 2 : 1;
    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &params);
}

static void usart_check_unimplemented(Efr32UsartState *s)
{
    uint32_t data_bits = usart_reg(s, USART_FRAME) & USART_FRAME_DATABITS_MASK;

    if ((usart_reg(s, USART_CTRL) & USART_CTRL_SYNC) && !s->master) {
        qemu_log_mask(LOG_UNIMP, "efr32-usart%u: synchronous secondary mode\n",
                      s->index);
    }
    if (usart_reg(s, USART_CTRL) & USART_CTRL_AUTOCS) {
        qemu_log_mask(LOG_UNIMP, "efr32-usart%u: AUTOCS\n", s->index);
    }
    if (data_bits > USART_FRAME_DATABITS_EIGHT) {
        qemu_log_mask(LOG_UNIMP, "efr32-usart%u: %u-bit frames\n", s->index,
                      data_bits + USART_FRAME_DATABITS_BIAS);
    }
    if (usart_reg(s, USART_IRCTRL) || usart_reg(s, USART_I2SCTRL) ||
        usart_reg(s, USART_TIMECMP0) || usart_reg(s, USART_TIMECMP1) ||
        usart_reg(s, USART_TIMECMP2)) {
        qemu_log_mask(LOG_UNIMP, "efr32-usart%u: IrDA, I2S or USART timer\n",
                      s->index);
    }
}

static void usart_tx_start(Efr32UsartState *s)
{
    int64_t frame_ns;

    if (s->tx_shifting || !s->tx_enabled || fifo8_is_empty(&s->tx_fifo)) {
        return;
    }
    frame_ns = usart_frame_ns(s);
    if (frame_ns <= 0) {
        /* No PCLK: the shifter does not run. */
        return;
    }
    s->tx_shift = fifo8_pop(&s->tx_fifo);
    s->tx_shifting = true;
    timer_mod(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + frame_ns);
}

static void usart_receive_frame(Efr32UsartState *s, uint8_t value);

/* The stop bit of the frame being transmitted has left the pin. */
static void usart_tx_frame_end(void *opaque)
{
    Efr32UsartState *s = opaque;
    uint8_t value = s->tx_shift;

    s->tx_shifting = false;
    trace_efr32bg22_usart_tx(s->index, value);
    if (usart_reg(s, USART_CTRL) & USART_CTRL_LOOPBK) {
        usart_receive_frame(s, value);
    } else if (usart_reg(s, USART_CTRL) & USART_CTRL_SYNC) {
        bool lsb_first = !(usart_reg(s, USART_CTRL) & USART_CTRL_MSBF);
        uint8_t wire = lsb_first ? revbit8(value) : value;
        uint8_t back = s->master ? ssi_transfer(s->ssi, wire) : 0;

        back = lsb_first ? revbit8(back) : back;
        trace_efr32_usart_sync(s->index, value, back);
        usart_receive_frame(s, back);
    } else {
        qemu_chr_fe_write_all(&s->chr, &value, 1);
    }
    usart_tx_start(s);
    if (usart_tx_idle(s)) {
        s->tx_complete = true;
        s->if_latched |= USART_IF_TXC | USART_IF_TXIDLE;
    }
    usart_update_irq(s);
}

static void usart_tx_push(Efr32UsartState *s, uint8_t value)
{
    if (fifo8_is_full(&s->tx_fifo)) {
        s->if_latched |= USART_IF_TXOF;
        return;
    }
    fifo8_push(&s->tx_fifo, value);
    s->tx_complete = false;
    usart_tx_start(s);
}

/* A complete frame reached the receiver's buffer input. */
static void usart_receive_frame(Efr32UsartState *s, uint8_t value)
{
    if (!s->rx_enabled) {
        return;
    }
    if (fifo8_is_full(&s->rx_fifo)) {
        s->if_latched |= USART_IF_RXOF;
        trace_efr32_usart_rx_overflow(s->index, value);
        return;
    }
    trace_efr32_usart_rx(s->index, value);
    fifo8_push(&s->rx_fifo, value);
}

static void usart_rx_frame_end(void *opaque)
{
    Efr32UsartState *s = opaque;

    s->rx_shifting = false;
    usart_receive_frame(s, s->rx_shift);
    usart_update_irq(s);
    qemu_chr_fe_accept_input(&s->chr);
}

/* One frame at a time is on the wire, and only to an enabled receiver. */
static int usart_can_receive(void *opaque)
{
    Efr32UsartState *s = opaque;

    /* In synchronous mode the receiver takes what the SPI bus returns. */
    return s->rx_enabled && !s->rx_shifting && usart_frame_ns(s) > 0 &&
           !(usart_reg(s, USART_CTRL) & USART_CTRL_SYNC);
}

static void usart_receive(void *opaque, const uint8_t *buf, int size)
{
    Efr32UsartState *s = opaque;

    if (size < 1 || !usart_can_receive(s)) {
        return;
    }
    s->rx_shift = buf[0];
    s->rx_shifting = true;
    timer_mod(s->rx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + usart_frame_ns(s));
}

static uint8_t usart_rx_pop(Efr32UsartState *s, bool peek)
{
    uint8_t value;

    if (fifo8_is_empty(&s->rx_fifo)) {
        s->if_latched |= USART_IF_RXUF;
        return 0;
    }
    if (peek) {
        return fifo8_peek(&s->rx_fifo);
    }
    value = fifo8_pop(&s->rx_fifo);
    return value;
}

static void usart_command(Efr32UsartState *s, uint32_t command)
{
    bool was_receiving = s->rx_enabled;

    if (command & USART_CMD_CLEARRX) {
        fifo8_reset(&s->rx_fifo);
    }
    if (command & USART_CMD_CLEARTX) {
        fifo8_reset(&s->tx_fifo);
    }
    if (command & USART_CMD_RXDIS) {
        s->rx_enabled = false;
    } else if (command & USART_CMD_RXEN) {
        s->rx_enabled = true;
    }
    if (command & USART_CMD_TXDIS) {
        s->tx_enabled = false;
    } else if (command & USART_CMD_TXEN) {
        s->tx_enabled = true;
    }
    if (command & USART_CMD_MASTERDIS) {
        s->master = false;
    } else if (command & USART_CMD_MASTEREN) {
        s->master = true;
    }
    if (command & (USART_CMD_RXEN | USART_CMD_TXEN | USART_CMD_MASTEREN)) {
        usart_check_unimplemented(s);
        usart_update_line_params(s);
    }
    usart_tx_start(s);
    if (s->rx_enabled && !was_receiving) {
        qemu_chr_fe_accept_input(&s->chr);
    }
}

static void usart_access(Efr32UsartState *s, hwaddr offset, unsigned size,
                         bool is_write)
{
    if (s->bus_model) {
        icount_bus_access(s->bus_model, current_cpu, s->base + offset, size,
                          is_write);
    }
}

static uint64_t usart_read(void *opaque, hwaddr addr, unsigned size)
{
    Efr32UsartState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)USART_ALIAS_MASK;
    uint32_t value;

    usart_access(s, addr, size, false);
    switch (offset) {
    case USART_STATUS:
        value = usart_status(s);
        break;
    case USART_IF:
        value = usart_if(s);
        break;
    case USART_RXDATA:
    case USART_RXDATAX:
        value = usart_rx_pop(s, false) & USART_RXDATAX_RXDATA_MASK;
        break;
    case USART_RXDATAXP:
        value = usart_rx_pop(s, true) & USART_RXDATAX_RXDATA_MASK;
        break;
    case USART_RXDOUBLE:
        value = usart_rx_pop(s, false);
        value |= (uint32_t)usart_rx_pop(s, false) << USART_DOUBLE_SHIFT;
        break;
    case USART_RXDOUBLEX:
        value = usart_rx_pop(s, false);
        value |= (uint32_t)usart_rx_pop(s, false) << USART_DOUBLEX_SHIFT;
        break;
    case USART_RXDOUBLEXP:
        /* Only one frame can be peeked without consuming the first. */
        value = usart_rx_pop(s, true);
        break;
    case USART_CMD:
    case USART_TXDATAX:
    case USART_TXDATA:
    case USART_TXDOUBLEX:
    case USART_TXDOUBLE:
        value = 0;
        break;
    default:
        if (offset >= USART_REGISTER_END) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "efr32-usart%u: read of reserved offset 0x%03"
                          HWADDR_PRIx "\n", s->index, offset);
            return 0;
        }
        value = usart_reg(s, offset);
        break;
    }
    usart_update_irq(s);
    return value;
}

static uint32_t usart_apply_alias(unsigned alias, uint32_t old, uint32_t value)
{
    switch (alias) {
    case USART_ALIAS_SET:
        return old | value;
    case USART_ALIAS_CLR:
        return old & ~value;
    case USART_ALIAS_TGL:
        return old ^ value;
    default:
        return value;
    }
}

static void usart_write(void *opaque, hwaddr addr, uint64_t data,
                        unsigned size)
{
    Efr32UsartState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)USART_ALIAS_MASK;
    unsigned alias = (addr & USART_ALIAS_MASK) >> USART_ALIAS_SHIFT;
    uint32_t value = data;

    usart_access(s, addr, size, true);
    switch (offset) {
    case USART_IPVERSION:
    case USART_STATUS:
    case USART_RXDATAX:
    case USART_RXDATA:
    case USART_RXDOUBLEX:
    case USART_RXDOUBLE:
    case USART_RXDATAXP:
    case USART_RXDOUBLEXP:
        break;
    case USART_CMD:
        usart_command(s, value);
        break;
    case USART_TXDATA:
    case USART_TXDATAX:
        usart_tx_push(s, value & USART_TXDATAX_TXDATAX_MASK);
        break;
    case USART_TXDOUBLE:
        usart_tx_push(s, value);
        usart_tx_push(s, value >> USART_DOUBLE_SHIFT);
        break;
    case USART_TXDOUBLEX:
        usart_tx_push(s, value);
        usart_tx_push(s, value >> USART_DOUBLEX_SHIFT);
        break;
    case USART_IF:
        /* Level flags follow their buffers whatever is written. */
        s->if_latched = usart_apply_alias(alias, s->if_latched, value) &
                        USART_IF_MASK & ~USART_IF_LEVEL;
        break;
    case USART_EN:
        s->regs[offset / 4] = usart_apply_alias(alias, usart_reg(s, offset),
                                                value) & USART_EN_EN;
        if (!(usart_reg(s, USART_EN) & USART_EN_EN)) {
            s->rx_enabled = false;
            s->tx_enabled = false;
        }
        break;
    case USART_CLKDIV:
    case USART_FRAME:
        s->regs[offset / 4] = usart_apply_alias(alias, usart_reg(s, offset),
                                                value);
        usart_update_line_params(s);
        break;
    default:
        if (offset >= USART_REGISTER_END) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "efr32-usart%u: write of reserved offset 0x%03"
                          HWADDR_PRIx "\n", s->index, offset);
            return;
        }
        s->regs[offset / 4] = usart_apply_alias(alias, usart_reg(s, offset),
                                                value);
        break;
    }
    usart_update_irq(s);
}

static const MemoryRegionOps usart_ops = {
    .read = usart_read,
    .write = usart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

static void usart_reset_hold(Object *obj, ResetType type)
{
    Efr32UsartState *s = EFR32_USART(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[USART_FRAME / 4] = USART_FRAME_RESETVALUE;
    s->if_latched = 0;
    s->rx_enabled = false;
    s->tx_enabled = false;
    s->master = false;
    s->tx_complete = false;
    s->tx_shifting = false;
    s->rx_shifting = false;
    fifo8_reset(&s->rx_fifo);
    fifo8_reset(&s->tx_fifo);
    timer_del(s->tx_timer);
    timer_del(s->rx_timer);
}

static void usart_realize(DeviceState *dev, Error **errp)
{
    Efr32UsartState *s = EFR32_USART(dev);
    g_autofree char *bus = g_strdup_printf("usart%u.ssi", s->index);

    /* "-device <spi-device>,bus=usart1.ssi" attaches to USART1's SPI bus. */
    s->ssi = ssi_create_bus(dev, bus);
    qemu_chr_fe_set_handlers(&s->chr, usart_can_receive, usart_receive,
                             NULL, NULL, s, NULL, true);
}

static void usart_init(Object *obj)
{
    Efr32UsartState *s = EFR32_USART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &usart_ops, s, TYPE_EFR32_USART,
                          EFR32_USART_BLOCK_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->rx_irq);
    sysbus_init_irq(sbd, &s->tx_irq);
    s->pclk = qdev_init_clock_in(DEVICE(obj), "pclk", NULL, NULL, 0);
    fifo8_create(&s->rx_fifo, USART_BUFFER_FRAMES);
    fifo8_create(&s->tx_fifo, USART_BUFFER_FRAMES);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, usart_tx_frame_end, s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, usart_rx_frame_end, s);
}

static void usart_finalize(Object *obj)
{
    Efr32UsartState *s = EFR32_USART(obj);

    timer_free(s->tx_timer);
    timer_free(s->rx_timer);
    fifo8_destroy(&s->rx_fifo);
    fifo8_destroy(&s->tx_fifo);
}

static Property usart_properties[] = {
    DEFINE_PROP_CHR("chardev", Efr32UsartState, chr),
    DEFINE_PROP_UINT32("index", Efr32UsartState, index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void usart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = usart_realize;
    rc->phases.hold = usart_reset_hold;
    device_class_set_props(dc, usart_properties);
}

static const TypeInfo usart_info = {
    .name = TYPE_EFR32_USART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32UsartState),
    .instance_init = usart_init,
    .instance_finalize = usart_finalize,
    .class_init = usart_class_init,
};

static void usart_register_types(void)
{
    type_register_static(&usart_info);
}
type_init(usart_register_types)

/*
 * The host device given to this USART: the chardev named "<soc>.<name>"
 * ("-chardev vhci,id=soc1.usart0" passes /dev/vhci through to soc1's
 * USART0), else the -serial at @serial_index.
 */
static Chardev *usart_host_device(Object *soc, const Efr32UsartConfig *config)
{
    const char *soc_name = object_get_canonical_path_component(soc);
    g_autofree char *id = g_strdup_printf("%s.%s", soc_name, config->name);
    Chardev *chr = qemu_chr_find(id);

    if (!chr && config->serial_index >= 0) {
        chr = serial_hd(config->serial_index);
    }
    return chr;
}

DeviceState *efr32_usart_create(Object *soc, MemoryRegion *memory,
                                DeviceState *nvic, Clock *pclk,
                                IcountBusModel *bus_model,
                                const Efr32UsartConfig *config)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_USART);
    Efr32UsartState *s = EFR32_USART(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    object_property_add_child(soc, config->name, OBJECT(dev));
    qdev_prop_set_uint32(dev, "index", config->index);
    qdev_prop_set_chr(dev, "chardev", usart_host_device(soc, config));
    qdev_connect_clock_in(dev, "pclk", pclk);
    s->bus_model = bus_model;
    s->base = config->base;
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion_overlap(memory, config->base,
                                        sysbus_mmio_get_region(sbd, 0), 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-usart-ns",
                             &s->mmio, 0, EFR32_USART_BLOCK_SIZE);
    memory_region_add_subregion_overlap(memory,
                                        config->base + config->ns_offset,
                                        &s->mmio_ns, 1);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(nvic, config->rx_irq));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(nvic, config->tx_irq));
    return dev;
}
