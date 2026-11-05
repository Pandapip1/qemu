/*
 * EFR32xG22 EUART0, enhanced UART (RM rev. 1.2 chapter 22), at 0x4A030000
 * (non-secure 0x5A030000), IRQs 62 (RX) and 63 (TX), bus clock
 * CMU_CLKEN0.EUART0 (bit 24), kernel clock CMU_EUART0CLKCTRL (EM01GRPACLK
 * or EM23GRPACLK).  Register layout and field encodings: the public
 * efr32bg22_eusart.h; behaviour: RM SS22.3.
 *
 * The line side is a QEMU character device, as for the USART
 * (efr32-usart.c): each frame occupies the wire for its real duration at
 * br = f_EUART / (oversample x (1 + CLKDIV / 256)) (RM Figure 22.3,
 * oversample 16/8/6/4, or 1 with OVS = DISABLE for LF operation), with one
 * start bit, 7/8/9 data bits (FRAMECFG.DATABITS 1/2/3, efr32bg22_eusart.h;
 * the RM's 4-bit table is the generic IP's), the parity bit and 0.5/1/1.5/2
 * stop bits.  A chardev "<soc>.euart0" claims the port.
 *
 * Modelled:
 *  - 4-entry TX and RX FIFOs plus shift registers (SS22.3.7, 22.3.11):
 *    STATUS.TXFL when the TX FIFO has room for at least CFG1.TXFIW + 1
 *    frames, RXFL when the RX FIFO holds at least RXFIW + 1, RXFULL,
 *    TXFCNT, TXIDLE, RXIDLE, TXC; TXOF (the write is dropped), RXOF (the
 *    frame is dropped), RXUF; RXDATAP peeks;
 *  - TXDATA's per-frame controls TXDISAT, RXENAT, UBRXAT, TXTRIAT and
 *    TXBREAK (a break condition on the chardev for a backend that has one);
 *    a break from the host side arrives as a frame of 0 with FERR
 *    (SS22.3.15: a break is a framing error), RXDATA.FERR set;
 *  - RXBLOCK (CMD.RXBLOCKEN/DIS), the programmable start frame (STARTF,
 *    CFG1.SFUBRX unblocking) and signal frame (SIGF), multi-processor
 *    mode (CFG0.MPM: a frame with the 9th bit equal to MPAB is an address,
 *    MPAF, loaded even while blocked), local loopback (CFG0.LOOPBK),
 *    TIMINGCFG.TXDELAY (0-3 bit periods before the first frame after idle);
 *  - IF is an INTFLAG register (only SET/CLR act); TXFL, RXFL and RXFULL
 *    set again while their condition holds (SS22.3.11);
 *  - register types (SS22.4): CONFIG registers (CFG0, CFG1, FRAMECFG,
 *    IRHFCFG, IRLFCFG, TIMINGCFG, STARTFRAMECFG, SIGFRAMECFG) fault when
 *    written with EN set (RM SS4.2.4.4.2); LFSYNC registers (CLKDIV,
 *    TRIGCTRL, CMD) take effect 3 kernel clock cycles after the write with
 *    SYNCBUSY set and fault on a second write while pending ([I] 3: the
 *    RM's 3-4; land at once with EN clear, as Silicon Labs' reset sequence
 *    writes CLKDIV twice in a row and must not fault);
 *  - PRS: consumer TRIGGER (TRIGCTRL.RXTEN/TXTEN enable the receiver or
 *    transmitter on a rising edge); producers TXC and RXFL (the status
 *    levels);
 *  - DMA requests: the TXFL and RXFL levels ("dma-req" 0 and 1);
 *  - CMU_CLKEN0.EUART0 clear: accesses fault ([I] RM SS4.2.1.2's rule).
 * [I] The RX/TX interrupt split follows the flag names, as for the USART.
 * [U] Not modelled: parity errors (the chardev carries bytes, not bits),
 * IrDA, auto-baud, collision detection, CTS/RTS flow control (the chardev
 * paces itself), the PRS RX input, the PRS producers TX/RTS/IRDATX, EM2
 * wake-up, DBGHALT.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "hw/arm/efr32-eusart.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32EusartState, EFR32_EUSART)

enum {
    EU_IPVERSION = 0x000,
    EU_EN = 0x004,
    EU_CFG0 = 0x008,
    EU_CFG1 = 0x00c,
    EU_FRAMECFG = 0x010,
    EU_IRHFCFG = 0x014,
    EU_IRLFCFG = 0x018,
    EU_TIMINGCFG = 0x01c,
    EU_STARTFRAMECFG = 0x020,
    EU_SIGFRAMECFG = 0x024,
    EU_CLKDIV = 0x028,
    EU_TRIGCTRL = 0x02c,
    EU_CMD = 0x030,
    EU_RXDATA = 0x034,
    EU_RXDATAP = 0x038,
    EU_TXDATA = 0x03c,
    EU_STATUS = 0x040,
    EU_IF = 0x044,
    EU_IEN = 0x048,
    EU_SYNCBUSY = 0x04c,
    EU_REGISTER_END = 0x050,
};

#define EU_CFG0_MASK 0xc1d264feu
#define EU_CFG0_LOOPBK (1u << 1)
#define EU_CFG0_MPM (1u << 3)
#define EU_CFG0_MPAB (1u << 4)
#define EU_CFG0_OVS_SHIFT 5
#define EU_CFG0_OVS_MASK 0x7u
#define EU_CFG0_AUTOTRI (1u << 17)
#define EU_CFG1_MASK 0x00db8e0fu
#define EU_CFG1_SFUBRX (1u << 11)
#define EU_CFG1_TXFIW_SHIFT 16
#define EU_CFG1_RXFIW_SHIFT 19
#define EU_CFG1_FIW_MASK 0x3u
#define EU_FRAMECFG_MASK 0x00003303u
#define EU_FRAMECFG_RESET 0x00001002u
#define EU_FRAMECFG_DATABITS_MASK 0x3u
#define EU_FRAMECFG_DATABITS_BIAS 6u
#define EU_FRAMECFG_PARITY_SHIFT 8
#define EU_FRAMECFG_PARITY_MASK 0x3u
#define EU_FRAMECFG_STOPBITS_SHIFT 12
#define EU_FRAMECFG_STOPBITS_MASK 0x3u
#define EU_TIMINGCFG_TXDELAY_MASK 0x3u
#define EU_CLKDIV_MASK 0x007ffff8u
#define EU_CLKDIV_UNITY 256u
#define EU_TRIGCTRL_MASK 0x3u
#define EU_TRIGCTRL_RXTEN (1u << 0)
#define EU_TRIGCTRL_TXTEN (1u << 1)
#define EU_CMD_RXEN (1u << 0)
#define EU_CMD_RXDIS (1u << 1)
#define EU_CMD_TXEN (1u << 2)
#define EU_CMD_TXDIS (1u << 3)
#define EU_CMD_RXBLOCKEN (1u << 4)
#define EU_CMD_RXBLOCKDIS (1u << 5)
#define EU_CMD_TXTRIEN (1u << 6)
#define EU_CMD_TXTRIDIS (1u << 7)
#define EU_CMD_CLEARTX (1u << 8)
#define EU_CMD_MASK 0x1ffu
#define EU_RXDATA_PERR (1u << 9)
#define EU_RXDATA_FERR (1u << 10)
#define EU_FRAME_DATA_MASK 0x1ffu
#define EU_TXDATA_UBRXAT (1u << 9)
#define EU_TXDATA_TXTRIAT (1u << 10)
#define EU_TXDATA_TXBREAK (1u << 11)
#define EU_TXDATA_TXDISAT (1u << 12)
#define EU_TXDATA_RXENAT (1u << 13)
#define EU_TXDATA_MASK 0x3fffu
#define EU_STATUS_RXENS (1u << 0)
#define EU_STATUS_TXENS (1u << 1)
#define EU_STATUS_RXBLOCK (1u << 3)
#define EU_STATUS_TXTRI (1u << 4)
#define EU_STATUS_TXC (1u << 5)
#define EU_STATUS_TXFL (1u << 6)
#define EU_STATUS_RXFL (1u << 7)
#define EU_STATUS_RXFULL (1u << 8)
#define EU_STATUS_RXIDLE (1u << 12)
#define EU_STATUS_TXIDLE (1u << 13)
#define EU_STATUS_TXFCNT_SHIFT 16
#define EU_STATUS_CLEARTXBUSY (1u << 19)
#define EU_IF_TXC (1u << 0)
#define EU_IF_TXFL (1u << 1)
#define EU_IF_RXFL (1u << 2)
#define EU_IF_RXFULL (1u << 3)
#define EU_IF_RXOF (1u << 4)
#define EU_IF_RXUF (1u << 5)
#define EU_IF_TXOF (1u << 6)
#define EU_IF_PERR (1u << 8)
#define EU_IF_FERR (1u << 9)
#define EU_IF_MPAF (1u << 10)
#define EU_IF_CCF (1u << 12)
#define EU_IF_TXIDLE (1u << 13)
#define EU_IF_STARTF (1u << 18)
#define EU_IF_SIGF (1u << 19)
#define EU_IF_AUTOBAUDDONE (1u << 24)
#define EU_IF_MASK 0x010c377fu
#define EU_IF_RX_LINE (EU_IF_RXFL | EU_IF_RXFULL | EU_IF_RXOF | EU_IF_RXUF | \
                       EU_IF_PERR | EU_IF_FERR | EU_IF_MPAF | EU_IF_STARTF |  \
                       EU_IF_SIGF | EU_IF_AUTOBAUDDONE)
#define EU_IF_TX_LINE (EU_IF_MASK & ~EU_IF_RX_LINE)
/* SYNCBUSY: DIV 0, RXTEN 1, TXTEN 2, then CMD bits 0-7 at 3-10. */
#define EU_SYNCBUSY_DIV (1u << 0)
#define EU_SYNCBUSY_RXTEN (1u << 1)
#define EU_SYNCBUSY_TXTEN (1u << 2)
#define EU_SYNCBUSY_CMD_SHIFT 3
#define EU_SYNCBUSY_CMD_BITS 0xffu
#define EU_FIFO_DEPTH 4
#define EU_SYNC_CYCLES 3
#define EU_CLKEN0_EUART0 (1u << 24)

enum { EU_PEND_CLKDIV, EU_PEND_TRIGCTRL, EU_PEND_CMD, EU_PEND_COUNT };

typedef struct EuFifo {
    uint32_t entry[EU_FIFO_DEPTH];
    unsigned count;
} EuFifo;

static unsigned eu_fifo_num_used(EuFifo *f)
{
    return f->count;
}

static bool eu_fifo_is_empty(EuFifo *f)
{
    return !f->count;
}

static bool eu_fifo_is_full(EuFifo *f)
{
    return f->count == EU_FIFO_DEPTH;
}

static void eu_fifo_reset(EuFifo *f)
{
    f->count = 0;
}

static void eu_fifo_push(EuFifo *f, uint32_t v)
{
    f->entry[f->count++] = v;
}

static uint32_t eu_fifo_peek(EuFifo *f)
{
    return f->entry[0];
}

static uint32_t eu_fifo_pop(EuFifo *f)
{
    uint32_t v = f->entry[0];

    memmove(f->entry, f->entry + 1, --f->count * sizeof(f->entry[0]));
    return v;
}

struct Efr32EusartState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    qemu_irq rx_irq;
    qemu_irq tx_irq;
    qemu_irq prs_txc;
    qemu_irq prs_rxfl;
    qemu_irq dma_req[2];
    CharBackend chr;
    const Efr32SocLink *link;
    hwaddr base;

    uint32_t en, cfg0, cfg1, framecfg, irhfcfg, irlfcfg, timingcfg;
    uint32_t startframe, sigframe, clkdiv, trigctrl, if_, ien;
    bool rx_enabled, tx_enabled, rx_block, tx_tri, tx_complete;
    /* TX FIFO entries: frame | TXDATA control bits. */
    EuFifo tx_fifo;
    /* RX FIFO entries: frame | RXDATA error bits. */
    EuFifo rx_fifo;
    bool tx_shifting;
    uint32_t tx_shift;
    bool tx_was_idle;
    QEMUTimer *tx_timer;
    bool rx_shifting;
    uint32_t rx_shift;
    QEMUTimer *rx_timer;
    struct {
        bool valid;
        uint32_t value;
        int64_t at_ns;
    } pend[EU_PEND_COUNT];
    QEMUTimer *sync_timer;
    bool prs_trigger;
    bool sent[4];
};

static uint64_t eu_clk_hz(Efr32EusartState *s)
{
    return efr32_link_tree_hz(s->link, EFR32_TREE_EUSART0CLK);
}

static unsigned eu_oversample(Efr32EusartState *s)
{
    static const unsigned ovs[] = {16, 8, 6, 4, 1, 1, 1, 1};

    return ovs[(s->cfg0 >> EU_CFG0_OVS_SHIFT) & EU_CFG0_OVS_MASK];
}

static unsigned eu_data_bits(Efr32EusartState *s)
{
    unsigned field = s->framecfg & EU_FRAMECFG_DATABITS_MASK;

    return (field ? field : 2) + EU_FRAMECFG_DATABITS_BIAS;
}

/* @half_bits bit periods / 2 at the programmed rate, in ns (-1: no clock). */
static int64_t eu_half_bits_ns(Efr32EusartState *s, uint64_t half_bits)
{
    uint64_t hz = eu_clk_hz(s);

    if (!hz) {
        return -1;
    }
    /* 10^9 / 512 is exact; muldiv64's divisor is 32 bits wide. */
    return muldiv64(half_bits * eu_oversample(s) *
                        (EU_CLKDIV_UNITY + (s->clkdiv & EU_CLKDIV_MASK)),
                    NANOSECONDS_PER_SECOND / (2 * EU_CLKDIV_UNITY), hz);
}

static int64_t eu_frame_ns(Efr32EusartState *s)
{
    unsigned parity = ((s->framecfg >> EU_FRAMECFG_PARITY_SHIFT) &
                       EU_FRAMECFG_PARITY_MASK) >= 2;
    unsigned stop_half = ((s->framecfg >> EU_FRAMECFG_STOPBITS_SHIFT) &
                          EU_FRAMECFG_STOPBITS_MASK) + 1;

    return eu_half_bits_ns(s, 2 * (1 + eu_data_bits(s) + parity) + stop_half);
}

static unsigned eu_tx_count(Efr32EusartState *s)
{
    return eu_fifo_num_used(&s->tx_fifo);
}

static bool eu_txfl(Efr32EusartState *s)
{
    unsigned fiw = (s->cfg1 >> EU_CFG1_TXFIW_SHIFT) & EU_CFG1_FIW_MASK;

    return EU_FIFO_DEPTH - eu_tx_count(s) >= fiw + 1;
}

static bool eu_rxfl(Efr32EusartState *s)
{
    unsigned fiw = (s->cfg1 >> EU_CFG1_RXFIW_SHIFT) & EU_CFG1_FIW_MASK;

    return eu_fifo_num_used(&s->rx_fifo) >= fiw + 1;
}

static bool eu_tx_idle(Efr32EusartState *s)
{
    return !s->tx_shifting && eu_fifo_is_empty(&s->tx_fifo);
}

static uint32_t eu_status(Efr32EusartState *s)
{
    return (s->rx_enabled ? EU_STATUS_RXENS : 0) |
           (s->tx_enabled ? EU_STATUS_TXENS : 0) |
           (s->rx_block ? EU_STATUS_RXBLOCK : 0) |
           (s->tx_tri && !(s->cfg0 & EU_CFG0_AUTOTRI) ? EU_STATUS_TXTRI : 0) |
           (s->tx_complete ? EU_STATUS_TXC : 0) |
           (eu_txfl(s) ? EU_STATUS_TXFL : 0) |
           (eu_rxfl(s) ? EU_STATUS_RXFL : 0) |
           (eu_fifo_is_full(&s->rx_fifo) ? EU_STATUS_RXFULL : 0) |
           (!s->rx_shifting ? EU_STATUS_RXIDLE : 0) |
           (eu_tx_idle(s) ? EU_STATUS_TXIDLE : 0) |
           (eu_tx_count(s) << EU_STATUS_TXFCNT_SHIFT);
}

/* The level flags set again while their condition holds (SS22.3.11). */
static void eu_level_flags(Efr32EusartState *s)
{
    uint32_t status = eu_status(s);

    s->if_ |= (status & EU_STATUS_TXFL ? EU_IF_TXFL : 0) |
              (status & EU_STATUS_RXFL ? EU_IF_RXFL : 0) |
              (status & EU_STATUS_RXFULL ? EU_IF_RXFULL : 0);
}

static void eu_set_line(Efr32EusartState *s, unsigned n, qemu_irq line,
                        bool level)
{
    if (s->sent[n] != level) {
        s->sent[n] = level;
        qemu_set_irq(line, level);
    }
}

static void eu_update(Efr32EusartState *s)
{
    uint32_t pending;

    eu_level_flags(s);
    pending = s->if_ & s->ien;
    qemu_set_irq(s->rx_irq, !!(pending & EU_IF_RX_LINE));
    qemu_set_irq(s->tx_irq, !!(pending & EU_IF_TX_LINE));
    eu_set_line(s, 0, s->prs_txc, s->tx_complete);
    eu_set_line(s, 1, s->prs_rxfl, eu_rxfl(s));
    eu_set_line(s, 2, s->dma_req[0], s->en && eu_txfl(s));
    eu_set_line(s, 3, s->dma_req[1], eu_rxfl(s));
}

static void eu_update_line_params(Efr32EusartState *s)
{
    static const char parity_names[] = {'N', 'N', 'E', 'O'};
    uint64_t hz = eu_clk_hz(s);
    QEMUSerialSetParams params;

    if (!hz) {
        return;
    }
    params.speed = muldiv64(EU_CLKDIV_UNITY, hz,
                            (uint64_t)eu_oversample(s) *
                                (EU_CLKDIV_UNITY +
                                 (s->clkdiv & EU_CLKDIV_MASK)));
    params.parity = parity_names[(s->framecfg >> EU_FRAMECFG_PARITY_SHIFT) &
                                 EU_FRAMECFG_PARITY_MASK];
    params.data_bits = MIN(eu_data_bits(s), 8);
    params.stop_bits = ((s->framecfg >> EU_FRAMECFG_STOPBITS_SHIFT) &
                        EU_FRAMECFG_STOPBITS_MASK) > 1 ? 2 : 1;
    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &params);
}

static void eu_receive_frame(Efr32EusartState *s, uint32_t frame,
                             uint32_t errors);

static void eu_tx_start(Efr32EusartState *s)
{
    int64_t frame_ns, delay_ns = 0;

    if (s->tx_shifting || !s->tx_enabled || eu_fifo_is_empty(&s->tx_fifo)) {
        return;
    }
    frame_ns = eu_frame_ns(s);
    if (frame_ns <= 0) {
        return;
    }
    if (s->tx_was_idle) {
        /* TIMINGCFG.TXDELAY bit periods before the first frame (SS22.3.9). */
        delay_ns = eu_half_bits_ns(s, 2 * (s->timingcfg &
                                           EU_TIMINGCFG_TXDELAY_MASK));
    }
    s->tx_was_idle = false;
    s->tx_shift = eu_fifo_pop(&s->tx_fifo);
    s->tx_shifting = true;
    s->tx_complete = false;
    timer_mod(s->tx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns + frame_ns);
}

static void eu_tx_frame_end(void *opaque)
{
    Efr32EusartState *s = opaque;
    uint32_t entry = s->tx_shift;
    uint8_t byte = entry & 0xff;

    s->tx_shifting = false;
    trace_efr32_eusart_tx(byte);
    if (entry & EU_TXDATA_TXBREAK) {
        int on = 1;

        qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK, &on);
        on = 0;
        qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK, &on);
    }
    if (s->cfg0 & EU_CFG0_LOOPBK) {
        eu_receive_frame(s, entry & EU_FRAME_DATA_MASK,
                         entry & EU_TXDATA_TXBREAK ? EU_RXDATA_FERR : 0);
    } else if (!(entry & EU_TXDATA_TXBREAK) && !s->tx_tri) {
        qemu_chr_fe_write_all(&s->chr, &byte, 1);
    }
    if (entry & EU_TXDATA_TXDISAT) {
        s->tx_enabled = false;
    }
    if (entry & EU_TXDATA_RXENAT) {
        s->rx_enabled = true;
        qemu_chr_fe_accept_input(&s->chr);
    }
    if (entry & EU_TXDATA_UBRXAT) {
        s->rx_block = false;
    }
    if (entry & EU_TXDATA_TXTRIAT) {
        s->tx_tri = true;
    }
    eu_tx_start(s);
    if (eu_tx_idle(s)) {
        s->tx_complete = true;
        s->tx_was_idle = true;
        s->if_ |= EU_IF_TXC | EU_IF_TXIDLE;
    }
    eu_update(s);
}

/* A complete frame reached the receiver (RM SS22.3.10-22.3.23). */
static void eu_receive_frame(Efr32EusartState *s, uint32_t frame,
                             uint32_t errors)
{
    bool address = false;

    if (!s->rx_enabled) {
        return;
    }
    if (errors & EU_RXDATA_FERR) {
        s->if_ |= EU_IF_FERR;
    }
    if (frame == (s->sigframe & EU_FRAME_DATA_MASK) && s->sigframe) {
        s->if_ |= EU_IF_SIGF;
    }
    if (frame == (s->startframe & EU_FRAME_DATA_MASK) && s->startframe) {
        s->if_ |= EU_IF_STARTF;
        if (s->cfg1 & EU_CFG1_SFUBRX) {
            s->rx_block = false;
        }
    }
    if ((s->cfg0 & EU_CFG0_MPM) && eu_data_bits(s) == 9 &&
        !!(frame & 0x100) == !!(s->cfg0 & EU_CFG0_MPAB)) {
        address = true;
        s->if_ |= EU_IF_MPAF;
    }
    if (s->rx_block && !address) {
        return;
    }
    if (eu_fifo_is_full(&s->rx_fifo)) {
        s->if_ |= EU_IF_RXOF;
        return;
    }
    trace_efr32_eusart_rx(frame);
    eu_fifo_push(&s->rx_fifo, frame | errors);
}

static void eu_rx_frame_end(void *opaque)
{
    Efr32EusartState *s = opaque;

    s->rx_shifting = false;
    eu_receive_frame(s, s->rx_shift & EU_FRAME_DATA_MASK,
                     s->rx_shift & EU_RXDATA_FERR);
    eu_update(s);
    qemu_chr_fe_accept_input(&s->chr);
}

static int eu_can_receive(void *opaque)
{
    Efr32EusartState *s = opaque;

    return s->en && s->rx_enabled && !s->rx_shifting && eu_frame_ns(s) > 0 &&
           !(s->cfg0 & EU_CFG0_LOOPBK);
}

static void eu_start_rx(Efr32EusartState *s, uint32_t shift)
{
    s->rx_shift = shift;
    s->rx_shifting = true;
    timer_mod(s->rx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + eu_frame_ns(s));
}

static void eu_receive(void *opaque, const uint8_t *buf, int size)
{
    Efr32EusartState *s = opaque;

    if (size >= 1 && eu_can_receive(s)) {
        eu_start_rx(s, buf[0]);
    }
}

static void eu_event(void *opaque, QEMUChrEvent event)
{
    Efr32EusartState *s = opaque;

    /* A break is a frame of 0 with a framing error (SS22.3.15). */
    if (event == CHR_EVENT_BREAK && eu_can_receive(s)) {
        eu_start_rx(s, EU_RXDATA_FERR);
    }
}

static void eu_command(Efr32EusartState *s, uint32_t cmd)
{
    bool was_receiving = s->rx_enabled;

    if (cmd & EU_CMD_CLEARTX) {
        eu_fifo_reset(&s->tx_fifo);
    }
    if (cmd & EU_CMD_RXDIS) {
        s->rx_enabled = false;
        if (s->rx_shifting) {
            /* "Any frame currently being received ... is discarded." */
            timer_del(s->rx_timer);
            s->rx_shifting = false;
        }
    } else if (cmd & EU_CMD_RXEN) {
        s->rx_enabled = true;
    }
    if (cmd & EU_CMD_TXDIS) {
        s->tx_enabled = false;
    } else if (cmd & EU_CMD_TXEN) {
        s->tx_enabled = true;
    }
    if (cmd & EU_CMD_RXBLOCKDIS) {
        s->rx_block = false;
    } else if (cmd & EU_CMD_RXBLOCKEN) {
        s->rx_block = true;
    }
    if (cmd & EU_CMD_TXTRIDIS) {
        s->tx_tri = false;
    } else if (cmd & EU_CMD_TXTRIEN) {
        s->tx_tri = true;
    }
    if (cmd & (EU_CMD_RXEN | EU_CMD_TXEN)) {
        eu_update_line_params(s);
        if (eu_data_bits(s) == 9) {
            qemu_log_mask(LOG_UNIMP, "efr32-eusart: 9-bit frames reach the "
                          "chardev as their low 8 bits\n");
        }
    }
    eu_tx_start(s);
    if (s->rx_enabled && !was_receiving) {
        qemu_chr_fe_accept_input(&s->chr);
    }
}

static void eu_commit(Efr32EusartState *s, unsigned which, uint32_t value)
{
    switch (which) {
    case EU_PEND_CLKDIV:
        s->clkdiv = value & EU_CLKDIV_MASK;
        eu_update_line_params(s);
        break;
    case EU_PEND_TRIGCTRL:
        s->trigctrl = value & EU_TRIGCTRL_MASK;
        break;
    case EU_PEND_CMD:
        eu_command(s, value);
        break;
    }
}

static void eu_sync_expired(void *opaque)
{
    Efr32EusartState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t next = INT64_MAX;

    for (unsigned i = 0; i < EU_PEND_COUNT; i++) {
        if (s->pend[i].valid && s->pend[i].at_ns <= now) {
            s->pend[i].valid = false;
            eu_commit(s, i, s->pend[i].value);
        }
    }
    for (unsigned i = 0; i < EU_PEND_COUNT; i++) {
        if (s->pend[i].valid) {
            next = MIN(next, s->pend[i].at_ns);
        }
    }
    if (next != INT64_MAX) {
        timer_mod(s->sync_timer, next);
    }
    eu_update(s);
}

/* An LFSYNC write: pending EU_SYNC_CYCLES kernel clocks while enabled. */
static bool eu_pend(Efr32EusartState *s, unsigned which, uint32_t value)
{
    uint64_t hz = eu_clk_hz(s);

    if (s->pend[which].valid) {
        return false;
    }
    if (!s->en || !hz) {
        eu_commit(s, which, value);
        return true;
    }
    s->pend[which].valid = true;
    s->pend[which].value = value;
    s->pend[which].at_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                           muldiv64_round_up(EU_SYNC_CYCLES,
                                             NANOSECONDS_PER_SECOND, hz);
    if (!timer_pending(s->sync_timer) ||
        timer_expire_time_ns(s->sync_timer) > s->pend[which].at_ns) {
        timer_mod(s->sync_timer, s->pend[which].at_ns);
    }
    return true;
}

static uint32_t eu_syncbusy(Efr32EusartState *s)
{
    uint32_t busy = 0;

    if (s->pend[EU_PEND_CLKDIV].valid) {
        busy |= EU_SYNCBUSY_DIV;
    }
    if (s->pend[EU_PEND_TRIGCTRL].valid) {
        busy |= EU_SYNCBUSY_RXTEN | EU_SYNCBUSY_TXTEN;
    }
    if (s->pend[EU_PEND_CMD].valid) {
        busy |= (s->pend[EU_PEND_CMD].value & EU_SYNCBUSY_CMD_BITS)
                << EU_SYNCBUSY_CMD_SHIFT;
    }
    return busy;
}

static bool eu_gated(Efr32EusartState *s)
{
    return !efr32_link_bus_clock(s->link, EFR32_LINK_CLKEN0,
                                 EU_CLKEN0_EUART0);
}

static MemTxResult eu_read(void *opaque, hwaddr addr, uint64_t *data,
                           unsigned size, MemTxAttrs attrs)
{
    Efr32EusartState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    uint32_t v = 0;

    efr32_link_bus_access(s->link, s->base + addr, size, false);
    *data = 0;
    if (eu_gated(s)) {
        return MEMTX_ERROR;
    }
    switch (offset) {
    case EU_EN:
        v = s->en;
        break;
    case EU_CFG0:
        v = s->cfg0;
        break;
    case EU_CFG1:
        v = s->cfg1;
        break;
    case EU_FRAMECFG:
        v = s->framecfg;
        break;
    case EU_IRHFCFG:
        v = s->irhfcfg;
        break;
    case EU_IRLFCFG:
        v = s->irlfcfg;
        break;
    case EU_TIMINGCFG:
        v = s->timingcfg;
        break;
    case EU_STARTFRAMECFG:
        v = s->startframe;
        break;
    case EU_SIGFRAMECFG:
        v = s->sigframe;
        break;
    case EU_CLKDIV:
        v = s->pend[EU_PEND_CLKDIV].valid ? s->pend[EU_PEND_CLKDIV].value
                                           : s->clkdiv;
        break;
    case EU_TRIGCTRL:
        v = s->pend[EU_PEND_TRIGCTRL].valid ? s->pend[EU_PEND_TRIGCTRL].value
                                             : s->trigctrl;
        break;
    case EU_RXDATA:
        if (eu_fifo_is_empty(&s->rx_fifo)) {
            s->if_ |= EU_IF_RXUF;
        } else {
            v = eu_fifo_pop(&s->rx_fifo);
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;
    case EU_RXDATAP:
        if (!eu_fifo_is_empty(&s->rx_fifo)) {
            v = eu_fifo_peek(&s->rx_fifo);
        }
        break;
    case EU_STATUS:
        v = eu_status(s);
        break;
    case EU_IF:
        eu_level_flags(s);
        v = s->if_;
        break;
    case EU_IEN:
        v = s->ien;
        break;
    case EU_SYNCBUSY:
        v = eu_syncbusy(s);
        break;
    default:
        break;
    }
    eu_update(s);
    *data = v;
    return MEMTX_OK;
}

static bool eu_config_reg(hwaddr offset)
{
    return offset >= EU_CFG0 && offset <= EU_SIGFRAMECFG;
}

static MemTxResult eu_write(void *opaque, hwaddr addr, uint64_t data,
                            unsigned size, MemTxAttrs attrs)
{
    Efr32EusartState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    unsigned alias = (addr & EFR32_ALIAS_MASK) >> EFR32_ALIAS_SHIFT;
    uint32_t value = data;
    bool ok = true;

    efr32_link_bus_access(s->link, s->base + addr, size, true);
    if (eu_gated(s)) {
        return MEMTX_ERROR;
    }
    if (eu_config_reg(offset) && s->en) {
        /* CONFIG register written while enabled (RM SS4.2.4.4.2). */
        return MEMTX_ERROR;
    }
    switch (offset) {
    case EU_EN:
        s->en = efr32_alias_apply(addr, s->en, value) & 1;
        if (!s->en) {
            s->rx_enabled = s->tx_enabled = false;
            s->rx_block = false;
        }
        break;
    case EU_CFG0:
        s->cfg0 = efr32_alias_apply(addr, s->cfg0, value) & EU_CFG0_MASK;
        break;
    case EU_CFG1:
        s->cfg1 = efr32_alias_apply(addr, s->cfg1, value) & EU_CFG1_MASK;
        break;
    case EU_FRAMECFG:
        s->framecfg = efr32_alias_apply(addr, s->framecfg, value) &
                      EU_FRAMECFG_MASK;
        break;
    case EU_IRHFCFG:
        s->irhfcfg = efr32_alias_apply(addr, s->irhfcfg, value) & 0xfu;
        if (s->irhfcfg & 1) {
            qemu_log_mask(LOG_UNIMP, "efr32-eusart: IrDA\n");
        }
        break;
    case EU_IRLFCFG:
        s->irlfcfg = efr32_alias_apply(addr, s->irlfcfg, value) & 1u;
        break;
    case EU_TIMINGCFG:
        s->timingcfg = efr32_alias_apply(addr, s->timingcfg, value) &
                       EU_TIMINGCFG_TXDELAY_MASK;
        break;
    case EU_STARTFRAMECFG:
        s->startframe = efr32_alias_apply(addr, s->startframe, value) &
                        EU_FRAME_DATA_MASK;
        break;
    case EU_SIGFRAMECFG:
        s->sigframe = efr32_alias_apply(addr, s->sigframe, value) &
                      EU_FRAME_DATA_MASK;
        break;
    case EU_CLKDIV:
        ok = eu_pend(s, EU_PEND_CLKDIV,
                     efr32_alias_apply(addr, s->clkdiv, value));
        break;
    case EU_TRIGCTRL:
        ok = eu_pend(s, EU_PEND_TRIGCTRL,
                     efr32_alias_apply(addr, s->trigctrl, value));
        break;
    case EU_CMD:
        ok = eu_pend(s, EU_PEND_CMD, value & EU_CMD_MASK);
        break;
    case EU_TXDATA:
        if (eu_fifo_is_full(&s->tx_fifo)) {
            s->if_ |= EU_IF_TXOF;
        } else {
            eu_fifo_push(&s->tx_fifo, value & EU_TXDATA_MASK);
            s->tx_complete = false;
            eu_tx_start(s);
        }
        break;
    case EU_IF:
        if (alias == 1 || alias == 2) {
            s->if_ = efr32_alias_apply(addr, s->if_, value) & EU_IF_MASK;
        }
        break;
    case EU_IEN:
        s->ien = efr32_alias_apply(addr, s->ien, value) & EU_IF_MASK;
        break;
    default:
        break;
    }
    eu_update(s);
    return ok ? MEMTX_OK : MEMTX_ERROR;
}

static const MemoryRegionOps eu_ops = {
    .read_with_attrs = eu_read,
    .write_with_attrs = eu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* Byte and half-word accesses act as word accesses (the LDMA writes
     * TXDATA a byte at a time). */
    .valid = {.min_access_size = 1, .max_access_size = 4},
    .impl = {.min_access_size = 4, .max_access_size = 4},
};

/* PRS consumer EUART0 TRIGGER (TRIGCTRL.RXTEN / TXTEN, SS22.3.27). */
static void eu_prs_trigger(void *opaque, int n, int level)
{
    Efr32EusartState *s = opaque;
    bool rising = level && !s->prs_trigger;

    s->prs_trigger = level;
    if (!rising || !s->en) {
        return;
    }
    if (s->trigctrl & EU_TRIGCTRL_TXTEN) {
        eu_command(s, EU_CMD_TXEN);
    }
    if (s->trigctrl & EU_TRIGCTRL_RXTEN) {
        eu_command(s, EU_CMD_RXEN);
    }
    eu_update(s);
}

static void eu_reset_hold(Object *obj, ResetType type)
{
    Efr32EusartState *s = EFR32_EUSART(obj);

    s->en = s->cfg0 = s->cfg1 = s->irhfcfg = s->irlfcfg = s->timingcfg = 0;
    s->framecfg = EU_FRAMECFG_RESET;
    s->startframe = s->sigframe = s->clkdiv = s->trigctrl = 0;
    s->if_ = s->ien = 0;
    s->rx_enabled = s->tx_enabled = s->rx_block = s->tx_tri = false;
    s->tx_complete = false;
    s->tx_was_idle = true;
    s->tx_shifting = s->rx_shifting = false;
    eu_fifo_reset(&s->tx_fifo);
    eu_fifo_reset(&s->rx_fifo);
    timer_del(s->tx_timer);
    timer_del(s->rx_timer);
    timer_del(s->sync_timer);
    memset(s->pend, 0, sizeof(s->pend));
}

static void eu_reset_exit(Object *obj, ResetType type)
{
    eu_update(EFR32_EUSART(obj));
}

static void eu_realize(DeviceState *dev, Error **errp)
{
    Efr32EusartState *s = EFR32_EUSART(dev);

    qemu_chr_fe_set_handlers(&s->chr, eu_can_receive, eu_receive, eu_event,
                             NULL, s, NULL, true);
}

static void eu_init(Object *obj)
{
    Efr32EusartState *s = EFR32_EUSART(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &eu_ops, s, TYPE_EFR32_EUSART,
                          0x4000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->rx_irq);
    sysbus_init_irq(sbd, &s->tx_irq);
    qdev_init_gpio_out_named(dev, &s->prs_txc, EFR32_EUSART_PRS_TXC, 1);
    qdev_init_gpio_out_named(dev, &s->prs_rxfl, EFR32_EUSART_PRS_RXFL, 1);
    qdev_init_gpio_out_named(dev, s->dma_req, EFR32_EUSART_DMA_REQ, 2);
    qdev_init_gpio_in_named(dev, eu_prs_trigger, EFR32_EUSART_PRS_TRIGGER, 1);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, eu_tx_frame_end, s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, eu_rx_frame_end, s);
    s->sync_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, eu_sync_expired, s);
}

static Property eu_properties[] = {
    DEFINE_PROP_CHR("chardev", Efr32EusartState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void eu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = eu_realize;
    rc->phases.hold = eu_reset_hold;
    rc->phases.exit = eu_reset_exit;
    device_class_set_props(dc, eu_properties);
}

static const TypeInfo eu_info = {
    .name = TYPE_EFR32_EUSART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32EusartState),
    .instance_init = eu_init,
    .class_init = eu_class_init,
};

static void eu_register_types(void)
{
    type_register_static(&eu_info);
}
type_init(eu_register_types)

DeviceState *efr32_eusart_create(Object *soc, MemoryRegion *memory,
                                 const Efr32SocLink *link, hwaddr base,
                                 hwaddr ns_offset, qemu_irq rx_irq,
                                 qemu_irq tx_irq)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_EUSART);
    Efr32EusartState *s = EFR32_EUSART(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    const char *soc_name = object_get_canonical_path_component(soc);
    g_autofree char *id = g_strdup_printf("%s.euart0", soc_name);

    object_property_add_child(soc, "euart0", OBJECT(dev));
    qdev_prop_set_chr(dev, "chardev", qemu_chr_find(id));
    s->link = link;
    s->base = base;
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion_overlap(memory, base, &s->mmio, 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-eusart-ns",
                             &s->mmio, 0, 0x4000);
    memory_region_add_subregion_overlap(memory, base + ns_offset, &s->mmio_ns,
                                        1);
    sysbus_connect_irq(sbd, 0, rx_irq);
    sysbus_connect_irq(sbd, 1, tx_irq);
    return dev;
}
