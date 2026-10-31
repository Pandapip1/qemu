/*
 * EFR32 Series 2 I2C, leader (master) mode (EFR32xG22 reference manual
 * rev. 1.2, chapter 23).
 *
 * Register layout and fields: the public efr32bg22_i2c.h (I2C_TypeDef, SET,
 * CLR and TGL aliases at +0x1000/+0x2000/+0x3000).  The line side is a QEMU
 * I2C bus: followers attach to it the way they would to SDA/SCL, a board
 * model or a host adapter through a passthrough device.
 *
 * Leader state machine (RM SS23.3.8, Figure 23.16, Tables 23.2-23.4):
 *  - CMD.START makes a START once the bus is idle; the address comes from
 *    the transmit buffer (bit 0 = R/W).  The follower's ACK/NACK sets
 *    IF.ACK or IF.NACK; a missing follower is a NACK.
 *  - Wherever the leader needs an interaction it consults the pending
 *    commands in the priority order of Table 23.2 (STOP, ABORT, CONT, NACK,
 *    ACK, address, START, data); with none applicable it holds the bus,
 *    sets IF.BUSHOLD and reports the table's I2C_STATE value.
 *  - After a NACK only CONT (with data), STOP or START continue.
 *  - Receiving, each byte waits for ACK or NACK (or AUTOACK); ACK/NACK with
 *    STOP or START ends the read with that condition.
 *  - STOP sets IF.MSTOP and frees the bus.
 *  - After reset the bus counts as busy (SS23.3.8.4) until CMD.ABORT, or
 *    until the bus idle timeout with CTRL.GIBITO.
 *  - Buffers (SS23.3.7): 2-level transmit and receive FIFOs; TXBL and
 *    RXDATAV/RXFULL in STATUS follow the levels, their IF bits latch; IF.TXBL
 *    is set when EN goes to 1.
 * Timing: every bit takes ((Nlow + Nhigh) x (DIV + 1) + 8) I2CCLK cycles
 * (Figure 23.11 with Nfall = Nrise = 0), a START or STOP one bit, an address
 * or data byte nine bits including the acknowledge.  I2CCLK is LSPCLK
 * (PCLK / 2, RM SS8.3.1.4) for I2C0.
 *
 * Fault injection (properties, settable at run time with qom-set):
 *  - "x-sda-stuck": something holds SDA low: no START can be generated, the
 *    leader stays busy with START pending (firmware timeouts decide).
 *  - "x-nack-addr": 7-bit address whose address phase is NACKed although a
 *    follower answers (an absent or wedged part), 0xffffffff for none.
 *
 * [U] Not modelled: follower mode, arbitration with other leaders, the clock
 * low timeout, SCL/SDA monitoring, 10-bit addresses, DMA and PRS, clock
 * stretching by the follower, and the exact timing of BUSHOLD and of the
 * flags inside a byte (all flags of a byte appear at its ninth bit).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/fifo8.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/arm/efr32-i2c.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32I2cState, EFR32_I2C)

enum {
    I2C_IPVERSION = 0x00,
    I2C_EN = 0x04,
    I2C_CTRL = 0x08,
    I2C_CMD = 0x0c,
    I2C_STATE = 0x10,
    I2C_STATUS = 0x14,
    I2C_CLKDIV = 0x18,
    I2C_SADDR = 0x1c,
    I2C_SADDRMASK = 0x20,
    I2C_RXDATA = 0x24,
    I2C_RXDOUBLE = 0x28,
    I2C_RXDATAP = 0x2c,
    I2C_RXDOUBLEP = 0x30,
    I2C_TXDATA = 0x34,
    I2C_TXDOUBLE = 0x38,
    I2C_IF = 0x3c,
    I2C_IEN = 0x40,
    I2C_REGISTER_END = 0x44,
};

#define I2C_ALIAS_SHIFT 12
#define I2C_ALIAS_MASK 0x3000u

#define I2C_EN_EN (1u << 0)
#define I2C_CTRL_SLAVE (1u << 1)
#define I2C_CTRL_AUTOACK (1u << 2)
#define I2C_CTRL_CLHR_SHIFT 8
#define I2C_CTRL_CLHR_MASK 0x3u
#define I2C_CTRL_BITO_SHIFT 12
#define I2C_CTRL_BITO_MASK 0x3u
#define I2C_CTRL_GIBITO (1u << 15)
#define I2C_CLKDIV_DIV_MASK 0x1ffu

#define I2C_CMD_START (1u << 0)
#define I2C_CMD_STOP (1u << 1)
#define I2C_CMD_ACK (1u << 2)
#define I2C_CMD_NACK (1u << 3)
#define I2C_CMD_CONT (1u << 4)
#define I2C_CMD_ABORT (1u << 5)
#define I2C_CMD_CLEARTX (1u << 6)
#define I2C_CMD_CLEARPC (1u << 7)

#define I2C_STATUS_PSTART (1u << 0)
#define I2C_STATUS_PSTOP (1u << 1)
#define I2C_STATUS_PACK (1u << 2)
#define I2C_STATUS_PNACK (1u << 3)
#define I2C_STATUS_PCONT (1u << 4)
#define I2C_STATUS_TXC (1u << 6)
#define I2C_STATUS_TXBL (1u << 7)
#define I2C_STATUS_RXDATAV (1u << 8)
#define I2C_STATUS_RXFULL (1u << 9)
#define I2C_STATUS_TXBUFCNT_SHIFT 10

#define I2C_STATE_BUSY (1u << 0)
#define I2C_STATE_MASTER (1u << 1)
#define I2C_STATE_TRANSMITTER (1u << 2)
/* I2C_STATE while the bus is held (RM Tables 23.3 and 23.4). */
#define I2C_STATE_START_SENT 0x67u
#define I2C_STATE_ADDRW_ACK 0x97u
#define I2C_STATE_ADDRW_NACK 0x9fu
#define I2C_STATE_ADDRR_ACK 0xa3u
#define I2C_STATE_ADDRR_NACK 0x9bu
#define I2C_STATE_DATA_ACK 0xd7u
#define I2C_STATE_DATA_NACK 0xdfu
#define I2C_STATE_DATA_RECEIVED 0xb3u

#define I2C_IF_START (1u << 0)
#define I2C_IF_RSTART (1u << 1)
#define I2C_IF_TXC (1u << 3)
#define I2C_IF_TXBL (1u << 4)
#define I2C_IF_RXDATAV (1u << 5)
#define I2C_IF_ACK (1u << 6)
#define I2C_IF_NACK (1u << 7)
#define I2C_IF_MSTOP (1u << 8)
#define I2C_IF_BUSHOLD (1u << 11)
#define I2C_IF_TXOF (1u << 12)
#define I2C_IF_RXUF (1u << 13)
#define I2C_IF_BITO (1u << 14)
#define I2C_IF_RXFULL (1u << 17)
#define I2C_IF_MASK 0x001fffffu

#define I2C_BUFFER_BYTES 2
/* Figure 23.11: the fixed eight I2CCLK cycles of every SCL period. */
#define I2C_BIT_FIXED_CYCLES 8u
/* An address or data byte with its acknowledge. */
#define I2C_BYTE_BITS 9u
#define I2C_ADDR_READ 1u
/* CTRL.CLHR: Nlow and Nhigh (RM SS23.5.3). */
static const unsigned i2c_clhr_cycles[] = { 4 + 4, 6 + 3, 11 + 6, 11 + 6 };
/* CTRL.BITO: prescaled clock cycles before the bus idle timeout. */
static const unsigned i2c_bito_pcc[] = { 0, 40, 80, 160 };

typedef enum {
    I2C_PHASE_IDLE,
    I2C_PHASE_START,     /* START or repeated START on the wire */
    I2C_PHASE_WAIT_ADDR, /* START done, address needed */
    I2C_PHASE_ADDR,      /* address byte on the wire */
    I2C_PHASE_TX,        /* data byte on the wire */
    I2C_PHASE_SENT,      /* address or data transmitted, acknowledged */
    I2C_PHASE_RX,        /* data byte from the follower on the wire */
    I2C_PHASE_RECEIVED,  /* data byte received, ACK/NACK needed */
    I2C_PHASE_STOP,      /* STOP on the wire */
} Efr32I2cPhase;

struct Efr32I2cState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    qemu_irq irq;
    Clock *pclk;
    I2CBus *bus;
    IcountBusModel *bus_model;
    hwaddr base;
    uint32_t index;
    /* Fault injection, see the header comment. */
    uint32_t sda_stuck;
    uint32_t nack_addr; /* UINT32_MAX: none */

    uint32_t ctrl;
    uint32_t clkdiv;
    uint32_t saddr;
    uint32_t saddrmask;
    uint32_t if_latched;
    uint32_t ien;
    bool enabled;
    uint32_t pending; /* I2C_STATUS_P* */
    Fifo8 tx_fifo;
    Fifo8 rx_fifo;

    Efr32I2cPhase phase;
    bool bus_busy;
    bool in_transfer;
    bool reading;
    bool nacked;
    bool repeated;
    bool held;
    /* The last byte sent in PHASE_SENT was the address (not data). */
    bool sent_addr;
    uint8_t shift;
    QEMUTimer *bit_timer;
    QEMUTimer *bito_timer;
};

static uint64_t i2c_clk_hz(Efr32I2cState *s)
{
    /* I2C0: LSPCLK, "always prescaled by two" from PCLK (RM SS8.3.1.4). */
    return clock_get_hz(s->pclk) / (s->index == 0 ? 2 : 1);
}

static int64_t i2c_bits_ns(Efr32I2cState *s, unsigned bits)
{
    uint64_t hz = i2c_clk_hz(s);
    unsigned clhr = (s->ctrl >> I2C_CTRL_CLHR_SHIFT) & I2C_CTRL_CLHR_MASK;
    uint64_t cycles = (uint64_t)i2c_clhr_cycles[clhr] *
                      ((s->clkdiv & I2C_CLKDIV_DIV_MASK) + 1) +
                      I2C_BIT_FIXED_CYCLES;

    return hz ? muldiv64(bits * cycles, NANOSECONDS_PER_SECOND, hz) : -1;
}

static uint32_t i2c_status(Efr32I2cState *s)
{
    uint32_t used = fifo8_num_used(&s->tx_fifo);

    return s->pending |
           (used == 0 && s->phase != I2C_PHASE_ADDR &&
            s->phase != I2C_PHASE_TX ? I2C_STATUS_TXC : 0) |
           (used == 0 ? I2C_STATUS_TXBL : 0) |
           (!fifo8_is_empty(&s->rx_fifo) ? I2C_STATUS_RXDATAV : 0) |
           (fifo8_is_full(&s->rx_fifo) ? I2C_STATUS_RXFULL : 0) |
           (used << I2C_STATUS_TXBUFCNT_SHIFT);
}

static uint32_t i2c_state(Efr32I2cState *s)
{
    uint32_t state = s->bus_busy ? I2C_STATE_BUSY : 0;

    if (s->held) {
        switch (s->phase) {
        case I2C_PHASE_WAIT_ADDR:
            return I2C_STATE_START_SENT;
        case I2C_PHASE_SENT:
            if (s->sent_addr && s->reading) {
                return s->nacked ? I2C_STATE_ADDRR_NACK : I2C_STATE_ADDRR_ACK;
            }
            if (s->sent_addr) {
                return s->nacked ? I2C_STATE_ADDRW_NACK : I2C_STATE_ADDRW_ACK;
            }
            return s->nacked ? I2C_STATE_DATA_NACK : I2C_STATE_DATA_ACK;
        case I2C_PHASE_RECEIVED:
            return I2C_STATE_DATA_RECEIVED;
        default:
            break;
        }
    }
    if (s->phase != I2C_PHASE_IDLE) {
        state |= I2C_STATE_MASTER | (s->reading ? 0 : I2C_STATE_TRANSMITTER);
    }
    return state;
}

static void i2c_update_irq(Efr32I2cState *s)
{
    qemu_set_irq(s->irq, !!(s->if_latched & s->ien));
}

static void i2c_flag(Efr32I2cState *s, uint32_t flags)
{
    s->if_latched |= flags;
}

static void i2c_shift(Efr32I2cState *s, Efr32I2cPhase phase, unsigned bits)
{
    int64_t ns = i2c_bits_ns(s, bits);

    s->phase = phase;
    s->held = false;
    if (ns < 0) {
        return; /* no clock: the bus never moves */
    }
    timer_mod(s->bit_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

static void i2c_hold(Efr32I2cState *s)
{
    if (!s->held) {
        s->held = true;
        i2c_flag(s, I2C_IF_BUSHOLD);
    }
}

static void i2c_begin_stop(Efr32I2cState *s)
{
    s->pending &= ~I2C_STATUS_PSTOP;
    i2c_shift(s, I2C_PHASE_STOP, 1);
}

static void i2c_begin_start(Efr32I2cState *s, bool repeated)
{
    s->pending &= ~I2C_STATUS_PSTART;
    s->repeated = repeated;
    i2c_shift(s, I2C_PHASE_START, 1);
}

/* Bus idle timeout while the bus is busy without us (SS23.3.8.4). */
static void i2c_arm_bito(Efr32I2cState *s)
{
    unsigned bito = (s->ctrl >> I2C_CTRL_BITO_SHIFT) & I2C_CTRL_BITO_MASK;
    uint64_t hz = i2c_clk_hz(s);

    if (!bito || !hz || timer_pending(s->bito_timer)) {
        return;
    }
    timer_mod(s->bito_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  muldiv64((uint64_t)i2c_bito_pcc[bito] *
                               ((s->clkdiv & I2C_CLKDIV_DIV_MASK) + 1),
                           NANOSECONDS_PER_SECOND, hz));
}

/* Take the highest-priority interaction available (RM Table 23.2). */
static void i2c_interact(Efr32I2cState *s)
{
    bool ack;

    if (!s->enabled) {
        return;
    }
    switch (s->phase) {
    case I2C_PHASE_IDLE:
        s->pending &= ~I2C_STATUS_PSTOP; /* nothing to stop */
        if (!(s->pending & I2C_STATUS_PSTART)) {
            return;
        }
        if (s->sda_stuck || s->bus_busy) {
            if (!s->sda_stuck) {
                i2c_arm_bito(s);
            }
            return;
        }
        i2c_begin_start(s, false);
        return;
    case I2C_PHASE_WAIT_ADDR:
        if (s->pending & I2C_STATUS_PSTOP) {
            i2c_begin_stop(s);
        } else if (!fifo8_is_empty(&s->tx_fifo)) {
            s->shift = fifo8_pop(&s->tx_fifo);
            s->reading = s->shift & I2C_ADDR_READ;
            if (fifo8_is_empty(&s->tx_fifo)) {
                i2c_flag(s, I2C_IF_TXBL);
            }
            i2c_shift(s, I2C_PHASE_ADDR, I2C_BYTE_BITS);
        } else {
            i2c_hold(s);
        }
        return;
    case I2C_PHASE_SENT:
        if (s->pending & I2C_STATUS_PSTOP) {
            i2c_begin_stop(s);
        } else if (s->pending & I2C_STATUS_PSTART) {
            i2c_begin_start(s, true);
        } else if (s->nacked && !(s->pending & I2C_STATUS_PCONT)) {
            i2c_hold(s);
        } else if (s->reading) {
            if (fifo8_is_full(&s->rx_fifo)) {
                i2c_hold(s);
                return;
            }
            s->pending &= ~I2C_STATUS_PCONT;
            s->nacked = false;
            i2c_shift(s, I2C_PHASE_RX, I2C_BYTE_BITS);
        } else if (!fifo8_is_empty(&s->tx_fifo)) {
            s->pending &= ~I2C_STATUS_PCONT;
            s->shift = fifo8_pop(&s->tx_fifo);
            if (fifo8_is_empty(&s->tx_fifo)) {
                i2c_flag(s, I2C_IF_TXBL);
            }
            i2c_shift(s, I2C_PHASE_TX, I2C_BYTE_BITS);
        } else {
            i2c_hold(s);
        }
        return;
    case I2C_PHASE_RECEIVED:
        ack = (s->ctrl & I2C_CTRL_AUTOACK) || (s->pending & I2C_STATUS_PACK);
        if (!ack && !(s->pending & I2C_STATUS_PNACK)) {
            i2c_hold(s);
            return;
        }
        if (ack) {
            s->pending &= ~I2C_STATUS_PACK;
            i2c_ack(s->bus);
        } else {
            s->pending &= ~I2C_STATUS_PNACK;
            i2c_nack(s->bus);
        }
        s->phase = I2C_PHASE_SENT;
        s->sent_addr = false;
        s->nacked = !ack;
        s->held = false;
        if (!ack && !(s->pending & (I2C_STATUS_PSTOP | I2C_STATUS_PSTART))) {
            i2c_hold(s);
            return;
        }
        i2c_interact(s);
        return;
    default:
        return; /* on the wire */
    }
}

static void i2c_bit_timer(void *opaque)
{
    Efr32I2cState *s = opaque;
    int result;

    switch (s->phase) {
    case I2C_PHASE_START:
        s->bus_busy = true;
        i2c_flag(s, I2C_IF_START | (s->repeated ? I2C_IF_RSTART : 0));
        s->phase = I2C_PHASE_WAIT_ADDR;
        trace_efr32_i2c_event(s->index, s->repeated ? "rstart" : "start", 0, 0);
        break;
    case I2C_PHASE_ADDR:
        if ((s->shift >> 1) == s->nack_addr) {
            result = 1;
        } else {
            result = i2c_start_transfer(s->bus, s->shift >> 1, s->reading);
        }
        s->in_transfer = result == 0;
        s->nacked = result != 0;
        s->sent_addr = true;
        s->phase = I2C_PHASE_SENT;
        i2c_flag(s, (s->nacked ? I2C_IF_NACK : I2C_IF_ACK) |
                        (fifo8_is_empty(&s->tx_fifo) ? I2C_IF_TXC : 0));
        trace_efr32_i2c_event(s->index, "addr", s->shift, result);
        break;
    case I2C_PHASE_TX:
        result = s->in_transfer ? i2c_send(s->bus, s->shift) : 1;
        s->nacked = result != 0;
        s->sent_addr = false;
        s->phase = I2C_PHASE_SENT;
        i2c_flag(s, (s->nacked ? I2C_IF_NACK : I2C_IF_ACK) |
                        (fifo8_is_empty(&s->tx_fifo) ? I2C_IF_TXC : 0));
        trace_efr32_i2c_event(s->index, "tx", s->shift, result);
        break;
    case I2C_PHASE_RX:
        s->shift = s->in_transfer ? i2c_recv(s->bus) : 0xff;
        fifo8_push(&s->rx_fifo, s->shift);
        i2c_flag(s, I2C_IF_RXDATAV |
                        (fifo8_is_full(&s->rx_fifo) ? I2C_IF_RXFULL : 0));
        s->phase = I2C_PHASE_RECEIVED;
        trace_efr32_i2c_event(s->index, "rx", s->shift, 0);
        break;
    case I2C_PHASE_STOP:
        if (s->in_transfer) {
            i2c_end_transfer(s->bus);
            s->in_transfer = false;
        }
        s->bus_busy = false;
        s->reading = false;
        s->nacked = false;
        s->phase = I2C_PHASE_IDLE;
        i2c_flag(s, I2C_IF_MSTOP);
        trace_efr32_i2c_event(s->index, "stop", 0, 0);
        break;
    default:
        break;
    }
    i2c_interact(s);
    i2c_update_irq(s);
}

static void i2c_bito_timer(void *opaque)
{
    Efr32I2cState *s = opaque;

    if (s->phase != I2C_PHASE_IDLE || !s->bus_busy) {
        return;
    }
    i2c_flag(s, I2C_IF_BITO);
    if (s->ctrl & I2C_CTRL_GIBITO) {
        s->bus_busy = false;
    }
    i2c_interact(s);
    i2c_update_irq(s);
}

static void i2c_abort(Efr32I2cState *s)
{
    timer_del(s->bit_timer);
    timer_del(s->bito_timer);
    if (s->in_transfer) {
        i2c_end_transfer(s->bus);
        s->in_transfer = false;
    }
    s->phase = I2C_PHASE_IDLE;
    s->held = false;
    s->reading = false;
    s->nacked = false;
    s->pending = 0;
    /* "the I2C module is instructed that the bus is idle" (SS23.3.8.4). */
    s->bus_busy = false;
}

static void i2c_command(Efr32I2cState *s, uint32_t cmd)
{
    if (cmd & I2C_CMD_CLEARPC) {
        s->pending = 0;
    }
    if (cmd & I2C_CMD_CLEARTX) {
        fifo8_reset(&s->tx_fifo);
    }
    if (cmd & I2C_CMD_ABORT) {
        i2c_abort(s);
        return;
    }
    s->pending |= (cmd & I2C_CMD_START ? I2C_STATUS_PSTART : 0) |
                  (cmd & I2C_CMD_STOP ? I2C_STATUS_PSTOP : 0) |
                  (cmd & I2C_CMD_ACK ? I2C_STATUS_PACK : 0) |
                  (cmd & I2C_CMD_NACK ? I2C_STATUS_PNACK : 0) |
                  (cmd & I2C_CMD_CONT ? I2C_STATUS_PCONT : 0);
    i2c_interact(s);
}

static void i2c_tx_push(Efr32I2cState *s, uint8_t value)
{
    if (fifo8_is_full(&s->tx_fifo)) {
        i2c_flag(s, I2C_IF_TXOF);
        return;
    }
    fifo8_push(&s->tx_fifo, value);
    i2c_interact(s);
}

static uint8_t i2c_rx_pop(Efr32I2cState *s, bool peek)
{
    uint8_t value;

    if (fifo8_is_empty(&s->rx_fifo)) {
        i2c_flag(s, I2C_IF_RXUF);
        return 0;
    }
    if (peek) {
        return fifo8_peek(&s->rx_fifo);
    }
    value = fifo8_pop(&s->rx_fifo);
    /* Space in the buffer lets a held receiver continue. */
    i2c_interact(s);
    return value;
}

static void i2c_access(Efr32I2cState *s, hwaddr offset, unsigned size,
                       bool is_write)
{
    if (s->bus_model) {
        icount_bus_access(s->bus_model, current_cpu, s->base + offset, size,
                          is_write);
    }
}

static uint64_t i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    Efr32I2cState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)I2C_ALIAS_MASK;
    uint32_t value = 0;

    i2c_access(s, addr, size, false);
    switch (offset) {
    case I2C_EN:
        value = s->enabled;
        break;
    case I2C_CTRL:
        value = s->ctrl;
        break;
    case I2C_STATE:
        value = i2c_state(s);
        break;
    case I2C_STATUS:
        value = i2c_status(s);
        break;
    case I2C_CLKDIV:
        value = s->clkdiv;
        break;
    case I2C_SADDR:
        value = s->saddr;
        break;
    case I2C_SADDRMASK:
        value = s->saddrmask;
        break;
    case I2C_RXDATA:
        value = i2c_rx_pop(s, false);
        break;
    case I2C_RXDATAP:
        value = i2c_rx_pop(s, true);
        break;
    case I2C_RXDOUBLE:
        value = i2c_rx_pop(s, false);
        value |= (uint32_t)i2c_rx_pop(s, false) << 8;
        break;
    case I2C_IF:
        value = s->if_latched;
        break;
    case I2C_IEN:
        value = s->ien;
        break;
    default:
        break;
    }
    i2c_update_irq(s);
    return value;
}

static uint32_t i2c_apply_alias(unsigned alias, uint32_t old, uint32_t value)
{
    switch (alias) {
    case 1:
        return old | value;
    case 2:
        return old & ~value;
    case 3:
        return old ^ value;
    default:
        return value;
    }
}

static void i2c_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    Efr32I2cState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)I2C_ALIAS_MASK;
    unsigned alias = (addr & I2C_ALIAS_MASK) >> I2C_ALIAS_SHIFT;
    uint32_t value = data;
    bool was_enabled = s->enabled;

    i2c_access(s, addr, size, true);
    switch (offset) {
    case I2C_EN:
        s->enabled = i2c_apply_alias(alias, s->enabled, value) & I2C_EN_EN;
        if (s->enabled && !was_enabled) {
            i2c_flag(s, I2C_IF_TXBL); /* SS23.3.7.1 */
        } else if (!s->enabled) {
            i2c_abort(s);
            /* Disabled, the bus state is unknown again (SS23.3.8.4). */
            s->bus_busy = true;
        }
        break;
    case I2C_CTRL:
        s->ctrl = i2c_apply_alias(alias, s->ctrl, value);
        if (s->ctrl & I2C_CTRL_SLAVE) {
            qemu_log_mask(LOG_UNIMP, "efr32-i2c%u: follower mode\n", s->index);
        }
        break;
    case I2C_CMD:
        if (s->enabled) {
            i2c_command(s, value);
        }
        break;
    case I2C_CLKDIV:
        s->clkdiv = i2c_apply_alias(alias, s->clkdiv, value) &
                    I2C_CLKDIV_DIV_MASK;
        break;
    case I2C_SADDR:
        s->saddr = i2c_apply_alias(alias, s->saddr, value);
        break;
    case I2C_SADDRMASK:
        s->saddrmask = i2c_apply_alias(alias, s->saddrmask, value);
        break;
    case I2C_TXDATA:
        i2c_tx_push(s, value);
        break;
    case I2C_TXDOUBLE:
        i2c_tx_push(s, value);
        i2c_tx_push(s, value >> 8);
        break;
    case I2C_IF:
        s->if_latched = i2c_apply_alias(alias, s->if_latched, value) &
                        I2C_IF_MASK;
        break;
    case I2C_IEN:
        s->ien = i2c_apply_alias(alias, s->ien, value) & I2C_IF_MASK;
        break;
    default:
        break;
    }
    i2c_update_irq(s);
}

static const MemoryRegionOps i2c_ops = {
    .read = i2c_read,
    .write = i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

static void i2c_reset_hold(Object *obj, ResetType type)
{
    Efr32I2cState *s = EFR32_I2C(obj);

    i2c_abort(s);
    s->ctrl = 0;
    s->clkdiv = 0;
    s->saddr = 0;
    s->saddrmask = 0;
    s->if_latched = 0;
    s->ien = 0;
    s->enabled = false;
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    s->bus_busy = true; /* SS23.3.8.4 */
    qemu_set_irq(s->irq, 0);
}

static void i2c_realize(DeviceState *dev, Error **errp)
{
    Efr32I2cState *s = EFR32_I2C(dev);
    g_autofree char *bus = g_strdup_printf("i2c%u", s->index);

    /* "-device <i2c-device>,bus=i2c0,address=..." attaches to I2C0. */
    s->bus = i2c_init_bus(dev, bus);
}

static void i2c_init(Object *obj)
{
    Efr32I2cState *s = EFR32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &i2c_ops, s, TYPE_EFR32_I2C,
                          EFR32_I2C_BLOCK_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    s->pclk = qdev_init_clock_in(DEVICE(obj), "pclk", NULL, NULL, 0);
    fifo8_create(&s->tx_fifo, I2C_BUFFER_BYTES);
    fifo8_create(&s->rx_fifo, I2C_BUFFER_BYTES);
    s->bit_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, i2c_bit_timer, s);
    s->bito_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, i2c_bito_timer, s);
    /* Fault injection, settable at run time (qom-set). */
    s->nack_addr = UINT32_MAX;
    object_property_add_uint32_ptr(obj, "x-sda-stuck", &s->sda_stuck,
                                 OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "x-nack-addr", &s->nack_addr,
                                   OBJ_PROP_FLAG_READWRITE);
}

static void i2c_finalize(Object *obj)
{
    Efr32I2cState *s = EFR32_I2C(obj);

    timer_free(s->bit_timer);
    timer_free(s->bito_timer);
    fifo8_destroy(&s->tx_fifo);
    fifo8_destroy(&s->rx_fifo);
}

static Property i2c_properties[] = {
    DEFINE_PROP_UINT32("index", Efr32I2cState, index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = i2c_realize;
    rc->phases.hold = i2c_reset_hold;
    device_class_set_props(dc, i2c_properties);
}

static const TypeInfo i2c_info = {
    .name = TYPE_EFR32_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32I2cState),
    .instance_init = i2c_init,
    .instance_finalize = i2c_finalize,
    .class_init = i2c_class_init,
};

static void i2c_register_types(void)
{
    type_register_static(&i2c_info);
}
type_init(i2c_register_types)

DeviceState *efr32_i2c_create(Object *soc, MemoryRegion *memory,
                              DeviceState *nvic, Clock *pclk,
                              IcountBusModel *bus_model, unsigned index,
                              hwaddr base, hwaddr ns_offset, unsigned irq)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_I2C);
    Efr32I2cState *s = EFR32_I2C(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    g_autofree char *name = g_strdup_printf("i2c%u", index);

    object_property_add_child(soc, name, OBJECT(dev));
    qdev_prop_set_uint32(dev, "index", index);
    qdev_connect_clock_in(dev, "pclk", pclk);
    s->bus_model = bus_model;
    s->base = base;
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion_overlap(memory, base,
                                        sysbus_mmio_get_region(sbd, 0), 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-i2c-ns",
                             &s->mmio, 0, EFR32_I2C_BLOCK_SIZE);
    memory_region_add_subregion_overlap(memory, base + ns_offset,
                                        &s->mmio_ns, 1);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(nvic, irq));
    return dev;
}
