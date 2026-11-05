/*
 * EFR32xG22 LDMA, linked DMA controller, and its request crossbar LDMAXBAR
 * (RM rev. 1.2 chapter 26), at 0x40040000 and 0x40044000 (non-secure
 * 0x5004xxxx), IRQ 21, bus clocks CMU_CLKEN0.LDMA (bit 0) and .LDMAXBAR
 * (bit 1).  Register layout: the public efr32bg22_ldma.h / _ldmaxbar.h.
 *
 * Modelled (RM SS26.3):
 *  - 8 channels with the descriptor registers CTRL, SRC, DST, LINK and
 *    CFG, LOOP; XFER descriptors (SIZE, SRCINC/DSTINC 1/2/4/none units,
 *    SRCINCSIGN/DSTINCSIGN, BYTESWAP, XFERCNT + 1 units, BLOCKSIZE units
 *    per arbitration, REQMODE BLOCK/ALL, DONEIEN, DECLOOPCNT looping),
 *    SYNC descriptors (SYNCSET/SYNCCLR, MATCHEN/MATCHVAL against
 *    SYNCSTATUS.SYNCTRIG) and WRI descriptors (one immediate write);
 *    absolute and relative SRC/DST/LINK addressing of linked descriptors
 *    (SS26.3.1.7); the initial descriptor written or loaded with LINKLOAD
 *    (which also enables the channel), STRUCTREQ;
 *  - requests: LDMAXBAR CHn_REQSEL SOURCESEL/SIGSEL (SS26.5) selects a
 *    request line; SWREQ; REQDIS masks peripheral requests; REQCLEAR;
 *    REQPEND, CHBUSY, CHSTATUS, CHDONE, STATUS.ANYBUSY/ANYREQ/CHGRANT/
 *    CHERROR, CHNUM;
 *  - arbitration: channels below CTRL.NUMFIXED by fixed priority, the rest
 *    round robin weighted by CFG.ARBSLOTS (Table 26.1's slot order);
 *  - IF.DONEn (DONEIEN), IF.ERROR on a bus error (STATUS.CHERROR, the
 *    channel stops); IRQ = IF & IEN;
 *  - SYNCSWSET/SYNCSWCLR, and SYNCHWEN/SYNCHWSEL: PRS channels 0-7 setting
 *    or clearing SYNCTRIG bits on the selected edge;
 *  - silicon (owo-open-fw docs/radio-tables.md SS7.74): an LDMA access to a
 *    radio register (RAC) ends the channel with IF.ERROR after the block;
 *    [I] the whole 0xA8000000-0xA80FFFFF radio register window is treated
 *    so.  Clearing EN does not stop a busy channel; CHDIS does.
 * Requests are levels; a rising edge is also latched as one pending request
 * (REQPEND), so a peripheral's one-cycle request pulse (a TIMER event) is
 * served once.  [I] The SREQ/REQ distinction (IGNORESREQ) is not made.
 * Timing [I] (no silicon figure for the LDMA's own throughput): each unit
 * transfer costs 2 HCLK cycles (a read and a write), a descriptor load 4,
 * arbitration 1; a block's data moves at the block's start.  The LDMA is
 * not a manager in the interconnect model (its contention on SRAM and
 * SEQRAM, SS7.74 table 6, is not reproduced).
 * [U] Not modelled: DBGHALT, CORERST, the FIFO level, EM2 wake.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/arm/efr32-ldma.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32LdmaState, EFR32_LDMA)

enum {
    LD_IPVERSION = 0x000,
    LD_EN = 0x004,
    LD_CTRL = 0x008,
    LD_STATUS = 0x00c,
    LD_SYNCSWSET = 0x010,
    LD_SYNCSWCLR = 0x014,
    LD_SYNCHWEN = 0x018,
    LD_SYNCHWSEL = 0x01c,
    LD_SYNCSTATUS = 0x020,
    LD_CHEN = 0x024,
    LD_CHDIS = 0x028,
    LD_CHSTATUS = 0x02c,
    LD_CHBUSY = 0x030,
    LD_CHDONE = 0x034,
    LD_DBGHALT = 0x038,
    LD_SWREQ = 0x03c,
    LD_REQDIS = 0x040,
    LD_REQPEND = 0x044,
    LD_LINKLOAD = 0x048,
    LD_REQCLEAR = 0x04c,
    LD_IF = 0x050,
    LD_IEN = 0x054,
    LD_CH0 = 0x058,
};
#define LD_CH_STRIDE 0x30u
#define LD_CH_CFG 0x04u
#define LD_CH_LOOP 0x08u
#define LD_CH_CTRL 0x0cu
#define LD_CH_SRC 0x10u
#define LD_CH_DST 0x14u
#define LD_CH_LINK 0x18u
#define LD_CHANNELS 8
#define LD_CH_MASK 0xffu

#define LD_IPVERSION_VALUE 0x0u
#define LD_CTRL_RESET 0x1e000000u
#define LD_CTRL_MASK 0x9f000000u
#define LD_CTRL_NUMFIXED_SHIFT 24
#define LD_CTRL_NUMFIXED_MASK 0x1fu
#define LD_STATUS_ANYBUSY (1u << 0)
#define LD_STATUS_ANYREQ (1u << 1)
#define LD_STATUS_CHGRANT_SHIFT 3
#define LD_STATUS_CHERROR_SHIFT 8
#define LD_STATUS_FIFOLEVEL_IDLE (0x10u << 16)
#define LD_STATUS_CHNUM (8u << 24)
#define LD_SYNC_MASK 0xffu
#define LD_SYNCHWEN_MASK 0x00ff00ffu
#define LD_IF_ERROR (1u << 31)
#define LD_IF_MASK 0x800000ffu
#define LD_CFG_MASK 0x00330000u
#define LD_CFG_ARBSLOTS_SHIFT 16
#define LD_CFG_ARBSLOTS_MASK 0x3u
#define LD_CFG_SRCINCSIGN (1u << 20)
#define LD_CFG_DSTINCSIGN (1u << 21)
#define LD_LOOP_MASK 0xffu
#define LD_CTRL_STRUCTTYPE_MASK 0x3u
#define LD_STRUCT_XFER 0u
#define LD_STRUCT_SYNC 1u
#define LD_STRUCT_WRI 2u
#define LD_CTRL_STRUCTREQ (1u << 3)
#define LD_CTRL_XFERCNT_SHIFT 4
#define LD_CTRL_XFERCNT_MASK 0x7ffu
#define LD_CTRL_BYTESWAP (1u << 15)
#define LD_CTRL_BLOCKSIZE_SHIFT 16
#define LD_CTRL_BLOCKSIZE_MASK 0xfu
#define LD_CTRL_DONEIEN (1u << 20)
#define LD_CTRL_REQMODE_ALL (1u << 21)
#define LD_CTRL_DECLOOPCNT (1u << 22)
#define LD_CTRL_SRCINC_SHIFT 24
#define LD_CTRL_SIZE_SHIFT 26
#define LD_CTRL_DSTINC_SHIFT 28
#define LD_CTRL_FIELD2_MASK 0x3u
#define LD_INC_NONE 3u
#define LD_CTRL_SRCMODE (1u << 30)
#define LD_CTRL_DSTMODE (1u << 31)
/* Software writes CTRL without the read-only STRUCTREQ, SRCMODE, DSTMODE. */
#define LD_CTRL_WRITABLE 0x3ffffff3u
#define LD_LINK_LINKMODE (1u << 0)
#define LD_LINK_LINK (1u << 1)
#define LD_LINK_ADDR_MASK 0xfffffffcu
#define LD_SYNC_SET_MASK 0xffu
#define LD_SYNC_CLR_SHIFT 8
#define LD_REQSEL_SIGSEL_MASK 0xfu
#define LD_REQSEL_SOURCESEL_SHIFT 16
#define LD_REQSEL_SOURCESEL_MASK 0x3fu
#define LD_REQSEL_MASK 0x003f000fu
#define LD_BLOCK_LOCK 15u
#define LD_UNIT_CYCLES 2
#define LD_DESCRIPTOR_CYCLES 4
#define LD_ARBITRATION_CYCLES 1
#define LD_CLKEN0_LDMA (1u << 0)
#define LD_CLKEN0_LDMAXBAR (1u << 1)
#define LD_RADIO_REG_BASE 0xa8000000u
#define LD_RADIO_REG_END 0xa8100000u

/* Table 26.2: BLOCKSIZE -> unit transfers per arbitration (15: lock). */
static const uint32_t ld_block_units[16] = {
    1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 64, 128, 256, 512, 1024, UINT32_MAX,
};
/* Table 26.1's slot order (slot weights 8, 4, 2, 1). */
static const uint8_t ld_slot_order[15] = {
    8, 4, 8, 2, 8, 4, 8, 1, 8, 4, 8, 2, 8, 4, 8,
};

typedef struct LdChannel {
    uint32_t cfg, loop, ctrl, src, dst, link;
    uint32_t reqsel;
    /* Address of the descriptor being executed (for LINK after a loop). */
    uint32_t desc_addr;
    bool enabled;
    /* Unit transfers left in this descriptor, and in the current request. */
    uint32_t remaining;
    uint32_t request_units;
    bool pending;          /* a latched request (edge, SWREQ, STRUCTREQ) */
    bool line;             /* the selected request line's level */
} LdChannel;

struct Efr32LdmaState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    MemoryRegion xbar;
    MemoryRegion xbar_ns;
    qemu_irq irq;
    const Efr32SocLink *link;
    hwaddr base;
    QEMUTimer *timer;

    uint32_t en, ctrl, synchwen, synchwsel, synctrig, chdone, reqdis;
    uint32_t if_, ien, dbghalt, cherror, chgrant;
    bool busy;
    unsigned rr_next[9];
    unsigned slot_pos;
    bool req_level[EFR32_LDMA_REQ_LINES];
    bool prs_level[8];
    LdChannel ch[LD_CHANNELS];
};

static void ld_update_irq(Efr32LdmaState *s)
{
    qemu_set_irq(s->irq, !!(s->if_ & s->ien));
}

static unsigned ld_req_line(LdChannel *c)
{
    return ((c->reqsel >> LD_REQSEL_SOURCESEL_SHIFT) &
            LD_REQSEL_SOURCESEL_MASK) * EFR32_LDMA_SIGNALS +
           (c->reqsel & LD_REQSEL_SIGSEL_MASK);
}

static bool ld_line_level(Efr32LdmaState *s, LdChannel *c)
{
    unsigned line = ld_req_line(c);

    return (c->reqsel >> LD_REQSEL_SOURCESEL_SHIFT) & LD_REQSEL_SOURCESEL_MASK
        && line < EFR32_LDMA_REQ_LINES && s->req_level[line];
}

static bool ld_requesting(Efr32LdmaState *s, unsigned n)
{
    LdChannel *c = &s->ch[n];
    bool periph = !(s->reqdis & (1u << n)) && ld_line_level(s, c);

    return c->enabled && (c->pending || periph || c->request_units);
}

static uint64_t ld_hclk_hz(Efr32LdmaState *s)
{
    uint64_t hz = efr32_link_tree_hz(s->link, EFR32_TREE_HCLK);

    return hz ? hz : 1;
}

static void ld_error(Efr32LdmaState *s, unsigned n)
{
    s->if_ |= LD_IF_ERROR;
    s->cherror = n;
    s->ch[n].enabled = false;
    s->ch[n].request_units = 0;
    qemu_log_mask(LOG_GUEST_ERROR, "efr32-ldma: channel %u bus error\n", n);
}

static bool ld_access(Efr32LdmaState *s, uint32_t addr, void *buf,
                      unsigned size, bool write)
{
    MemTxAttrs attrs = { .secure = 1 };
    MemTxResult r;

    if (addr >= LD_RADIO_REG_BASE && addr < LD_RADIO_REG_END) {
        return false;
    }
    if (!s->link || !s->link->dma_as) {
        return false;
    }
    r = write ? address_space_write(s->link->dma_as, addr, attrs, buf, size)
              : address_space_read(s->link->dma_as, addr, attrs, buf, size);
    return r == MEMTX_OK;
}

/* Load the descriptor at @addr into channel @n (linked: modes apply). */
static bool ld_load_descriptor(Efr32LdmaState *s, unsigned n, uint32_t addr,
                               bool linked)
{
    LdChannel *c = &s->ch[n];
    uint32_t word[4];

    if (!ld_access(s, addr, word, sizeof(word), false)) {
        ld_error(s, n);
        return false;
    }
    for (unsigned i = 0; i < 4; i++) {
        word[i] = le32_to_cpu(word[i]);
    }
    c->desc_addr = addr;
    c->ctrl = word[0];
    if (linked && (word[0] & LD_CTRL_SRCMODE)) {
        c->src += word[1];
    } else {
        c->src = word[1];
    }
    if (linked && (word[0] & LD_CTRL_DSTMODE)) {
        c->dst += word[2];
    } else {
        c->dst = word[2];
    }
    /*
     * A relative link is relative to the descriptor's own address, also for
     * the one LINKLOAD fetches (emlib's LINKREL descriptor arrays start so).
     */
    if (word[3] & LD_LINK_LINKMODE) {
        c->link = ((addr + (word[3] & LD_LINK_ADDR_MASK)) & LD_LINK_ADDR_MASK) |
                  (word[3] & ~LD_LINK_ADDR_MASK);
    } else {
        c->link = word[3];
    }
    c->remaining = ((c->ctrl >> LD_CTRL_XFERCNT_SHIFT) & LD_CTRL_XFERCNT_MASK)
                   + 1;
    c->request_units = 0;
    if (c->ctrl & LD_CTRL_STRUCTREQ) {
        c->pending = true;
    }
    return true;
}

static void ld_set_done(Efr32LdmaState *s, unsigned n)
{
    if (s->ch[n].ctrl & LD_CTRL_DONEIEN) {
        s->if_ |= 1u << n;
    }
}

/*
 * The current descriptor of channel @n has finished: loop, link or stop.
 * Returns the extra cycles spent loading descriptors.
 */
static unsigned ld_descriptor_done(Efr32LdmaState *s, unsigned n);

/* Run SYNC and WRI descriptors at the head of channel @n. */
static unsigned ld_run_control(Efr32LdmaState *s, unsigned n)
{
    LdChannel *c = &s->ch[n];
    unsigned cycles = 0;

    for (unsigned guard = 0; c->enabled && guard < 64; guard++) {
        uint32_t type = c->ctrl & LD_CTRL_STRUCTTYPE_MASK;

        if (type == LD_STRUCT_WRI) {
            uint32_t v = cpu_to_le32(c->src);

            if (!ld_access(s, c->dst, &v, 4, true)) {
                ld_error(s, n);
                return cycles;
            }
            cycles += LD_UNIT_CYCLES + ld_descriptor_done(s, n);
        } else if (type == LD_STRUCT_SYNC) {
            uint32_t set = c->src & LD_SYNC_SET_MASK;
            uint32_t clr = (c->src >> LD_SYNC_CLR_SHIFT) & LD_SYNC_SET_MASK;
            uint32_t matchval = c->dst & LD_SYNC_SET_MASK;
            uint32_t matchen = (c->dst >> LD_SYNC_CLR_SHIFT) & LD_SYNC_SET_MASK;

            s->synctrig = (s->synctrig & ~clr) | set;
            c->src &= ~(LD_SYNC_SET_MASK | (LD_SYNC_SET_MASK <<
                                             LD_SYNC_CLR_SHIFT));
            if ((s->synctrig & matchen) != (matchval & matchen)) {
                return cycles;  /* waits for the match */
            }
            cycles += ld_descriptor_done(s, n);
        } else {
            return cycles;
        }
    }
    return cycles;
}

static unsigned ld_descriptor_done(Efr32LdmaState *s, unsigned n)
{
    LdChannel *c = &s->ch[n];
    bool loop = (c->ctrl & LD_CTRL_DECLOOPCNT) && c->loop;
    uint32_t next;

    ld_set_done(s, n);
    c->request_units = 0;
    if (c->ctrl & LD_CTRL_DECLOOPCNT) {
        if (loop) {
            c->loop--;
            next = c->link & LD_LINK_ADDR_MASK;
        } else if (c->link & LD_LINK_LINK) {
            next = c->desc_addr + 16;
        } else {
            goto stop;
        }
    } else if (c->link & LD_LINK_LINK) {
        next = c->link & LD_LINK_ADDR_MASK;
    } else {
        goto stop;
    }
    if (!ld_load_descriptor(s, n, next, true)) {
        return LD_DESCRIPTOR_CYCLES;
    }
    return LD_DESCRIPTOR_CYCLES + ld_run_control(s, n);

stop:
    s->chdone |= 1u << n;
    c->enabled = false;
    c->pending = false;
    return 0;
}

static uint32_t ld_unit_bytes(uint32_t ctrl)
{
    return 1u << ((ctrl >> LD_CTRL_SIZE_SHIFT) & LD_CTRL_FIELD2_MASK);
}

static int32_t ld_step(uint32_t ctrl, unsigned shift, bool negative)
{
    unsigned inc = (ctrl >> shift) & LD_CTRL_FIELD2_MASK;
    int32_t step;

    if (inc == LD_INC_NONE) {
        return 0;
    }
    step = (int32_t)(ld_unit_bytes(ctrl) << inc);
    return negative ? -step : step;
}

/* One unit transfer of channel @n. */
static bool ld_unit(Efr32LdmaState *s, unsigned n)
{
    LdChannel *c = &s->ch[n];
    unsigned bytes = ld_unit_bytes(c->ctrl);
    uint8_t buf[4];

    if (bytes > 4) {
        bytes = 4;
    }
    if (!ld_access(s, c->src, buf, bytes, false)) {
        return false;
    }
    if ((c->ctrl & LD_CTRL_BYTESWAP) && bytes > 1) {
        for (unsigned i = 0; i < bytes / 2; i++) {
            uint8_t t = buf[i];

            buf[i] = buf[bytes - 1 - i];
            buf[bytes - 1 - i] = t;
        }
    }
    if (!ld_access(s, c->dst, buf, bytes, true)) {
        return false;
    }
    c->src += ld_step(c->ctrl, LD_CTRL_SRCINC_SHIFT,
                      c->cfg & LD_CFG_SRCINCSIGN);
    c->dst += ld_step(c->ctrl, LD_CTRL_DSTINC_SHIFT,
                      c->cfg & LD_CFG_DSTINCSIGN);
    c->remaining--;
    c->ctrl = (c->ctrl & ~(LD_CTRL_XFERCNT_MASK << LD_CTRL_XFERCNT_SHIFT)) |
              (((c->remaining ? c->remaining - 1 : 0) & LD_CTRL_XFERCNT_MASK)
               << LD_CTRL_XFERCNT_SHIFT);
    return true;
}

static int ld_arbitrate(Efr32LdmaState *s)
{
    unsigned numfixed = MIN((s->ctrl >> LD_CTRL_NUMFIXED_SHIFT) &
                            LD_CTRL_NUMFIXED_MASK, LD_CHANNELS);

    for (unsigned n = 0; n < numfixed; n++) {
        if (ld_requesting(s, n)) {
            return n;
        }
    }
    /* Round robin, weighted by ARBSLOTS in Table 26.1's slot order. */
    for (unsigned tries = 0; tries < ARRAY_SIZE(ld_slot_order); tries++) {
        unsigned slot = ld_slot_order[s->slot_pos];

        s->slot_pos = (s->slot_pos + 1) % ARRAY_SIZE(ld_slot_order);
        for (unsigned k = 0; k < LD_CHANNELS; k++) {
            unsigned n = (s->rr_next[slot] + k) % LD_CHANNELS;
            unsigned weight = 1u << ((s->ch[n].cfg >> LD_CFG_ARBSLOTS_SHIFT) &
                                     LD_CFG_ARBSLOTS_MASK);

            if (n >= numfixed && weight == slot && ld_requesting(s, n)) {
                s->rr_next[slot] = n + 1;
                return n;
            }
        }
    }
    return -1;
}

static void ld_schedule(Efr32LdmaState *s, int64_t delay_ns)
{
    if (!s->busy) {
        s->busy = true;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
    }
}

static void ld_kick(Efr32LdmaState *s)
{
    for (unsigned n = 0; n < LD_CHANNELS; n++) {
        if (ld_requesting(s, n)) {
            ld_schedule(s, muldiv64(LD_ARBITRATION_CYCLES,
                                    NANOSECONDS_PER_SECOND, ld_hclk_hz(s)));
            return;
        }
    }
}

/* Serve one block of the channel that wins arbitration. */
static void ld_service(void *opaque)
{
    Efr32LdmaState *s = opaque;
    unsigned cycles = LD_ARBITRATION_CYCLES;
    int n;
    LdChannel *c;
    uint32_t units;

    s->busy = false;
    n = ld_arbitrate(s);
    if (n < 0) {
        ld_update_irq(s);
        return;
    }
    c = &s->ch[n];
    s->chgrant = n;
    cycles += ld_run_control(s, n);
    if (!c->enabled || (c->ctrl & LD_CTRL_STRUCTTYPE_MASK) != LD_STRUCT_XFER) {
        goto out;
    }
    if (!c->request_units) {
        /* A new request: one block, or the whole descriptor (ALL). */
        c->pending = false;
        c->request_units = c->ctrl & LD_CTRL_REQMODE_ALL ? c->remaining
                         : ld_block_units[(c->ctrl >> LD_CTRL_BLOCKSIZE_SHIFT) &
                                          LD_CTRL_BLOCKSIZE_MASK];
    }
    units = MIN(MIN(c->request_units,
                    ld_block_units[(c->ctrl >> LD_CTRL_BLOCKSIZE_SHIFT) &
                                   LD_CTRL_BLOCKSIZE_MASK]),
                c->remaining);
    for (uint32_t i = 0; i < units; i++) {
        if (!ld_unit(s, n)) {
            ld_error(s, n);
            goto out;
        }
        cycles += LD_UNIT_CYCLES;
        c->request_units--;
    }
    trace_efr32_ldma_block(n, units, c->remaining);
    if (!c->remaining) {
        cycles += ld_descriptor_done(s, n);
    }
out:
    ld_update_irq(s);
    ld_schedule(s, muldiv64(cycles, NANOSECONDS_PER_SECOND, ld_hclk_hz(s)));
}

static void ld_req_in(void *opaque, int line, int level)
{
    Efr32LdmaState *s = opaque;
    bool rising = level && !s->req_level[line];

    s->req_level[line] = level;
    if (rising) {
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            if (ld_req_line(&s->ch[n]) == (unsigned)line &&
                (s->ch[n].reqsel >> LD_REQSEL_SOURCESEL_SHIFT) &
                    LD_REQSEL_SOURCESEL_MASK &&
                !(s->reqdis & (1u << n))) {
                s->ch[n].pending = true;
            }
        }
    }
    ld_kick(s);
}

/* PRS channels 0-7 into SYNCTRIG (SYNCHWEN, SYNCHWSEL edges). */
static void ld_prs_in(void *opaque, int ch, int level)
{
    Efr32LdmaState *s = opaque;
    bool rising = level && !s->prs_level[ch];
    bool falling = !level && s->prs_level[ch];
    bool set_edge = (s->synchwsel >> ch) & 1 ? falling : rising;
    bool clr_edge = (s->synchwsel >> (16 + ch)) & 1 ? falling : rising;
    uint32_t bit = 1u << ch;

    s->prs_level[ch] = level;
    if ((s->synchwen & (bit << 16)) && clr_edge) {
        s->synctrig &= ~bit;
    }
    if ((s->synchwen & bit) && set_edge) {
        s->synctrig |= bit;
    }
    ld_kick(s);
}

static bool ld_gated(Efr32LdmaState *s, uint32_t bit)
{
    return !efr32_link_bus_clock(s->link, EFR32_LINK_CLKEN0, bit);
}

static uint32_t ld_busy_mask(Efr32LdmaState *s)
{
    uint32_t busy = 0;

    for (unsigned n = 0; n < LD_CHANNELS; n++) {
        if (s->ch[n].enabled && (s->ch[n].request_units ||
                                 ld_requesting(s, n))) {
            busy |= 1u << n;
        }
    }
    return busy;
}

static uint32_t ld_reqpend(Efr32LdmaState *s)
{
    uint32_t pend = 0;

    for (unsigned n = 0; n < LD_CHANNELS; n++) {
        if (s->ch[n].pending ||
            (!(s->reqdis & (1u << n)) && ld_line_level(s, &s->ch[n]))) {
            pend |= 1u << n;
        }
    }
    return pend;
}

static MemTxResult ld_read(void *opaque, hwaddr addr, uint64_t *data,
                           unsigned size, MemTxAttrs attrs)
{
    Efr32LdmaState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    uint32_t v = 0;

    efr32_link_bus_access(s->link, s->base + addr, size, false);
    *data = 0;
    if (ld_gated(s, LD_CLKEN0_LDMA)) {
        return MEMTX_ERROR;
    }
    if (offset >= LD_CH0 && offset < LD_CH0 + LD_CHANNELS * LD_CH_STRIDE) {
        LdChannel *c = &s->ch[(offset - LD_CH0) / LD_CH_STRIDE];

        switch ((offset - LD_CH0) % LD_CH_STRIDE) {
        case LD_CH_CFG:
            v = c->cfg;
            break;
        case LD_CH_LOOP:
            v = c->loop;
            break;
        case LD_CH_CTRL:
            v = c->ctrl;
            break;
        case LD_CH_SRC:
            v = c->src;
            break;
        case LD_CH_DST:
            v = c->dst;
            break;
        case LD_CH_LINK:
            v = c->link;
            break;
        }
        *data = v;
        return MEMTX_OK;
    }
    switch (offset) {
    case LD_IPVERSION:
        v = LD_IPVERSION_VALUE;
        break;
    case LD_EN:
        v = s->en;
        break;
    case LD_CTRL:
        v = s->ctrl;
        break;
    case LD_STATUS:
        v = (ld_busy_mask(s) ? LD_STATUS_ANYBUSY : 0) |
            (ld_reqpend(s) ? LD_STATUS_ANYREQ : 0) |
            (s->chgrant << LD_STATUS_CHGRANT_SHIFT) |
            (s->cherror << LD_STATUS_CHERROR_SHIFT) |
            LD_STATUS_FIFOLEVEL_IDLE | LD_STATUS_CHNUM;
        break;
    case LD_SYNCHWEN:
        v = s->synchwen;
        break;
    case LD_SYNCHWSEL:
        v = s->synchwsel;
        break;
    case LD_SYNCSTATUS:
        v = s->synctrig;
        break;
    case LD_CHSTATUS:
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            v |= s->ch[n].enabled ? 1u << n : 0;
        }
        break;
    case LD_CHBUSY:
        v = ld_busy_mask(s);
        break;
    case LD_CHDONE:
        v = s->chdone;
        break;
    case LD_DBGHALT:
        v = s->dbghalt;
        break;
    case LD_REQDIS:
        v = s->reqdis;
        break;
    case LD_REQPEND:
        v = ld_reqpend(s);
        break;
    case LD_IF:
        v = s->if_;
        break;
    case LD_IEN:
        v = s->ien;
        break;
    }
    *data = v;
    return MEMTX_OK;
}

static void ld_channel_write(Efr32LdmaState *s, LdChannel *c, unsigned reg,
                             hwaddr addr, uint32_t value)
{
    switch (reg) {
    case LD_CH_CFG:
        c->cfg = efr32_alias_apply(addr, c->cfg, value) & LD_CFG_MASK;
        break;
    case LD_CH_LOOP:
        c->loop = efr32_alias_apply(addr, c->loop, value) & LD_LOOP_MASK;
        break;
    case LD_CH_CTRL:
        c->ctrl = (c->ctrl & ~LD_CTRL_WRITABLE) |
                  (efr32_alias_apply(addr, c->ctrl, value) & LD_CTRL_WRITABLE);
        c->remaining = ((c->ctrl >> LD_CTRL_XFERCNT_SHIFT) &
                        LD_CTRL_XFERCNT_MASK) + 1;
        c->request_units = 0;
        break;
    case LD_CH_SRC:
        c->src = efr32_alias_apply(addr, c->src, value);
        break;
    case LD_CH_DST:
        c->dst = efr32_alias_apply(addr, c->dst, value);
        break;
    case LD_CH_LINK:
        c->link = efr32_alias_apply(addr, c->link, value);
        break;
    }
}

static MemTxResult ld_write(void *opaque, hwaddr addr, uint64_t data,
                            unsigned size, MemTxAttrs attrs)
{
    Efr32LdmaState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    unsigned alias = (addr & EFR32_ALIAS_MASK) >> EFR32_ALIAS_SHIFT;
    uint32_t value = data;
    uint32_t mask = value & LD_CH_MASK;

    efr32_link_bus_access(s->link, s->base + addr, size, true);
    if (ld_gated(s, LD_CLKEN0_LDMA)) {
        return MEMTX_ERROR;
    }
    if (offset >= LD_CH0 && offset < LD_CH0 + LD_CHANNELS * LD_CH_STRIDE) {
        ld_channel_write(s, &s->ch[(offset - LD_CH0) / LD_CH_STRIDE],
                         (offset - LD_CH0) % LD_CH_STRIDE, addr, value);
        ld_kick(s);
        return MEMTX_OK;
    }
    /* The command registers act on the bits written, whatever the alias. */
    switch (offset) {
    case LD_EN:
        s->en = efr32_alias_apply(addr, s->en, value) & 1;
        break;
    case LD_CTRL:
        s->ctrl = efr32_alias_apply(addr, s->ctrl, value) & LD_CTRL_MASK;
        break;
    case LD_SYNCSWSET:
        s->synctrig |= value & LD_SYNC_MASK;
        break;
    case LD_SYNCSWCLR:
        s->synctrig &= ~(value & LD_SYNC_MASK);
        break;
    case LD_SYNCHWEN:
        s->synchwen = efr32_alias_apply(addr, s->synchwen, value) &
                      LD_SYNCHWEN_MASK;
        break;
    case LD_SYNCHWSEL:
        s->synchwsel = efr32_alias_apply(addr, s->synchwsel, value) &
                       LD_SYNCHWEN_MASK;
        break;
    case LD_CHEN:
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            if (mask & (1u << n)) {
                s->ch[n].enabled = true;
            }
        }
        break;
    case LD_CHDIS:
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            if (mask & (1u << n)) {
                s->ch[n].enabled = false;
            }
        }
        break;
    case LD_CHDONE:
        s->chdone = efr32_alias_apply(addr, s->chdone, value) & LD_CH_MASK;
        break;
    case LD_DBGHALT:
        s->dbghalt = efr32_alias_apply(addr, s->dbghalt, value) & LD_CH_MASK;
        break;
    case LD_SWREQ:
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            if (mask & (1u << n)) {
                s->ch[n].pending = true;
            }
        }
        break;
    case LD_REQDIS:
        s->reqdis = efr32_alias_apply(addr, s->reqdis, value) & LD_CH_MASK;
        break;
    case LD_LINKLOAD:
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            if (mask & (1u << n)) {
                /* "load the next DMA structure and enable the channel" */
                s->ch[n].enabled = true;
                if (ld_load_descriptor(s, n, s->ch[n].link & LD_LINK_ADDR_MASK,
                                       false)) {
                    ld_run_control(s, n);
                }
            }
        }
        break;
    case LD_REQCLEAR:
        for (unsigned n = 0; n < LD_CHANNELS; n++) {
            if (mask & (1u << n)) {
                s->ch[n].pending = false;
                s->ch[n].request_units = 0;
            }
        }
        break;
    case LD_IF:
        s->if_ = efr32_alias_apply(addr, s->if_, value) & LD_IF_MASK;
        break;
    case LD_IEN:
        s->ien = efr32_alias_apply(addr, s->ien, value) & LD_IF_MASK;
        break;
    default:
        (void)alias;
        break;
    }
    ld_update_irq(s);
    ld_kick(s);
    return MEMTX_OK;
}

static MemTxResult ld_xbar_read(void *opaque, hwaddr addr, uint64_t *data,
                                unsigned size, MemTxAttrs attrs)
{
    Efr32LdmaState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;

    efr32_link_bus_access(s->link, EFR32_LDMAXBAR_BASE + addr, size, false);
    *data = 0;
    if (ld_gated(s, LD_CLKEN0_LDMAXBAR)) {
        return MEMTX_ERROR;
    }
    if (offset < 4 * LD_CHANNELS) {
        *data = s->ch[offset / 4].reqsel;
    }
    return MEMTX_OK;
}

static MemTxResult ld_xbar_write(void *opaque, hwaddr addr, uint64_t data,
                                 unsigned size, MemTxAttrs attrs)
{
    Efr32LdmaState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;

    efr32_link_bus_access(s->link, EFR32_LDMAXBAR_BASE + addr, size, true);
    if (ld_gated(s, LD_CLKEN0_LDMAXBAR)) {
        return MEMTX_ERROR;
    }
    if (offset < 4 * LD_CHANNELS) {
        LdChannel *c = &s->ch[offset / 4];

        c->reqsel = efr32_alias_apply(addr, c->reqsel, data) & LD_REQSEL_MASK;
        ld_kick(s);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps ld_ops = {
    .read_with_attrs = ld_read,
    .write_with_attrs = ld_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

static const MemoryRegionOps ld_xbar_ops = {
    .read_with_attrs = ld_xbar_read,
    .write_with_attrs = ld_xbar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

static void ld_reset_hold(Object *obj, ResetType type)
{
    Efr32LdmaState *s = EFR32_LDMA(obj);

    timer_del(s->timer);
    s->busy = false;
    s->en = s->synchwen = s->synchwsel = s->synctrig = s->chdone = 0;
    s->reqdis = s->if_ = s->ien = s->dbghalt = s->cherror = s->chgrant = 0;
    s->ctrl = LD_CTRL_RESET;
    s->slot_pos = 0;
    memset(s->rr_next, 0, sizeof(s->rr_next));
    memset(s->ch, 0, sizeof(s->ch));
}

static void ld_reset_exit(Object *obj, ResetType type)
{
    ld_update_irq(EFR32_LDMA(obj));
}

static void ld_init(Object *obj)
{
    Efr32LdmaState *s = EFR32_LDMA(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &ld_ops, s, TYPE_EFR32_LDMA, 0x4000);
    memory_region_init_io(&s->xbar, obj, &ld_xbar_ops, s, "efr32-ldmaxbar",
                          0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->xbar);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(dev, ld_req_in, EFR32_LDMA_REQ,
                            EFR32_LDMA_REQ_LINES);
    qdev_init_gpio_in_named(dev, ld_prs_in, EFR32_LDMA_PRS_SYNC, 8);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ld_service, s);
}

static void ld_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = ld_reset_hold;
    rc->phases.exit = ld_reset_exit;
}

static const TypeInfo ld_info = {
    .name = TYPE_EFR32_LDMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32LdmaState),
    .instance_init = ld_init,
    .class_init = ld_class_init,
};

static void ld_register_types(void)
{
    type_register_static(&ld_info);
}
type_init(ld_register_types)

DeviceState *efr32_ldma_create(Object *soc, MemoryRegion *memory,
                               const Efr32SocLink *link, hwaddr ns_offset,
                               qemu_irq irq)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_LDMA);
    Efr32LdmaState *s = EFR32_LDMA(dev);

    object_property_add_child(soc, "ldma", OBJECT(dev));
    s->link = link;
    s->base = EFR32_LDMA_BASE;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion_overlap(memory, EFR32_LDMA_BASE, &s->mmio, 1);
    memory_region_add_subregion_overlap(memory, EFR32_LDMAXBAR_BASE, &s->xbar,
                                        1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-ldma-ns",
                             &s->mmio, 0, 0x4000);
    memory_region_init_alias(&s->xbar_ns, OBJECT(dev), "efr32-ldmaxbar-ns",
                             &s->xbar, 0, 0x4000);
    memory_region_add_subregion_overlap(memory, EFR32_LDMA_BASE + ns_offset,
                                        &s->mmio_ns, 1);
    memory_region_add_subregion_overlap(memory,
                                        EFR32_LDMAXBAR_BASE + ns_offset,
                                        &s->xbar_ns, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    return dev;
}
