/*
 * EFR32xG22 PRS, Peripheral Reflex System (RM rev. 1.2 chapter 13).
 *
 * Register layout and encodings: RM SS13.4-13.5 and the public
 * efr32bg22_prs.h / efr32bg22_prs_signals.h.
 *
 * Modelled (RM SS13.3):
 *  - 12 asynchronous channels: SOURCESEL/SIGSEL select a producer level
 *    (input A; SOURCESEL 0 is a constant 0), AUXSEL any asynchronous
 *    channel's output (input B), FNSEL the two-input look-up table
 *    (output = FNSEL[2A + B], Table 13.1), and the result is XORed with
 *    SWLEVEL and SWPULSE (SS13.3.1).  Channels feed each other through
 *    AUXSEL and through the PRSL/PRS producers (SOURCESEL 0x0A/0x0B); a
 *    change is propagated until every channel is stable.
 *  - 4 synchronous channels: SOURCESEL/SIGSEL route a TIMER or IADC0
 *    producer unchanged (no logic, no software control).
 *  - ASYNC_PEEK / SYNC_PEEK read the channel outputs.
 *  - Every consumer register (PRSSEL, and SPRSSEL where the register has
 *    it: IADC0 triggers and TIMERn CC0-2) drives its consumer line.
 *  - The bus clock gate CMU_CLKEN0.PRS: reads and writes fault while it is
 *    clear (silicon: owo-open-fw docs/radio-tables.md SS7.71 "TIMER1 and
 *    PRS registers fault", SS7.73 PRS SWLEVEL writes 5/5 bus errors).  The
 *    channels themselves are combinational and keep routing.
 *
 * Timing: propagation is immediate.  Silicon: a PRS SWPULSE store and the
 * TIMER1 CMD.START it brackets act within +-1 TIMER1 cycle of each other,
 * and one store pulsing two channels is captured on the same TIMER1 tick
 * by both (SS7.73, n = 16 each).  [I] SWPULSE is one EM01GRPACLK cycle
 * wide on silicon (SS13.3.1); here it is a raise and a lower at the same
 * instant.  No consumer modelled here can tell the difference: edge
 * consumers act on the rise, and the TIMER's level sampler needs two
 * cycles (RM SS19.3.3.1), which neither width meets.
 *
 * [U] Not modelled: the EM2/EM3 availability split (synchronous channels
 * only in EM0/EM1) and channel-to-channel propagation delays.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/arm/efr32-prs.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32PrsState, EFR32_PRS)

#define PRS_IPVERSION 0x000u
#define PRS_IPVERSION_VALUE 0x1u
#define PRS_ASYNC_SWPULSE 0x008u
#define PRS_ASYNC_SWLEVEL 0x00cu
#define PRS_ASYNC_PEEK 0x010u
#define PRS_SYNC_PEEK 0x014u
#define PRS_ASYNC_CH_CTRL(n) (0x018u + 4u * (n))
#define PRS_SYNC_CH_CTRL(n) (0x048u + 4u * (n))
#define PRS_CONSUMER(n) (EFR32_PRS_CONSUMER_FIRST + 4u * (n))
#define PRS_REGISTER_END PRS_CONSUMER(EFR32_PRS_CONSUMERS)

#define PRS_ASYNC_MASK 0xfffu
#define PRS_SYNC_MASK 0xfu
#define PRS_ASYNC_CH_CTRL_RESET 0x000c0000u
#define PRS_ASYNC_CH_CTRL_MASK 0x0f0f7f07u
#define PRS_SYNC_CH_CTRL_MASK 0x00007f07u
#define PRS_CH_SIGSEL_MASK 0x7u
#define PRS_CH_SOURCESEL_SHIFT 8
#define PRS_CH_SOURCESEL_MASK 0x7fu
#define PRS_CH_FNSEL_SHIFT 16
#define PRS_CH_FNSEL_MASK 0xfu
#define PRS_CH_AUXSEL_SHIFT 24
#define PRS_CH_AUXSEL_MASK 0xfu
#define PRS_CONSUMER_PRSSEL_MASK 0xfu
#define PRS_CONSUMER_SPRSSEL_SHIFT 8
#define PRS_CONSUMER_SPRSSEL_MASK 0x3u
/* The PRSL source carries channels 0-7, PRS channels 8-11 (Table 13.5). */
#define PRS_PRS_SOURCE_FIRST_CHANNEL 8

#define PRS_CLKEN0_PRS (1u << 27)

struct Efr32PrsState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    const Efr32SocLink *link;
    hwaddr base;

    uint32_t swlevel;
    uint32_t swpulse;
    uint32_t async_ctrl[EFR32_PRS_ASYNC_CHANNELS];
    uint32_t sync_ctrl[EFR32_PRS_SYNC_CHANNELS];
    uint32_t consumer[EFR32_PRS_CONSUMERS];

    bool async_in[EFR32_PRS_ASYNC_INPUTS];
    bool sync_in[EFR32_PRS_SYNC_INPUTS];
    uint32_t async_out;
    uint32_t sync_out;
    bool consumer_level[EFR32_PRS_CONSUMERS];
    bool consumer_sync_level[EFR32_PRS_CONSUMERS];
    bool settling;

    qemu_irq consumer_line[EFR32_PRS_CONSUMERS];
    qemu_irq consumer_sync_line[EFR32_PRS_CONSUMERS];
    qemu_irq async_ch_line[EFR32_PRS_ASYNC_CHANNELS];
    qemu_irq ldma_sync_line[EFR32_PRS_LDMA_SYNC_CHANNELS];
    qemu_irq sync_ch_line[EFR32_PRS_SYNC_CHANNELS];
};

/* Consumer registers with an SPRSSEL field (efr32bg22_prs.h masks 0x30F). */
static bool prs_consumer_has_sync(unsigned n)
{
    if (n == EFR32_PRS_CON_IADC0_SCANTRIGGER ||
        n == EFR32_PRS_CON_IADC0_SINGLETRIGGER) {
        return true;
    }
    if (n >= EFR32_PRS_CON_TIMER_CC(0, 0) &&
        n < EFR32_PRS_CON_TIMER_CC(5, 0)) {
        return (n - EFR32_PRS_CON_TIMER0_CC0) % EFR32_PRS_TIMER_CONSUMERS < 3;
    }
    return false;
}

/* Words of the consumer range that the device header leaves reserved. */
static bool prs_consumer_reserved(unsigned n)
{
    uint32_t offset = PRS_CONSUMER(n);

    return offset == 0x060u || (offset >= 0x08cu && offset < 0x0c0u) ||
           offset == 0x0f4u;
}

static bool prs_async_input(Efr32PrsState *s, uint32_t ctrl, uint32_t out)
{
    unsigned source = (ctrl >> PRS_CH_SOURCESEL_SHIFT) & PRS_CH_SOURCESEL_MASK;
    unsigned signal = ctrl & PRS_CH_SIGSEL_MASK;

    switch (source) {
    case 0:
        return false;
    case EFR32_PRS_SRC_PRSL:
        return out & (1u << signal);
    case EFR32_PRS_SRC_PRS:
        return signal < EFR32_PRS_ASYNC_CHANNELS - PRS_PRS_SOURCE_FIRST_CHANNEL &&
               (out & (1u << (PRS_PRS_SOURCE_FIRST_CHANNEL + signal)));
    default:
        return source < EFR32_PRS_ASYNC_SOURCES &&
               s->async_in[EFR32_PRS_LINE(source, signal)];
    }
}

static uint32_t prs_async_evaluate(Efr32PrsState *s, uint32_t out)
{
    uint32_t next = 0;

    for (unsigned ch = 0; ch < EFR32_PRS_ASYNC_CHANNELS; ch++) {
        uint32_t ctrl = s->async_ctrl[ch];
        unsigned aux = (ctrl >> PRS_CH_AUXSEL_SHIFT) & PRS_CH_AUXSEL_MASK;
        unsigned fnsel = (ctrl >> PRS_CH_FNSEL_SHIFT) & PRS_CH_FNSEL_MASK;
        bool a = prs_async_input(s, ctrl, out);
        bool b = aux < EFR32_PRS_ASYNC_CHANNELS && (out & (1u << aux));
        bool level = (fnsel >> (2 * a + b)) & 1;

        level ^= !!((s->swlevel ^ s->swpulse) & (1u << ch));
        next |= (uint32_t)level << ch;
    }
    return next;
}

static uint32_t prs_sync_evaluate(Efr32PrsState *s)
{
    uint32_t next = 0;

    for (unsigned ch = 0; ch < EFR32_PRS_SYNC_CHANNELS; ch++) {
        uint32_t ctrl = s->sync_ctrl[ch];
        unsigned source = (ctrl >> PRS_CH_SOURCESEL_SHIFT) &
                          PRS_CH_SOURCESEL_MASK;
        unsigned signal = ctrl & PRS_CH_SIGSEL_MASK;

        if (source && source < EFR32_PRS_SYNC_SOURCES &&
            s->sync_in[EFR32_PRS_LINE(source, signal)]) {
            next |= 1u << ch;
        }
    }
    return next;
}

static void prs_drive_consumers(Efr32PrsState *s)
{
    for (unsigned n = 0; n < EFR32_PRS_CONSUMERS; n++) {
        unsigned ch = s->consumer[n] & PRS_CONSUMER_PRSSEL_MASK;
        bool level = ch < EFR32_PRS_ASYNC_CHANNELS &&
                     (s->async_out & (1u << ch));

        if (level != s->consumer_level[n]) {
            s->consumer_level[n] = level;
            qemu_set_irq(s->consumer_line[n], level);
        }
        if (prs_consumer_has_sync(n)) {
            unsigned sch = (s->consumer[n] >> PRS_CONSUMER_SPRSSEL_SHIFT) &
                           PRS_CONSUMER_SPRSSEL_MASK;

            level = s->sync_out & (1u << sch);
            if (level != s->consumer_sync_level[n]) {
                s->consumer_sync_level[n] = level;
                qemu_set_irq(s->consumer_sync_line[n], level);
            }
        }
    }
}

/*
 * Settle every channel and drive what changed.  A consumer acting on a
 * change may make its own producer change (a TIMER started by a PRS edge
 * that then overflows); that re-enters here and is settled in turn.
 */
static void prs_update(Efr32PrsState *s)
{
    uint32_t out = s->async_out, sync_out;
    unsigned pass;

    for (pass = 0; pass <= EFR32_PRS_ASYNC_CHANNELS; pass++) {
        uint32_t next = prs_async_evaluate(s, out);

        if (next == out) {
            break;
        }
        out = next;
    }
    if (pass > EFR32_PRS_ASYNC_CHANNELS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "efr32-prs: asynchronous channels oscillate (AUXSEL "
                      "loop)\n");
    }
    sync_out = prs_sync_evaluate(s);
    for (unsigned ch = 0; ch < EFR32_PRS_ASYNC_CHANNELS; ch++) {
        if ((out ^ s->async_out) & (1u << ch)) {
            trace_efr32_prs_async(ch, !!(out & (1u << ch)));
        }
    }
    {
        uint32_t changed = out ^ s->async_out;
        uint32_t sync_changed = sync_out ^ s->sync_out;

        s->async_out = out;
        s->sync_out = sync_out;
        for (unsigned ch = 0; ch < EFR32_PRS_ASYNC_CHANNELS; ch++) {
            if (changed & (1u << ch)) {
                qemu_set_irq(s->async_ch_line[ch], !!(out & (1u << ch)));
                if (ch < EFR32_PRS_LDMA_SYNC_CHANNELS) {
                    qemu_set_irq(s->ldma_sync_line[ch], !!(out & (1u << ch)));
                }
            }
        }
        for (unsigned ch = 0; ch < EFR32_PRS_SYNC_CHANNELS; ch++) {
            if (sync_changed & (1u << ch)) {
                qemu_set_irq(s->sync_ch_line[ch], !!(sync_out & (1u << ch)));
            }
        }
    }
    prs_drive_consumers(s);
}

static void prs_async_in(void *opaque, int n, int level)
{
    Efr32PrsState *s = opaque;

    if (s->async_in[n] == !!level) {
        return;
    }
    s->async_in[n] = level;
    prs_update(s);
}

static void prs_sync_in(void *opaque, int n, int level)
{
    Efr32PrsState *s = opaque;

    if (s->sync_in[n] == !!level) {
        return;
    }
    s->sync_in[n] = level;
    prs_update(s);
}

static bool prs_gated(Efr32PrsState *s)
{
    return !efr32_link_bus_clock(s->link, EFR32_LINK_CLKEN0, PRS_CLKEN0_PRS);
}

static MemTxResult prs_read(void *opaque, hwaddr addr, uint64_t *data,
                            unsigned size, MemTxAttrs attrs)
{
    Efr32PrsState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;

    efr32_link_bus_access(s->link, s->base + addr, size, false);
    *data = 0;
    if (prs_gated(s)) {
        return MEMTX_ERROR;
    }
    switch (offset) {
    case PRS_IPVERSION:
        *data = PRS_IPVERSION_VALUE;
        break;
    case PRS_ASYNC_SWPULSE:
        break;
    case PRS_ASYNC_SWLEVEL:
        *data = s->swlevel;
        break;
    case PRS_ASYNC_PEEK:
        *data = s->async_out;
        break;
    case PRS_SYNC_PEEK:
        *data = s->sync_out;
        break;
    default:
        if (offset >= PRS_ASYNC_CH_CTRL(0) &&
            offset < PRS_ASYNC_CH_CTRL(EFR32_PRS_ASYNC_CHANNELS)) {
            *data = s->async_ctrl[(offset - PRS_ASYNC_CH_CTRL(0)) / 4];
        } else if (offset >= PRS_SYNC_CH_CTRL(0) &&
                   offset < PRS_SYNC_CH_CTRL(EFR32_PRS_SYNC_CHANNELS)) {
            *data = s->sync_ctrl[(offset - PRS_SYNC_CH_CTRL(0)) / 4];
        } else if (offset >= PRS_CONSUMER(0) && offset < PRS_REGISTER_END) {
            *data = s->consumer[(offset - PRS_CONSUMER(0)) / 4];
        }
        break;
    }
    return MEMTX_OK;
}

static MemTxResult prs_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size, MemTxAttrs attrs)
{
    Efr32PrsState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    uint32_t value = data;

    efr32_link_bus_access(s->link, s->base + addr, size, true);
    if (prs_gated(s)) {
        return MEMTX_ERROR;
    }
    switch (offset) {
    case PRS_ASYNC_SWPULSE:
        /* Write-only strobes: a SET alias store pulses the same way. */
        s->swpulse = value & PRS_ASYNC_MASK;
        if (s->swpulse) {
            trace_efr32_prs_swpulse(s->swpulse);
            prs_update(s);
            s->swpulse = 0;
            prs_update(s);
        }
        return MEMTX_OK;
    case PRS_ASYNC_SWLEVEL:
        s->swlevel = efr32_alias_apply(addr, s->swlevel, value) &
                     PRS_ASYNC_MASK;
        break;
    default:
        if (offset >= PRS_ASYNC_CH_CTRL(0) &&
            offset < PRS_ASYNC_CH_CTRL(EFR32_PRS_ASYNC_CHANNELS)) {
            uint32_t *ctrl =
                &s->async_ctrl[(offset - PRS_ASYNC_CH_CTRL(0)) / 4];

            *ctrl = efr32_alias_apply(addr, *ctrl, value) &
                    PRS_ASYNC_CH_CTRL_MASK;
        } else if (offset >= PRS_SYNC_CH_CTRL(0) &&
                   offset < PRS_SYNC_CH_CTRL(EFR32_PRS_SYNC_CHANNELS)) {
            uint32_t *ctrl = &s->sync_ctrl[(offset - PRS_SYNC_CH_CTRL(0)) / 4];

            *ctrl = efr32_alias_apply(addr, *ctrl, value) &
                    PRS_SYNC_CH_CTRL_MASK;
        } else if (offset >= PRS_CONSUMER(0) && offset < PRS_REGISTER_END) {
            unsigned n = (offset - PRS_CONSUMER(0)) / 4;
            uint32_t mask = PRS_CONSUMER_PRSSEL_MASK;

            if (prs_consumer_reserved(n)) {
                return MEMTX_OK;
            }
            if (prs_consumer_has_sync(n)) {
                mask |= PRS_CONSUMER_SPRSSEL_MASK << PRS_CONSUMER_SPRSSEL_SHIFT;
            }
            s->consumer[n] = efr32_alias_apply(addr, s->consumer[n], value) &
                             mask;
        } else {
            return MEMTX_OK;
        }
        break;
    }
    prs_update(s);
    return MEMTX_OK;
}

static const MemoryRegionOps prs_ops = {
    .read_with_attrs = prs_read,
    .write_with_attrs = prs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

static void prs_reset_hold(Object *obj, ResetType type)
{
    Efr32PrsState *s = EFR32_PRS(obj);

    s->swlevel = 0;
    s->swpulse = 0;
    for (unsigned ch = 0; ch < EFR32_PRS_ASYNC_CHANNELS; ch++) {
        s->async_ctrl[ch] = PRS_ASYNC_CH_CTRL_RESET;
    }
    memset(s->sync_ctrl, 0, sizeof(s->sync_ctrl));
    memset(s->consumer, 0, sizeof(s->consumer));
}

/*
 * Producers keep their levels through a reset of the PRS registers (each
 * resets on its own), so re-settle in the exit phase, after they did.
 */
static void prs_reset_exit(Object *obj, ResetType type)
{
    prs_update(EFR32_PRS(obj));
}

static void prs_init(Object *obj)
{
    Efr32PrsState *s = EFR32_PRS(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &prs_ops, s, TYPE_EFR32_PRS,
                          0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in_named(dev, prs_async_in, EFR32_PRS_ASYNC_IN,
                            EFR32_PRS_ASYNC_INPUTS);
    qdev_init_gpio_in_named(dev, prs_sync_in, EFR32_PRS_SYNC_IN,
                            EFR32_PRS_SYNC_INPUTS);
    qdev_init_gpio_out_named(dev, s->consumer_line, EFR32_PRS_CONSUMER,
                             EFR32_PRS_CONSUMERS);
    qdev_init_gpio_out_named(dev, s->consumer_sync_line,
                             EFR32_PRS_CONSUMER_SYNC, EFR32_PRS_CONSUMERS);
    qdev_init_gpio_out_named(dev, s->async_ch_line, EFR32_PRS_ASYNC_CH,
                             EFR32_PRS_ASYNC_CHANNELS);
    qdev_init_gpio_out_named(dev, s->sync_ch_line, EFR32_PRS_SYNC_CH,
                             EFR32_PRS_SYNC_CHANNELS);
    qdev_init_gpio_out_named(dev, s->ldma_sync_line, EFR32_PRS_ASYNC_CH_LDMA,
                             EFR32_PRS_LDMA_SYNC_CHANNELS);
}

static void prs_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = prs_reset_hold;
    rc->phases.exit = prs_reset_exit;
}

static const TypeInfo prs_info = {
    .name = TYPE_EFR32_PRS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32PrsState),
    .instance_init = prs_init,
    .class_init = prs_class_init,
};

static void prs_register_types(void)
{
    type_register_static(&prs_info);
}
type_init(prs_register_types)

bool efr32_prs_selected(DeviceState *dev, unsigned source, unsigned signal)
{
    Efr32PrsState *s = EFR32_PRS(dev);

    for (unsigned ch = 0; ch < EFR32_PRS_ASYNC_CHANNELS; ch++) {
        uint32_t ctrl = s->async_ctrl[ch];

        if (((ctrl >> PRS_CH_SOURCESEL_SHIFT) & PRS_CH_SOURCESEL_MASK) ==
                source &&
            (ctrl & PRS_CH_SIGSEL_MASK) == signal) {
            return true;
        }
    }
    return false;
}

DeviceState *efr32_prs_create(Object *soc, MemoryRegion *memory,
                              const Efr32SocLink *link, hwaddr base,
                              hwaddr ns_offset)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_PRS);
    Efr32PrsState *s = EFR32_PRS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    object_property_add_child(soc, "prs", OBJECT(dev));
    s->link = link;
    s->base = base;
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion_overlap(memory, base, &s->mmio, 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-prs-ns",
                             &s->mmio, 0, 0x4000);
    memory_region_add_subregion_overlap(memory, base + ns_offset, &s->mmio_ns,
                                        1);
    return dev;
}
