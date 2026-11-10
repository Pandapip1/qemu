/*
 * BCM4387 GCI core (0x840 rev 19) at 0x18010000: the always-on "global
 * coexistence interface" block that holds the chip-level control and status
 * files, the LPO selection, the OTP window and the semaphores.
 *
 * Modelled:
 *  - the indirect files: GCI+0x40 selects an index, +0x200 is chipcontrol
 *    [index] (read/write), +0x204 chipstatus[index] (read-only). [B] layout
 *    (bcmdhd sbgci.h gci_indirect_addr/gci_chipctrl/gci_chipsts).
 *  - the index write is posted through the always-on register sync shared
 *    with the PMU (bcm4387-pmu.c): a data access right after it, with no
 *    read-back, still sees the old index, and a second index write before
 *    the first has landed wedges the sync. [F] queue run 15.
 *  - chipcontrol holds, at handoff, what the previous stock run left (the
 *    block is not reset by a firmware reload; queue run 15 read stock's
 *    values at handoff). The values are stock's at its first PHY write
 *    (fw/trace/indirect_stock_run14.tsv). [F] An index stock never uses is
 *    not modelled (reads 0xdead and are logged). [U]
 *  - chipstatus[15] bit 1: the FLL 1 MHz LPO is locked
 *    (GCI_CS_4387_FLL1MHZ_LOCK_MASK). [B] Stock sets pmucontrol_ext
 *    FASTLPO_ENAB and polls this (RAM 0x82b6f8, 300 x 10 us) before setting
 *    FASTLPO_SWENAB. Other chipstatus entries read 0. [U]
 *  - +0x310 handshake: bit 12 follows the request (+0x314 bit 15, written
 *    as the inverse of what the firmware then waits for, RAM 0x76378c). [I]
 *  - +0x31c OTP layout 0xf2e0 (the only layout the firmware accepts, RAM
 *    0x8254c0/0x82563c). [I]
 *  - LPO: +0xc20 bit 3 holds the LPO in reset, bit 2 enables it, status
 *    [7:4] reads >8 while running (ROM 0x416e18); +0xc1c [21:18] is one-hot
 *    for the source selected in +0xc04 [10:9] (ROM 0x416f50). [I]
 *  - semaphores +0x38/+0x3c: writing a bit toggles it (ROM 0x4123b4 /
 *    0x41249c acquire and release by writing the same bit). [I]
 *  - CNCB LUTs: +0x484 selects the entry of the 2G (+0x488) and 5G
 *    (+0x48c) tables (gci_cncb_lut_indirect_addr). [B] layout; [U] effect.
 *  - LHL ARM timer 0 (+0xc40 intrp_adr, +0xc44 st_adr, +0xc48 armtim0_adr;
 *    bcmdhd sbgci.h / hndlhl.c si_lhl_timer_config(LHL_ARM_TIMER),
 *    si_lhl_timer_reset: count 0 then status write-1-to-clear) [B]. A
 *    non-zero write to +0xc48 starts it; it expires that many ILP cycles
 *    later, sets +0xc44 bit 0 and asserts the PMU ARM resource request
 *    (res_req_mask/timer) until the status is cleared or the count is
 *    rewritten. [I] (notes/36 §6.1: stock starts it with 0x39 at seq 63244;
 *    queue run 31 tests whether the expiry brings up XTAL_HQ). No
 *    interrupt is raised. [U] The interrupt enable bits are stored only.
 *  - LHL MAC timers 0/1/2 (+0xc70..+0xc9c, bcmdhd sbgci.h
 *    lhl_wl_mactimN_intrp_adr / _st_adr / lhl_wl_mactim_intN_adr /
 *    _fracN_adr) [B]: stock's si_lhl_timer_config sets intrp_adr bits 0/1
 *    (LHL_WL_MACTIM_INTRP_EN | _EDGE_TRIGGER) and then writes int_adr = 0,
 *    "Reset MAC Main Timer if in case it is running due to previous
 *    instance. This also resets the interrupt status" (hndlhl.c) [B]. The
 *    block is always-on: at handoff 0xc70 reads 3 and 0xc78 0, stock's
 *    values (run 30). [F] The timers do not count and never raise their
 *    status (st_adr bit 0, write 1 to clear per LHL_WL_MACTIMER_INT_ST_MASK
 *    [B]), so int_adr writes only clear it; the PMU MAC resource request
 *    does not depend on these registers in the model: run 30 had them at
 *    stock's values and nothing was granted before the MAC asked. [F]
 *    Whether they take part in the grant at all is unknown. [U]
 *  - every other register in gci_regs[] is plain storage: its effect is
 *    not known and it is listed as a gap. [U]
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/arm/bcm4387.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Bcm4387GciState, BCM4387_GCI)

#define GCI_INDIRECT    0x040
#define GCI_CHIPCTRL    0x200
#define GCI_CHIPSTS     0x204
#define GCI_NFILE       32
#define GCI_OTP         0x1000      /* OTP window, 0x1000 bytes */
#define GCI_CNCB_LUT_ADDR 0x484
#define GCI_CNCB_LUT_2G 0x488
#define GCI_CNCB_LUT_5G 0x48c

/* chipcontrol at handoff = stock's at its first PHY write (run 14) [F] */
static const struct { uint8_t idx; uint32_t v; } gci_cc_handoff[] = {
    { 0, 0x00000001 }, { 1, 0x0dd03333 }, { 2, 0x20000000 },
    { 3, 0x00000044 }, { 4, 0x0011455a }, { 8, 0x6dde8800 },
    { 13, 0x81000000 }, { 15, 0x60000000 }, { 16, 0x6a240006 },
    { 17, 0x8620c220 }, { 20, 0x94005400 }, { 23, 0x60008dff },
    { 25, 0xfff94d30 }, { 26, 0xfff94380 },
};

static const Bcm4387Reg gci_regs[] = {
    /* low byte 0x13 = core rev 19: stock caches it at si_pub+0x44 [F] */
    { 0x000, "corecaps0", 0x00000013, 0, "F low byte (si_pub cache); rest U" },
    { 0x00c, "corectrl", 0, ~0u, "U (stock sets 0x4)" },
    { 0x018, "intmask", 0, ~0u, "U (stock 0x02000000)" },
    { 0x01c, "wakemask", 0, ~0u, "U (stock 0x04000000)" },
    { 0x044, "gpioctl", 0, ~0u, "U (stock 0x89)" },
    { 0x048, "gpiostatus", 0, ~0u, "U (stock writes 0xf)" },
    { 0x058, "gpiointmask", 0, ~0u, "U" },
    { 0x05c, "gpiowakemask", 0, ~0u, "U" },
    { 0x16c, "gci_0x16c", 0, ~0u, "U (stock 0x01000000, RAM 0x48290b)" },
    { 0x25c, "reset_override", 0, ~0u, "B name; U" },
    { 0x260, "gci_0x260", 0, ~0u, "U (stock pulses 1/0 with the semaphores)" },
    { 0x314, "handshake_req", 0, ~0u, "I see +0x310" },
    { 0x318, "otp_ctrl", 0, ~0u, "U" },
    { 0x320, "otplayoutextension", 0, 0, "B name; U value (0 assumed)" },
    { 0x324, "otpcontrol1", 0, ~0u, "B name; U" },
    { 0x328, "otpprogdata", 0, ~0u, "B name; U" },
    { 0x418, "gci_0x418", 0, ~0u, "U (stock 0x101)" },
    { 0x420, "gci_0x420", 0, ~0u, "U (stock 0x16ec32)" },
    { 0x45c, "gci_0x45c", 0, ~0u, "U" },
    { 0x460, "gci_0x460", 0, ~0u, "U" },
    { 0x464, "gci_0x464", 0, ~0u, "U" },
    { 0x468, "gci_0x468", 0, ~0u, "U" },
    { 0x46c, "gci_0x46c", 0, ~0u, "U" },
    { 0x47c, "cncb_2g_reset_pulse_width", 0, ~0u, "B name; U" },
    { 0x480, "cncb_5g_reset_pulse_width", 0, ~0u, "B name; U" },
    { 0x490, "cncb_glitch_filter_width", 0, ~0u, "B name; U" },
    { 0x500, "gci_0x500", 0, 0, "U read-only, 0 assumed" },
    { 0x514, "gci_0x514", 0, 0, "U read-only, 0 assumed" },
    { 0x738, "gci_0x738", 0, ~0u, "U" },
    { 0x73c, "gci_0x73c", 0, 0, "U read-only, 0 assumed" },
    { 0x748, "gci_0x748", 0, ~0u, "U (stock 0x01dcd650)" },
    { 0x74c, "gci_0x74c", 0, ~0u, "U (stock 0xc587)" },
    /* low byte 7: stock caches it at si_pub+0x46 [F] */
    { 0xc00, "gci_0xc00", 0x00000007, 0, "F low byte (si_pub cache); rest U" },
    { 0xc04, "lpo_sel", 0, ~0u, "I see LPO" },
    /* LHL ARM timer 0, see LHL ARM timer above */
    { 0xc40, "lhl_wl_armtim0_intrp_adr", 0x3, 0x3,
      "B name/bits; I handoff 3 (stock's, persists like 0xc70)" },
    { 0xc44, "lhl_wl_armtim0_st_adr", 0, 0, "B name, W1C bit 0; I set on expiry" },
    { 0xc48, "lhl_wl_armtim0_adr", 0, ~0u, "B name; I ILP count to expiry" },
    /* LHL MAC timers 0/1/2 (main/aux/scan), see LHL MAC timers above */
    { 0xc70, "lhl_wl_mactim0_intrp_adr", 0x3, 0x3,
      "B name/bits; F handoff 3 (run 30); U effect" },
    { 0xc74, "lhl_wl_mactim0_st_adr", 0, 0, "B name; I W1C status, never set" },
    { 0xc78, "lhl_wl_mactim_int0_adr", 0, ~0u,
      "B name; F handoff 0 (run 30); I write clears status; U count" },
    { 0xc7c, "lhl_wl_mactim_frac0_adr", 0, ~0u, "B name; U" },
    { 0xc80, "lhl_wl_mactim1_intrp_adr", 0, 0x3, "B name/bits; U effect" },
    { 0xc84, "lhl_wl_mactim1_st_adr", 0, 0, "B name; I as mactim0_st" },
    { 0xc88, "lhl_wl_mactim_int1_adr", 0, ~0u, "B name; I as mactim_int0" },
    { 0xc8c, "lhl_wl_mactim_frac1_adr", 0, ~0u, "B name; U" },
    { 0xc90, "lhl_wl_mactim2_intrp_adr", 0, 0x3, "B name/bits; U effect" },
    { 0xc94, "lhl_wl_mactim2_st_adr", 0, 0, "B name; I as mactim0_st" },
    { 0xc98, "lhl_wl_mactim_int2_adr", 0, ~0u, "B name; I as mactim_int0" },
    { 0xc9c, "lhl_wl_mactim_frac2_adr", 0, ~0u, "B name; U" },
    { 0xd70, "gci_0xd70", 0, ~0u, "U (stock table RAM 0x82b3a7)" },
    { 0xd74, "gci_0xd74", 0, ~0u, "U" },
    { 0xdc4, "gci_0xdc4", 0, ~0u, "U" },
    { 0xdc8, "gci_0xdc8", 0, ~0u, "U" },
    { 0xdcc, "gci_0xdcc", 0, ~0u, "U" },
    { 0xe04, "gci_0xe04", 0, 0, "U read-only, 0 assumed" },
    { 0xe68, "gci_0xe68", 0, ~0u, "U" },
    { 0xe74, "gci_0xe74", 0, ~0u, "U" },
    { 0xe78, "gci_0xe78", 0, ~0u, "U" },
    { 0xe7c, "gci_0xe7c", 0, ~0u, "U" },
    { 0xe80, "gci_0xe80", 0, ~0u, "U" },
    { 0xe84, "gci_0xe84", 0, ~0u, "U" },
    { 0xeb8, "gci_0xeb8", 0, ~0u, "U" },
    { 0xebc, "gci_0xebc", 0, ~0u, "U" },
    { 0xec0, "gci_0xec0", 0, ~0u, "U" },
    { 0xec4, "gci_0xec4", 0, ~0u, "U" },
    { 0xec8, "gci_0xec8", 0, ~0u, "U" },
    { 0xedc, "gci_0xedc", 0, ~0u, "U" },
    { 0xee0, "gci_0xee0", 0, ~0u, "U" },
    { 0xee4, "gci_0xee4", 0, ~0u, "U" },
    { 0xee8, "gci_0xee8", 0, ~0u, "U" },
    { 0xeec, "gci_0xeec", 0, ~0u, "U" },
    { 0xef0, "gci_0xef0", 0, ~0u, "U" },
    { 0xef4, "gci_0xef4", 0, ~0u, "U" },
};
#define GCI_NREGS ARRAY_SIZE(gci_regs)

struct Bcm4387GciState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    Bcm4387PmuState *pmu;

    uint32_t reg[GCI_NREGS];
    uint32_t index;
    uint32_t chipctrl[GCI_NFILE];
    uint32_t cc_known;          /* chipcontrol entries with a known value */
    uint32_t sema_req, sema_rsv;
    uint32_t lut_addr, lut[2][1024];
    uint32_t hs_status, otp_layout, lpo_ctl, lpo_status;
    QEMUTimer *armtim;
};

static uint32_t *gci_reg(Bcm4387GciState *s, hwaddr off)
{
    int i = bcm4387_reg_index(gci_regs, GCI_NREGS, off);

    return i < 0 ? NULL : &s->reg[i];
}

static uint32_t gci_chipstatus(Bcm4387GciState *s, uint32_t idx)
{
    switch (idx) {
    case 15:
        return bcm4387_pmu_fll_locked(s->pmu) ? 0x2 : 0;
    default:
        return 0;
    }
}

static void gci_armtim_expire(void *opaque)
{
    Bcm4387GciState *s = opaque;

    *gci_reg(s, 0xc44) |= 1;
    trace_bcm4387_gci_armtim(*gci_reg(s, 0xc48), true);
    bcm4387_pmu_arm_timer_req(s->pmu, true);
}

static void gci_set_index(void *opaque, hwaddr addr, uint32_t v)
{
    Bcm4387GciState *s = opaque;

    trace_bcm4387_gci_index(s->index, v);
    s->index = v;
}

static MemTxResult gci_read(void *opaque, hwaddr off, uint64_t *data,
                            unsigned size, MemTxAttrs attrs)
{
    Bcm4387GciState *s = opaque;
    uint32_t *r, idx = s->index & (GCI_NFILE - 1);
    uint32_t v;

    switch (off) {
    case GCI_INDIRECT:
        bcm4387_pmu_sync_read(s->pmu, BCM4387_GCI_BASE + off);
        v = s->index;
        break;
    case GCI_CHIPCTRL:
        if (s->index >= GCI_NFILE || !(s->cc_known & BIT(idx))) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387-gci: unknown chipcontrol"
                          "[%u] read (0xdead) pc~0x%08x\n", s->index,
                          bcm4387_pc());
            v = 0xdead;
        } else {
            v = s->chipctrl[idx];
        }
        break;
    case GCI_CHIPSTS:
        v = gci_chipstatus(s, s->index);
        break;
    case 0x038:
        v = s->sema_req;
        break;
    case GCI_CNCB_LUT_ADDR:
        v = s->lut_addr;
        break;
    case GCI_CNCB_LUT_2G:
    case GCI_CNCB_LUT_5G:
        v = s->lut[off == GCI_CNCB_LUT_5G][s->lut_addr & 1023];
        break;
    case 0x03c:
        v = s->sema_rsv;
        break;
    case 0x310:
        v = (s->hs_status & ~BIT(12)) |
            (!(*gci_reg(s, 0x314) & BIT(15)) << 12);
        break;
    case 0x31c:
        v = (s->otp_layout & ~0xffffu) | 0xf2e0;
        break;
    case 0xc20:
        v = (s->lpo_ctl & ~0xf0u) |
            ((s->lpo_ctl & 4) && !(s->lpo_ctl & 8) ? 0xf0 : 0);
        break;
    case 0xc1c:
        v = (s->lpo_status & ~0x3c0000u) |
            BIT(18 + ((*gci_reg(s, 0xc04) >> 9) & 3));
        break;
    default:
        if (off >= GCI_OTP && off < GCI_OTP + 0x1000) {
            /* OTP contents: not read on silicon yet; blank [U] */
            v = 0;
            break;
        }
        r = gci_reg(s, off & ~3);
        if (!r || (size != 4 && !(off >= 0xc00 && off < 0xc10))) {
            return bcm4387_unknown("gci", BCM4387_GCI_BASE, off, false, data, size);
        }
        v = *r;
        break;
    }
    *data = extract32(v, (off & 3) * 8, size * 8);
    trace_bcm4387_gci_read(off, v);
    bcm4387_pmu_sync_read_done(s->pmu);
    return MEMTX_OK;
}

static MemTxResult gci_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size, MemTxAttrs attrs)
{
    Bcm4387GciState *s = opaque;
    uint32_t *r, idx = s->index & (GCI_NFILE - 1);

    trace_bcm4387_gci_write(off, val);
    switch (off) {
    case GCI_INDIRECT:
        bcm4387_pmu_sync_post(s->pmu, gci_set_index, s,
                              BCM4387_GCI_BASE + off, val);
        return MEMTX_OK;
    case GCI_CHIPCTRL:
        if (s->index < GCI_NFILE) {
            trace_bcm4387_gci_chipctrl(s->index, s->chipctrl[idx], val);
            s->chipctrl[idx] = val;
            s->cc_known |= BIT(idx);
        }
        return MEMTX_OK;
    case GCI_CHIPSTS:
        return MEMTX_OK;            /* read-only */
    case 0x038:
        s->sema_req ^= val;
        return MEMTX_OK;
    case GCI_CNCB_LUT_ADDR:
        s->lut_addr = val;
        return MEMTX_OK;
    case GCI_CNCB_LUT_2G:
    case GCI_CNCB_LUT_5G:
        s->lut[off == GCI_CNCB_LUT_5G][s->lut_addr & 1023] = val;
        return MEMTX_OK;
    case 0x03c:
        s->sema_rsv ^= val;
        return MEMTX_OK;
    case 0x310:
        s->hs_status = val;
        return MEMTX_OK;
    case 0x31c:
        s->otp_layout = val;
        return MEMTX_OK;
    case 0xc20:
        s->lpo_ctl = val;
        return MEMTX_OK;
    case 0xc1c:
        s->lpo_status = val;
        return MEMTX_OK;
    }
    r = gci_reg(s, off);
    if (!r || size != 4) {
        return bcm4387_unknown("gci", BCM4387_GCI_BASE, off, true, &val, size);
    }
    if (off == 0xc78 || off == 0xc88 || off == 0xc98) {
        *gci_reg(s, off - 4) = 0;
    }
    if (off == 0xc44) {
        *r &= ~(val & 1);
        bcm4387_pmu_arm_timer_req(s->pmu, *r & 1);
        return MEMTX_OK;
    }
    if (off == 0xc48) {
        *r = val;
        trace_bcm4387_gci_armtim(val, false);
        bcm4387_pmu_arm_timer_req(s->pmu, false);
        *gci_reg(s, 0xc44) &= ~1;
        if (val) {
            timer_mod(s->armtim, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      muldiv64(val, NANOSECONDS_PER_SECOND, BCM4387_ILP_HZ));
        } else {
            timer_del(s->armtim);
        }
        return MEMTX_OK;
    }
    *r = (*r & ~gci_regs[r - s->reg].wmask) |
         (val & gci_regs[r - s->reg].wmask);
    return MEMTX_OK;
}

static const MemoryRegionOps gci_ops = {
    .read_with_attrs = gci_read,
    .write_with_attrs = gci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void gci_reset(DeviceState *dev)
{
    Bcm4387GciState *s = BCM4387_GCI(dev);

    for (int i = 0; i < GCI_NREGS; i++) {
        s->reg[i] = gci_regs[i].reset;
    }
    memset(s->chipctrl, 0, sizeof(s->chipctrl));
    s->cc_known = 0;
    for (int i = 0; i < ARRAY_SIZE(gci_cc_handoff); i++) {
        s->chipctrl[gci_cc_handoff[i].idx] = gci_cc_handoff[i].v;
        s->cc_known |= BIT(gci_cc_handoff[i].idx);
    }
    s->index = 0;
    s->sema_req = s->sema_rsv = 0;
    s->lut_addr = 0;
    memset(s->lut, 0, sizeof(s->lut));
    s->hs_status = s->otp_layout = s->lpo_ctl = s->lpo_status = 0;
    timer_del(s->armtim);
}

static void gci_init(Object *obj)
{
    Bcm4387GciState *s = BCM4387_GCI(obj);

    s->armtim = timer_new_ns(QEMU_CLOCK_VIRTUAL, gci_armtim_expire, s);
    memory_region_init_io(&s->mmio, obj, &gci_ops, s, "bcm4387.gci",
                          2 * BCM4387_CORE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void gci_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), gci_reset);
}

static const TypeInfo gci_types[] = {
    {
        .name = TYPE_BCM4387_GCI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Bcm4387GciState),
        .instance_init = gci_init,
        .class_init = gci_class_init,
    },
};
DEFINE_TYPES(gci_types)

Bcm4387GciState *bcm4387_gci_create(hwaddr base, Bcm4387PmuState *pmu)
{
    DeviceState *dev = qdev_new(TYPE_BCM4387_GCI);
    Bcm4387GciState *s = BCM4387_GCI(dev);

    s->pmu = pmu;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return s;
}
