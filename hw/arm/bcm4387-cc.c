/*
 * BCM4387 ChipCommon (0x800 rev 69) at 0x18000000.
 *
 * Register layout and names: bcmdhd 101.10 sbchipc.h chipcregs_t, bit
 * names hndsoc.h CCS_* and sbchipc.h SRPWR_*. [B]
 *
 * Modelled:
 *  - identification: chipid 0x10074387, capabilities 0x18400009,
 *    chipstatus 0x0e63be0c, capabilities_ext 0x0002044e (read on silicon
 *    and cached by stock in si_pub, fw/dump/ram_identity.bin), eromptr. [F]
 *  - clk_ctl_st: the request bits [15:0] are as written; ALP_AVAIL and
 *    HT_AVAIL follow PMU resources ALP_AVAIL (19) and HT_AVAIL (28), and
 *    the backplane runs on HT while HT is up. [B] Bit 25 reads set at
 *    handoff and in every run. [F] (meaning unknown [U])
 *    Bit 24 (CCS_SECI_AVAIL) answers the SECI clock request (bit 8,
 *    CCS_SECICLKREQ) [B] once the FLL LPO runs (pmucontrol_ext
 *    FASTLPO_ENAB, FLL locked). [I] hypothesis: our firmware's 0x110
 *    request left bit 24 clear (queue run 11); stock enables the FLL LPO
 *    (RAM 0x82b6f8) before its SECI clock request (ROM 0x483964) and gets
 *    0x030b0152.
 *  - powerctl (srpwr, +0x1e8): PowerOnRequest [12:8] per domain (0 PCIE,
 *    1 ARM/BP, 2 MAC aux, 3 MAC main, 4 MAC scan), ExtPwrStatus [20:16]
 *    and BT status [21] read-only. [B] Domains 0, 1 and the BT bit read on
 *    at handoff (status 0x23). [F] A MAC domain request asks the PMU for
 *    its CORE_RDY resource (aux 18, main 22, scan 13), and the domain reads
 *    on while that resource is up. [B] names, [F] check: requesting 0x1e
 *    adds exactly CORE_RDY_* and PWRSW_* to res_state (0x1e4efbdf, run 11)
 *    and status reads 0x3f. Once the always-on synchroniser has wedged,
 *    the request reads back but is not acted on (status stays 0x23). [F]
 *    queue run 15.
 *  - SECI_status (+0x134) is write-one-to-clear and has no event source. [I]
 *  - RNG (+0x3c0): FIFO always full, data a deterministic xorshift. [I]
 *  - every other register in cc_regs[] is plain storage. [U]
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/arm/bcm4387.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Bcm4387CcState, BCM4387_CC)

#define CC_CLK_CTL_ST   0x1e0
#define CC_POWERCTL     0x1e8
#define CC_SECI_STATUS  0x134
#define CC_RNG_DATA     0x3e0
#define CC_RNG_COUNT    0x3e4

#define CCS_FORCEALP    0x00000001
#define CCS_FORCEHT     0x00000002
#define CCS_ALPAREQ     0x00000008
#define CCS_HTAREQ      0x00000010
#define CCS_SECICLKREQ  0x00000100
#define CCS_ALPAVAIL    0x00010000
#define CCS_HTAVAIL     0x00020000
#define CCS_BP_ON_ALP   0x00040000
#define CCS_BP_ON_HT    0x00080000
#define CCS_SECI_AVAIL  0x01000000
#define CCS_ERSRC1_STS  0x02000000

#define SRPWR_REQ_SHIFT     8
#define SRPWR_STATUS_SHIFT  16
#define SRPWR_DOMAINS       0x1f
#define SRPWR_ALWAYS_ON     0x23    /* PCIE, ARM/BP, BT status at handoff [F] */

#define EROM_BASE       0x18105000

static const Bcm4387Reg cc_regs[] = {
    { 0x000, "chipid", 0x10074387, 0, "F" },
    { 0x004, "capabilities", 0x18400009, 0, "F" },
    { 0x008, "corecontrol", 0, ~0u, "B name; U" },
    { 0x020, "intstatus", 0, 0, "B name; U (no interrupt source)" },
    { 0x024, "intmask", 0, ~0u, "B name; U" },
    { 0x02c, "chipstatus", 0x0e63be0c, 0, "F" },
    { 0x058, "gpiopullup", 0, ~0u, "B name; U" },
    { 0x05c, "gpiopulldown", 0, ~0u, "B name; U" },
    { 0x064, "gpioout", 0, ~0u, "B name; U" },
    { 0x068, "gpioouten", 0, ~0u, "B name; U" },
    { 0x06c, "gpiocontrol", 0, ~0u, "B name; U" },
    { 0x0ac, "capabilities_ext", 0x0002044e, 0, "F" },
    { 0x0f8, "fabid", 0, 0, "B name; U value (0 assumed)" },
    { 0x0fc, "eromptr", EROM_BASE, 0, "F" },
    { 0x130, "SECI_config", 0, ~0u, "B name; U" },
    { 0x138, "SECI_statusmask", 0, ~0u, "B name; U" },
    { 0x18c, "cc_0x18c", 0, ~0u, "U (stock writes 1, ROM 0x4833ef)" },
    { 0x1c4, "seci_uart_bauddiv", 0, ~0u, "B name; U" },
    { 0x1cc, "seci_uart_lcr", 0, ~0u, "B name; U" },
    { 0x1d0, "seci_uart_mcr", 0, ~0u, "B name; U" },
    { 0x1dc, "seci_uart_baudadj", 0, ~0u, "B name; U" },
    { 0x3c0, "rng_ctrl_0", 0, ~0u, "B name; U" },
    { 0x3c4, "rng_0x3c4", 0, ~0u, "U" },
    { 0x3c8, "rng_0x3c8", 0, ~0u, "U" },
    { 0x3cc, "rng_0x3cc", 0, ~0u, "U" },
    { 0x3d0, "rng_0x3d0", 0, ~0u, "U" },
    { 0x3d4, "rng_0x3d4", 0, ~0u, "U" },
    { 0x3d8, "rng_0x3d8", 0, ~0u, "U" },
    { 0x3dc, "rng_0x3dc", 0, ~0u, "U" },
};
#define CC_NREGS ARRAY_SIZE(cc_regs)

struct Bcm4387CcState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    Bcm4387PmuState *pmu;

    uint32_t reg[CC_NREGS];
    uint32_t clk_req;           /* clk_ctl_st [15:0] */
    uint32_t srpwr_req;         /* powerctl request domains */
    uint32_t seci_status;
    uint32_t rng;
};

static uint32_t *cc_reg(Bcm4387CcState *s, hwaddr off)
{
    int i = bcm4387_reg_index(cc_regs, CC_NREGS, off);

    return i < 0 ? NULL : &s->reg[i];
}

static const uint8_t domain_res[5] = {
    0, 0, BCM4387_RES_CORE_RDY_AUX, BCM4387_RES_CORE_RDY_MAIN,
    BCM4387_RES_CORE_RDY_SCAN,
};

uint32_t bcm4387_cc_srpwr_status(Bcm4387CcState *s)
{
    uint32_t st = SRPWR_ALWAYS_ON;

    for (int d = 2; d < 5; d++) {
        if (bcm4387_pmu_res_up(s->pmu, domain_res[d])) {
            st |= BIT(d);
        }
    }
    return st;
}

static void srpwr_request(Bcm4387CcState *s)
{
    uint32_t m = 0;

    for (int d = 2; d < 5; d++) {
        if (s->srpwr_req & BIT(d)) {
            m |= BIT(domain_res[d]);
        }
    }
    bcm4387_pmu_request(s->pmu, BCM4387_REQ_SRPWR, m);
}

static void clk_request(Bcm4387CcState *s)
{
    uint32_t m = 0;

    if (s->clk_req & (CCS_FORCEHT | CCS_HTAREQ)) {
        m |= BIT(BCM4387_RES_HT_AVAIL);
    }
    if (s->clk_req & (CCS_FORCEALP | CCS_ALPAREQ)) {
        m |= BIT(BCM4387_RES_ALP_AVAIL);
    }
    bcm4387_pmu_request(s->pmu, BCM4387_REQ_CC, m);
}

static uint32_t clk_ctl_st(Bcm4387CcState *s)
{
    uint32_t v = s->clk_req | CCS_ERSRC1_STS;
    bool ht = bcm4387_pmu_res_up(s->pmu, BCM4387_RES_HT_AVAIL);
    bool alp = bcm4387_pmu_res_up(s->pmu, BCM4387_RES_ALP_AVAIL);

    v |= (alp ? CCS_ALPAVAIL : 0) | (ht ? CCS_HTAVAIL | CCS_BP_ON_HT : 0) |
         (!ht && alp ? CCS_BP_ON_ALP : 0);
    if ((s->clk_req & CCS_SECICLKREQ) && bcm4387_pmu_fll_locked(s->pmu)) {
        v |= CCS_SECI_AVAIL;
    }
    return v;
}

static MemTxResult cc_read(void *opaque, hwaddr off, uint64_t *data,
                           unsigned size, MemTxAttrs attrs)
{
    Bcm4387CcState *s = opaque;
    uint32_t v, *r;

    if (size != 4) {
        return bcm4387_unknown("cc", BCM4387_CC_BASE, off, false, data, size);
    }
    switch (off) {
    case CC_CLK_CTL_ST:
        v = clk_ctl_st(s);
        break;
    case CC_POWERCTL:
        v = bcm4387_cc_srpwr_status(s) << SRPWR_STATUS_SHIFT |
            s->srpwr_req << SRPWR_REQ_SHIFT;
        break;
    case CC_SECI_STATUS:
        v = s->seci_status;
        break;
    case CC_RNG_COUNT:
        v = 0x10;
        break;
    case CC_RNG_DATA:
        s->rng ^= s->rng << 13;
        s->rng ^= s->rng >> 17;
        s->rng ^= s->rng << 5;
        v = s->rng;
        break;
    default:
        r = cc_reg(s, off);
        if (!r) {
            return bcm4387_unknown("cc", BCM4387_CC_BASE, off, false, data, size);
        }
        v = *r;
    }
    *data = v;
    trace_bcm4387_cc_read(off, v);
    return MEMTX_OK;
}

static MemTxResult cc_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size, MemTxAttrs attrs)
{
    Bcm4387CcState *s = opaque;
    uint32_t *r;

    if (size != 4) {
        return bcm4387_unknown("cc", BCM4387_CC_BASE, off, true, &val, size);
    }
    trace_bcm4387_cc_write(off, val);
    switch (off) {
    case CC_CLK_CTL_ST:
        s->clk_req = val & 0xffff;
        clk_request(s);
        return MEMTX_OK;
    case CC_POWERCTL:
        s->srpwr_req = (val >> SRPWR_REQ_SHIFT) & SRPWR_DOMAINS;
        trace_bcm4387_cc_srpwr(s->srpwr_req);
        if (bcm4387_pmu_sync_wedged(s->pmu)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: srpwr request 0x%x not "
                          "acted on (always-on sync wedged) pc~0x%08x\n",
                          s->srpwr_req, bcm4387_pc());
            return MEMTX_OK;
        }
        srpwr_request(s);
        return MEMTX_OK;
    case CC_SECI_STATUS:
        s->seci_status &= ~val;
        return MEMTX_OK;
    case CC_RNG_COUNT:
    case CC_RNG_DATA:
        return MEMTX_OK;
    }
    r = cc_reg(s, off);
    if (!r) {
        return bcm4387_unknown("cc", BCM4387_CC_BASE, off, true, &val, size);
    }
    *r = (*r & ~cc_regs[r - s->reg].wmask) | (val & cc_regs[r - s->reg].wmask);
    return MEMTX_OK;
}

static const MemoryRegionOps cc_ops = {
    .read_with_attrs = cc_read,
    .write_with_attrs = cc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void cc_reset(DeviceState *dev)
{
    Bcm4387CcState *s = BCM4387_CC(dev);

    for (int i = 0; i < CC_NREGS; i++) {
        s->reg[i] = cc_regs[i].reset;
    }
    s->clk_req = 0x40;          /* HQCLKREQ at handoff (0x020b0040) [F] */
    s->srpwr_req = 0;
    s->seci_status = 0;
    s->rng = 0x4387c2;
    clk_request(s);
    srpwr_request(s);
}

static void cc_init(Object *obj)
{
    Bcm4387CcState *s = BCM4387_CC(obj);

    memory_region_init_io(&s->mmio, obj, &cc_ops, s, "bcm4387.cc",
                          BCM4387_CORE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void cc_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), cc_reset);
}

static const TypeInfo cc_types[] = {
    {
        .name = TYPE_BCM4387_CC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Bcm4387CcState),
        .instance_init = cc_init,
        .class_init = cc_class_init,
    },
};
DEFINE_TYPES(cc_types)

Bcm4387CcState *bcm4387_cc_create(hwaddr base, Bcm4387PmuState *pmu)
{
    DeviceState *dev = qdev_new(TYPE_BCM4387_CC);
    Bcm4387CcState *s = BCM4387_CC(dev);

    s->pmu = pmu;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return s;
}
