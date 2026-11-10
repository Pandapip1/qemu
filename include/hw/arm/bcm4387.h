/*
 * Broadcom BCM4387 (rev C2) backplane model: what the per-core devices share.
 *
 * Evidence classes used in every register table and comment:
 *   [F] silicon fact: read on the chip (fw/dump/queued-*, phylog snapshots,
 *       stock RAM dumps), or the value stock firmware cached from the chip.
 *   [B] Broadcom/Linux semantics (bcmdhd, bcma, brcmsmac, brcmfmac).
 *   [I] inferred: the minimal behaviour the stock firmware needs, not yet
 *       confirmed on silicon.
 *   [U] unknown: stored and traced only, no effect modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_BCM4387_H
#define HW_ARM_BCM4387_H

#include "exec/hwaddr.h"
#include "exec/memattrs.h"

#define BCM4387_CC_BASE     0x18000000
#define BCM4387_PCIE_BASE   0x18001000
#define BCM4387_OOBR_BASE   0x18006000
#define BCM4387_GCI_BASE    0x18010000
#define BCM4387_PMU_BASE    0x18012000
#define BCM4387_CR4_BASE    0x18030000
#define BCM4387_D11_BASE(n) (0x18031000 + (n) * 0x1000)
#define BCM4387_WRAP(core)  ((core) + 0x100000)
#define BCM4387_CORE_SIZE   0x1000

#define BCM4387_ILP_HZ      32768

/* Unmodelled MMIO is an AXI slave error (machine property strict=on). */
extern bool bcm4387_strict;

uint32_t bcm4387_pc(void);
uint32_t bcm4387_lr(void);

/*
 * A plain register: what it holds at the host->firmware handoff, which bits
 * firmware can change, and the evidence for both. Devices keep the value in
 * their own state; the table is also the gap list (docs/bcm4387-model-
 * fidelity.md is generated from the same comments).
 */
typedef struct Bcm4387Reg {
    uint16_t off;
    const char *name;
    uint32_t reset;
    uint32_t wmask;
    const char *ev;
} Bcm4387Reg;

int bcm4387_reg_index(const Bcm4387Reg *tab, int n, hwaddr off);

/*
 * An access the device does not know: an AXI slave error under strict;
 * with strict=off (exploration) plain storage, logged as "MMIO".
 */
MemTxResult bcm4387_unknown(const char *dev, hwaddr base, hwaddr off,
                            bool write, uint64_t *data, unsigned size);

/* ---- PMU ------------------------------------------------------------------ */

#define TYPE_BCM4387_PMU "bcm4387-pmu"
typedef struct Bcm4387PmuState Bcm4387PmuState;

/*
 * Resource requesters outside the PMU's own registers. Each sets the
 * resources it asks for; the PMU brings up the dependency closure.
 */
typedef enum Bcm4387PmuReq {
    BCM4387_REQ_EXTERNAL,   /* host / BT side, seen as the handoff state */
    BCM4387_REQ_SRPWR,      /* save/restore power domains (CC +0x1e8) */
    BCM4387_REQ_CC,         /* ChipCommon clk_ctl_st requests */
    BCM4387_REQ_MAC0,       /* d11 slice n clk_ctl_st requests */
    BCM4387_REQ_MAC1,
    BCM4387_REQ_MAC2,
    BCM4387_REQ_N,
} Bcm4387PmuReq;

void bcm4387_pmu_request(Bcm4387PmuState *pmu, Bcm4387PmuReq who,
                         uint32_t mask);
uint32_t bcm4387_pmu_res_state(Bcm4387PmuState *pmu);
void bcm4387_pmu_mac_active(Bcm4387PmuState *pmu, int n, bool on);
uint32_t bcm4387_pmu_reg(Bcm4387PmuState *pmu, hwaddr off);
bool bcm4387_pmu_res_up(Bcm4387PmuState *pmu, unsigned res);
bool bcm4387_pmu_fll_locked(Bcm4387PmuState *pmu);
Bcm4387PmuState *bcm4387_pmu_create(hwaddr base);

/* Always-on write synchroniser (see bcm4387-pmu.c). */
typedef void (*Bcm4387SyncApply)(void *opaque, hwaddr addr, uint32_t val);
void bcm4387_pmu_sync_post(Bcm4387PmuState *pmu, Bcm4387SyncApply apply,
                           void *opaque, hwaddr addr, uint32_t val);
void bcm4387_pmu_sync_read(Bcm4387PmuState *pmu, hwaddr addr);
void bcm4387_pmu_sync_read_done(Bcm4387PmuState *pmu);
bool bcm4387_pmu_sync_wedged(Bcm4387PmuState *pmu);

/* 4387 resources (bcmdhd RES4387_*) the other blocks refer to [B] */
#define BCM4387_RES_CORE_RDY_SCAN   13
#define BCM4387_RES_CORE_RDY_AUX    18
#define BCM4387_RES_ALP_AVAIL       19
#define BCM4387_RES_CORE_RDY_MAIN   22
#define BCM4387_RES_MACPHY_CLK_SCAN 24
#define BCM4387_RES_HT_AVAIL        28
#define BCM4387_RES_MACPHY_CLK_AUX  29
#define BCM4387_RES_MACPHY_CLK_MAIN 30

/* ---- GCI ------------------------------------------------------------------ */

#define TYPE_BCM4387_GCI "bcm4387-gci"
typedef struct Bcm4387GciState Bcm4387GciState;
Bcm4387GciState *bcm4387_gci_create(hwaddr base, Bcm4387PmuState *pmu);

/* ---- ChipCommon ----------------------------------------------------------- */

#define TYPE_BCM4387_CC "bcm4387-cc"
typedef struct Bcm4387CcState Bcm4387CcState;

Bcm4387CcState *bcm4387_cc_create(hwaddr base, Bcm4387PmuState *pmu);
uint32_t bcm4387_cc_srpwr_status(Bcm4387CcState *cc);

/* ---- d11 template RAM + SampleCollect (bcm4387-d11sc.c) ----------------- */

typedef struct Bcm4387Sc Bcm4387Sc;
typedef struct Bcm4387ScHooks {
    void *opaque;
    bool (*phy_live)(void *opaque, int slice);
    uint16_t (*phyreg)(void *opaque, int slice, uint16_t reg);
} Bcm4387ScHooks;

void bcm4387_iq_load(const char *path, const char *fmt);
Bcm4387Sc *bcm4387_sc_new(int slice, const Bcm4387ScHooks *hooks);
void bcm4387_sc_reset(Bcm4387Sc *s);
/* true if off is a template-RAM or SampleCollect register of the slice */
bool bcm4387_sc_access(Bcm4387Sc *s, hwaddr off, unsigned size, bool wr,
                       uint64_t *val);

/* ---- d11 PSM, the microcode CPU (bcm4387-psm.c) ------------------------- */

typedef struct Bcm4387Psm Bcm4387Psm;
typedef struct Bcm4387PsmHooks {
    void *opaque;
    /* the slice's PSM gets its clock */
    bool (*clocked)(void *opaque, int slice);
    /* IHR idx (host d11+0x400+2*idx) if the machine models it */
    bool (*ihr)(void *opaque, int slice, uint16_t idx, bool wr, uint16_t *v);
    /* PSM-side PHY (port 0) / radio (port 1) access; false = result unknown */
    bool (*port)(void *opaque, int slice, int port, bool wr, uint16_t addr,
                 uint16_t *v);
} Bcm4387PsmHooks;

Bcm4387Psm *bcm4387_psm_new(int slice, const Bcm4387PsmHooks *hooks);
void bcm4387_psm_reset(Bcm4387Psm *p);
void bcm4387_psm_core_reset(Bcm4387Psm *p);
void bcm4387_psm_control(Bcm4387Psm *p, uint32_t maccontrol);
/* objaddr +0x160 / objdata +0x164, +0x166; false if not modelled */
bool bcm4387_psm_objmem(Bcm4387Psm *p, hwaddr off, unsigned size, bool wr,
                        uint64_t *val);

#endif
