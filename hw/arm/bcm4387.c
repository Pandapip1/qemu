/*
 * Broadcom BCM4387 (rev C2) Wi-Fi dongle ARM core, as used in Apple T6000
 * Macs. Reverse-engineering scaffold, not a faithful silicon model; see
 * docs/bcm4387-model-fidelity.md for what each piece is based on.
 *
 * It provides:
 *   - TCM laid out as the firmware and brcmfmac expect (RAM 0x740000-0x939000,
 *     ATCM also visible at 0),
 *   - the on-chip ROM window 0x400000-0x57ffff, from a dump file if one is
 *     given, else filled with "bx lr" (use contrib/plugins/romtrap to log and
 *     stub calls into it),
 *   - just enough of the AI backplane for early boot: ChipCommon chip ID and
 *     EROM pointer, a synthesised EROM, the ARM CR4 core's bank registers,
 *   - the host's pre-boot writes (firmware at rambase, NVRAM + random seed at
 *     the top of RAM, reset vector) and a watch on the shared-RAM word the
 *     firmware posts when it has booted, printing its console when it does,
 *   - a catch-all tracer: every other access is logged with the PC and
 *     reads back the last value written (0 if never written).
 *
 * Machine properties: firmware=, hostblob= (scripts/mkhostblob.py output),
 * rom= (optional ROM dump, up to 0x19b000 bytes), atcm-size=.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "target/arm/cpu.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/loader.h"
#include "hw/arm/machines-qom.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

#define TYPE_BCM4387_MACHINE MACHINE_TYPE_NAME("bcm4387")
OBJECT_DECLARE_SIMPLE_TYPE(BCM4387MachineState, BCM4387_MACHINE)

#define RAM_BASE        0x740000
#define RAM_SIZE        0x1f9000
#define RAM_TOP         (RAM_BASE + RAM_SIZE)

/*
 * RAM above the TCM. The firmware's ROM-patch veneers (0x750000) branch to
 * patch code in 0x939000-0xb36000, above the TCM window; the image is not
 * compressed, so the firmware is expected to populate this region itself at
 * runtime. Map it as plain RAM (separate from the TCM banks so the ramsize
 * calc at RAM 0x82c354 is unaffected) so those writes land instead of being
 * dropped by the catch-all tracer.
 */
#define SOCRAM_BASE     RAM_TOP
#define SOCRAM_SIZE     0x207000
#define ROM_BASE        0x400000
/*
 * Mask ROM 0x400000-0x59b000, as copied out in full by the open firmware at
 * reset (reads fault at 0x59b000, where the ARM default slave begins). [F]
 */
#define ROM_SIZE        0x19b000

/*
 * Scratch region below RAM for hand-modelled ROM functions (finding Z1). Not
 * real silicon: [SCRATCH_BASE] holds a bump-allocator cursor, allocations are
 * handed out from SCRATCH_BASE + 0x40 upward. [I].
 */
#define SCRATCH_BASE    0x600000
#define SCRATCH_SIZE    0x140000
#define HEAP_START      (SCRATCH_BASE + 0x40)

/* Hot ROM functions hand-modelled as native Thumb stubs patched into the ROM. */
#define ROM_MALLOC      0x47eadc

/*
 * Firmware ROM-patch table (static image data): entries of {rom_addr,
 * veneer_addr}, 8 bytes each. The firmware programs a ROM-patch redirect unit
 * so reads of each rom_addr return the 8-byte RAM veneer; it verifies this at
 * RAM 0x82c3f0. See the pre-apply in bcm4387_init(). [F]
 */
#define PATCH_TBL       0x8a0338
#define PATCH_TBL_END   0x8a1d70

#define CC_BASE         0x18000000
#define CC_RNG          (CC_BASE + 0x3c0)
#define CR4_BASE        0x18030000
#define D11_BASE(n)     (0x18031000 + (n) * 0x1000)    /* 3 slices */
#define OOBR_BASE       0x18006000
#define GCI_BASE        0x18010000
#define PMU_BASE        0x18012000
#define PMU_TIMER       (PMU_BASE + 0x614)
#define ILP_HZ          32768
#define EROM_BASE       0x18f00000
#define EROM_SIZE       0x1000

struct BCM4387MachineState {
    MachineState parent;

    char *hostblob;
    char *rom;
    char *patch;
    uint32_t atcm_size;
    bool identity;
    bool relocate;
    bool relocraw;

    ARMCPU *cpu;
    MemoryRegion atcm, atcm_alias, btcm, socram, rom_mr, erom, scratch, trace;
    GHashTable *store;
    uint32_t cr4_bankidx;
    uint32_t bank_info[8];
    int nab, nbb;
    QEMUTimer *watch;
    uint32_t shared;
    uint32_t rng_state;
    uint16_t phy_addr[3];
    GHashTable *phyregs;
};

/* ---- catch-all tracer and modelled backplane registers ------------------ */

static uint32_t current_pc(void)
{
    if (!current_cpu) {
        return 0;
    }
    return cpu_env(current_cpu)->regs[15];
}

static uint32_t chipid(void)
{
    /* id 0x4387, rev 7 ("BCM4387 r7" on the console), AXI backplane (1) */
    return 0x4387 | (7 << 16) | (0 << 20) | (2 << 24) | (1u << 28);
}

static uint32_t store_get(BCM4387MachineState *s, hwaddr a)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->store, GUINT_TO_POINTER(a)));
}

static void store_set(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    g_hash_table_insert(s->store, GUINT_TO_POINTER(a), GUINT_TO_POINTER(v));
}

static bool model_read(BCM4387MachineState *s, hwaddr a, uint32_t *v)
{
    if (a >= 0x18000000 && a < 0x18100000 && (a & 0xfff) == 0x1e0) {
        /*
         * clk_ctl_st on every core: clocks are up (silicon ChipCommon reads
         * 0x020b0040 at reset): ALP avail 0x10000, HT avail 0x20000, backplane
         * on HT 0x80000; external-resource status [26:24] follows the
         * request bits [10:8]. [F]/[I]
         */
        uint32_t r = store_get(s, a) & 0xffff;

        *v = r | 0x000b0000 | ((r & 0x700) << 16);
        return true;
    }
    switch (a) {
    case CC_BASE + 0x00:
        *v = chipid();
        return true;
    case CC_BASE + 0xfc:
        *v = EROM_BASE;
        return true;
    /*
     * Read-only identification registers, at the values the firmware cached
     * in si_pub on silicon (fw/dump/ram_identity.bin 0x823084..0x8230b4). [F]
     */
    case CC_BASE + 0x04:            /* capabilities */
        *v = 0x18400009;
        return true;
    case CC_BASE + 0x2c:            /* chipstatus */
        *v = 0x0e63be0c;
        return true;
    case CC_BASE + 0xac:            /* capabilities_ext */
        *v = 0x0002044e;
        return true;
    case PMU_BASE + 0x604:          /* pmucapabilities (PMU rev 0x27) */
        *v = 0x9c565f27;
        return true;
    /*
     * d11 PHY version (regs +0x3e0): analog rev [15:12], PHY type [11:8]
     * (0xb), PHY rev [7:0]. Silicon decodes 0xdb32 for the d11 cores at
     * 0x18031000/0x18032000 (2-core PHYs) and 0xdb34 for 0x18033000 (1-core);
     * per-slice struct at [wlc_hw+4], d11 base at [wlc_hw+0xc8], in
     * fw/dump/ram_identity.bin. [F]
     */
    case D11_BASE(0) + 0x3e0:
    case D11_BASE(1) + 0x3e0:
        *v = 0xdb32;
        return true;
    case D11_BASE(2) + 0x3e0:
        *v = 0xdb34;
        return true;
    /*
     * d11 wrapper iostatus (sflags): PHY band straps SISF_2G_PHY 0x1,
     * SISF_5G_PHY 0x2, plus SISF_FCLKA 0x4. Inferred from the board NVRAM
     * (bandcap=2 for slice 0, slice/1/bandcap=1, slice 2 default both). [I]
     */
    case D11_BASE(0) + 0x100000 + 0x500:
        *v = 0x2 | 0x4;
        return true;
    case D11_BASE(1) + 0x100000 + 0x500:
        *v = 0x1 | 0x4;
        return true;
    case D11_BASE(2) + 0x100000 + 0x500:
        *v = 0x3 | 0x4;
        return true;
    case CC_RNG + 0x24:             /* RNG_FIFO_COUNT: always full */
        *v = 0x10;
        return true;
    case CC_RNG + 0x20:             /* RNG_FIFO_DATA: deterministic xorshift */
        s->rng_state ^= s->rng_state << 13;
        s->rng_state ^= s->rng_state >> 17;
        s->rng_state ^= s->rng_state << 5;
        *v = s->rng_state;
        return true;
    case CR4_BASE + 0x04:
        /*
         * Capabilities: nab in bits[3:0], nbb in bits[7:4] (Linux brcmfmac
         * ARMCR4_CAP layout), and the total bank count in bits[11:8], which is
         * where the on-die firmware's TCM-ramsize loop (RAM 0x82c354) reads the
         * iteration count. Without bits[11:8] that loop runs zero times and
         * ramsize comes back 0, tripping the section-bound assert at 0x82c594.
         */
        *v = s->nab | (s->nbb << 4) | ((s->nab + s->nbb) << 8);
        return true;
    case CR4_BASE + 0x40:
        *v = s->cr4_bankidx;
        return true;
    case CR4_BASE + 0x44:
        *v = s->cr4_bankidx < s->nab + s->nbb ?
             s->bank_info[s->cr4_bankidx] : 0;
        return true;
    case CR4_BASE + 0x70:
        /*
         * Patch-unit command/status: the firmware writes a command here (and an
         * operand at +0x74) and polls bit31 for completion (RAM 0x82c440).
         * Report "done" so the per-patch commit doesn't burn its full retry
         * budget (~101 polls each) and fall through to an error path.
         */
        *v = 0x80000000;
        return true;
    case GCI_BASE + 0x310:
        /*
         * Handshake status: bit12 acknowledges the request in GCI+0x314 bit15
         * (written as !want; the firmware then waits for bit12 == want, RAM
         * 0x76378c). Acknowledge immediately.
         */
        *v = (store_get(s, a) & ~(1u << 12)) |
             (!(store_get(s, GCI_BASE + 0x314) & (1u << 15)) << 12);
        return true;
    case GCI_BASE + 0x31c:
        /*
         * OTP layout: size code 15 in bits[15:12] (the only one the firmware
         * accepts, RAM 0x8254c0) and the HW-region offset 0x2e0 in bits[11:0],
         * which the firmware checks against its own layout template (RAM
         * 0x82563c).
         */
        *v = (store_get(s, a) & ~0xffffu) | 0xf2e0;
        return true;
    case GCI_BASE + 0xc20: {
        /*
         * LPO control/status (ROM 0x416e18, "LPO is not available"): bit3 holds
         * the LPO in reset, bit2 enables it; status bits[7:4] read 0 while it
         * is stopped and >8 once it is running. Report stopped/running at once.
         */
        uint32_t c = store_get(s, a) & ~0xf0u;

        *v = c | ((c & 4) && !(c & 8) ? 0xf0 : 0);
        return true;
    }
    case GCI_BASE + 0xc1c:
        /*
         * LPO source status (ROM 0x416f50): bits[21:18] are one-hot for the
         * source selected in GCI+0xc04 bits[10:9]. Follow the selection.
         */
        *v = (store_get(s, a) & ~0x3c0000u) |
             (1u << (18 + ((store_get(s, GCI_BASE + 0xc04) >> 9) & 3)));
        return true;
    case PMU_TIMER:
        /* free-running ILP (32.768 kHz) tick counter; PMU waits time out on it */
        *v = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), ILP_HZ,
                      NANOSECONDS_PER_SECOND);
        return true;
    }
    return false;
}

static bool model_write(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    switch (a) {
    case GCI_BASE + 0x38:
    case GCI_BASE + 0x3c:
        /*
         * GCI semaphore request / wait registers: writing a bit toggles it
         * (acquire sets, release clears; ROM 0x4123b4 / 0x41249c).
         */
        store_set(s, a, store_get(s, a) ^ v);
        return true;
    case CR4_BASE + 0x40:
        s->cr4_bankidx = v & 0xff;
        return true;
    }
    return false;
}

/* ---- d11 PHY register file (indirect port +0x3fc addr / +0x3fe data) ----- */

#define PHY_KEY(n, r)   GUINT_TO_POINTER(((n) << 16) | (r))

/*
 * Reset values the firmware depends on, as decoded on silicon into the
 * per-slice struct at [wlc_hw+4] (fw/dump/ram_identity.bin): PHY reg 0x02
 * bits[3:0] = 1 on every slice, PHY reg 0x0b bits[2:0] = number of PHY cores
 * (2, 2, 1). Only those bit fields are known; the rest of each value is 0. [F]
 */
static void phy_init(BCM4387MachineState *s)
{
    static const uint8_t ncores[3] = { 2, 2, 1 };

    s->phyregs = g_hash_table_new(g_direct_hash, g_direct_equal);
    for (int n = 0; n < 3; n++) {
        g_hash_table_insert(s->phyregs, PHY_KEY(n, 0x02), GUINT_TO_POINTER(1));
        g_hash_table_insert(s->phyregs, PHY_KEY(n, 0x0b),
                            GUINT_TO_POINTER(ncores[n]));
    }
}

static int phy_slice(hwaddr a)
{
    for (int n = 0; n < 3; n++) {
        if (a >= D11_BASE(n) + 0x3fc && a < D11_BASE(n) + 0x400) {
            return n;
        }
    }
    return -1;
}

static uint16_t phy_get(BCM4387MachineState *s, int n)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->phyregs,
                                                PHY_KEY(n, s->phy_addr[n])));
}

static void phy_put(BCM4387MachineState *s, int n, uint16_t v)
{
    g_hash_table_insert(s->phyregs, PHY_KEY(n, s->phy_addr[n]),
                        GUINT_TO_POINTER(v));
    qemu_log_mask(LOG_UNIMP, "bcm4387: PHY%d W 0x%04x <- 0x%04x pc~0x%08x\n",
                  n, s->phy_addr[n], v, current_pc());
}

/* Returns true if the access hit a PHY port; *v receives read data. */
static bool phy_port(BCM4387MachineState *s, hwaddr a, unsigned size,
                     bool wr, uint32_t val, uint32_t *v)
{
    int n = phy_slice(a);
    hwaddr off;

    if (n < 0) {
        return false;
    }
    off = a - D11_BASE(n);
    if (wr) {
        if (off == 0x3fc) {
            s->phy_addr[n] = val;
            if (size == 4) {
                phy_put(s, n, val >> 16);
            }
        } else if (off == 0x3fe) {
            phy_put(s, n, val);
        }
        return true;
    }
    if (off == 0x3fc) {
        *v = s->phy_addr[n] | (size == 4 ? (uint32_t)phy_get(s, n) << 16 : 0);
    } else {
        *v = off == 0x3fe ? phy_get(s, n) : 0;
    }
    if (off != 0x3fc || size == 4) {
        qemu_log_mask(LOG_UNIMP, "bcm4387: PHY%d R 0x%04x -> 0x%04x pc~0x%08x\n",
                      n, s->phy_addr[n], phy_get(s, n), current_pc());
    }
    return true;
}

static uint64_t trace_read(void *opaque, hwaddr addr, unsigned size)
{
    BCM4387MachineState *s = opaque;
    uint32_t v = 0;
    bool modelled;

    if (phy_port(s, addr, size, false, 0, &v)) {
        return v;
    }
    modelled = model_read(s, addr, &v);

    if (!modelled) {
        gpointer p = g_hash_table_lookup(s->store, GUINT_TO_POINTER(addr));
        v = GPOINTER_TO_UINT(p);
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s R%u 0x%08" HWADDR_PRIx
                  " -> 0x%08x pc~0x%08x\n", modelled ? "MODEL" : "MMIO ",
                  size * 8, addr, v, current_pc());
    return v;
}

static void trace_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    BCM4387MachineState *s = opaque;
    bool modelled;

    if (phy_port(s, addr, size, true, val, NULL)) {
        return;
    }
    modelled = model_write(s, addr, val);

    if (!modelled) {
        g_hash_table_insert(s->store, GUINT_TO_POINTER(addr),
                            GUINT_TO_POINTER((uint32_t)val));
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s W%u 0x%08" HWADDR_PRIx
                  " <- 0x%08" PRIx64 " pc~0x%08x\n",
                  modelled ? "MODEL" : "MMIO ", size * 8, addr, val,
                  current_pc());
}

static const MemoryRegionOps trace_ops = {
    .read = trace_read,
    .write = trace_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- OOB router (HND_OOBR, core 0x85c) ----------------------------------- */

/*
 * Interrupt routing. hnd_oobr_get_intr_config (RAM 0x82b424) takes the source
 * core's OOB output selectors (percore[i] +0x10, one byte each: bit7 valid,
 * bits[6:0] signal id) and finds the ARM CR4 input carrying the same signal
 * (percore[cr4] +0x00); the signal id is the interrupt line. percore[i] +0x38
 * holds the output (bits[15:8]) and input (bits[23:16]) counts, +0x00 of the
 * router the core count. The lines are as registered on silicon (ISR list in
 * fw/dump/ram_identity.bin): ChipCommon 0, PCIe 2, d11 cores 5/7/9; the
 * per-core output counts filling the gaps and the identity input map are
 * inferred. Seeded into the register store, so firmware writes still apply.
 */
static void oobr_init(BCM4387MachineState *s)
{
    /* si core index, first signal id, number of outputs */
    static const struct { int idx, sig, n; } out[] = {
        { 0, 0, 2 }, { 1, 2, 3 }, { 3, 5, 2 }, { 4, 7, 2 }, { 5, 9, 2 },
    };
    const int ncores = 18, cr4 = 2, nin = 16;

    store_set(s, OOBR_BASE, ncores);
    for (int i = 0; i < ARRAY_SIZE(out); i++) {
        hwaddr pc = OOBR_BASE + 0x100 + out[i].idx * 0x80;

        for (int j = 0; j < out[i].n; j++) {
            hwaddr w = pc + 0x10 + (j & ~3);

            store_set(s, w, store_get(s, w) |
                      ((0x80u | (out[i].sig + j)) << (8 * (j & 3))));
        }
        store_set(s, pc + 0x38, out[i].n << 8);
    }
    for (int k = 0; k < nin; k++) {
        hwaddr w = OOBR_BASE + 0x100 + cr4 * 0x80 + (k & ~3);

        store_set(s, w, store_get(s, w) | ((0x80u | k) << (8 * (k & 3))));
    }
    store_set(s, OOBR_BASE + 0x100 + cr4 * 0x80 + 0x38,
              store_get(s, OOBR_BASE + 0x100 + cr4 * 0x80 + 0x38) | nin << 16);
}

/* ---- EROM --------------------------------------------------------------- */

/*
 * The chip's enumeration ROM, rebuilt from the firmware's own parse of it in a
 * hardware RAM coredump (fw/dump/ram_identity.bin: the si_info core arrays at
 * 0x8231fc.. and the AXI wrapper list at 0x8b4564). 18 cores: ChipCommon,
 * PCIe Gen2 (0x83c), ARM CR4, 3x d11 (0x812), 0x857, 0x855 x2, 0x85c, 0x841 x5,
 * GCI (0x840), PMU (0x827), the ARM default slave (0xfff) at 0x59b000, plus the
 * ARM bridge components (0x135, 0x31) whose wrappers the firmware error-logs.
 * CIA/CIB words, both slave ports, sizes and every wrapper address are as
 * recorded on silicon; master-port IDs and the bridge windows (which the
 * firmware does not keep) are placeholders. [F]
 */
static const uint32_t erom_words[] = {
    0x4bf80001, 0x45084411, 0x00000003, 0x18000005, 0x1c000135, 0x04000000,
    0x181000c5, 0x18103185, 0x4bf83c01, 0x47084411, 0x00000103, 0x18001005,
    0x18002005, 0x08000135, 0x08000000, 0x181010c5, 0x18102185, 0x4bf83e01,
    0x12084411, 0x00000203, 0x18030005, 0x00000135, 0x0059b000, 0x181300c5,
    0x1813c185, 0x4bf81201, 0x5508c431, 0x00000303, 0x00000413, 0x00000523,
    0x18031005, 0xe8800135, 0x00800000, 0x181310c5, 0x181341c5, 0x181352c5,
    0x18136185, 0x4bf81201, 0x5508c431, 0x00000603, 0x00000713, 0x00000823,
    0x18032005, 0xe8000135, 0x00800000, 0x181320c5, 0x181371c5, 0x181382c5,
    0x18139185, 0x4bf81201, 0x55084411, 0x00000903, 0x18033005, 0xe9000135,
    0x00800000, 0x181330c5, 0x1813a185, 0x4bf85701, 0x02084211, 0x00000a03,
    0x19000035, 0x01000000, 0x181080c5, 0x18109085, 0x4bf85501, 0x03000201,
    0x18004005, 0x4bf85501, 0x03000201, 0x18005005, 0x4bf85c01, 0x00000201,
    0x18006005, 0x4bf84101, 0x09000201, 0x18007005, 0x4bf84001, 0x13000201,
    0x18010015, 0x4bf82701, 0x27000201, 0x18012005, 0x4bf84101, 0x09000201,
    0x18013005, 0x4bf84101, 0x09000201, 0x18014005, 0x4bf84101, 0x09000201,
    0x1801f005, 0x4bf84101, 0x09000201, 0x18020005, 0x43b13501, 0x00080201,
    0x18200045, 0x18106085, 0x43b13501, 0x00080201, 0x18201045, 0x18107085,
    0x43b13501, 0x00080201, 0x18202045, 0x1813d085, 0x43b03101, 0x00080201,
    0x18203045, 0x1810a085, 0x43b03101, 0x00084201, 0x18204045, 0x1810b0c5,
    0x1813e085, 0x43b03101, 0x00084201, 0x18205045, 0x1810c0c5, 0x1813f085,
    0x43bfff01, 0x00080201, 0x0059b035, 0x001a5000, 0x18104085, 0x0000000f,
};

static void erom_build(BCM4387MachineState *s)
{
    uint32_t *e = memory_region_get_ram_ptr(&s->erom);

    QEMU_BUILD_BUG_ON(sizeof(erom_words) > EROM_SIZE);
    memset(e, 0, EROM_SIZE);
    for (size_t i = 0; i < ARRAY_SIZE(erom_words); i++) {
        e[i] = cpu_to_le32(erom_words[i]);
    }
}

/* ---- CR4 TCM banks -------------------------------------------------------- */

static uint32_t bank_encode(uint32_t size)
{
    if (size % (8 * KiB) == 0 && size / (8 * KiB) <= 128) {
        return size / (8 * KiB) - 1;
    }
    assert(size % KiB == 0 && size / KiB <= 128);
    return (1 << 9) | (size / KiB - 1);
}

static int banks_fill(uint32_t *info, int n, uint32_t size)
{
    while (size) {
        uint32_t chunk;

        if (size >= MiB) {
            chunk = MiB;
        } else if (size >= 8 * KiB) {
            chunk = size & ~(8 * KiB - 1);
        } else {
            chunk = size;
        }
        info[n++] = bank_encode(chunk);
        size -= chunk;
    }
    return n;
}

/* ---- host side: shared-RAM watch and console ---------------------------- */

static uint32_t ram_ld(hwaddr a)
{
    return address_space_ldl_le(&address_space_memory, a,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static void dump_console(uint32_t shared)
{
    uint32_t cons = ram_ld(shared + 20);
    uint32_t buf = ram_ld(cons + 8), size = ram_ld(cons + 12);
    uint32_t widx = ram_ld(cons + 16);
    g_autofree char *text = g_malloc0(size + 1);

    if (!size || size > 64 * KiB) {
        return;
    }
    address_space_read(&address_space_memory, buf, MEMTXATTRS_UNSPECIFIED,
                       text, size);
    fprintf(stderr, "bcm4387: console base 0x%x buf 0x%x size %u widx %u\n",
            cons, buf, size, widx);
    /* oldest data starts at the write index once the ring has wrapped */
    fwrite(text + widx, 1, strnlen(text + widx, size - widx), stderr);
    fwrite(text, 1, strnlen(text, widx), stderr);
    fputc('\n', stderr);
}

static void watch_shared(void *opaque)
{
    BCM4387MachineState *s = opaque;
    uint32_t v = ram_ld(RAM_TOP - 4);

    if (v != s->shared) {
        s->shared = v;
        fprintf(stderr, "bcm4387: shared-RAM word at 0x%x = 0x%08x\n",
                RAM_TOP - 4, v);
        if (v >= RAM_BASE && v < RAM_TOP) {
            dump_console(v);
        }
    }
    timer_mod(s->watch, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

/* ---- machine ------------------------------------------------------------ */

/*
 * A tiny Thumb bump allocator (finding Z1, [I]): r0=size in, r0=pointer out.
 * It reads/advances the cursor word at SCRATCH_BASE and never frees. Assembled
 * from mstub.s; the movw/movt immediates encode SCRATCH_BASE (0x00600000).
 */
static const uint16_t malloc_stub[] = {
    0xf240, 0x0300,   /* movw r3, #0x0000      */
    0xf2c0, 0x0360,   /* movt r3, #0x0060      */
    0x6819,           /* ldr  r1, [r3]         */
    0xf100, 0x0007,   /* add.w r0, r0, #7      */
    0xf020, 0x0007,   /* bic.w r0, r0, #7      */
    0xeb01, 0x0200,   /* add.w r2, r1, r0      */
    0x601a,           /* str  r2, [r3]         */
    0x4608,           /* mov  r0, r1           */
    0x4770,           /* bx   lr               */
};

static void install_rom_stub(BCM4387MachineState *s, uint32_t addr,
                             const uint16_t *code, size_t n)
{
    uint16_t *rom = memory_region_get_ram_ptr(&s->rom_mr);
    uint32_t off = (addr - ROM_BASE) / 2;

    assert(addr >= ROM_BASE && addr + 2 * n <= ROM_BASE + ROM_SIZE);
    for (size_t i = 0; i < n; i++) {
        rom[off + i] = cpu_to_le16(code[i]);
    }
}

static void bcm4387_reset(void *opaque)
{
    BCM4387MachineState *s = opaque;

    cpu_reset(CPU(s->cpu));
    /* brcmf_chip_cr4_set_active(): the host writes rstvec to TCM offset 0 */
    s->shared = ram_ld(RAM_TOP - 4);
    address_space_stl_le(&address_space_memory, SCRATCH_BASE, HEAP_START,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    timer_mod(s->watch, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

static void bcm4387_init(MachineState *machine)
{
    BCM4387MachineState *s = BCM4387_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    int64_t sz;

    if (!machine->firmware) {
        error_report("bcm4387: firmware=<file> (or -bios) is required");
        exit(1);
    }
    if (s->atcm_size == 0 || s->atcm_size >= RAM_SIZE ||
        s->atcm_size % KiB) {
        error_report("bcm4387: bad atcm-size 0x%x", s->atcm_size);
        exit(1);
    }

    cpuobj = object_new(machine->cpu_type);
    object_property_set_bool(cpuobj, "cfgte", true, &error_fatal);
    object_property_set_uint(cpuobj, "atcm-region", 0x1, &error_fatal);
    object_property_set_uint(cpuobj, "btcm-region",
                             (RAM_BASE + s->atcm_size) | 0x1, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    s->cpu = ARM_CPU(cpuobj);

    /* ATCM at 0, aliased at rambase; BTCM follows it up to RAM_TOP */
    memory_region_init_ram(&s->atcm, NULL, "bcm4387.atcm", s->atcm_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0, &s->atcm);
    memory_region_init_alias(&s->atcm_alias, NULL, "bcm4387.atcm-alias",
                             &s->atcm, 0, s->atcm_size);
    memory_region_add_subregion(sysmem, RAM_BASE, &s->atcm_alias);
    memory_region_init_ram(&s->btcm, NULL, "bcm4387.btcm",
                           RAM_SIZE - s->atcm_size, &error_fatal);
    memory_region_add_subregion(sysmem, RAM_BASE + s->atcm_size, &s->btcm);

    /* RAM above the TCM for the firmware's ROM-patch code (see SOCRAM_BASE).
     * socram=<file> pre-loads it from a hardware dump of 0x939000 upward (the
     * firmware would otherwise populate it from the init path we can't yet
     * reach). */
    memory_region_init_ram(&s->socram, NULL, "bcm4387.socram", SOCRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SOCRAM_BASE, &s->socram);
    if (s->patch) {
        int64_t psz = get_image_size(s->patch, &error_fatal);

        if (psz <= 0 || psz > SOCRAM_SIZE ||
            load_image_size(s->patch,
                            memory_region_get_ram_ptr(&s->socram), psz) != psz) {
            error_report("bcm4387: cannot load socram %s", s->patch);
            exit(1);
        }
        info_report("bcm4387: loaded socram %s at 0x%x (0x%" PRIx64 " bytes)",
                    s->patch, SOCRAM_BASE, psz);
    }

    s->nab = banks_fill(s->bank_info, 0, s->atcm_size);
    s->nbb = banks_fill(s->bank_info, s->nab, RAM_SIZE - s->atcm_size) -
             s->nab;

    memory_region_init_rom(&s->rom_mr, NULL, "bcm4387.rom", ROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, ROM_BASE, &s->rom_mr);
    {
        /* Default the whole ROM window to "bx lr" + the malloc stub, then
         * overlay any real ROM given via rom= on top. rom= is a ';'-separated
         * list of chunks, each "path" (loaded at ROM_BASE) or "path@0xADDR"
         * (loaded at that absolute address in the ROM window). Partial chunks
         * are fine; whatever isn't covered stays a stub. */
        uint16_t *h = memory_region_get_ram_ptr(&s->rom_mr);

        for (int i = 0; i < ROM_SIZE / 2; i++) {
            h[i] = cpu_to_le16(0x4770);     /* bx lr */
        }
        install_rom_stub(s, ROM_MALLOC, malloc_stub, ARRAY_SIZE(malloc_stub));

        if (s->rom) {
            g_auto(GStrv) chunks = g_strsplit(s->rom, ";", 0);

            for (int i = 0; chunks[i]; i++) {
                g_auto(GStrv) pa = g_strsplit(chunks[i], "@", 2);
                hwaddr base = pa[1] ? strtoull(pa[1], NULL, 0) : ROM_BASE;
                int64_t fsz = get_image_size(pa[0], NULL);

                if (!pa[0][0]) {
                    continue;
                }
                if (fsz < 0 || base < ROM_BASE ||
                    base + fsz > ROM_BASE + ROM_SIZE) {
                    error_report("bcm4387: rom chunk %s: bad path/addr/size",
                                 chunks[i]);
                    exit(1);
                }
                if (load_image_size(pa[0], (uint8_t *)h + (base - ROM_BASE),
                                    fsz) != fsz) {
                    error_report("bcm4387: rom chunk %s: load failed",
                                 chunks[i]);
                    exit(1);
                }
                info_report("bcm4387: mapped ROM chunk %s at 0x%" HWADDR_PRIx
                            " (0x%" PRIx64 " bytes)", pa[0], base, fsz);
            }
        }
    }

    memory_region_init_ram(&s->scratch, NULL, "bcm4387.scratch", SCRATCH_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SCRATCH_BASE, &s->scratch);

    memory_region_init_rom(&s->erom, NULL, "bcm4387.erom", EROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, EROM_BASE, &s->erom);
    erom_build(s);

    s->store = g_hash_table_new(g_direct_hash, g_direct_equal);
    oobr_init(s);
    phy_init(s);
    memory_region_init_io(&s->trace, NULL, &trace_ops, s, "bcm4387.trace",
                          UINT64_MAX);
    memory_region_add_subregion_overlap(sysmem, 0, &s->trace, -1000);

    /*
     * Host download (brcmf_pcie_download_fw_nvram). Read the image into a
     * buffer so we can apply identity relocation before it is installed, then
     * register it as the firmware rom blob. (load_image_targphys defers the
     * load to reset, so patching memory afterwards gets clobbered.)
     *
     * Identity relocation: force the two ASLR placement helpers (0x8a3e48
     * intra-page, 0x8a3eb8 page) to "movs r0,#0; bx lr" so every section's ASLR
     * offset is 0 and code stays at its link address — matching the hw patch.
     *
     * The relocation dispatcher at 0x8a40e4 runs the placed init function (via
     * blx) and, because the ASLR scheme expects that call not to return here,
     * tail-calls the panic at 0x8a3ea0. Under identity the placed function is at
     * its link address and returns normally, so replace that panic tail at
     * 0x8a4120 with "bx lr" to return to the dispatcher's caller (0x840f72).
     */
    {
        g_autofree char *fw = NULL;
        gsize fwlen = 0;

        if (!g_file_get_contents(machine->firmware, &fw, &fwlen, NULL) ||
            fwlen == 0 || fwlen > RAM_SIZE) {
            error_report("bcm4387: cannot read firmware %s", machine->firmware);
            exit(1);
        }
        if (s->relocraw) {
            /*
             * relocraw=on: run the real ASLR relocator with its genuine
             * intra-page placement helper (0x8a3e48, which returns the real
             * +0x400), and the genuine PAGE placement helper (0x8a3eb8) — RNG,
             * bounds checks and all, so the RNG stream (stack canary, later
             * placements) stays faithful — but force the page offset it RETURNS
             * to 0 by patching its epilogue "mov r0,r4" (0x8a3f04) to
             * "movs r0,#0".
             *
             * Why: each section descriptor is filled as [+0]=place_page() (page
             * ASLR) and [+4]=place_intra() (=+0x400); the re-linker computes a
             * branch fixup delta of [+0]-[+4]. For a branch whose target is in a
             * FIXED section (ROM at 0x4xxxxx, which does not move in our model —
             * we map it once at 0x400000), the correct delta is -0x400 (only the
             * instruction moved +0x400, the ROM target stays put), which needs
             * [+0]=0. Genuine place_page instead returns a random page offset
             * (observed -0x270be8) that lands in ROM and corrupts every
             * ROM-target branch (e.g. 0x82c430 "bl 0x47eb84" -> bl 0x20df9c,
             * which sleds into the dead gap and aborts). On real hardware the
             * host applies that page placement by actually loading the image at
             * the random page; our model keeps the image at its link base and
             * only honours the +0x400 intra-page shift, so the faithful page
             * offset here is 0.
             *
             * TODO (fidelity): boot then stops at the ROM-patch verify (RAM
             * 0x82c3f0), because the re-linker rewrites the veneers at 0x750000
             * (e.g. 5c f3 1e bf -> 5d f3 1e b9) after the load-time veneer->ROM
             * copy below has already run. Faithful fix: make ROM reads of a
             * patched address return the live RAM veneer (a dynamic TCAM
             * redirect) instead of a static copy.
             */
            static const uint8_t movs_r0_0[] = { 0x00, 0x20 };
            gsize op = 0x8a3f04 - RAM_BASE;

            if (op + 2 > fwlen) {
                error_report("bcm4387: firmware too small for relocraw patch");
                exit(1);
            }
            memcpy(fw + op, movs_r0_0, sizeof(movs_r0_0));
        } else if (s->identity && !s->relocate) {
            static const uint8_t ret0[] = { 0x00, 0x20, 0x70, 0x47 };
            static const uint8_t bxlr[] = { 0x70, 0x47, 0x00, 0xbf };
            gsize o1 = 0x8a3e48 - RAM_BASE, o2 = 0x8a3eb8 - RAM_BASE;
            gsize o3 = 0x8a4120 - RAM_BASE;

            if (o3 + 4 > fwlen) {
                error_report("bcm4387: firmware too small for identity patch");
                exit(1);
            }
            memcpy(fw + o1, ret0, 4);
            memcpy(fw + o2, ret0, 4);
            memcpy(fw + o3, bxlr, 4);
        } else if (s->relocate) {
            /*
             * relocate=on: let the real ASLR relocator run, but force a
             * deterministic, model-mapped section offset. Keep each placement
             * helper's prologue (which sets up the [ctx+0x40] magic the copy
             * loop needs) and override only its RETURN: intra-page offset 0,
             * page offset +0x108ffc (into mapped BTCM/socram). The relocator
             * then re-links per its tables by itself.
             * - 0x8a3e4e (after 0x8a3e48's prologue): "movs r0,#0; pop {r3,pc}"
             * - 0x8a3ec0 (after 0x8a3eb8's prologue): "ldr r0,[pc,#0];
             *   pop {r4,r5,r6,pc}; .word 0x108ffc"
             * STATUS: the relocator runs and re-links (the stack-canary RNG
             * 0x47eb10 is re-linked away and the canary clears), but the
             * re-link currently also moves a ROM target (0x40a5b4 -> 0x5135b0),
             * faulting. The per-region (RAM +0x108ffc, ROM 0) descriptor setup
             * still needs work.
             */
            static const uint8_t intra0[] = { 0x00, 0x20, 0x08, 0xbd };
            static const uint8_t page_off[] = { 0x00, 0x48, 0x70, 0xbd,
                                                0xfc, 0x8f, 0x10, 0x00 };
            gsize o1 = (0x8a3e48 + 6) - RAM_BASE;
            gsize o2 = (0x8a3eb8 + 8) - RAM_BASE;

            if (o2 + 8 > fwlen) {
                error_report("bcm4387: firmware too small for relocate patch");
                exit(1);
            }
            memcpy(fw + o1, intra0, sizeof(intra0));
            memcpy(fw + o2, page_off, sizeof(page_off));
        }
        /*
         * Mirror the ROM-patch redirect unit's read-side effect: copy each
         * veneer from the image into its ROM slot so reads of a patched ROM
         * address return the veneer, as the firmware's verify at RAM 0x82c3f0
         * expects. Only meaningful under identity (link addresses) with a real
         * ROM mapped.
         */
        if (s->identity && s->rom) {
            uint8_t *rom = memory_region_get_ram_ptr(&s->rom_mr);

            for (gsize t = PATCH_TBL - RAM_BASE;
                 t + 8 <= PATCH_TBL_END - RAM_BASE && t + 8 <= fwlen; t += 8) {
                uint32_t src = ldl_le_p(fw + t), dst = ldl_le_p(fw + t + 4);
                gsize so = src - ROM_BASE, vo = dst - RAM_BASE;

                if (src >= ROM_BASE && so + 8 <= ROM_SIZE &&
                    dst >= RAM_BASE && vo + 8 <= fwlen) {
                    memcpy(rom + so, fw + vo, 8);
                }
            }
        }
        rom_add_blob_fixed("bcm4387.fw", fw, fwlen, RAM_BASE);
    }

    if (s->hostblob) {
        sz = get_image_size(s->hostblob, &error_fatal);
        if (sz <= 0 || sz > RAM_SIZE / 2 ||
            load_image_targphys(s->hostblob, RAM_TOP - sz, sz,
                                &error_fatal) < 0) {
            error_report("bcm4387: cannot load hostblob %s", s->hostblob);
            exit(1);
        }
    }

    s->watch = timer_new_ms(QEMU_CLOCK_VIRTUAL, watch_shared, s);
    qemu_register_reset(bcm4387_reset, s);
}

static char *get_str(char **field)
{
    return g_strdup(*field);
}

#define STR_PROP(name, field)                                               \
    static char *get_##field(Object *o, Error **e)                          \
    {                                                                       \
        return get_str(&BCM4387_MACHINE(o)->field);                         \
    }                                                                       \
    static void set_##field(Object *o, const char *v, Error **e)            \
    {                                                                       \
        g_free(BCM4387_MACHINE(o)->field);                                  \
        BCM4387_MACHINE(o)->field = g_strdup(v);                            \
    }

STR_PROP("hostblob", hostblob)
STR_PROP("rom", rom)
STR_PROP("socram", patch)

static void get_atcm(Object *o, Visitor *v, const char *name, void *opaque,
                     Error **errp)
{
    visit_type_uint32(v, name, &BCM4387_MACHINE(o)->atcm_size, errp);
}

static void set_atcm(Object *o, Visitor *v, const char *name, void *opaque,
                     Error **errp)
{
    visit_type_uint32(v, name, &BCM4387_MACHINE(o)->atcm_size, errp);
}

static bool get_identity(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->identity;
}

static void set_identity(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->identity = v;
}

static bool get_relocate(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->relocate;
}

static void set_relocate(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->relocate = v;
}

static bool get_relocraw(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->relocraw;
}

static void set_relocraw(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->relocraw = v;
}

static void bcm4387_instance_init(Object *obj)
{
    /* CP15 at reset on silicon: ATCM region 0-8 MiB, BTCM base 0x800000 */
    BCM4387_MACHINE(obj)->atcm_size = 0x800000 - RAM_BASE;
    BCM4387_MACHINE(obj)->rng_state = 0x4387c2;
    BCM4387_MACHINE(obj)->identity = true;
}

static void bcm4387_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-r5"),
        NULL
    };

    mc->desc = "Broadcom BCM4387 Wi-Fi dongle ARM core (RE scaffold)";
    mc->init = bcm4387_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-r5");
    mc->valid_cpu_types = valid_cpu_types;

    object_class_property_add_str(oc, "hostblob", get_hostblob, set_hostblob);
    object_class_property_add_str(oc, "rom", get_rom, set_rom);
    object_class_property_add_str(oc, "socram", get_patch, set_patch);
    object_class_property_add(oc, "atcm-size", "uint32", get_atcm, set_atcm,
                              NULL, NULL);
    object_class_property_add_bool(oc, "identity", get_identity, set_identity);
    object_class_property_add_bool(oc, "relocate", get_relocate, set_relocate);
    object_class_property_add_bool(oc, "relocraw", get_relocraw, set_relocraw);
}

static const TypeInfo bcm4387_machine_types[] = {
    {
        .name = TYPE_BCM4387_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(BCM4387MachineState),
        .instance_init = bcm4387_instance_init,
        .class_init = bcm4387_class_init,
        .interfaces = arm_machine_interfaces,
    },
};

DEFINE_TYPES(bcm4387_machine_types)
