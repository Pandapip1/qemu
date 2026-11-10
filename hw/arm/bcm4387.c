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
#include "qemu/bitops.h"
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
#include "system/runstate.h"

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
#define EROM_BASE       0x18105000
#define EROM_SIZE       0x1000

/*
 * Address 0 is its own ECC-protected TCM (not an alias of 0x740000): the
 * host writes only image word 0 there, and reading a never-written word
 * there (e.g. 0x1e0) data-aborts with DFSR 0x409 on silicon. Its size is not
 * known; model up to the ROM. The hole between the end of ROM and RAM
 * (0x59b000-0x740000) aborts the same way (seen at 0x59b000 and 0x5a0000). [F]
 */
#define LOWTCM_SIZE     ROM_BASE
#define HOLE_BASE       (ROM_BASE + ROM_SIZE)
#define HOLE_SIZE       (RAM_BASE - HOLE_BASE)

#define TCAM_SLOTS      1024
#define SRPWR           (CC_BASE + 0x1e8)
#define SRPWR_BASE_ON   0x23    /* power-domain status at handoff [F] */
#define PHYPLL_LOCK_NS  10000   /* PHYPLL "available" this long after request */

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
    MemoryRegion atcm, btcm, socram, rom_mr, erom, trace;
    GHashTable *store;
    uint32_t cr4_bankidx;
    uint32_t bank_info[8];
    int nab, nbb;
    QEMUTimer *watch;
    uint32_t shared;
    uint32_t rng_state;
    uint16_t phy_addr[3], radio_addr[3];
    GHashTable *phyregs;        /* values written through the ports */
    GHashTable *phyknown;       /* values known (silicon / exploration seeds) */

    bool strict;                /* unmodelled MMIO -> AXI slave error */
    MemoryRegion lowtcm, hole;
    uint8_t *lowtcm_mem;
    unsigned long *lowtcm_valid;    /* ECC: one bit per written word */
    uint32_t rstvec;                /* image word 0, written to 0 at reset */
    uint8_t *rom_pristine;
    uint32_t tcam_rom[TCAM_SLOTS];  /* programmed ROM address per slot, 0=off */
    uint32_t tcam_base, tcam_op;
    bool d11_phyflags[3];   /* left reset with PCLKE|PRST (0xc) held */
    int64_t d11_phypll_t[3];    /* when PHYPLL was requested, -1 = not */
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
    /* as read on silicon: id 0x4387, rev 7, AXI backplane (type 1) [F] */
    return 0x10074387;
}

static uint32_t store_get(BCM4387MachineState *s, hwaddr a)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->store, GUINT_TO_POINTER(a)));
}

static void store_set(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    g_hash_table_insert(s->store, GUINT_TO_POINTER(a), GUINT_TO_POINTER(v));
}

#define D11_WRAP(n)     (D11_BASE(n) + 0x100000)
#define WRAP_IOCTL      0x408
#define WRAP_IOSTATUS   0x500
#define WRAP_RESETCTRL  0x800

/* save/restore power domain of each d11 slice (srpwr bit), RAM 0x8491ae [F] */
static const uint32_t d11_domain[3] = { 0x8, 0x4, 0x10 };

static uint32_t srpwr_req(BCM4387MachineState *s)
{
    return (store_get(s, SRPWR) >> 8) & 0xff;
}

/* domains requested are on at once (requesting 0x1e gave status 0x3f) [F] */
static uint32_t srpwr_status(BCM4387MachineState *s)
{
    return SRPWR_BASE_ON | srpwr_req(s);
}

static bool d11_phypll(BCM4387MachineState *s, int n)
{
    return s->d11_phypll_t[n] >= 0 &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->d11_phypll_t[n] >=
           PHYPLL_LOCK_NS;
}

/*
 * Extra master wrappers (PHY agents) of each d11 core, from the silicon EROM:
 * core 0 0x18134000/0x18135000, core 1 0x18137000/0x18138000, core 2 none.
 */
static const hwaddr d11_agents[3][2] = {
    { 0x18134000, 0x18135000 }, { 0x18137000, 0x18138000 }, { 0, 0 },
};

/*
 * The slice's extra wrappers (PHY agents) are out of reset and clocked. ROM
 * ai_core_reset (0x400fac) resets wrapba3, then wrapba2, then the main
 * wrapper; slice 2 has no extra wrappers (treated as up). [F]
 */
static bool d11_agents_up(BCM4387MachineState *s, int n)
{
    for (int i = 0; i < 2 && d11_agents[n][i]; i++) {
        hwaddr w = d11_agents[n][i];

        if ((store_get(s, w + WRAP_RESETCTRL) & 1) ||
            !(store_get(s, w + WRAP_IOCTL) & 1)) {
            return false;
        }
    }
    return true;
}

/*
 * The PHY window (radio port, phyversion, PHY port) answers once the PHY
 * side is clocked: the core's PHYPLL is locked (queue runs 7/8, even with the
 * PHY agents still in reset), or the slice's PHY agents are up (stock reads
 * phyversion that way without a PLL request). Neither: slave error (run 6).
 * [F]/[I: the agents-only path is inferred from stock]
 */
static bool d11_phy_ready(BCM4387MachineState *s, int n)
{
    return d11_phypll(s, n) || (d11_agents[n][0] && d11_agents_up(s, n));
}

/* index of the d11 core whose register space contains a, or -1 */
static int d11_core(hwaddr a)
{
    for (int n = 0; n < 3; n++) {
        if (a >= D11_BASE(n) && a < D11_BASE(n) + 0x1000) {
            return n;
        }
    }
    return -1;
}

static bool model_read(BCM4387MachineState *s, hwaddr a, uint32_t *v)
{
    if (a >= 0x18000000 && a < 0x18100000 && (a & 0xfff) == 0x1e0) {
        /*
         * clk_ctl_st: request bits [15:0] as written; ALP/HT available and
         * the backplane on HT (0xb0000) as at handoff; resource-available
         * bits [28:24]. ChipCommon reads 0x020b0040 at handoff and 0x030b0152
         * with ERSRC request 0x100. A d11 core out of reset reads 0x150b0040;
         * requesting D11PLL (0x100) adds bit 24 and PHYPLL (0x200) adds bits
         * 25/27 once the PLL locks (stock: 0x1f0b0342). [F]
         */
        uint32_t r = store_get(s, a) & 0xffff;
        int n = d11_core(a);

        *v = r | 0x000b0000;
        if (a == CC_BASE + 0x1e0) {
            *v |= 0x02000000 | ((r & 0x700) << 16);
        } else if (n >= 0) {
            *v |= 0x15000000 | (r & 0x100 ? 0x01000000 : 0) |
                  (d11_phypll(s, n) ? 0x0a000000 : 0);
        }
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
    case SRPWR:
        /* save/restore power: request [15:8], status [23:16] [F] */
        *v = (srpwr_status(s) << 16) | (srpwr_req(s) << 8);
        return true;
    case PMU_BASE + 0x60c: {
        /*
         * res_state: 0x1e0a83df at handoff; with a d11 PHYPLL up the stock
         * firmware reads 0x5ecefbff, i.e. resources 0x40c47820 added. [F]
         */
        bool pll = d11_phypll(s, 0) || d11_phypll(s, 1) || d11_phypll(s, 2);

        *v = 0x1e0a83df | (pll ? 0x40c47820 : 0);
        return true;
    }
    case PMU_BASE + 0x610:          /* res_pending: transitions are instant */
        *v = 0;
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

static void tcam_program(BCM4387MachineState *s, uint32_t slot);

static bool model_write(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    int n = d11_core(a);

    if (n >= 0 && a == D11_BASE(n) + 0x1e0) {
        uint32_t old = store_get(s, a);

        if ((v & 0x200) && !(old & 0x200)) {
            s->d11_phypll_t[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        } else if (!(v & 0x200)) {
            s->d11_phypll_t[n] = -1;
        }
        store_set(s, a, v);
        return true;
    }
    for (n = 0; n < 3; n++) {
        if (a == D11_WRAP(n) + WRAP_RESETCTRL) {
            /* note whether the core left reset with PHY clock+reset held */
            if ((store_get(s, a) & 1) && !(v & 1)) {
                s->d11_phyflags[n] =
                    (store_get(s, D11_WRAP(n) + WRAP_IOCTL) & 0xc) == 0xc;
            }
            store_set(s, a, v);
            return true;
        }
    }
    switch (a) {
    case CR4_BASE + 0x6c:           /* TCAM replacement base */
        s->tcam_base = v;
        store_set(s, a, v);
        return true;
    case CR4_BASE + 0x74:           /* TCAM operand: rom address | enable */
        s->tcam_op = v;
        store_set(s, a, v);
        return true;
    case CR4_BASE + 0x70:           /* TCAM command: 0x20000 | slot = write */
        store_set(s, a, v);
        if (v & 0x20000) {
            tcam_program(s, v & (TCAM_SLOTS - 1));
        }
        return true;
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

/* ---- d11 PHY and radio register files (indirect ports) --------------------- */

/*
 * Each d11 slice has a PHY port (address +0x3fc, data +0x3fe; a 32-bit write
 * to +0x3fc is address | data << 16) and a radio port (address +0x3d8, data
 * +0x3da). Registers are kept per slice and port; writes are logged as
 * "PHYn"/"RADn" lines for the trace tooling.
 */
enum { PORT_PHY, PORT_RADIO };
#define PORT_KEY(port, n, r) GUINT_TO_POINTER((port) << 24 | (n) << 16 | (r))

static void port_seed(BCM4387MachineState *s, int port, int n, uint16_t r,
                      uint16_t v)
{
    g_hash_table_insert(s->phyknown, PORT_KEY(port, n, r), GUINT_TO_POINTER(v));
}

/*
 * Register values known from silicon: radio 0 and 1 read 0x000c each (radio
 * ID 0x000c000c, slice 0, runs 7/8; assumed the same on the other slices).
 * PHY register contents with the PHY agents up are not known yet, so in
 * strict mode every such read is logged as unmodelled and returns
 * PHY_UNKNOWN. With strict=off (stock exploration) PHY registers read back
 * what was written, seeded with the two fields the stock firmware decodes on
 * silicon (PHY 0x02 bits[3:0] = 1, PHY 0x0b bits[2:0] = PHY cores 2/2/1,
 * from the per-slice struct at [wlc_hw+4] in fw/dump/ram_identity.bin; the
 * rest of those values is unknown). [F]/[I]
 */
#define PHY_UNKNOWN     0xdead
#define PHY_IN_RESET    0xce00  /* PHY data read with the agents in reset [F] */

static void phy_init(BCM4387MachineState *s)
{
    static const uint8_t ncores[3] = { 2, 2, 1 };

    s->phyregs = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->phyknown = g_hash_table_new(g_direct_hash, g_direct_equal);
    for (int n = 0; n < 3; n++) {
        port_seed(s, PORT_RADIO, n, 0, 0x000c);
        port_seed(s, PORT_RADIO, n, 1, 0x000c);
        if (!s->strict) {
            port_seed(s, PORT_PHY, n, 0x02, 1);
            port_seed(s, PORT_PHY, n, 0x0b, ncores[n]);
        }
    }
}

static void port_put(BCM4387MachineState *s, int port, int n, uint16_t v)
{
    uint16_t a = port == PORT_PHY ? s->phy_addr[n] : s->radio_addr[n];

    g_hash_table_insert(s->phyregs, PORT_KEY(port, n, a), GUINT_TO_POINTER(v));
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s%d W 0x%04x <- 0x%04x pc~0x%08x\n",
                  port == PORT_PHY ? "PHY" : "RAD", n, a, v, current_pc());
}

/*
 * A data read through a port. PHY reads while the slice's PHY agents are in
 * reset return 0xce00 without faulting (queue run 8: regs 0x000-0x00f,
 * 0x400-0x40f, 0x480-0x48f, 0x720-0x72f, PLL locked, main wrapper only).
 */
static uint16_t port_read(BCM4387MachineState *s, int port, int n)
{
    uint16_t a = port == PORT_PHY ? s->phy_addr[n] : s->radio_addr[n];
    gpointer key = PORT_KEY(port, n, a);
    uint16_t v;

    if (port == PORT_PHY && !d11_agents_up(s, n)) {
        v = PHY_IN_RESET;
    } else if (g_hash_table_contains(s->phyknown, key) &&
               (s->strict || !g_hash_table_contains(s->phyregs, key))) {
        v = GPOINTER_TO_UINT(g_hash_table_lookup(s->phyknown, key));
    } else if (s->strict) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: unmodelled %s%d reg 0x%04x "
                      "read (returns 0x%04x) pc~0x%08x\n",
                      port == PORT_PHY ? "PHY" : "RAD", n, a, PHY_UNKNOWN,
                      current_pc());
        v = PHY_UNKNOWN;
    } else {
        v = GPOINTER_TO_UINT(g_hash_table_lookup(s->phyregs, key));
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s%d R 0x%04x -> 0x%04x pc~0x%08x\n",
                  port == PORT_PHY ? "PHY" : "RAD", n, a, v, current_pc());
    return v;
}

/* Returns true if the access hit a PHY or radio port; *v receives read data. */
static bool phy_port(BCM4387MachineState *s, hwaddr a, unsigned size,
                     bool wr, uint32_t val, uint32_t *v)
{
    int n = d11_core(a), port;
    hwaddr off;
    uint16_t *areg;

    if (n < 0) {
        return false;
    }
    off = a - D11_BASE(n);
    if (off == 0x3fc || off == 0x3fe) {
        port = PORT_PHY;
        areg = &s->phy_addr[n];
    } else if (off == 0x3d8 || off == 0x3da) {
        port = PORT_RADIO;
        areg = &s->radio_addr[n];
    } else {
        return false;
    }
    if (wr) {
        if (off == 0x3fc || off == 0x3d8) {
            *areg = val;
            if (size == 4) {
                port_put(s, port, n, val >> 16);
            }
        } else {
            port_put(s, port, n, val);
        }
        return true;
    }
    if (off == 0x3fc || off == 0x3d8) {
        *v = *areg;
        if (size == 4) {
            *v |= (uint32_t)port_read(s, port, n) << 16;
        }
    } else {
        *v = port_read(s, port, n);
    }
    return true;
}

/* ---- ROM-patch (TCAM) unit ----------------------------------------------- */

/*
 * The firmware programs slots through CR4+0x74 (operand: ROM address | 1 to
 * enable, 0 to clear) and CR4+0x70 (0x20000 | slot); CR4+0x6c holds the base
 * of the 8-byte replacement words (0x750000 in stock). Reads of a patched
 * 8-byte ROM slot then return base + 8 * slot. Model it by copying those RAM
 * bytes into the ROM when the slot is programmed (restoring the pristine
 * bytes when it is cleared), so a moved table or a bad slot shows up exactly
 * as it would on silicon. Later changes to the RAM copy are not tracked. [F]
 */
static void tcam_program(BCM4387MachineState *s, uint32_t slot)
{
    uint32_t base = s->tcam_base ? s->tcam_base : 0x750000;
    uint32_t old = s->tcam_rom[slot], rom = s->tcam_op & ~7u;
    uint8_t buf[8];

    if (old) {
        address_space_write_rom(&address_space_memory, old,
                                MEMTXATTRS_UNSPECIFIED,
                                s->rom_pristine + (old - ROM_BASE), 8);
        s->tcam_rom[slot] = 0;
    }
    if (!(s->tcam_op & 1)) {
        return;
    }
    if (rom < ROM_BASE || rom + 8 > ROM_BASE + ROM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: TCAM slot %u: ROM address "
                      "0x%08x outside the ROM, ignored\n", slot, rom);
        return;
    }
    address_space_read(&address_space_memory, base + 8 * slot,
                       MEMTXATTRS_UNSPECIFIED, buf, 8);
    address_space_write_rom(&address_space_memory, rom, MEMTXATTRS_UNSPECIFIED,
                            buf, 8);
    s->tcam_rom[slot] = rom;
}

/* ---- address 0 (ECC TCM) and the unbacked hole above the ROM -------------- */

static MemTxResult lowtcm_read(void *opaque, hwaddr addr, uint64_t *data,
                               unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;

    for (hwaddr w = addr >> 2; w <= (addr + size - 1) >> 2; w++) {
        if (!test_bit(w, s->lowtcm_valid)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: ECC abort: read of "
                          "never-written TCM word 0x%08" HWADDR_PRIx
                          " pc~0x%08x\n", w << 2, current_pc());
            return MEMTX_ACCESS_ERROR;
        }
    }
    *data = ldn_le_p(s->lowtcm_mem + addr, size);
    return MEMTX_OK;
}

static MemTxResult lowtcm_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;

    stn_le_p(s->lowtcm_mem + addr, size, data);
    for (hwaddr w = addr >> 2; w <= (addr + size - 1) >> 2; w++) {
        set_bit(w, s->lowtcm_valid);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps lowtcm_ops = {
    .read_with_attrs = lowtcm_read,
    .write_with_attrs = lowtcm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static MemTxResult hole_read(void *opaque, hwaddr addr, uint64_t *data,
                             unsigned size, MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: ECC abort: read of unbacked "
                  "0x%08" HWADDR_PRIx " pc~0x%08x\n", HOLE_BASE + addr,
                  current_pc());
    return MEMTX_ACCESS_ERROR;
}

/* writes to the hole: not observed on silicon; dropped. [I] */
static MemTxResult hole_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size, MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: write to unbacked 0x%08"
                  HWADDR_PRIx " dropped pc~0x%08x\n", HOLE_BASE + addr,
                  current_pc());
    return MEMTX_OK;
}

static const MemoryRegionOps hole_ops = {
    .read_with_attrs = hole_read,
    .write_with_attrs = hole_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- bus rules: d11 access, strict whitelist --------------------------------- */

/*
 * Silicon behaviour of the d11 cores (fw/open queue runs 3-6, notes/50-openfw.md):
 * - power domain off (srpwr) or core in reset / clock off: AXI slave error.
 * - core left reset without PHY clock+reset held (ioctl & 0xc) and later
 *   given bandwidth/0x100 cflags: register reads get no response (stall).
 * - PHY window (radio port 0x3d8/0x3da, phyversion 0x3e0, PHY port
 *   0x3fc/0x3fe): slave error until the PHY side is clocked
 *   (d11_phy_ready). [F]/[I]
 */
static MemTxResult d11_gate(BCM4387MachineState *s, hwaddr a)
{
    int n = d11_core(a);
    uint32_t ioctl;

    if (n < 0) {
        return MEMTX_OK;
    }
    ioctl = store_get(s, D11_WRAP(n) + WRAP_IOCTL);
    if (!(srpwr_status(s) & d11_domain[n])) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d "
                      "power domain off, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, current_pc());
        return MEMTX_ERROR;
    }
    if ((store_get(s, D11_WRAP(n) + WRAP_RESETCTRL) & 1) || !(ioctl & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d in "
                      "reset or unclocked, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, current_pc());
        return MEMTX_ERROR;
    }
    if (!s->d11_phyflags[n] && (ioctl & 0x1c0)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: STALL: d11 %d left reset "
                      "without PHY flags, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, current_pc());
        qemu_system_vmstop_request_prepare();
        qemu_system_vmstop_request(RUN_STATE_PAUSED);
        cpu_exit(current_cpu);
        return MEMTX_OK;
    }
    if (a - D11_BASE(n) >= 0x3d8 && !d11_phy_ready(s, n)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d PHY "
                      "window without PHYPLL, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, current_pc());
        return MEMTX_ERROR;
    }
    return MEMTX_OK;
}

/*
 * Registers the model knows (modelled or plain storage with known
 * semantics). In strict mode every other access is an AXI slave error, so
 * firmware that touches something we have not characterised fails visibly.
 */
static bool reg_known(hwaddr a)
{
    static const struct { hwaddr base, len; } ranges[] = {
        { CC_BASE + 0x000, 0x08 },      /* chipid, capabilities */
        { CC_BASE + 0x02c, 0x04 },      /* chipstatus */
        { CC_BASE + 0x0ac, 0x04 },      /* capabilities_ext */
        { CC_BASE + 0x0fc, 0x04 },      /* EROM pointer */
        { CC_BASE + 0x1e0, 0x04 },      /* clk_ctl_st */
        { SRPWR, 0x04 },
        { CC_RNG, 0x28 },
        { CR4_BASE + 0x04, 0x04 }, { CR4_BASE + 0x40, 0x08 },
        { CR4_BASE + 0x68, 0x10 },      /* TCAM */
        { PMU_BASE + 0x600, 0x20 },     /* ctl, caps, status, res_* , timer */
        { GCI_BASE + 0x038, 0x08 },     /* semaphores */
        { GCI_BASE + 0x310, 0x1c },     /* handshake, OTP control/status */
        { GCI_BASE + 0xc04, 0x04 }, { GCI_BASE + 0xc1c, 0x08 },  /* LPO */
        { OOBR_BASE, 0xa00 },
    };

    for (int i = 0; i < ARRAY_SIZE(ranges); i++) {
        if (a >= ranges[i].base && a < ranges[i].base + ranges[i].len) {
            return true;
        }
    }
    if (a >= 0x18100000 && a < 0x18200000) {    /* AI wrappers */
        hwaddr o = a & 0xfff;

        return o == WRAP_IOCTL || o == WRAP_IOSTATUS || o == WRAP_RESETCTRL ||
               o == WRAP_RESETCTRL + 4 || (o >= 0x900 && o < 0x940);
    }
    if (d11_core(a) >= 0) {
        hwaddr o = a - D11_BASE(d11_core(a));

        return o == 0x1e0 || o >= 0x3d8;
    }
    return false;
}

static MemTxResult trace_read(void *opaque, hwaddr addr, uint64_t *data,
                              unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;
    uint32_t v = 0;
    bool modelled;
    MemTxResult r = d11_gate(s, addr);

    if (r != MEMTX_OK) {
        return r;
    }
    if (phy_port(s, addr, size, false, 0, &v)) {
        *data = v;
        return MEMTX_OK;
    }
    modelled = model_read(s, addr, &v);
    if (!modelled) {
        if (s->strict && !reg_known(addr)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: "
                          "unmodelled R%u 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                          size * 8, addr, current_pc());
            return MEMTX_ERROR;
        }
        v = store_get(s, addr);
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s R%u 0x%08" HWADDR_PRIx
                  " -> 0x%08x pc~0x%08x\n", modelled ? "MODEL" : "MMIO ",
                  size * 8, addr, v, current_pc());
    *data = v;
    return MEMTX_OK;
}

static MemTxResult trace_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;
    bool modelled;
    MemTxResult r = d11_gate(s, addr);

    if (r != MEMTX_OK) {
        return r;
    }
    if (phy_port(s, addr, size, true, val, NULL)) {
        return MEMTX_OK;
    }
    if (s->strict && !reg_known(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: unmodelled "
                      "W%u 0x%08" HWADDR_PRIx " <- 0x%08" PRIx64
                      " pc~0x%08x\n", size * 8, addr, val, current_pc());
        return MEMTX_ERROR;
    }
    modelled = model_write(s, addr, val);
    if (!modelled) {
        store_set(s, addr, val);
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s W%u 0x%08" HWADDR_PRIx
                  " <- 0x%08" PRIx64 " pc~0x%08x\n",
                  modelled ? "MODEL" : "MMIO ", size * 8, addr, val,
                  current_pc());
    return MEMTX_OK;
}

static const MemoryRegionOps trace_ops = {
    .read_with_attrs = trace_read,
    .write_with_attrs = trace_write,
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
 * The chip's enumeration ROM as read on silicon by the open firmware
 * (fw/dump/erom_silicon.bin, 190 words at 0x18105000 = ChipCommon+0xfc). It
 * includes 64-bit address descriptors (type 0xD) that a reconstruction from
 * the firmware's parsed core table lacked. [F]
 */
static const uint32_t erom_words[] = {
    0x4bf80001, 0x45084411, 0x00000003, 0x18000005, 0x1c000135, 0x04000000,
    0x181000c5, 0x18103185, 0x4bf83c01, 0x47084411, 0x00000103, 0x18001005,
    0x18002005, 0x18003005, 0x08000135, 0x08000000, 0x0000013d, 0x80000000,
    0x00000008, 0x80000000, 0x181010c5, 0x18102185, 0x4bf83e01, 0x12084411,
    0x00000203, 0x18030005, 0x18034025, 0x00000135, 0x0059b000, 0x00740135,
    0x001f9000, 0x181300c5, 0x1813c185, 0x4bf81201, 0x5508c431, 0x00000303,
    0x00000413, 0x00000523, 0x18031005, 0xe8800135, 0x00800000, 0x181310c5,
    0x181341c5, 0x181352c5, 0x18136185, 0x4bf81201, 0x5508c431, 0x00000603,
    0x00000713, 0x00000823, 0x18032005, 0xe8000135, 0x00800000, 0x181320c5,
    0x181371c5, 0x181382c5, 0x18139185, 0x4bf81201, 0x55084411, 0x00000903,
    0x18033005, 0xe9000135, 0x00800000, 0x181330c5, 0x1813a185, 0x4bf85701,
    0x02084211, 0x00000a03, 0x19000035, 0x01000000, 0x181080c5, 0x18109085,
    0x4bf85501, 0x03000201, 0x18004005, 0x4bf85501, 0x03000201, 0x18005005,
    0x4bf85c01, 0x00000201, 0x18006005, 0x4bf84101, 0x09000201, 0x18007005,
    0x4bf84001, 0x13000201, 0x18010015, 0x4bf82701, 0x27000201, 0x18012005,
    0x4bf84101, 0x09000201, 0x18013005, 0x4bf84101, 0x09000201, 0x18014005,
    0x4bf84a01, 0x0c000201, 0x18015005, 0x18016005, 0x18017005, 0x1801c005,
    0x4bf84a01, 0x0e000201, 0x18018005, 0x1801d005, 0x4bf84a01, 0x0d000201,
    0x18019005, 0x1801a005, 0x1801b005, 0x1801e005, 0x4bf84101, 0x09000201,
    0x1801f005, 0x4bf84101, 0x09000201, 0x18020005, 0x43b13501, 0x00080201,
    0x18000075, 0x00006000, 0x18106085, 0x43b13501, 0x00080201, 0x18010075,
    0x00010000, 0x18020055, 0x18107085, 0x43b13501, 0x00080201, 0x18030075,
    0x00008000, 0x1813d085, 0x43b36601, 0x00000201, 0x18105005, 0x43b36601,
    0x00000201, 0x18140005, 0x43b30101, 0x00000201, 0x18200035, 0x00100000,
    0x43b30101, 0x00000201, 0x18300035, 0x00100000, 0x43b03101, 0x00080011,
    0x00000b03, 0x1810a085, 0x43b03101, 0x00084011, 0x00000c03, 0x1810b0c5,
    0x1813e085, 0x43b03101, 0x00084011, 0x00000d03, 0x1810c0c5, 0x1813f085,
    0x43bfff01, 0x00080201, 0x0059b035, 0x001a5000, 0x00939035, 0x076c7000,
    0x10000035, 0x08000000, 0x18008035, 0x00008000, 0x18021035, 0x0000f000,
    0x18038035, 0x000c8000, 0x1810d035, 0x00023000, 0x18141035, 0x000bf000,
    0x18400035, 0x00c00000, 0x1a000035, 0x02000000, 0x20000035, 0xc8000000,
    0xe9800035, 0x16800000, 0x18104085, 0x0000000f,
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

static void bcm4387_reset(void *opaque)
{
    BCM4387MachineState *s = opaque;

    cpu_reset(CPU(s->cpu));
    /*
     * brcmf_chip_cr4_set_active(): the host writes only image word 0 (the
     * reset vector) to address 0; the rest of that TCM is never written.
     */
    memset(s->lowtcm_mem, 0, LOWTCM_SIZE);
    bitmap_zero(s->lowtcm_valid, LOWTCM_SIZE / 4);
    stl_le_p(s->lowtcm_mem, s->rstvec);
    set_bit(0, s->lowtcm_valid);
    s->shared = ram_ld(RAM_TOP - 4);
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

    /*
     * CP15 as read on silicon at reset: ATCM region 0x39 (base 0, 8 MiB),
     * BTCM region 0x00800031 (base 0x800000, 2 MiB), SCTLR 0x40e50878 (TE
     * from cfgte). Never-written TCM words read as ECC (parity) aborts. [F]
     */
    cpuobj = object_new(machine->cpu_type);
    object_property_set_bool(cpuobj, "cfgte", true, &error_fatal);
    object_property_set_bool(cpuobj, "access-error-parity", true, &error_fatal);
    object_property_set_uint(cpuobj, "atcm-region", 0x39, &error_fatal);
    object_property_set_uint(cpuobj, "btcm-region", 0x00800031, &error_fatal);
    ARM_CPU(cpuobj)->reset_sctlr = 0x00e50878;
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    s->cpu = ARM_CPU(cpuobj);

    /* RAM: the ATCM part at rambase, BTCM from 0x800000 up to RAM_TOP */
    memory_region_init_ram(&s->atcm, NULL, "bcm4387.atcm", s->atcm_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, RAM_BASE, &s->atcm);
    s->lowtcm_mem = g_malloc0(LOWTCM_SIZE);
    s->lowtcm_valid = bitmap_new(LOWTCM_SIZE / 4);
    memory_region_init_io(&s->lowtcm, NULL, &lowtcm_ops, s, "bcm4387.tcm0",
                          LOWTCM_SIZE);
    memory_region_add_subregion(sysmem, 0, &s->lowtcm);
    memory_region_init_io(&s->hole, NULL, &hole_ops, s, "bcm4387.hole",
                          HOLE_SIZE);
    memory_region_add_subregion(sysmem, HOLE_BASE, &s->hole);
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
        /* Default the whole ROM window to "bx lr", then overlay any real ROM
         * given via rom= on top. rom= is a ';'-separated
         * list of chunks, each "path" (loaded at ROM_BASE) or "path@0xADDR"
         * (loaded at that absolute address in the ROM window). Partial chunks
         * are fine; whatever isn't covered stays a stub. */
        uint16_t *h = memory_region_get_ram_ptr(&s->rom_mr);

        for (int i = 0; i < ROM_SIZE / 2; i++) {
            h[i] = cpu_to_le16(0x4770);     /* bx lr */
        }

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

    s->rom_pristine = g_memdup2(memory_region_get_ram_ptr(&s->rom_mr),
                                ROM_SIZE);

    memory_region_init_rom(&s->erom, NULL, "bcm4387.erom", EROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, EROM_BASE, &s->erom);
    erom_build(s);

    s->store = g_hash_table_new(g_direct_hash, g_direct_equal);
    oobr_init(s);
    phy_init(s);
    /* register state at the host->firmware handoff, read on silicon [F] */
    store_set(s, CC_BASE + 0x1e0, 0x40);
    store_set(s, PMU_BASE + 0x600, 0x380);
    store_set(s, PMU_BASE + 0x608, 0x1613e);
    store_set(s, PMU_BASE + 0x618, 1);
    store_set(s, PMU_BASE + 0x61c, 0x7fffffff);
    {
        static const uint32_t iostatus[3] = { 0x11002, 0x11001, 0x11018 };

        for (int n = 0; n < 3; n++) {
            store_set(s, D11_WRAP(n) + WRAP_IOCTL, 0);
            store_set(s, D11_WRAP(n) + WRAP_RESETCTRL, 1);
            store_set(s, D11_WRAP(n) + WRAP_IOSTATUS, iostatus[n]);
            store_set(s, D11_BASE(n) + 0x1e0, 0x40);
            s->d11_phypll_t[n] = -1;
        }
    }
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
     * (Same two patches as scripts/rompatch.py applies on hardware.)
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
            /* stock image only: the patch sites must hold the stock bytes */
            static const uint8_t ret0[] = { 0x00, 0x20, 0x70, 0x47 };
            static const uint8_t orig1[] = { 0x08, 0xb5, 0x07, 0x4b };
            static const uint8_t orig2[] = { 0x13, 0x4b, 0x70, 0xb5 };
            gsize o1 = 0x8a3e48 - RAM_BASE, o2 = 0x8a3eb8 - RAM_BASE;

            if (o2 + 4 <= fwlen && !memcmp(fw + o1, orig1, 4) &&
                !memcmp(fw + o2, orig2, 4)) {
                memcpy(fw + o1, ret0, 4);
                memcpy(fw + o2, ret0, 4);
            } else {
                info_report("bcm4387: not the stock image, identity patch "
                            "not applied");
            }
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
        s->rstvec = ldl_le_p(fw);
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

static bool get_strict(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->strict;
}

static void set_strict(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->strict = v;
}

static void bcm4387_instance_init(Object *obj)
{
    /* CP15 at reset on silicon: ATCM region 0-8 MiB, BTCM base 0x800000 */
    BCM4387_MACHINE(obj)->atcm_size = 0x800000 - RAM_BASE;
    BCM4387_MACHINE(obj)->rng_state = 0x4387c2;
    BCM4387_MACHINE(obj)->identity = true;
    BCM4387_MACHINE(obj)->strict = true;
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
    object_class_property_add_bool(oc, "strict", get_strict, set_strict);
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
