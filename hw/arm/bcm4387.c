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
 * rom= (optional ROM dump, 0x180000 bytes), atcm-size=.
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
 * The dumped ROM is 0x180000 (0x400000-0x580000), but one ROM-patch entry
 * redirects 0x58a758, so the real ROM extends past the dump; size the window to
 * cover it (the uncovered tail stays stub/veneer, never real ROM code). [F]
 */
#define ROM_SIZE        0x18b000

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
#define CR4_WRAP        0x18130000
#define EROM_BASE       0x18f00000
#define EROM_SIZE       0x1000

#define BCMA_CORE_CHIPCOMMON    0x800
#define BCMA_CORE_ARM_CR4       0x83e
#define BRCM_DESIGNER           0x4bf

struct BCM4387MachineState {
    MachineState parent;

    char *hostblob;
    char *rom;
    char *patch;
    uint32_t atcm_size;
    bool identity;
    bool relocate;

    ARMCPU *cpu;
    MemoryRegion atcm, atcm_alias, btcm, socram, rom_mr, erom, scratch, trace;
    GHashTable *store;
    uint32_t cr4_bankidx;
    uint32_t bank_info[8];
    int nab, nbb;
    QEMUTimer *watch;
    uint32_t shared;
    uint32_t rng_state;
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

static bool model_read(BCM4387MachineState *s, hwaddr a, uint32_t *v)
{
    switch (a) {
    case CC_BASE + 0x00:
        *v = chipid();
        return true;
    case CC_BASE + 0xfc:
        *v = EROM_BASE;
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
    }
    return false;
}

static bool model_write(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    switch (a) {
    case CR4_BASE + 0x40:
        s->cr4_bankidx = v & 0xff;
        return true;
    }
    return false;
}

static uint64_t trace_read(void *opaque, hwaddr addr, unsigned size)
{
    BCM4387MachineState *s = opaque;
    uint32_t v = 0;
    bool modelled = model_read(s, addr, &v);

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
    bool modelled = model_write(s, addr, val);

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

/* ---- EROM --------------------------------------------------------------- */

static uint32_t *erom_put_core(uint32_t *p, uint16_t id, uint8_t rev,
                               uint32_t base, uint32_t wrap)
{
    /* CIA: designer, part number, class; CIB: rev, 1 slave wrapper, 1 port */
    *p++ = (BRCM_DESIGNER << 20) | (id << 8) | 0x1;
    *p++ = (rev << 24) | (1 << 19) | (1 << 9) | 0x1;
    /* slave port 0, 4 KiB */
    *p++ = base | 0x5;
    /* slave wrapper (type 2), 4 KiB */
    *p++ = wrap | (2 << 6) | 0x5;
    return p;
}

static void erom_build(BCM4387MachineState *s)
{
    uint32_t *e = memory_region_get_ram_ptr(&s->erom), *p = e;

    memset(e, 0, EROM_SIZE);
    p = erom_put_core(p, BCMA_CORE_CHIPCOMMON, 0, CC_BASE, 0x18100000);
    p = erom_put_core(p, BCMA_CORE_ARM_CR4, 0, CR4_BASE, CR4_WRAP);
    *p++ = 0xf;
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
        if (s->identity && !s->relocate) {
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

static void bcm4387_instance_init(Object *obj)
{
    BCM4387_MACHINE(obj)->atcm_size = 0x140000;
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
