/*
 * QEMU machine model for the Intel Wi-Fi 6E AX210 (Typhoon Peak / "Ty")
 * on-die ARCompact (ARC700) MAC processor.
 *
 * This is a reverse-engineering / security-research scaffold, not a faithful
 * silicon model. It provides:
 *   - RAM over the firmware's LMAC/UMAC/uncached address windows,
 *   - traced MMIO stubs over the known peripheral windows (crypto coprocessor,
 *     control registers), and
 *   - a low-priority catch-all tracer so any unmodelled access is logged
 *     (with the faulting PC) and returns a programmable value instead of
 *     aborting, so firmware init can be driven and its hardware dependencies
 *     discovered iteratively.
 *
 * Firmware sections are loaded by the harness over the gdbstub; this board
 * only lays out memory and peripherals and brings up one ARC700 core.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "hw/arc/cpudevs.h"
#include "boot.h"

/* ---- traced MMIO -------------------------------------------------------- */

typedef struct {
    const char *name;
    hwaddr base;
} TraceRegion;

static FILE *trace_fp;
static unsigned long trace_count;
static unsigned long trace_cap = 200000;

/* Override table: reads from these addresses return the given value instead of
 * 0. Populated as we learn which status/"ready" registers firmware polls. */
#define MAX_OVERRIDES 64
static struct { hwaddr addr; uint32_t val; } overrides[MAX_OVERRIDES];
static int n_overrides;

static uint32_t override_lookup(hwaddr addr, int *hit)
{
    for (int i = 0; i < n_overrides; i++) {
        if (overrides[i].addr == addr) { *hit = 1; return overrides[i].val; }
    }
    *hit = 0;
    return 0;
}

static target_ulong cur_pc(void)
{
    if (current_cpu) {
        ARCCPU *c = ARC_CPU(current_cpu);
        return c->env.pc;
    }
    return 0;
}

static void trace_log(const char *tag, hwaddr addr, uint64_t val, unsigned size)
{
    if (!trace_fp) {
        const char *p = getenv("AX210_TRACE");
        trace_fp = p ? fopen(p, "w") : stderr;
        if (!trace_fp) trace_fp = stderr;
    }
    if (trace_count++ < trace_cap) {
        fprintf(trace_fp, "%-5s pc=%08x addr=%08x val=%08x sz=%u\n",
                tag, (uint32_t)cur_pc(), (uint32_t)addr, (uint32_t)val, size);
        fflush(trace_fp);
    }
}

static uint64_t trace_read(void *opaque, hwaddr addr, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    int hit; uint32_t v = override_lookup(base + addr, &hit);
    trace_log(hit ? "R.ovr" : "R", base + addr, v, size);
    return v;
}

static void trace_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    trace_log("W", base + addr, val, size);
}

static const MemoryRegionOps trace_ops = {
    .read = trace_read,
    .write = trace_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void add_ram(const char *name, hwaddr base, uint64_t size)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_ram(r, NULL, name, size, &error_fatal);
    memory_region_add_subregion(get_system_memory(), base, r);
}

static void add_trace(const char *name, hwaddr base, uint64_t size, int prio)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_io(r, NULL, &trace_ops, (void *)(uintptr_t)base,
                          name, size);
    memory_region_add_subregion_overlap(get_system_memory(), base, r, prio);
}

/* ---- machine ------------------------------------------------------------ */

static struct arc_boot_info ax210_boot_info;   /* zeroed: no kernel/cmdline */

static void ax210_init(MachineState *machine)
{
    ARCCPU *cpu = ARC_CPU(object_new(machine->cpu_type));
    object_property_set_uint(OBJECT(cpu), "freq_hz", 1000000, &error_fatal);
    object_property_set_bool(OBJECT(cpu), "rtc-opt", true, &error_fatal);
    object_property_set_bool(OBJECT(cpu), "realized", true, &error_fatal);
    cpu_arc_pic_init(cpu);
    cpu_arc_clock_init(cpu);
    cpu->env.boot_info = &ax210_boot_info;
    qemu_register_reset(arc_cpu_reset, cpu);

    /* Catch-all tracer over the whole 32-bit space (lowest priority). */
    add_trace("ax210.catchall", 0x00000000, 0x100000000ULL, -1000);

    /* RAM windows (firmware load addresses). ARC cache aliases 0x8.. and 0xc..
     * are modelled as independent RAM here; coherence between aliases is a
     * known simplification to revisit if firmware relies on it. */
    add_ram("ax210.lmac",  0x00000000, 0x01100000); /* LMAC code/data, paged, DCCM-low, CSS */
    add_ram("ax210.umac",  0x80000000, 0x00800000); /* UMAC code/data (0x80400000+) */
    add_ram("ax210.unc",   0xc0000000, 0x01000000); /* uncached window: code @0xc0080000, bss @0xc0880000 */

    /* Traced peripheral windows (overlay RAM; higher priority). */
    add_trace("ax210.crypto",   0x00a20000, 0x00010000, 10); /* RSA/SHA coproc 0xa248/49/4a */
    add_trace("ax210.ctl_d",    0xc0d00000, 0x00010000, 10); /* control regs polled by IML */
    add_trace("ax210.crypto_c", 0xc0e20000, 0x00010000, 10); /* remapped crypto window */
    add_trace("ax210.csr",      0xf0000000, 0x01000000, 10); /* chip CSRs */

    if (getenv("AX210_TRACECAP")) trace_cap = strtoul(getenv("AX210_TRACECAP"), 0, 0);

    /* MMIO read overrides, discovered iteratively, as
     * AX210_OVR="c0d02c68=0x100,c0d02c7c=0xffffffff,...". */
    const char *ovr = getenv("AX210_OVR");
    if (ovr) {
        char *s = g_strdup(ovr), *tok, *save = NULL;
        for (tok = strtok_r(s, ",", &save); tok && n_overrides < MAX_OVERRIDES;
             tok = strtok_r(NULL, ",", &save)) {
            char *eq = strchr(tok, '=');
            if (!eq) continue;
            *eq = 0;
            overrides[n_overrides].addr = strtoull(tok, 0, 16);
            overrides[n_overrides].val  = strtoul(eq + 1, 0, 0);
            n_overrides++;
        }
        g_free(s);
    }
}

static void ax210_machine_init(MachineClass *mc)
{
    mc->desc = "Intel AX210 Wi-Fi MAC (ARC700) RE scaffold";
    mc->init = ax210_init;
    mc->max_cpus = 1;
    mc->no_serial = 1;
    mc->default_cpu_type = TYPE_ARC_CPU_ARC700;
}

DEFINE_MACHINE("ax210", ax210_machine_init)
