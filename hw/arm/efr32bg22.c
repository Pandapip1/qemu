/*
 * Silicon Labs EFR32BG22 (Cortex-M33) machine with an out-of-process
 * peripheral model.
 *
 * Flash and SRAM are modelled in QEMU. Every access to the peripheral
 * window 0x40000000..0x5fffffff is forwarded synchronously over a chardev to
 * an external server (see tools/efr32bg22/server.py). Each reply carries the
 * level of every NVIC interrupt line and the virtual time at which the server
 * wants to be called next, so peripherals can be written in Python without
 * touching QEMU.
 *
 * Wire format (little endian, packed):
 *   request : u8 op, u8 size, u16 pad, u32 addr, u32 value, u32 pad, u64 now_ns
 *   reply   : u32 value, u32 nwr|flags, u64 irq_mask, u64 next_ns, then nwr x
 *             { u32 addr, u32 len, len bytes padded to 4 } guest memory writes
 *   op: 0 read, 1 write, 2 tick, 3 reset notification
 *   flags (bit 31 of the second word): the server asks for a system reset
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties-system.h"
#include "hw/loader.h"
#include "chardev/char-fe.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"
#include "target/arm/idau.h"
#include "target/arm/cpu.h"

#define TYPE_OWO_BRIDGE "owo-mmio-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(OwoBridge, OWO_BRIDGE)

#define BRIDGE_BASE 0x40000000
#define BRIDGE_SIZE 0x20000000
#define RADIO_BASE  0xA8000000
#define RADIO_SIZE  0x04000000
#define NUM_IRQ     64
#define FLASH_SIZE  (512 * 1024)
#define SRAM_SIZE   (32 * 1024)

typedef struct __attribute__((packed)) {
    uint8_t op, size;
    uint16_t pad0;
    uint32_t addr, value, pc;
    uint64_t now_ns;
} BridgeReq;

typedef struct __attribute__((packed)) {
    uint32_t value, nwr_flags;
    uint64_t irq_mask, next_ns;
} BridgeRep;

#define REP_NWR(r)        ((r)->nwr_flags & 0xffff)
#define REP_RESET         0x80000000u

struct OwoBridge {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    MemoryRegion mmio_radio;     /* 0xA8000000.. (radio and friends) */
    CharBackend chr;
    qemu_irq irq[NUM_IRQ];
    QEMUTimer *timer;
    uint64_t irq_mask;
    bool wdog_valid[2];
    uint32_t wdog_val[2];
    int64_t wdog_last_cmd;
};

static void bridge_apply(OwoBridge *s, const BridgeRep *r)
{
    uint64_t diff = s->irq_mask ^ r->irq_mask;

    for (int i = 0; i < NUM_IRQ && diff; i++) {
        if (diff & (1ULL << i)) {
            qemu_set_irq(s->irq[i], !!(r->irq_mask & (1ULL << i)));
        }
    }
    s->irq_mask = r->irq_mask;
    if (r->next_ns) {
        timer_mod(s->timer, r->next_ns);
    } else {
        timer_del(s->timer);
    }
}

static uint32_t bridge_xfer(OwoBridge *s, uint8_t op, hwaddr addr,
                            uint32_t val, unsigned size)
{
    BridgeReq q = { .op = op, .size = size, .addr = addr,
                    .value = val,
                    .pc = first_cpu ? ARM_CPU(first_cpu)->env.regs[15] : 0,
                    .now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) };
    BridgeRep r;

    if (qemu_chr_fe_write_all(&s->chr, (uint8_t *)&q, sizeof(q)) != sizeof(q) ||
        qemu_chr_fe_read_all(&s->chr, (uint8_t *)&r, sizeof(r)) != sizeof(r)) {
        error_report("efr32bg22: peripheral server went away");
        exit(EXIT_FAILURE);
    }
    for (uint32_t i = 0; i < REP_NWR(&r); i++) {
        uint32_t hdr[2];
        g_autofree uint8_t *buf = NULL;

        if (qemu_chr_fe_read_all(&s->chr, (uint8_t *)hdr, 8) != 8) {
            error_report("efr32bg22: truncated memory write from server");
            exit(EXIT_FAILURE);
        }
        buf = g_malloc(ROUND_UP(hdr[1], 4));
        if (qemu_chr_fe_read_all(&s->chr, buf, ROUND_UP(hdr[1], 4)) !=
            ROUND_UP(hdr[1], 4)) {
            error_report("efr32bg22: truncated memory write from server");
            exit(EXIT_FAILURE);
        }
        address_space_write(&address_space_memory, hdr[0],
                            MEMTXATTRS_UNSPECIFIED, buf, hdr[1]);
    }
    bridge_apply(s, &r);
    if (r.nwr_flags & REP_RESET) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
    return r.value;
}

/*
 * Fast path for the watchdog: the stock firmware's main loop spins in WDOG_Feed (read EN,
 * read SYNCBUSY, write CMD) millions of times, and each access would be a round trip to the
 * peripheral server. EN and SYNCBUSY only change through writes other than CMD, so reads
 * of them are cached until such a write; CMD writes are forwarded at most every 500 us of
 * virtual time (the watchdog period is milliseconds).
 */
#define WDOG_BASE 0x4A018000u
#define WDOG_NS_BIT 0x10000000u

static bool wdog_reg(hwaddr a, unsigned *reg)
{
    uint32_t x = (uint32_t)a + BRIDGE_BASE;
    x &= ~WDOG_NS_BIT;
    if (x < WDOG_BASE || x >= WDOG_BASE + 0x4000) {
        return false;
    }
    *reg = (x - WDOG_BASE) & 0xFFF;
    return true;
}

static uint64_t bridge_read(void *opaque, hwaddr addr, unsigned size)
{
    OwoBridge *s = opaque;
    unsigned reg;

    if (wdog_reg(addr, &reg) && (reg == 0x04 || reg == 0x24)) {
        int i = reg == 0x04 ? 0 : 1;
        if (!s->wdog_valid[i]) {
            s->wdog_val[i] = bridge_xfer(s, 0, addr + BRIDGE_BASE, 0, size);
            s->wdog_valid[i] = true;
        }
        return s->wdog_val[i];
    }
    return bridge_xfer(opaque, 0, addr + BRIDGE_BASE, 0, size);
}

static uint64_t bridge_read_radio(void *opaque, hwaddr addr, unsigned size)
{
    return bridge_xfer(opaque, 0, addr + RADIO_BASE, 0, size);
}

static void bridge_write_radio(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    bridge_xfer(opaque, 1, addr + RADIO_BASE, val, size);
}

static void bridge_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    OwoBridge *s = opaque;
    unsigned reg;

    if (wdog_reg(addr, &reg)) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        if (reg == 0x0C) {
            if (now - s->wdog_last_cmd < 500000 && s->wdog_last_cmd) {
                return;
            }
            s->wdog_last_cmd = now;
        } else {
            s->wdog_valid[0] = s->wdog_valid[1] = false;
        }
    }
    bridge_xfer(opaque, 1, addr + BRIDGE_BASE, val, size);
}

static void bridge_tick(void *opaque)
{
    bridge_xfer(opaque, 2, 0, 0, 4);
}

static const MemoryRegionOps bridge_ops = {
    .read = bridge_read,
    .write = bridge_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps bridge_radio_ops = {
    .read = bridge_read_radio,
    .write = bridge_write_radio,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bridge_init(Object *obj)
{
    OwoBridge *s = OWO_BRIDGE(obj);

    memory_region_init_io(&s->mmio, obj, &bridge_ops, s, "efr32bg22-periph",
                          BRIDGE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    memory_region_init_io(&s->mmio_radio, obj, &bridge_radio_ops, s,
                          "efr32bg22-radio", RADIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio_radio);
    for (int i = 0; i < NUM_IRQ; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
}

static void bridge_realize(DeviceState *dev, Error **errp)
{
    OwoBridge *s = OWO_BRIDGE(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bridge_tick, s);
    /* Blocking reads are used, so no handlers are installed. */
}

static void bridge_reset_hold(Object *obj, ResetType type)
{
    OwoBridge *s = OWO_BRIDGE(obj);

    s->irq_mask = 0;
    s->wdog_valid[0] = s->wdog_valid[1] = false;
    s->wdog_last_cmd = 0;
    if (s->timer) {
        timer_del(s->timer);
    }
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        bridge_xfer(s, 3, 0, 0, 4);
    }
}

static Property bridge_props[] = {
    DEFINE_PROP_CHR("chardev", OwoBridge, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void bridge_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = bridge_realize;
    rc->phases.hold = bridge_reset_hold;
    device_class_set_props(dc, bridge_props);
}

static const TypeInfo bridge_info = {
    .name = TYPE_OWO_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OwoBridge),
    .instance_init = bridge_init,
    .class_init = bridge_class_init,
};

/* ---- machine ---------------------------------------------------------- */

struct Efr32State {
    MachineState parent_obj;
    ARMv7MState armv7m;
    MemoryRegion flash, sram;
    Clock *sysclk;
    char *flash_image;
};

#define TYPE_EFR32BG22_MACHINE MACHINE_TYPE_NAME("efr32bg22")
OBJECT_DECLARE_SIMPLE_TYPE(Efr32State, EFR32BG22_MACHINE)

/*
 * The CPU is reset once before the machine is complete; reset it again after the
 * flash holds its contents so it picks up the bootloader's vector table.
 */
static void efr32_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void efr32_init(MachineState *machine)
{
    Efr32State *s = EFR32BG22_MACHINE(machine);
    DeviceState *nvic;
    DeviceState *br;
    Chardev *chr = serial_hd(0);

    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, 38400000);

    memory_region_init_ram(&s->flash, NULL, "efr32.flash", FLASH_SIZE,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, &s->flash);
    memory_region_init_ram(&s->sram, NULL, "efr32.sram", SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x20000000, &s->sram);

    object_initialize_child(OBJECT(machine), "armv7m", &s->armv7m,
                            TYPE_ARMV7M);
    nvic = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(nvic, "num-irq", NUM_IRQ);
    qdev_connect_clock_in(nvic, "cpuclk", s->sysclk);
    qdev_prop_set_string(nvic, "cpu-type", machine->cpu_type);
    object_property_set_link(OBJECT(&s->armv7m), "idau", OBJECT(machine),
                             &error_abort);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(get_system_memory()), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), &error_fatal);

    if (!chr) {
        error_report("efr32bg22: needs -serial chardev:<id> for the "
                     "peripheral server");
        exit(EXIT_FAILURE);
    }
    br = qdev_new(TYPE_OWO_BRIDGE);
    qdev_prop_set_chr(br, "chardev", chr);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(br), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(br), 0, BRIDGE_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(br), 1, RADIO_BASE);
    {   /* Non-secure alias of the radio block, and the radio RAMs */
        MemoryRegion *sm = get_system_memory();
        MemoryRegion *ra = g_new(MemoryRegion, 1);
        MemoryRegion *radio_ram = g_new(MemoryRegion, 1);
        MemoryRegion *radio_ram_ns = g_new(MemoryRegion, 1);
        MemoryRegion *devinfo = g_new(MemoryRegion, 1);
        SysBusDevice *sbd = SYS_BUS_DEVICE(br);

        memory_region_init_alias(ra, NULL, "radio-ns", sysbus_mmio_get_region(sbd, 1),
                                 0, RADIO_SIZE);
        memory_region_add_subregion(sm, 0xB8000000, ra);
        /* SEQRAM (16K) + FRCRAM (16K) */
        memory_region_init_ram(radio_ram, NULL, "efr32.radioram", 0x8000, &error_fatal);
        memory_region_add_subregion(sm, 0xA0000000, radio_ram);
        memory_region_init_alias(radio_ram_ns, NULL, "efr32.radioram-ns", radio_ram,
                                 0, 0x8000);
        memory_region_add_subregion(sm, 0xB0000000, radio_ram_ns);
        /* USERDATA / DEVINFO / CHIPCONFIG (populated by the server at reset) */
        memory_region_init_ram(devinfo, NULL, "efr32.devinfo", 0x10000, &error_fatal);
        memory_region_add_subregion(sm, 0x0FE00000, devinfo);
    }
    for (int i = 0; i < NUM_IRQ; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(br), i, qdev_get_gpio_in(nvic, i));
    }

    /*
     * Flash is initialised once from a full flash image (bootloader at 0,
     * application at its slot, ...) and then only changes through the MSC
     * model, so it survives system resets like the real part. Execution starts
     * at the reset vector in the bootloader at address 0.
     */
    if (s->flash_image) {
        gchar *data;
        gsize len;

        if (!g_file_get_contents(s->flash_image, &data, &len, NULL) ||
            len > FLASH_SIZE) {
            error_report("efr32bg22: cannot load flash image %s", s->flash_image);
            exit(EXIT_FAILURE);
        }
        memcpy(memory_region_get_ram_ptr(&s->flash), data, len);
        g_free(data);
    }
    qemu_register_reset(efr32_cpu_reset, ARM_CPU(first_cpu));
}

/*
 * External secure attribution unit (AN1374 section 3.3). Peripherals at
 * 0x4xxxxxxx and radio at 0xAxxxxxxx are always Secure, 0x5xxxxxxx and
 * 0xBxxxxxxx always Non-secure. Flash and RAM follow movable boundaries set
 * through the SMU; they are modelled as entirely Secure, which is the reset
 * state used by the bootloader and by apps that do not use TrustZone.
 */
static void efr32_idau_check(IDAUInterface *ii, uint32_t address,
                             int *iregion, bool *exempt, bool *ns, bool *nsc)
{
    int region = extract32(address, 28, 4);

    *iregion = region;
    *exempt = (address & 0xeff00000) == 0xe0000000;
    *ns = region == 5 || region == 0xb;
    *nsc = false;
}

static char *efr32_get_flash(Object *obj, Error **errp)
{
    return g_strdup(EFR32BG22_MACHINE(obj)->flash_image);
}

static void efr32_set_flash(Object *obj, const char *v, Error **errp)
{
    Efr32State *s = EFR32BG22_MACHINE(obj);

    g_free(s->flash_image);
    s->flash_image = g_strdup(v);
}

static void efr32_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    IDAUInterfaceClass *iic = IDAU_INTERFACE_CLASS(oc);

    iic->check = efr32_idau_check;
    static const char *const valid[] = { ARM_CPU_TYPE_NAME("cortex-m33"),
                                         NULL };

    mc->desc = "Silicon Labs EFR32BG22 (Cortex-M33), peripherals out of process";
    mc->init = efr32_init;
    mc->valid_cpu_types = valid;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    mc->default_ram_size = FLASH_SIZE; /* only bounds -device loader */
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash", efr32_get_flash, efr32_set_flash);
    object_class_property_set_description(oc, "flash",
        "512K flash image (bootloader at 0, application at its slot)");
}

static const TypeInfo efr32_info = {
    .name = TYPE_EFR32BG22_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Efr32State),
    .class_init = efr32_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_IDAU_INTERFACE },
        { }
    },
};

static void efr32_register(void)
{
    type_register_static(&bridge_info);
    type_register_static(&efr32_info);
}
type_init(efr32_register)
