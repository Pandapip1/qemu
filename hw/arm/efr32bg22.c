/*
 * Silicon Labs EFR32BG22 native QEMU machine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/boards.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
#include "sysemu/reset.h"
#include "target/arm/cpu.h"
#include "target/arm/idau.h"

#define TYPE_EFR32_PERIPH "efr32bg22-periph"
OBJECT_DECLARE_SIMPLE_TYPE(Efr32Periph, EFR32_PERIPH)
#define PERIPH_BASE 0x40000000u
#define PERIPH_SIZE 0x20000000u
#define RADIO_BASE 0xa8000000u
#define RADIO_SIZE 0x04000000u
#define FLASH_SIZE (512 * 1024)
#define SRAM_SIZE (32 * 1024)

/* Radio block addresses and source-backed register offsets. */
#define EFR32_FRC_BASE      0xa8004000u
#define EFR32_RFCRC_BASE    0xa8010000u
#define EFR32_MODEM_BASE    0xa8014000u
#define EFR32_SYNTH_BASE    0xa8018000u
#define EFR32_PROTIMER_BASE 0xa801c000u
#define EFR32_RAC_BASE      0xa8020000u

#define EFR32_RAC_STATUS    0x00cu
#define EFR32_RAC_CMD       0x010u
#define EFR32_RAC_IF         0x01cu
#define EFR32_RAC_IEN        0x020u
#define EFR32_RAC_SEQIF      0x028u
#define EFR32_RAC_SEQIEN     0x02cu
#define EFR32_RAC_CMD_TXEN  (1u << 0)
#define EFR32_RAC_CMD_TXDIS (1u << 5)
#define EFR32_RAC_CMD_RXDIS (1u << 8)
#define EFR32_RAC_STATE_OFF 0u
#define EFR32_RAC_STATE_TX  9u

#define EFR32_BUFC_BASE        0xaa000000u
#define EFR32_BUFC_FIRST       0x00cu
#define EFR32_BUFC_STRIDE      0x040u
#define EFR32_BUFC_IF          0x114u
#define EFR32_BUFC_IEN         0x118u
#define EFR32_BUFC_WRITEOFFSET 0x008u
#define EFR32_BUFC_READOFFSET  0x00cu
#define EFR32_BUFC_READDATA    0x014u
#define EFR32_BUFC_WRITEDATA   0x018u
#define EFR32_BUFC_XWRITEDATA  0x01cu
#define EFR32_BUFC_STATUS      0x020u
#define EFR32_BUFC_THRESHOLD   0x024u
#define EFR32_BUFC_CMD         0x028u

struct Efr32Periph {
    SysBusDevice parent_obj;
    MemoryRegion peripheral_mmio;
    MemoryRegion radio_mmio;
    GHashTable *registers;
    uint32_t trng_state;
    uint32_t rac_state;
    GByteArray *bufc_data[4];
    qemu_irq irq[64];
};

static uint32_t efr32_register_get(Efr32Periph *s, uint32_t address);
static void efr32_register_put(Efr32Periph *s, uint32_t address,
                               uint32_t value);

static void efr32_bufc_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IEN);

    qemu_set_irq(s->irq[32], !!(pending & enabled));
}

typedef struct Efr32RegisterMask {
    uint16_t offset;
    uint32_t writable_mask;
} Efr32RegisterMask;

static const Efr32RegisterMask efr32_frc_masks[] = {
    {0x00c, 0x01ffff7f}, {0x010, 0x0000ffff}, {0x020, 0x00000fff},
    {0x024, 0x00000fff}, {0x030, 0x00001f7f}, {0x034, 0x0000ffff},
    {0x038, 0x0000ffff}, {0x03c, 0x003ffff7}, {0x040, 0xfffffffc},
    {0x044, 0xfffffffc}, {0x048, 0x001f3ff7}, {0x04c, 0x000007ff},
    {0x050, 0x007fffff}, {0x05c, 0x00037f7f}, {0x060, 0x00007f7f},
    {0x0a8, 0x0000ffff}, {0x0bc, 0x00000fff}, {0x0c0, 0x0001ffff},
    {0x0c4, 0x7fffff7f}, {0x0c8, 0x00000fff}, {0x0cc, 0x0103ffff},
    {0x104, 0x0001ffff}, {0x108, 0x0001ffff}, {0x10c, 0x0001ffff},
    {0x110, 0x0001ffff},
};

static uint32_t efr32_apply_register_mask(const Efr32RegisterMask *masks,
                                          size_t count, uint32_t offset,
                                          uint32_t value)
{
    for (size_t i = 0; i < count; i++) {
        if (offset == masks[i].offset) {
            return value & masks[i].writable_mask;
        }
    }

    return value;
}

static bool efr32_bufc_decode(uint32_t offset, unsigned *channel,
                              uint32_t *register_offset)
{
    uint32_t relative;

    if (offset < EFR32_BUFC_FIRST ||
        offset >= EFR32_BUFC_FIRST + 4 * EFR32_BUFC_STRIDE) {
        return false;
    }

    relative = offset - EFR32_BUFC_FIRST;
    *channel = relative / EFR32_BUFC_STRIDE;
    *register_offset = relative % EFR32_BUFC_STRIDE;
    return true;
}

static uint32_t efr32_bufc_channel_base(unsigned channel)
{
    return EFR32_BUFC_BASE + EFR32_BUFC_FIRST +
           channel * EFR32_BUFC_STRIDE;
}

static void efr32_bufc_update_threshold(Efr32Periph *s, unsigned channel)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t config = efr32_register_get(s, base + EFR32_BUFC_THRESHOLD);
    uint32_t count = s->bufc_data[channel]->len;
    uint32_t threshold = config & 0xfff;
    uint32_t flag = 1u << (8 * channel + 2);
    bool hit = config & 0x2000 ? count <= threshold : count > threshold;
    uint32_t pending = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF);

    pending = hit ? pending | flag : pending & ~flag;
    efr32_register_put(s, EFR32_BUFC_BASE + EFR32_BUFC_IF, pending);
    efr32_bufc_update_irq(s);
}

static void efr32_bufc_write_data(Efr32Periph *s, unsigned channel,
                                  uint8_t value, bool xor_write)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t position = efr32_register_get(s, base + EFR32_BUFC_WRITEOFFSET) &
                        0x1fff;
    GByteArray *data = s->bufc_data[channel];

    if (position >= data->len) {
        g_byte_array_set_size(data, position + 1);
    }
    data->data[position] = xor_write ? data->data[position] ^ value : value;
    efr32_register_put(s, base + EFR32_BUFC_WRITEOFFSET, position + 1);
    efr32_bufc_update_threshold(s, channel);
}

/* Each Series-2 block has normal, SET, CLR, and TGL 4-KiB windows. */
static uint32_t efr32_register_key(uint32_t address) {
    return (address & ~0x3fffu) | (address & 0xffcu);
}

static uint32_t efr32_register_get(Efr32Periph *s, uint32_t address) {
    return GPOINTER_TO_UINT(g_hash_table_lookup(
        s->registers, GUINT_TO_POINTER(efr32_register_key(address))));
}

static void efr32_register_put(Efr32Periph *s, uint32_t address,
                               uint32_t value) {
    g_hash_table_insert(s->registers,
                        GUINT_TO_POINTER(efr32_register_key(address)),
                        GUINT_TO_POINTER(value));
}

static void efr32_rac_update_irqs(Efr32Periph *s)
{
    qemu_set_irq(s->irq[37], !!(efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IF) &
                                 efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IEN)));
    qemu_set_irq(s->irq[38], !!(efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF) &
                                 efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIEN)));
}

static uint32_t efr32_radio_mask_write(uint32_t address, uint32_t value) {
    uint32_t offset = address & 0xfff;

    if ((address & ~0x3fffu) == 0xa8018000 && offset >= 0x34 &&
        offset <= 0x44 && !(offset & 3)) {
        static const uint32_t m[] = {0x0fffffffu, 0x001fffffu, 0x1ff, 0x3f,
                                     0x3ffff};
        return value & m[(offset - 0x34) / 4];
    }
    if ((address & ~0x3fffu) == 0xa8010000) {
        if (offset == 4)
            return value & 1;
        if (offset == 8)
            return value & 0x1fef;
    }
    if ((address & ~0x3fffu) == EFR32_FRC_BASE) {
        return efr32_apply_register_mask(efr32_frc_masks,
                                         G_N_ELEMENTS(efr32_frc_masks),
                                         offset, value);
    }
    return value;
}

static uint32_t efr32_rac_status(Efr32Periph *s)
{
    uint32_t state = s->rac_state;

    return state << 24 | (state != EFR32_RAC_STATE_OFF) << 28 |
           (state == EFR32_RAC_STATE_TX) << 30;
}

static uint32_t efr32_peripheral_read_word(Efr32Periph *s, uint32_t address) {
    uint32_t offset = address & 0xfff;
    unsigned channel;
    uint32_t register_offset;

    if ((address & ~0x3fffu) == EFR32_BUFC_BASE &&
        efr32_bufc_decode(offset, &channel, &register_offset)) {
        uint32_t base = efr32_bufc_channel_base(channel);

        if (register_offset == EFR32_BUFC_READDATA) {
            uint32_t position =
                efr32_register_get(s, base + EFR32_BUFC_READOFFSET) & 0x1fff;
            uint32_t value = position < s->bufc_data[channel]->len ?
                                 s->bufc_data[channel]->data[position] :
                                 0;

            efr32_register_put(s, base + EFR32_BUFC_READOFFSET, position + 1);
            efr32_bufc_update_threshold(s, channel);
            return value;
        }
        if (register_offset == EFR32_BUFC_STATUS) {
            uint32_t read_offset =
                efr32_register_get(s, base + EFR32_BUFC_READOFFSET) & 0x1fff;
            uint32_t count = s->bufc_data[channel]->len > read_offset ?
                                 s->bufc_data[channel]->len - read_offset :
                                 0;
            uint32_t threshold_flag =
                efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF) &
                        (1u << (8 * channel + 2)) ?
                    1u << 20 :
                    0;

            return (count & 0x1fff) | threshold_flag;
        }
    }
    if (address == 0x4400003c)
        return 0x20; /* RADIOAES.STATUS WAITING */
    if ((address & ~0xfffu) == 0x4c024000) {
        uint32_t x = s->trng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return s->trng_state = x;
    }
    if (address == 0x4c021004 || address == 0x4c02100c)
        return 64;
    if (address == 0x40008008)
        return UINT32_MAX; /* CMU.STATUS */
    if (address == 0x4001000c || address == 0x40018004 || address == 0x40028008)
        return 0x00010001;
    if (address == 0x4000c058)
        return 0x000b0003;
    if (address == 0x4009402c)
        return efr32_register_get(s, 0x40094008) & 1 ? 6 : 1;
    if (address == 0x4a018004 || address == 0x4a018024)
        return 0;
    if (address == EFR32_RAC_BASE + EFR32_RAC_STATUS)
        return efr32_rac_status(s);
    if (((address & ~0x3fffu) == 0xa8028000 ||
         (address & ~0x3fffu) == 0xa802c000) &&
        offset == 0x40)
        return 0;
    return efr32_register_get(s, address);
}
static uint64_t efr32_peripheral_read(Efr32Periph *s, uint32_t address,
                                      unsigned size) {
    return extract32(efr32_peripheral_read_word(s, address & ~3u),
                     (address & 3) * 8, size * 8);
}
static void efr32_peripheral_write(Efr32Periph *s, uint32_t address,
                                   uint64_t value, unsigned size) {
    unsigned channel;
    uint32_t register_offset;
    uint32_t word = address & ~3u;
    uint32_t register_address = efr32_register_key(word);
    uint32_t old = efr32_register_get(s, register_address),
             shift = (address & 3) * 8,
             mask = size == 4 ? UINT32_MAX : ((1u << (size * 8)) - 1) << shift,
             next = (old & ~mask) | (((uint32_t)value << shift) & mask);
    switch ((address >> 12) & 3) {
    case 1:
        next = old | next;
        break;
    case 2:
        next = old & ~next;
        break;
    case 3:
        next = old ^ next;
    }

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CMD) {
        if (next & (EFR32_RAC_CMD_TXDIS | EFR32_RAC_CMD_RXDIS))
            s->rac_state = EFR32_RAC_STATE_OFF;
        else if (next & EFR32_RAC_CMD_TXEN)
            s->rac_state = EFR32_RAC_STATE_TX;
        return;
    }
    if ((register_address & ~0x3fffu) == EFR32_BUFC_BASE &&
        efr32_bufc_decode(register_address & 0xfff, &channel,
                          &register_offset)) {
        uint32_t base = efr32_bufc_channel_base(channel);

        switch (register_offset) {
        case EFR32_BUFC_WRITEDATA:
            efr32_bufc_write_data(s, channel, next, false);
            return;
        case EFR32_BUFC_XWRITEDATA:
            efr32_bufc_write_data(s, channel, next, true);
            return;
        case EFR32_BUFC_CMD:
            efr32_register_put(s, register_address, next & 0xf);
            if (next & 1) {
                g_byte_array_set_size(s->bufc_data[channel], 0);
                efr32_register_put(s, base + EFR32_BUFC_WRITEOFFSET, 0);
                efr32_register_put(s, base + EFR32_BUFC_READOFFSET, 0);
                efr32_bufc_update_threshold(s, channel);
            }
            return;
        default:
            break;
        }
    }
    efr32_register_put(s, register_address,
                       efr32_radio_mask_write(register_address, next));

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_IF ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_IEN ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_SEQIF ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_SEQIEN) {
        efr32_rac_update_irqs(s);
    }

    if (register_address == EFR32_BUFC_BASE + EFR32_BUFC_IF ||
        register_address == EFR32_BUFC_BASE + EFR32_BUFC_IEN) {
        efr32_bufc_update_irq(s);
    }

    if ((register_address & ~0x3fffu) == EFR32_BUFC_BASE &&
        efr32_bufc_decode(register_address & 0xfff, &channel,
                          &register_offset) &&
        (register_offset == 0 || register_offset == EFR32_BUFC_WRITEOFFSET ||
         register_offset == EFR32_BUFC_READOFFSET ||
         register_offset == EFR32_BUFC_THRESHOLD)) {
        efr32_bufc_update_threshold(s, channel);
    }
}
static uint64_t efr32_peripheral_mmio_read(void *opaque, hwaddr offset,
                                           unsigned size) {
    return efr32_peripheral_read(EFR32_PERIPH(opaque), PERIPH_BASE + offset,
                                 size);
}
static void efr32_peripheral_mmio_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size) {
    efr32_peripheral_write(EFR32_PERIPH(opaque), PERIPH_BASE + offset, value,
                           size);
}
static uint64_t efr32_radio_mmio_read(void *opaque, hwaddr offset,
                                      unsigned size) {
    return efr32_peripheral_read(EFR32_PERIPH(opaque), RADIO_BASE + offset,
                                 size);
}
static void efr32_radio_mmio_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size) {
    efr32_peripheral_write(EFR32_PERIPH(opaque), RADIO_BASE + offset, value,
                           size);
}
static const MemoryRegionOps po = {
    .read = efr32_peripheral_mmio_read,
    .write = efr32_peripheral_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4}};
static const MemoryRegionOps ro = {
    .read = efr32_radio_mmio_read,
    .write = efr32_radio_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4}};
static void efr32_periph_init(Object *o) {
    Efr32Periph *s = EFR32_PERIPH(o);
    s->registers = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->trng_state = 0x2545f491;
    s->rac_state = EFR32_RAC_STATE_OFF;
    for (unsigned channel = 0; channel < G_N_ELEMENTS(s->bufc_data); channel++) {
        s->bufc_data[channel] = g_byte_array_new();
    }
    memory_region_init_io(&s->peripheral_mmio, o, &po, s, "efr32bg22-periph",
                          PERIPH_SIZE);
    memory_region_init_io(&s->radio_mmio, o, &ro, s, "efr32bg22-radio",
                          RADIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(o), &s->peripheral_mmio);
    sysbus_init_mmio(SYS_BUS_DEVICE(o), &s->radio_mmio);
    for (unsigned irq = 0; irq < G_N_ELEMENTS(s->irq); irq++) {
        sysbus_init_irq(SYS_BUS_DEVICE(o), &s->irq[irq]);
    }
}
static void efr32_periph_reset_hold(Object *o, ResetType t) {
    Efr32Periph *s = EFR32_PERIPH(o);
    g_hash_table_remove_all(s->registers);
    s->trng_state = 0x2545f491;
    s->rac_state = EFR32_RAC_STATE_OFF;
    for (unsigned irq = 0; irq < G_N_ELEMENTS(s->irq); irq++) {
        qemu_set_irq(s->irq[irq], 0);
    }
    for (unsigned channel = 0; channel < G_N_ELEMENTS(s->bufc_data); channel++) {
        g_byte_array_set_size(s->bufc_data[channel], 0);
    }
}
static void efr32_periph_class_init(ObjectClass *o, void *d) {
    RESETTABLE_CLASS(o)->phases.hold = efr32_periph_reset_hold;
}
static const TypeInfo periph_info = {.name = TYPE_EFR32_PERIPH,
                                     .parent = TYPE_SYS_BUS_DEVICE,
                                     .instance_size = sizeof(Efr32Periph),
                                     .instance_init = efr32_periph_init,
                                     .class_init = efr32_periph_class_init};

struct Efr32State {
    MachineState parent_obj;
    ARMv7MState armv7m;
    MemoryRegion flash, sram;
    Clock *clk;
    char *image;
};
#define TYPE_EFR32BG22_MACHINE MACHINE_TYPE_NAME("efr32bg22")
OBJECT_DECLARE_SIMPLE_TYPE(Efr32State, EFR32BG22_MACHINE)
static void cpu_reset_cb(void *o) { cpu_reset(CPU(o)); }
static void init(MachineState *m) {
    Efr32State *s = EFR32BG22_MACHINE(m);
    DeviceState *nvic, *p;
    MemoryRegion *sm = get_system_memory(), *alias = g_new(MemoryRegion, 1),
                 *ram = g_new(MemoryRegion, 1), *ns = g_new(MemoryRegion, 1),
                 *di = g_new(MemoryRegion, 1);
    s->clk = clock_new(OBJECT(m), "SYSCLK");
    clock_set_hz(s->clk, 38400000);
    memory_region_init_ram(&s->flash, NULL, "efr32.flash", FLASH_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sm, 0, &s->flash);
    memory_region_init_ram(&s->sram, NULL, "efr32.sram", SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sm, 0x20000000, &s->sram);
    object_initialize_child(OBJECT(m), "armv7m", &s->armv7m, TYPE_ARMV7M);
    nvic = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(nvic, "num-irq", 64);
    qdev_connect_clock_in(nvic, "cpuclk", s->clk);
    qdev_prop_set_string(nvic, "cpu-type", m->cpu_type);
    object_property_set_link(OBJECT(&s->armv7m), "idau", OBJECT(m),
                             &error_abort);
    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(sm),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), &error_fatal);
    p = qdev_new(TYPE_EFR32_PERIPH);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(p), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(p), 0, PERIPH_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(p), 1, RADIO_BASE);
    for (unsigned irq = 0; irq < 64; irq++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(p), irq, qdev_get_gpio_in(nvic, irq));
    }
    memory_region_init_alias(alias, NULL, "radio-ns",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(p), 1), 0,
                             RADIO_SIZE);
    memory_region_add_subregion(sm, 0xb8000000, alias);
    memory_region_init_ram(ram, NULL, "efr32.radioram", 0x8000, &error_fatal);
    memory_region_add_subregion(sm, 0xa0000000, ram);
    memory_region_init_alias(ns, NULL, "efr32.radioram-ns", ram, 0, 0x8000);
    memory_region_add_subregion(sm, 0xb0000000, ns);
    memory_region_init_ram(di, NULL, "efr32.devinfo", 0x10000, &error_fatal);
    memory_region_add_subregion(sm, 0x0fe00000, di);
    if (s->image) {
        gchar *d;
        gsize n;
        if (!g_file_get_contents(s->image, &d, &n, NULL) || n > FLASH_SIZE) {
            error_report("efr32bg22: cannot load flash image %s", s->image);
            exit(EXIT_FAILURE);
        }
        memcpy(memory_region_get_ram_ptr(&s->flash), d, n);
        g_free(d);
    }
    qemu_register_reset(cpu_reset_cb, ARM_CPU(first_cpu));
}
static void idau(IDAUInterface *i, uint32_t a, int *r, bool *e, bool *n,
                 bool *c) {
    *r = extract32(a, 28, 4);
    *e = (a & 0xeff00000) == 0xe0000000;
    *n = *r == 5 || *r == 0xb;
    *c = false;
}
static char *image_get(Object *o, Error **e) {
    return g_strdup(EFR32BG22_MACHINE(o)->image);
}
static void image_set(Object *o, const char *v, Error **e) {
    Efr32State *s = EFR32BG22_MACHINE(o);
    g_free(s->image);
    s->image = g_strdup(v);
}
static void mc(ObjectClass *o, void *d) {
    MachineClass *m = MACHINE_CLASS(o);
    IDAUInterfaceClass *i = IDAU_INTERFACE_CLASS(o);
    static const char *const cpu[] = {ARM_CPU_TYPE_NAME("cortex-m33"), NULL};
    i->check = idau;
    m->desc = "Silicon Labs EFR32BG22 (Cortex-M33), native peripherals";
    m->init = init;
    m->valid_cpu_types = cpu;
    m->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    m->default_ram_size = FLASH_SIZE;
    m->ignore_memory_transaction_failures = true;
    object_class_property_add_str(o, "flash", image_get, image_set);
}
static const TypeInfo machine_info = {
    .name = TYPE_EFR32BG22_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Efr32State),
    .class_init = mc,
    .interfaces = (InterfaceInfo[]){{TYPE_IDAU_INTERFACE}, {}}};
static void register_types(void) {
    type_register_static(&periph_info);
    type_register_static(&machine_info);
}
type_init(register_types)
