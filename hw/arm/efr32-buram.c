/*
 * EFR32xG22 BURAM, backup RAM (RM rev. 1.2 chapter 17), at 0x40080000
 * (non-secure 0x50080000), bus clock CMU_CLKEN0.BURAM (bit 28).
 *
 * 32 retention registers RET0-31_REG (SS17.2-17.4), with the SET, CLR and
 * TGL aliases at +0x1000/+0x2000/+0x3000 (SS17.3).  "Any written RETREG
 * values will be retained through any event other than a brownout or
 * power-on reset" (SS17.4.1): the contents survive every reset this machine
 * models (system, pin, watchdog, lockup, QMP system_reset) and are zero
 * when the machine is created ("undefined out of reset" on silicon).
 * Byte and half-word accesses act on their lanes (Zephyr's
 * retained_mem_silabs_buram copies with memcpy).
 * [I] An access with CMU_CLKEN0.BURAM clear is a bus fault, RM SS4.2.1.2's
 * rule for a gated peripheral; not measured for this block.
 * [U] Not modelled: EM4 itself (nothing else survives it here either).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/arm/efr32-buram.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32BuramState, EFR32_BURAM)

#define BURAM_WORDS 32
#define BURAM_SIZE (BURAM_WORDS * 4)
#define BURAM_BLOCK_SIZE 0x4000u
#define BURAM_CLKEN0_BURAM (1u << 28)

struct Efr32BuramState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    const Efr32SocLink *link;
    hwaddr base;
    uint32_t ret[BURAM_WORDS];
};

static MemTxResult buram_read(void *opaque, hwaddr addr, uint64_t *data,
                              unsigned size, MemTxAttrs attrs)
{
    Efr32BuramState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;

    efr32_link_bus_access(s->link, s->base + addr, size, false);
    *data = 0;
    if (!efr32_link_bus_clock(s->link, EFR32_LINK_CLKEN0,
                              BURAM_CLKEN0_BURAM)) {
        return MEMTX_ERROR;
    }
    if (offset < BURAM_SIZE) {
        *data = extract32(s->ret[offset / 4], (offset & 3) * 8, size * 8);
    }
    return MEMTX_OK;
}

static MemTxResult buram_write(void *opaque, hwaddr addr, uint64_t data,
                               unsigned size, MemTxAttrs attrs)
{
    Efr32BuramState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    unsigned shift = (offset & 3) * 8;
    uint32_t lanes = size == 4 ? UINT32_MAX : MAKE_64BIT_MASK(shift, size * 8);
    uint32_t *word;

    efr32_link_bus_access(s->link, s->base + addr, size, true);
    if (!efr32_link_bus_clock(s->link, EFR32_LINK_CLKEN0,
                              BURAM_CLKEN0_BURAM)) {
        return MEMTX_ERROR;
    }
    if (offset >= BURAM_SIZE) {
        return MEMTX_OK;
    }
    word = &s->ret[offset / 4];
    *word = (*word & ~lanes) |
            (efr32_alias_apply(addr, *word, (uint32_t)data << shift) & lanes);
    return MEMTX_OK;
}

static const MemoryRegionOps buram_ops = {
    .read_with_attrs = buram_read,
    .write_with_attrs = buram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4},
    .impl = {.min_access_size = 1, .max_access_size = 4},
};

static void buram_init(Object *obj)
{
    Efr32BuramState *s = EFR32_BURAM(obj);

    memory_region_init_io(&s->mmio, obj, &buram_ops, s, TYPE_EFR32_BURAM,
                          BURAM_BLOCK_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const TypeInfo buram_info = {
    .name = TYPE_EFR32_BURAM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32BuramState),
    .instance_init = buram_init,
};

static void buram_register_types(void)
{
    type_register_static(&buram_info);
}
type_init(buram_register_types)

DeviceState *efr32_buram_create(Object *soc, MemoryRegion *memory,
                                const Efr32SocLink *link, hwaddr base,
                                hwaddr ns_offset)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_BURAM);
    Efr32BuramState *s = EFR32_BURAM(dev);

    object_property_add_child(soc, "buram", OBJECT(dev));
    s->link = link;
    s->base = base;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion_overlap(memory, base, &s->mmio, 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-buram-ns",
                             &s->mmio, 0, BURAM_BLOCK_SIZE);
    memory_region_add_subregion_overlap(memory, base + ns_offset, &s->mmio_ns,
                                        1);
    return dev;
}
