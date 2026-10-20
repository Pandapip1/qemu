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
#include "qemu/timer.h"
#include "hw/boards.h"
#include "hw/arm/efr32-rf-medium.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
#include "sysemu/reset.h"
#include "target/arm/cpu.h"
#include "target/arm/idau.h"
#include "trace.h"

#define TYPE_EFR32_PERIPH "efr32bg22-periph"
OBJECT_DECLARE_SIMPLE_TYPE(Efr32Periph, EFR32_PERIPH)
#define PERIPH_BASE 0x40000000u
#define PERIPH_SIZE 0x20000000u
#define RADIO_BASE 0xa8000000u
#define RADIO_SIZE 0x04000000u
#define FLASH_SIZE (512 * 1024)
#define SRAM_SIZE (32 * 1024)

/* Radio block addresses and source-backed register offsets. */
#define EFR32_FRC_BASE 0xa8004000u
#define EFR32_AGC_BASE 0xa800c000u
#define EFR32_RFCRC_BASE 0xa8010000u
#define EFR32_MODEM_BASE 0xa8014000u
#define EFR32_SYNTH_BASE 0xa8018000u
#define EFR32_PROTIMER_BASE 0xa801c000u
#define EFR32_PROTIMER_NS_BASE 0xb801c000u
#define EFR32_RAC_BASE 0xa8020000u

#define EFR32_FRC_IF 0x068u
#define EFR32_FRC_IEN 0x06cu
#define EFR32_FRC_RXRAWDATA 0x090u
#define EFR32_FRC_IF_TXDONE (1u << 0)
#define EFR32_FRC_IF_RXDONE (1u << 4)

#define EFR32_AGC_RSSI 0x018u
#define EFR32_AGC_FRAMERSSI 0x01cu
#define EFR32_AGC_IF 0x044u
#define EFR32_AGC_IEN 0x048u
#define EFR32_AGC_CMD 0x04cu

#define EFR32_RAC_STATUS 0x00cu
#define EFR32_RAC_RXENSRCEN 0x008u
#define EFR32_RAC_CMD 0x010u
#define EFR32_RAC_CTRL 0x014u
#define EFR32_RAC_IF 0x01cu
#define EFR32_RAC_IEN 0x020u
#define EFR32_RAC_SEQIF 0x028u
#define EFR32_RAC_SEQIEN 0x02cu
#define EFR32_RAC_CMD_TXEN (1u << 0)
#define EFR32_RAC_CMD_TXDIS (1u << 5)
#define EFR32_RAC_CMD_RXDIS (1u << 8)
#define EFR32_RAC_CTRL_FORCEDISABLE (1u << 0)
#define EFR32_RAC_RXENSRCEN_SWRXEN (1u << 1)
#define EFR32_RAC_STATE_OFF 0u
#define EFR32_RAC_STATE_RXSEARCH 2u
#define EFR32_RAC_STATE_TX 9u

/* Synthetic model delay, not a measured on-air transmission duration. */
#define EFR32_RAC_TX_COMPLETION_DELAY_NS 150000

/* Observed in rfhal_protimer.o and the retained stock trace; latch only. */
#define EFR32_PROTIMER_EN 0x004u
#define EFR32_PROTIMER_CTRL 0x008u
#define EFR32_PROTIMER_CMD 0x00cu
#define EFR32_PROTIMER_COUNTER 0x020u
#define EFR32_PROTIMER_STATE 0x030u
#define EFR32_PROTIMER_PERIOD 0x038u
#define EFR32_PROTIMER_TRACE_60 0x060u
#define EFR32_PROTIMER_TRACE_64 0x064u
#define EFR32_PROTIMER_TRACE_74 0x074u
#define EFR32_PROTIMER_TRACE_78 0x078u
#define EFR32_PROTIMER_CMP0 0x100u
#define EFR32_PROTIMER_CMP1 0x110u
#define EFR32_PROTIMER_CMP2 0x120u
#define EFR32_PROTIMER_CMP3 0x130u
#define EFR32_PROTIMER_CMP4 0x140u

/* Minimal clock/power state required by the stock boot path. */
#define EFR32_CMU_BASE 0x40008000u
#define EFR32_CMU_SYSCLKCTRL 0x070u
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_MASK 0x7u
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_FSRCO 1u
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_HFXO 3u
#define EFR32_HFXO_BASE 0x4000c000u
#define EFR32_HFXO_SECURE_BASE 0x5000c000u
#define EFR32_HFXO_CTRL 0x028u
#define EFR32_HFXO_CMD 0x050u
#define EFR32_HFXO_STATUS 0x058u
#define EFR32_HFXO_CTRL_FORCEEN (1u << 0)
#define EFR32_HFXO_CTRL_DISONDEMAND (1u << 1)
#define EFR32_HFXO_CMD_MANUALOVERRIDE (1u << 1)
#define EFR32_HFXO_STATUS_RDY (1u << 0)
#define EFR32_HFXO_STATUS_COREBIASOPTRDY (1u << 1)
#define EFR32_HFXO_STATUS_ENS (1u << 16)
#define EFR32_HFXO_STATUS_HWREQ (1u << 17)
#define EFR32_HFXO_STATUS_ISWARM (1u << 19)
#define EFR32_HFXO_STATUS_FSMLOCK (1u << 30)
#define EFR32_HFXO_READY_STATUS \
    (EFR32_HFXO_STATUS_RDY | EFR32_HFXO_STATUS_COREBIASOPTRDY | \
     EFR32_HFXO_STATUS_ENS | EFR32_HFXO_STATUS_HWREQ | EFR32_HFXO_STATUS_ISWARM)

#define EFR32_DCDC_BASE 0x40094000u
#define EFR32_DCDC_SECURE_BASE 0x50094000u
#define EFR32_DCDC_CTRL 0x008u
#define EFR32_DCDC_IF 0x024u
#define EFR32_DCDC_STATUS 0x02cu
#define EFR32_DCDC_IF_REGULATION (1u << 5)
#define EFR32_DCDC_STATUS_WARM (1u << 1)
#define EFR32_DCDC_STATUS_RUNNING (1u << 2)

#define EFR32_EMU_BASE 0x40004000u
#define EFR32_EMU_STATUS 0x084u
#define EFR32_EMU_STATUS_RESET 0x00000080u

#define EFR32_MSC_BASE 0x40030000u
#define EFR32_MSC_STATUS 0x01cu
#define EFR32_MSC_STATUS_RESET 0x08000008u

#define EFR32_BUFC_BASE 0xaa000000u
#define EFR32_BUFC_FIRST 0x00cu
#define EFR32_BUFC_STRIDE 0x040u
#define EFR32_BUFC_IF 0x114u
#define EFR32_BUFC_IEN 0x118u
#define EFR32_BUFC_CTRL 0x000u
#define EFR32_BUFC_WRITEOFFSET 0x008u
#define EFR32_BUFC_READOFFSET 0x00cu
#define EFR32_BUFC_READDATA 0x014u
#define EFR32_BUFC_WRITEDATA 0x018u
#define EFR32_BUFC_XWRITEDATA 0x01cu
#define EFR32_BUFC_STATUS 0x020u
#define EFR32_BUFC_THRESHOLD 0x024u
#define EFR32_BUFC_CMD 0x028u

struct Efr32Periph
{
    SysBusDevice parent_obj;
    MemoryRegion peripheral_mmio;
    MemoryRegion radio_mmio;
    GHashTable *registers;
    uint32_t trng_state;
    uint32_t rac_state;
    bool hfxo_fsm_locked;
    bool rx_offer_pending;
    Clock *protimer_clk;
    ptimer_state *protimer_counter;
    QEMUTimer *rac_tx_timer;
    QEMUTimer *rac_rx_timer;
    GByteArray *bufc_data[4];
    Efr32RfMedium *rf_medium;
    qemu_irq irq[64];
};

static uint32_t efr32_register_get(Efr32Periph *s, uint32_t address);
static void efr32_register_put(Efr32Periph *s, uint32_t address,
                               uint32_t value);

static void efr32_agc_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_AGC_BASE + EFR32_AGC_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_AGC_BASE + EFR32_AGC_IEN);

    qemu_set_irq(s->irq[31], !!(pending & enabled));
}

static void efr32_bufc_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IEN);

    qemu_set_irq(s->irq[32], !!(pending & enabled));
}

static void efr32_frc_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_FRC_BASE + EFR32_FRC_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_FRC_BASE + EFR32_FRC_IEN);

    qemu_set_irq(s->irq[34], !!(pending & enabled));
}

static void efr32_frc_raise(Efr32Periph *s, uint32_t flags)
{
    efr32_register_put(s, EFR32_FRC_BASE + EFR32_FRC_IF,
                       efr32_register_get(s, EFR32_FRC_BASE + EFR32_FRC_IF) |
                           flags);
    efr32_frc_update_irq(s);
}

typedef struct Efr32RegisterMask
{
    uint16_t offset;
    uint32_t writable_mask;
} Efr32RegisterMask;

static const Efr32RegisterMask efr32_frc_masks[] = {
    {0x004, 0x00000001}, {0x00c, 0x01ffff7f}, {0x010, 0x0000ffff},
    {0x018, 0x000000ff}, {0x020, 0x00000fff},
    {0x024, 0x00000fff}, {0x030, 0x00001f7f}, {0x034, 0x0000ffff},
    {0x038, 0x0000ffff}, {0x03c, 0x003ffff7}, {0x040, 0xfffffffc},
    {0x044, 0xfffffffc}, {0x048, 0x001f3ff7}, {0x04c, 0x000007ff},
    {0x050, 0x007fffff}, {0x054, 0x0000003f}, {0x05c, 0x00037f7f},
    {0x060, 0x00007f7f},
    {0x068, 0x7f7fffff}, {0x06c, 0x7f7fffff},
    {0x078, 0x0000000f},
    {0x084, 0x0003ffff},
    {0x08c, 0x000021bf},
    {0x0a8, 0x0000ffff}, {0x0bc, 0x00000fff}, {0x0c0, 0x0001ffff},
    {0x0c4, 0x7fffff7f}, {0x0c8, 0x00000fff}, {0x0cc, 0x0103ffff},
    {0x0b4, 0x7f7fffff}, {0x0b8, 0x7f7fffff},
    {0x104, 0x0001ffff}, {0x108, 0x0001ffff}, {0x10c, 0x0001ffff},
    {0x110, 0x0001ffff},
};

static const Efr32RegisterMask efr32_agc_masks[] = {
    {0x004, 0x00000001}, {0x020, 0xfecfffff}, {0x024, 0xffffffff},
    {0x028, 0xe3ffffff},
    {0x02c, 0xffffffff}, {0x030, 0xfe00ffff}, {0x034, 0x00ffffff},
    {0x038, 0xc0000000}, {0x03c, 0x3fffffff}, {0x040, 0x0001ffff},
    {EFR32_AGC_IF, 0x0000037d}, {EFR32_AGC_IEN, 0x0000037d},
    {0x050, 0x0fffffff}, {0x054, 0xffffffff}, {0x058, 0xffffffff},
    {0x05c, 0x0003ffff}, {0x064, 0x3fffffff}, {0x068, 0x3fffffff},
    {0x06c, 0x3fffffff}, {0x070, 0x03ffffff}, {0x074, 0x07ffffff},
    {0x078, 0x3fffffff}, {0x07c, 0x3fffffff}, {0x080, 0xffffffff},
    {0x084, 0x00000fff}, {0x088, 0x0000007f},
};

static const Efr32RegisterMask efr32_modem_masks[] = {
    {0x004, 0x00000001}, {0x01c, 0x0000001f}, {0x020, 0xffffffff},
    {0x024, 0xffffdfff},
    {0x028, 0xffffffff}, {0x02c, 0xffffff81}, {0x030, 0xbfffffff},
    {0x034, 0x607007fe}, {0x038, 0xf6000000}, {0x058, 0x00ffffff},
    {0x05c, 0x00001fff}, {0x060, 0xcfffffff}, {0x064, 0xffff1fff},
    {0x068, 0xffffffff}, {0x06c, 0xffffffff}, {0x098, 0xffffffff},
    {0x09c, 0xffffffff}, {0x0a0, 0xffffffff}, {0x0a4, 0xffffffff},
    {0x0a8, 0xffffffff}, {0x0ac, 0xffffffff}, {0x0b0, 0xffffffff},
    {0x0b4, 0xffffffff}, {0x0b8, 0xffffffff}, {0x0bc, 0xffffffff},
    {0x0c0, 0xffffffff}, {0x0e0, 0x000001ff}, {0x0e4, 0x7fffffff},
    {0x0ec, 0xeffff8ff}, {0x0f4, 0xffffffff}, {0x0f8, 0x7fffffff},
    {0x0fc, 0xffefffff}, {0x100, 0xffffffff}, {0x104, 0xffffffff},
    {0x10c, 0x007fffff}, {0x110, 0x7fffffff}, {0x114, 0xffffffff},
    {0x124, 0x0000ffff}, {0x128, 0x0000ffff}, {0x130, 0x7ffffeff},
    {0x134, 0x00001f0f}, {0x138, 0x7fffffff}, {0x13c, 0x3fff7fff},
    {0x140, 0xffffffff}, {0x144, 0xffffffff}, {0x148, 0xffffffff},
    {0x14c, 0x0fffffff}, {0x150, 0xfff7ffff}, {0x154, 0x000001ff},
    {0x168, 0xffffffff}, {0x16c, 0x07ffffff}, {0x170, 0x0000fff3},
    {0x230, 0xffffffff}, {0x234, 0x3fffffff}, {0x238, 0x000000ff},
    {0x23c, 0x000001ff}, {0x244, 0x00001e00}, {0x248, 0xe01fffff},
    {0x208, 0x0007ffff}, {0x20c, 0x0007ffff}, {0x228, 0x0000ffbf},
    {0x254, 0x3ffff7ff}, {0x258, 0x01ffffff}, {0x25c, 0x0003ffff},
    {0x260, 0x0003ffff}, {0x264, 0xffffffff}, {0x268, 0xbfffc3ff},
    {0x26c, 0x3fffffff}, {0x270, 0x0003ffff}, {0x274, 0x00ffffff},
    {0x278, 0x80000000}, {0x27c, 0x000001ff}, {0x280, 0x000007ff},
};

static const Efr32RegisterMask efr32_rac_masks[] = {
    {0x004, 0x00000001}, {0x008, 0x00003fff}, {0x014, 0x1f0107ef},
    {0x018, 0x0000000f}, {0x01c, 0x00ff000f}, {0x020, 0x00ff000f},
    {0x024, 0x00000003}, {0x028, 0x3fff000f}, {0x02c, 0x3fff000f},
};

static uint32_t efr32_apply_register_mask(const Efr32RegisterMask *masks,
                                          size_t count, uint32_t offset,
                                          uint32_t value)
{
    for (size_t i = 0; i < count; i++)
    {
        if (offset == masks[i].offset)
        {
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
        offset >= EFR32_BUFC_FIRST + 4 * EFR32_BUFC_STRIDE)
    {
        return false;
    }

    relative = offset - EFR32_BUFC_FIRST;
    *channel = relative / EFR32_BUFC_STRIDE;
    *register_offset = relative % EFR32_BUFC_STRIDE;
    return true;
}

static uint32_t efr32_bufc_channel_base(unsigned channel)
{
    return EFR32_BUFC_BASE + EFR32_BUFC_FIRST + channel * EFR32_BUFC_STRIDE;
}

static uint32_t efr32_bufc_capacity(Efr32Periph *s, unsigned channel)
{
    uint32_t config =
        efr32_register_get(s, efr32_bufc_channel_base(channel) + EFR32_BUFC_CTRL);

    /* RAIL encodes supported 64..4096-byte power-of-two FIFO sizes as 0..6. */
    return config <= 6 ? 64u << config : 0;
}

static uint32_t efr32_bufc_storage_offset(Efr32Periph *s, unsigned channel,
                                          uint32_t offset)
{
    uint32_t capacity = efr32_bufc_capacity(s, channel);

    return capacity ? offset & (capacity - 1) : offset;
}

static uint32_t efr32_bufc_count(Efr32Periph *s, unsigned channel)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t write_offset =
        efr32_register_get(s, base + EFR32_BUFC_WRITEOFFSET) & 0x1fff;
    uint32_t read_offset =
        efr32_register_get(s, base + EFR32_BUFC_READOFFSET) & 0x1fff;

    return (write_offset - read_offset) & 0x1fff;
}

static void efr32_bufc_update_threshold(Efr32Periph *s, unsigned channel)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t config = efr32_register_get(s, base + EFR32_BUFC_THRESHOLD);
    uint32_t count = efr32_bufc_count(s, channel);
    uint32_t capacity = efr32_bufc_capacity(s, channel);
    uint32_t threshold = config & 0xfff;
    uint32_t flag = 1u << (8 * channel + 2);
    bool hit = config & 0x2000 ? count <= threshold : count > threshold;
    uint32_t pending = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF);

    pending = hit ? pending | flag : pending & ~flag;
    if (capacity && count > capacity)
    {
        pending |= 1u << (8 * channel);
    }
    efr32_register_put(s, EFR32_BUFC_BASE + EFR32_BUFC_IF, pending);
    efr32_bufc_update_irq(s);
}

/*
 * BUFC data ports are the boundary between the packet engines and the FIFO
 * backing store.  Host/RF ingress is not modeled yet, but keeping this write
 * path separate from MMIO dispatch gives a future FRC/RF producer the same
 * circular-buffer and guest-RAM behavior as sequencer programmed I/O.
 */
static void efr32_bufc_write_data_port(Efr32Periph *s, unsigned channel,
                                       uint8_t value, bool xor_write)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t buffer_address = efr32_register_get(s, base + 0x004);
    uint32_t write_offset =
        efr32_register_get(s, base + EFR32_BUFC_WRITEOFFSET) & 0x1fff;
    uint32_t position = efr32_bufc_storage_offset(s, channel, write_offset);
    GByteArray *data = s->bufc_data[channel];

    if (position >= data->len)
    {
        g_byte_array_set_size(data, position + 1);
    }
    data->data[position] = xor_write ? data->data[position] ^ value : value;
    if (buffer_address)
    {
        address_space_write(&address_space_memory, buffer_address + position,
                            MEMTXATTRS_UNSPECIFIED, &data->data[position], 1);
    }
    efr32_register_put(s, base + EFR32_BUFC_WRITEOFFSET, write_offset + 1);
    efr32_bufc_update_threshold(s, channel);
}

static uint8_t efr32_bufc_read_data_port(Efr32Periph *s, unsigned channel)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t buffer_address = efr32_register_get(s, base + 0x004);
    uint32_t read_offset =
        efr32_register_get(s, base + EFR32_BUFC_READOFFSET) & 0x1fff;
    uint32_t position = efr32_bufc_storage_offset(s, channel, read_offset);
    uint8_t value = position < s->bufc_data[channel]->len
                        ? s->bufc_data[channel]->data[position]
                        : 0;

    if (buffer_address)
    {
        address_space_read(&address_space_memory, buffer_address + position,
                           MEMTXATTRS_UNSPECIFIED, &value, 1);
    }

    efr32_register_put(s, base + EFR32_BUFC_READOFFSET, read_offset + 1);
    efr32_bufc_update_threshold(s, channel);
    return value;
}

/*
 * Capture the software-visible TX FIFO at a model-defined snapshot boundary.
 * A direct BUFC data-port user is represented by bufc_data.  RAIL's normal
 * SetTxFifo path instead preloads guest RAM and describes it with ADDR and
 * the two offsets, so prefer that circular backing store when it is present.
 * This is intentionally not a claim that hardware consumes the FIFO at TXEN.
 */
static GByteArray *efr32_bufc_snapshot_data(Efr32Periph *s, unsigned channel)
{
    uint32_t base = efr32_bufc_channel_base(channel);
    uint32_t buffer_address = efr32_register_get(s, base + 0x004);
    uint32_t capacity = efr32_bufc_capacity(s, channel);
    uint32_t read_offset;
    uint32_t length;
    GByteArray *snapshot;

    if (!buffer_address || !capacity) {
        return g_byte_array_ref(s->bufc_data[channel]);
    }

    read_offset = efr32_register_get(s, base + EFR32_BUFC_READOFFSET) & 0x1fff;
    length = MIN(efr32_bufc_count(s, channel), capacity);
    snapshot = g_byte_array_sized_new(length);
    g_byte_array_set_size(snapshot, length);
    for (uint32_t index = 0; index < length; index++) {
        uint8_t value;
        uint32_t offset = efr32_bufc_storage_offset(
            s, channel, read_offset + index);

        address_space_read(&address_space_memory, buffer_address + offset,
                           MEMTXATTRS_UNSPECIFIED, &value, 1);
        snapshot->data[index] = value;
    }
    return snapshot;
}

static void efr32_bufc_clear(Efr32Periph *s, unsigned channel)
{
    uint32_t base = efr32_bufc_channel_base(channel);

    g_byte_array_set_size(s->bufc_data[channel], 0);
    efr32_register_put(s, base + EFR32_BUFC_WRITEOFFSET, 0);
    efr32_register_put(s, base + EFR32_BUFC_READOFFSET, 0);
    efr32_bufc_update_threshold(s, channel);
}

/* Each Series-2 block has normal, SET, CLR, and TGL 4-KiB windows. */
static uint32_t efr32_peripheral_canonical_address(uint32_t address)
{
    /*
     * RAIL accesses PROTIMER through its non-secure window at 0xb801c000,
     * while stock setup traces also use the secure 0xa801c000 window.  They
     * are views of the same block, including the standard SET/CLR/TGL pages.
     */
    if ((address & ~0x3fffu) == EFR32_PROTIMER_NS_BASE) {
        address = EFR32_PROTIMER_BASE + (address & 0x3fffu);
    }

    /* HFXO0 is visible through both the secure and non-secure peripheral map. */
    if ((address & ~0x3fffu) == EFR32_HFXO_SECURE_BASE) {
        address = EFR32_HFXO_BASE + (address & 0x3fffu);
    }

    /* DCDC0 has the same secure/non-secure peripheral-map arrangement. */
    if ((address & ~0x3fffu) == EFR32_DCDC_SECURE_BASE) {
        address = EFR32_DCDC_BASE + (address & 0x3fffu);
    }

    return address;
}

static uint32_t efr32_register_key(uint32_t address)
{
    address = efr32_peripheral_canonical_address(address);

    return (address & ~0x3fffu) | (address & 0xffcu);
}

static uint32_t efr32_register_get(Efr32Periph *s, uint32_t address)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(
        s->registers, GUINT_TO_POINTER(efr32_register_key(address))));
}

static void efr32_register_put(Efr32Periph *s, uint32_t address, uint32_t value)
{
    g_hash_table_insert(s->registers,
                        GUINT_TO_POINTER(efr32_register_key(address)),
                        GUINT_TO_POINTER(value));
}

static bool efr32_hfxo_enabled(Efr32Periph *s)
{
    uint32_t hfxo_ctrl =
        efr32_register_get(s, EFR32_HFXO_BASE + EFR32_HFXO_CTRL);
    uint32_t sysclk_ctrl =
        efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL);

    return (hfxo_ctrl & EFR32_HFXO_CTRL_FORCEEN) ||
           ((sysclk_ctrl & EFR32_CMU_SYSCLKCTRL_CLKSEL_MASK) ==
            EFR32_CMU_SYSCLKCTRL_CLKSEL_HFXO);
}

static void efr32_rac_update_irqs(Efr32Periph *s)
{
    qemu_set_irq(s->irq[37],
                 !!(efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IF) &
                    efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IEN)));
    qemu_set_irq(s->irq[38],
                 !!(efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF) &
                    efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIEN)));
}

static void efr32_rac_set_state(Efr32Periph *s, uint32_t state)
{
    uint32_t previous_state;
    uint32_t sequence_flag;

    if (s->rac_state == state)
    {
        return;
    }

    previous_state = s->rac_state;
    s->rac_state = state;
    efr32_register_put(s, EFR32_RAC_BASE + EFR32_RAC_IF,
                       efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IF) |
                           1);
    switch (state)
    {
    case EFR32_RAC_STATE_OFF:
        sequence_flag = 1u << 16;
        break;
    case EFR32_RAC_STATE_RXSEARCH:
        sequence_flag = 1u << 18;
        break;
    case EFR32_RAC_STATE_TX:
        sequence_flag = 1u << 25;
        break;
    default:
        g_assert_not_reached();
    }
    efr32_register_put(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF,
                       efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF) |
                           sequence_flag);
    efr32_rac_update_irqs(s);
    trace_efr32bg22_rac_state(previous_state, state);
}

static void efr32_rac_tx_complete(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    efr32_rac_set_state(s, EFR32_RAC_STATE_OFF);
    efr32_frc_raise(s, EFR32_FRC_IF_TXDONE);
}

static void efr32_rac_cancel_tx_completion(Efr32Periph *s)
{
    timer_del(s->rac_tx_timer);
}

static void efr32_rac_schedule_tx_completion(Efr32Periph *s)
{
    timer_mod(s->rac_tx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  EFR32_RAC_TX_COMPLETION_DELAY_NS);
}

/*
 * An RX offer is deferred out of the transmitter's RAC.CMD write path.  This
 * is a QEMU scheduling boundary, not a claim about propagation or packet-air
 * time.  It also gives a receiver that turns RX off before dispatch a chance
 * to leave its offer pending until a later RXSEARCH entry.
 */
static void efr32_rac_schedule_rx_handoff(Efr32Periph *s)
{
    timer_mod(s->rac_rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void efr32_rac_rx_handoff(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    Efr32RfRxDeliveryRequest *request;
    const Efr32RfRxMetadata *metadata;
    const uint8_t *data;
    gsize length;

    if (s->rac_state != EFR32_RAC_STATE_RXSEARCH) {
        return;
    }

    request = efr32_rf_medium_take_rx_offer_for_radio(
        s->rf_medium, EFR32_RF_RADIO(s));
    if (!request) {
        s->rx_offer_pending = false;
        return;
    }

    metadata = efr32_rf_rx_delivery_request_get_metadata(request);
    g_assert(metadata->buffer_channel == EFR32_RF_RX_BUFC_CHANNEL);
    data = g_bytes_get_data(efr32_rf_rx_delivery_request_get_data(request),
                            &length);

    /*
     * The retained sequencer reads FRC.RXRAWDATA then writes its four byte
     * lanes to secure BUFC channel 1 WRITEDATA.  Replay only that observed
     * data path: these bytes are not framed, CRC checked, or made into a
     * channel-2 descriptor.
     */
    for (gsize index = 0; index < length; index++) {
        efr32_register_put(s, EFR32_FRC_BASE + EFR32_FRC_RXRAWDATA,
                           data[index]);
        efr32_bufc_write_data_port(s, EFR32_RF_RX_BUFC_CHANNEL,
                                   data[index], false);
    }
    efr32_rf_rx_delivery_request_free(request);
    efr32_frc_raise(s, EFR32_FRC_IF_RXDONE);

    /* One mailbox record is consumed per deferred handoff. */
    s->rx_offer_pending = efr32_rf_medium_has_rx_offer_for_radio(
        s->rf_medium, EFR32_RF_RADIO(s));
    if (s->rx_offer_pending) {
        efr32_rac_schedule_rx_handoff(s);
    }
}

static void efr32_rac_capture_tx_snapshot(Efr32Periph *s)
{
    Efr32RfTxMetadata metadata = {
        .trigger = EFR32_RF_TX_TRIGGER_RAC_CMD_TXEN,
        .buffer_channel = EFR32_RF_TX_BUFC_CHANNEL,
    };
    GByteArray *data = efr32_bufc_snapshot_data(
        s, EFR32_RF_TX_BUFC_CHANNEL);
    Efr32RfTxSnapshot *snapshot;
    const uint8_t *bytes;
    gsize length;
    uint32_t first_word = 0;

    /*
     * The retained blob evidence places the TX handoff at RAC.CMD.TXEN.
     * BUFC channel 0 is copied here, before the synthetic RAC completion
     * timer is armed. This does not initiate RF delivery.
     */
    snapshot = efr32_rf_medium_snapshot_tx(
        s->rf_medium, EFR32_RF_RADIO(s), &metadata, data->data, data->len,
        &error_abort);
    g_byte_array_unref(data);
    bytes = g_bytes_get_data(efr32_rf_tx_snapshot_get_data(snapshot), &length);
    for (unsigned index = 0; index < MIN(length, sizeof(first_word)); index++) {
        first_word |= (uint32_t)bytes[index] << (index * 8);
    }
    trace_efr32bg22_tx_snapshot(efr32_rf_tx_snapshot_get_sequence(snapshot),
                                 length, first_word);
    efr32_rf_medium_set_latest_tx_snapshot(s->rf_medium, snapshot);
    trace_efr32bg22_tx_offer_count(
        efr32_rf_medium_offer_latest_tx(s->rf_medium, &error_abort));
}

static uint32_t efr32_radio_mask_write(uint32_t address, uint32_t value)
{
    uint32_t offset = address & 0xfff;

    if ((address & ~0x3fffu) == EFR32_SYNTH_BASE && offset == 0x004)
    {
        return value & 1;
    }
    if ((address & ~0x3fffu) == EFR32_SYNTH_BASE && offset >= 0x34 &&
        offset <= 0x44 && !(offset & 3))
    {
        static const uint32_t m[] = {0x0fffffffu, 0x001fffffu, 0x1ff, 0x3f,
                                     0x3ffff};
        return value & m[(offset - 0x34) / 4];
    }
    if ((address & ~0x3fffu) == 0xa8010000)
    {
        if (offset == 4)
            return value & 1;
        if (offset == 8)
            return value & 0x1fef;
    }
    if ((address & ~0x3fffu) == EFR32_BUFC_BASE &&
        (offset == EFR32_BUFC_IF || offset == EFR32_BUFC_IEN))
    {
        return value & 0x8f0f0f0f;
    }
    if ((address & ~0x3fffu) == EFR32_FRC_BASE)
    {
        return efr32_apply_register_mask(
            efr32_frc_masks, G_N_ELEMENTS(efr32_frc_masks), offset, value);
    }
    if ((address & ~0x3fffu) == EFR32_AGC_BASE)
    {
        return efr32_apply_register_mask(
            efr32_agc_masks, G_N_ELEMENTS(efr32_agc_masks), offset, value);
    }
    if ((address & ~0x3fffu) == EFR32_MODEM_BASE)
    {
        return efr32_apply_register_mask(
            efr32_modem_masks, G_N_ELEMENTS(efr32_modem_masks), offset, value);
    }
    if ((address & ~0x3fffu) == EFR32_RAC_BASE)
    {
        return efr32_apply_register_mask(
            efr32_rac_masks, G_N_ELEMENTS(efr32_rac_masks), offset, value);
    }
    return value;
}

static uint32_t efr32_bufc_mask_write(uint32_t offset, uint32_t value)
{
    switch (offset)
    {
    case EFR32_BUFC_CTRL:
        return value & 0x7;
    case EFR32_BUFC_WRITEOFFSET:
    case EFR32_BUFC_READOFFSET:
        return value & 0x1fff;
    case EFR32_BUFC_THRESHOLD:
        return value & 0x2fff;
    default:
        return value;
    }
}

static uint32_t efr32_rac_status(Efr32Periph *s)
{
    uint32_t state = s->rac_state;

    return state << 24 | (state != EFR32_RAC_STATE_OFF) << 28 |
           (state == EFR32_RAC_STATE_TX) << 30 |
           (state == EFR32_RAC_STATE_RXSEARCH) << 31;
}

static void efr32_protimer_counter_expired(void *opaque)
{
    /* Free-running counter rollover has no observed side effects. */
}

static void efr32_protimer_clock_update(void *opaque, ClockEvent event)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    ptimer_transaction_begin(s->protimer_counter);
    ptimer_set_period_from_clock(s->protimer_counter, s->protimer_clk, 1);
    ptimer_transaction_commit(s->protimer_counter);
}

/*
 * RAIL_GetTime reads PROTIMER +0x020. The public RAIL API documents its
 * timer tick as roughly 0.5 microseconds, so the machine provides a dedicated
 * 2 MHz input clock. This deliberately remains independent of SYSCLKCTRL.
 *
 * The stock helper at flash +0x26e0c does derive a setup value from the
 * SYSCLKCTRL-selected rate and RADIO RAM +0x4fe0 bit 16, then the PROTIMER
 * setup at +0x29974 writes that derived state after CTRL. That establishes
 * setup arithmetic, not the source or divisor of the public free-running
 * counter. Do not couple this clock to those inputs without that missing
 * hardware-level evidence. The ptimer owns scheduling and the wrapping
 * counter; compare/capture, commands, and interrupt behavior remain
 * unmodeled.
 */
static uint32_t efr32_protimer_counter(Efr32Periph *s)
{
    return UINT32_MAX - ptimer_get_count(s->protimer_counter);
}

static uint32_t efr32_peripheral_read_word(Efr32Periph *s, uint32_t address)
{
    address = efr32_peripheral_canonical_address(address);
    uint32_t offset = address & 0xfff;
    unsigned channel;
    uint32_t register_offset;

    if ((address & ~0x3fffu) == EFR32_BUFC_BASE &&
        efr32_bufc_decode(offset, &channel, &register_offset))
    {
        if (register_offset == EFR32_BUFC_READDATA)
        {
            return efr32_bufc_read_data_port(s, channel);
        }
        if (register_offset == EFR32_BUFC_STATUS)
        {
            uint32_t count = efr32_bufc_count(s, channel);
            uint32_t threshold_flag =
                efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF) &
                        (1u << (8 * channel + 2))
                    ? 1u << 20
                    : 0;

            return (count & 0x1fff) | threshold_flag;
        }
    }
    if (address == 0x4400003c)
        return 0x20; /* RADIOAES.STATUS WAITING */
    if ((address & ~0xfffu) == 0x4c024000)
    {
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
    if (address == EFR32_HFXO_BASE + EFR32_HFXO_STATUS)
    {
        if (!efr32_hfxo_enabled(s))
        {
            return 0;
        }
        return EFR32_HFXO_READY_STATUS |
               (s->hfxo_fsm_locked ? EFR32_HFXO_STATUS_FSMLOCK : 0);
    }
    if (address == EFR32_DCDC_BASE + EFR32_DCDC_IF)
        return efr32_register_get(s, EFR32_DCDC_BASE + EFR32_DCDC_CTRL) & 1
                   ? EFR32_DCDC_IF_REGULATION
                   : 0;
    if (address == EFR32_DCDC_BASE + EFR32_DCDC_STATUS)
        return efr32_register_get(s, EFR32_DCDC_BASE + EFR32_DCDC_CTRL) & 1
                   ? EFR32_DCDC_STATUS_WARM | EFR32_DCDC_STATUS_RUNNING
                   : EFR32_DCDC_STATUS_WARM;
    if (address == 0x4a018004 || address == 0x4a018024)
        return 0;
    if (address == EFR32_RAC_BASE + EFR32_RAC_STATUS)
        return efr32_rac_status(s);
    if (((address & ~0x3fffu) == EFR32_PROTIMER_BASE ||
         (address & ~0x3fffu) == EFR32_PROTIMER_NS_BASE) &&
        offset == EFR32_PROTIMER_COUNTER)
    {
        return efr32_protimer_counter(s);
    }
    if (address == EFR32_AGC_BASE + EFR32_AGC_CMD)
        return 0; /* AGC.CMD is write-only in the public register model. */
    if (((address & ~0x3fffu) == 0xa8028000 ||
         (address & ~0x3fffu) == 0xa802c000) &&
        offset == 0x40)
        return 0;
    return efr32_register_get(s, address);
}
static uint64_t efr32_peripheral_read(Efr32Periph *s, uint32_t address,
                                      unsigned size)
{
    return extract32(efr32_peripheral_read_word(s, address & ~3u),
                     (address & 3) * 8, size * 8);
}
static void efr32_peripheral_write(Efr32Periph *s, uint32_t address,
                                   uint64_t value, unsigned size)
{
    unsigned channel;
    uint32_t register_offset;
    uint32_t word = address & ~3u;
    uint32_t register_address = efr32_register_key(word);
    uint32_t old = efr32_register_get(s, register_address),
             shift = (address & 3) * 8,
             mask = size == 4 ? UINT32_MAX : ((1u << (size * 8)) - 1) << shift,
             next = (old & ~mask) | (((uint32_t)value << shift) & mask);
    bool hfxo_was_enabled = efr32_hfxo_enabled(s);

    if (word >= RADIO_BASE && word < RADIO_BASE + RADIO_SIZE)
    {
        uint32_t pc = current_cpu ? ARM_CPU(current_cpu)->env.regs[15] : 0;

        trace_efr32bg22_radio_write(pc, word, (uint32_t)value, size);
    }

    switch ((address >> 12) & 3)
    {
    case 1:
        next = old | next;
        break;
    case 2:
        next = old & ~next;
        break;
    case 3:
        next = old ^ next;
    }

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CMD)
    {
        next &= 0xc000fdff;
        if (next & (EFR32_RAC_CMD_TXDIS | EFR32_RAC_CMD_RXDIS))
        {
            efr32_rac_cancel_tx_completion(s);
            timer_del(s->rac_rx_timer);
            efr32_rac_set_state(s, EFR32_RAC_STATE_OFF);
        }
        else if (next & EFR32_RAC_CMD_TXEN)
        {
            timer_del(s->rac_rx_timer);
            efr32_rac_capture_tx_snapshot(s);
            efr32_rac_set_state(s, EFR32_RAC_STATE_TX);
            efr32_rac_schedule_tx_completion(s);
        }
        trace_efr32bg22_rac_command(next, s->rac_state);
        return;
    }
    if ((register_address & ~0x3fffu) == EFR32_BUFC_BASE &&
        efr32_bufc_decode(register_address & 0xfff, &channel, &register_offset))
    {
        switch (register_offset)
        {
        case EFR32_BUFC_WRITEDATA:
            efr32_bufc_write_data_port(s, channel, next, false);
            return;
        case EFR32_BUFC_XWRITEDATA:
            efr32_bufc_write_data_port(s, channel, next, true);
            return;
        case EFR32_BUFC_CMD:
            efr32_register_put(s, register_address, next & 0xf);
            if (next & 1)
            {
                efr32_bufc_clear(s, channel);
            }
            return;
        default:
            next = efr32_bufc_mask_write(register_offset, next);
            break;
        }
    }

    /* FRC drives RXRAWDATA; the captured sequencer only reads it. */
    if (register_address == EFR32_FRC_BASE + EFR32_FRC_RXRAWDATA)
    {
        return;
    }

    /* AGC RSSI registers are generated-model read-only status values. */
    if (register_address == EFR32_AGC_BASE + EFR32_AGC_RSSI ||
        register_address == EFR32_AGC_BASE + EFR32_AGC_FRAMERSSI)
    {
        return;
    }

    /* RSSISTART is accepted, but this model has no RF input to sample. */
    if (register_address == EFR32_AGC_BASE + EFR32_AGC_CMD)
    {
        return;
    }

    efr32_register_put(s, register_address,
                       efr32_radio_mask_write(register_address, next));

    /* A new HFXO request starts a fresh lock sequence. */
    if ((register_address == EFR32_HFXO_BASE + EFR32_HFXO_CTRL ||
         register_address == EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL) &&
        !hfxo_was_enabled && efr32_hfxo_enabled(s))
    {
        s->hfxo_fsm_locked = true;
    }

    if (register_address == EFR32_HFXO_BASE + EFR32_HFXO_CMD &&
        (next & EFR32_HFXO_CMD_MANUALOVERRIDE))
    {
        s->hfxo_fsm_locked = false;
    }

    /* CTRL.FORCEDISABLE cancels an active RAC operation. */
    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CTRL &&
        (next & EFR32_RAC_CTRL_FORCEDISABLE))
    {
        efr32_rac_cancel_tx_completion(s);
        timer_del(s->rac_rx_timer);
        efr32_rac_set_state(s, EFR32_RAC_STATE_OFF);
    }

    /*
     * RAIL_StartRx writes SWRXEN through the RXENSRCEN SET alias.  The
     * hardware sequencer has many intermediate states; RXSEARCH is the
     * first stable receive state that its public register model exposes.
     */
    if (register_address == EFR32_RAC_BASE + EFR32_RAC_RXENSRCEN &&
        ((word >> 12) & 3) == 1 &&
        (next & EFR32_RAC_RXENSRCEN_SWRXEN))
    {
        /* RX replaces this model's active synthetic TX operation. */
        efr32_rac_cancel_tx_completion(s);
        efr32_rac_set_state(s, EFR32_RAC_STATE_RXSEARCH);
        if (s->rx_offer_pending) {
            efr32_rac_schedule_rx_handoff(s);
        }
    }

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_IF ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_IEN ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_SEQIF ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_SEQIEN)
    {
        efr32_rac_update_irqs(s);
    }

    if (register_address == EFR32_BUFC_BASE + EFR32_BUFC_IF ||
        register_address == EFR32_BUFC_BASE + EFR32_BUFC_IEN)
    {
        efr32_bufc_update_irq(s);
    }

    if (register_address == EFR32_FRC_BASE + EFR32_FRC_IF ||
        register_address == EFR32_FRC_BASE + EFR32_FRC_IEN)
    {
        efr32_frc_update_irq(s);
    }

    if (register_address == EFR32_AGC_BASE + EFR32_AGC_IF ||
        register_address == EFR32_AGC_BASE + EFR32_AGC_IEN)
    {
        efr32_agc_update_irq(s);
    }

    if ((register_address & ~0x3fffu) == EFR32_BUFC_BASE &&
        efr32_bufc_decode(register_address & 0xfff, &channel,
                          &register_offset) &&
        (register_offset == 0 || register_offset == EFR32_BUFC_WRITEOFFSET ||
         register_offset == EFR32_BUFC_READOFFSET ||
         register_offset == EFR32_BUFC_THRESHOLD))
    {
        efr32_bufc_update_threshold(s, channel);
    }
}
static uint64_t efr32_peripheral_mmio_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    return efr32_peripheral_read(EFR32_PERIPH(opaque), PERIPH_BASE + offset,
                                 size);
}
static void efr32_peripheral_mmio_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size)
{
    efr32_peripheral_write(EFR32_PERIPH(opaque), PERIPH_BASE + offset, value,
                           size);
}
static uint64_t efr32_radio_mmio_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    return efr32_peripheral_read(EFR32_PERIPH(opaque), RADIO_BASE + offset,
                                 size);
}
static void efr32_radio_mmio_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
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
static void efr32_periph_init(Object *o)
{
    Efr32Periph *s = EFR32_PERIPH(o);
    s->registers = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->trng_state = 0x2545f491;
    s->rac_state = EFR32_RAC_STATE_OFF;
    s->hfxo_fsm_locked = true;
    s->rx_offer_pending = false;
    s->protimer_clk = qdev_init_clock_in(DEVICE(o), "protimerclk",
                                         efr32_protimer_clock_update, s,
                                         ClockUpdate);
    s->protimer_counter = ptimer_init(efr32_protimer_counter_expired, s,
                                      PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                                      PTIMER_POLICY_NO_COUNTER_ROUND_DOWN |
                                      PTIMER_POLICY_TRIGGER_ONLY_ON_DECREMENT);
    s->rac_tx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_tx_complete, s);
    s->rac_rx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_rx_handoff, s);
    for (unsigned channel = 0; channel < G_N_ELEMENTS(s->bufc_data); channel++)
    {
        s->bufc_data[channel] = g_byte_array_new();
    }
    memory_region_init_io(&s->peripheral_mmio, o, &po, s, "efr32bg22-periph",
                          PERIPH_SIZE);
    memory_region_init_io(&s->radio_mmio, o, &ro, s, "efr32bg22-radio",
                          RADIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(o), &s->peripheral_mmio);
    sysbus_init_mmio(SYS_BUS_DEVICE(o), &s->radio_mmio);
    for (unsigned irq = 0; irq < G_N_ELEMENTS(s->irq); irq++)
    {
        sysbus_init_irq(SYS_BUS_DEVICE(o), &s->irq[irq]);
    }
}
static void efr32_periph_reset_hold(Object *o, ResetType t)
{
    Efr32Periph *s = EFR32_PERIPH(o);
    g_hash_table_remove_all(s->registers);
    s->trng_state = 0x2545f491;
    s->rac_state = EFR32_RAC_STATE_OFF;
    s->hfxo_fsm_locked = true;
    ptimer_transaction_begin(s->protimer_counter);
    ptimer_stop(s->protimer_counter);
    ptimer_set_period_from_clock(s->protimer_counter, s->protimer_clk, 1);
    ptimer_set_limit(s->protimer_counter, UINT32_MAX, 1);
    ptimer_run(s->protimer_counter, 0);
    ptimer_transaction_commit(s->protimer_counter);
    efr32_rac_cancel_tx_completion(s);
    timer_del(s->rac_rx_timer);
    s->rx_offer_pending = false;
    efr32_rf_medium_reset_radio(s->rf_medium, EFR32_RF_RADIO(s));
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL,
                       EFR32_CMU_SYSCLKCTRL_CLKSEL_FSRCO);
    efr32_register_put(s, EFR32_HFXO_BASE + EFR32_HFXO_CTRL,
                       EFR32_HFXO_CTRL_DISONDEMAND);
    efr32_register_put(s, EFR32_EMU_BASE + EFR32_EMU_STATUS,
                       EFR32_EMU_STATUS_RESET);
    efr32_register_put(s, EFR32_MSC_BASE + EFR32_MSC_STATUS,
                       EFR32_MSC_STATUS_RESET);
    efr32_register_put(s, EFR32_AGC_BASE + EFR32_AGC_RSSI, 0x8000);
    efr32_register_put(s, EFR32_AGC_BASE + EFR32_AGC_FRAMERSSI, 0x8000);
    for (unsigned irq = 0; irq < G_N_ELEMENTS(s->irq); irq++)
    {
        qemu_set_irq(s->irq[irq], 0);
    }
    for (unsigned channel = 0; channel < G_N_ELEMENTS(s->bufc_data); channel++)
    {
        g_byte_array_set_size(s->bufc_data[channel], 0);
    }
}

static void efr32_periph_finalize(Object *obj)
{
    Efr32Periph *s = EFR32_PERIPH(obj);

    timer_free(s->rac_rx_timer);
    timer_free(s->rac_tx_timer);
    ptimer_free(s->protimer_counter);
    for (unsigned channel = 0; channel < G_N_ELEMENTS(s->bufc_data); channel++) {
        g_byte_array_unref(s->bufc_data[channel]);
    }
    g_hash_table_destroy(s->registers);
}

static bool efr32_periph_is_rx_ready(Efr32RfRadio *radio)
{
    Efr32Periph *s = EFR32_PERIPH(radio);

    return s->rac_state == EFR32_RAC_STATE_RXSEARCH;
}

static void efr32_periph_rx_offer_available(Efr32RfRadio *radio)
{
    Efr32Periph *s = EFR32_PERIPH(radio);

    /* Delivery occurs from the virtual-clock callback, never in TXEN's path. */
    s->rx_offer_pending = true;
    efr32_rac_schedule_rx_handoff(s);
}

static void efr32_periph_class_init(ObjectClass *o, void *d)
{
    Efr32RfRadioClass *radio_class = EFR32_RF_RADIO_CLASS(o);

    RESETTABLE_CLASS(o)->phases.hold = efr32_periph_reset_hold;
    radio_class->is_rx_ready = efr32_periph_is_rx_ready;
    radio_class->rx_offer_available = efr32_periph_rx_offer_available;
}
static const TypeInfo periph_info = {.name = TYPE_EFR32_PERIPH,
                                     .parent = TYPE_SYS_BUS_DEVICE,
                                     .instance_size = sizeof(Efr32Periph),
                                     .instance_init = efr32_periph_init,
                                     .instance_finalize = efr32_periph_finalize,
                                     .class_init = efr32_periph_class_init,
                                     .interfaces =
                                         (InterfaceInfo[]){{
                                             TYPE_EFR32_RF_RADIO,
                                         },
                                                            {}}};

/*
 * The SoC owns the guest-visible address map.  The board supplies its root
 * MemoryRegion.  This lets a future test board give two EFR32 instances
 * independent same-address maps while sharing an in-process RF medium.
 */
#define TYPE_EFR32BG22_SOC "efr32bg22-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Efr32Bg22SoCState, EFR32BG22_SOC)

struct Efr32Bg22SoCState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;
    Efr32Periph periph;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion radio_ns;
    MemoryRegion radio_ram;
    MemoryRegion radio_ram_ns;
    MemoryRegion devinfo;
    Clock *clk;
    Clock *protimer_clk;

    MemoryRegion *memory;
    Efr32RfMedium *rf_medium;
    Object *idau;
};

struct Efr32State
{
    MachineState parent_obj;
    Efr32Bg22SoCState soc;
    Efr32RfMedium *rf_medium;
    char *image;
};
#define TYPE_EFR32BG22_MACHINE MACHINE_TYPE_NAME("efr32bg22")
OBJECT_DECLARE_SIMPLE_TYPE(Efr32State, EFR32BG22_MACHINE)

/*
 * Test-only topology: two complete EFR32 address spaces joined by one
 * in-process RF medium.  The peer map starts above the 32-bit address range,
 * so qtest can drive both MCUs without inventing a host packet ingress.
 */
#define TYPE_EFR32BG22_DUAL_MACHINE MACHINE_TYPE_NAME("efr32bg22-dual")
#define EFR32BG22_PEER_PERIPH_BASE 0x60000000u
#define EFR32BG22_PEER_RADIO_BASE 0x80000000u
OBJECT_DECLARE_SIMPLE_TYPE(Efr32DualState, EFR32BG22_DUAL_MACHINE)

struct Efr32DualState {
    MachineState parent_obj;
    Efr32Bg22SoCState soc[2];
    MemoryRegion memory[2];
    MemoryRegion peer_periph;
    MemoryRegion peer_radio;
    Efr32RfMedium *rf_medium;
};

/* ARMv7M CPUs require an explicit board reset registration. */
static void efr32_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void efr32_initialize_devinfo(MemoryRegion *devinfo)
{
    uint8_t *contents = memory_region_get_ram_ptr(devinfo);
    uint32_t value;

    /* USERDATA/CHIPCONFIG are erased.  DEVINFO holds only public size data
     * and an intentionally synthetic EUI-64; calibration words stay zero. */
    memset(contents, 0xff, 0x8000);
    memset(contents + 0x8000, 0, 0x2000);
    value = cpu_to_le32(512 | (32 << 16));
    memcpy(contents + 0x800c, &value, sizeof(value));
    value = cpu_to_le32(0x0000beef);
    memcpy(contents + 0x8048, &value, sizeof(value));
    value = cpu_to_le32(0x00000a00);
    memcpy(contents + 0x804c, &value, sizeof(value));
}

static void efr32_soc_init(Object *obj)
{
    Efr32Bg22SoCState *s = EFR32BG22_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "periph", &s->periph, TYPE_EFR32_PERIPH);
}

static void efr32_soc_realize(DeviceState *dev, Error **errp)
{
    Efr32Bg22SoCState *s = EFR32BG22_SOC(dev);
    DeviceState *nvic = DEVICE(&s->armv7m);
    SysBusDevice *periph = SYS_BUS_DEVICE(&s->periph);
    g_autofree gchar *name = object_get_canonical_path_component(OBJECT(s));
    g_autofree gchar *flash_name = NULL;
    g_autofree gchar *sram_name = NULL;
    g_autofree gchar *radio_ram_name = NULL;
    g_autofree gchar *devinfo_name = NULL;

    if (name && strcmp(name, "soc")) {
        flash_name = g_strdup_printf("efr32.%s.flash", name);
        sram_name = g_strdup_printf("efr32.%s.sram", name);
        radio_ram_name = g_strdup_printf("efr32.%s.radioram", name);
        devinfo_name = g_strdup_printf("efr32.%s.devinfo", name);
    } else {
        flash_name = g_strdup("efr32.flash");
        sram_name = g_strdup("efr32.sram");
        radio_ram_name = g_strdup("efr32.radioram");
        devinfo_name = g_strdup("efr32.devinfo");
    }

    if (!s->memory) {
        error_setg(errp, "memory property was not set");
        return;
    }
    if (!s->rf_medium) {
        error_setg(errp, "rf-medium property was not set");
        return;
    }
    if (!s->idau) {
        error_setg(errp, "idau property was not set");
        return;
    }

    s->clk = clock_new(OBJECT(s), "SYSCLK");
    clock_set_hz(s->clk, 38400000);
    s->protimer_clk = clock_new(OBJECT(s), "PROTIMERCLK");
    clock_set_hz(s->protimer_clk, 2000000);

    memory_region_init_ram(&s->flash, OBJECT(s), flash_name, FLASH_SIZE,
                           &error_fatal);
    memory_region_add_subregion(s->memory, 0, &s->flash);
    memory_region_init_ram(&s->sram, OBJECT(s), sram_name, SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(s->memory, 0x20000000, &s->sram);

    qdev_prop_set_uint32(nvic, "num-irq", 64);
    qdev_connect_clock_in(nvic, "cpuclk", s->clk);
    qdev_prop_set_string(nvic, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    object_property_set_link(OBJECT(&s->armv7m), "idau", s->idau,
                             &error_abort);
    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(s->memory),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    s->periph.rf_medium = s->rf_medium;
    qdev_connect_clock_in(DEVICE(&s->periph), "protimerclk", s->protimer_clk);
    if (!sysbus_realize(periph, errp)) {
        return;
    }
    efr32_rf_medium_attach(s->rf_medium, EFR32_RF_RADIO(&s->periph), errp);
    if (*errp) {
        return;
    }
    memory_region_add_subregion(s->memory, PERIPH_BASE,
                                sysbus_mmio_get_region(periph, 0));
    memory_region_add_subregion(s->memory, RADIO_BASE,
                                sysbus_mmio_get_region(periph, 1));
    for (unsigned irq = 0; irq < 64; irq++) {
        sysbus_connect_irq(periph, irq, qdev_get_gpio_in(nvic, irq));
    }

    memory_region_init_alias(&s->radio_ns, OBJECT(s), "radio-ns",
                             sysbus_mmio_get_region(periph, 1), 0, RADIO_SIZE);
    memory_region_add_subregion(s->memory, 0xb8000000, &s->radio_ns);
    memory_region_init_ram(&s->radio_ram, OBJECT(s), radio_ram_name, 0x8000,
                           &error_fatal);
    memory_region_add_subregion(s->memory, 0xa0000000, &s->radio_ram);
    memory_region_init_alias(&s->radio_ram_ns, OBJECT(s), "efr32.radioram-ns",
                             &s->radio_ram, 0, 0x8000);
    memory_region_add_subregion(s->memory, 0xb0000000, &s->radio_ram_ns);
    memory_region_init_ram(&s->devinfo, OBJECT(s), devinfo_name, 0x10000,
                           &error_fatal);
    memory_region_add_subregion(s->memory, 0x0fe00000, &s->devinfo);
    efr32_initialize_devinfo(&s->devinfo);
}

static Property efr32_soc_properties[] = {
    DEFINE_PROP_LINK("memory", Efr32Bg22SoCState, memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("rf-medium", Efr32Bg22SoCState, rf_medium,
                     TYPE_EFR32_RF_MEDIUM, Efr32RfMedium *),
    DEFINE_PROP_LINK("idau", Efr32Bg22SoCState, idau, TYPE_IDAU_INTERFACE,
                     Object *),
    DEFINE_PROP_END_OF_LIST(),
};

static void efr32_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = efr32_soc_realize;
    device_class_set_props(dc, efr32_soc_properties);
}

static const TypeInfo efr32_soc_info = {
    .name = TYPE_EFR32BG22_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32Bg22SoCState),
    .instance_init = efr32_soc_init,
    .class_init = efr32_soc_class_init,
};

static void init(MachineState *m)
{
    Efr32State *s = EFR32BG22_MACHINE(m);
    s->rf_medium = EFR32_RF_MEDIUM(object_new(TYPE_EFR32_RF_MEDIUM));
    object_property_add_child(OBJECT(m), "rf-medium", OBJECT(s->rf_medium));
    object_initialize_child(OBJECT(m), "soc", &s->soc, TYPE_EFR32BG22_SOC);
    object_property_set_link(OBJECT(&s->soc), "memory",
                             OBJECT(get_system_memory()), &error_fatal);
    object_property_set_link(OBJECT(&s->soc), "rf-medium", OBJECT(s->rf_medium),
                             &error_fatal);
    object_property_set_link(OBJECT(&s->soc), "idau", OBJECT(m), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);
    if (s->image)
    {
        gchar *d;
        gsize n;
        if (!g_file_get_contents(s->image, &d, &n, NULL) || n > FLASH_SIZE)
        {
            error_report("efr32bg22: cannot load flash image %s", s->image);
            exit(EXIT_FAILURE);
        }
        memcpy(memory_region_get_ram_ptr(&s->soc.flash), d, n);
        g_free(d);
    }
    qemu_register_reset(efr32_cpu_reset, s->soc.armv7m.cpu);
}
static void idau(IDAUInterface *i, uint32_t a, int *r, bool *e, bool *n,
                 bool *c)
{
    *r = extract32(a, 28, 4);
    *e = (a & 0xeff00000) == 0xe0000000;
    *n = *r == 5 || *r == 0xb;
    *c = false;
}
static char *image_get(Object *o, Error **e)
{
    return g_strdup(EFR32BG22_MACHINE(o)->image);
}
static void image_set(Object *o, const char *v, Error **e)
{
    Efr32State *s = EFR32BG22_MACHINE(o);
    g_free(s->image);
    s->image = g_strdup(v);
}
static void mc(ObjectClass *o, void *d)
{
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

static void dual_init(MachineState *m)
{
    Efr32DualState *s = EFR32BG22_DUAL_MACHINE(m);

    s->rf_medium = EFR32_RF_MEDIUM(object_new(TYPE_EFR32_RF_MEDIUM));
    object_property_add_child(OBJECT(m), "rf-medium", OBJECT(s->rf_medium));
    for (unsigned index = 0; index < G_N_ELEMENTS(s->soc); index++) {
        gchar *name = g_strdup_printf("soc%u", index);
        gchar *memory_name = g_strdup_printf("efr32bg22-dual.memory%u", index);

        memory_region_init(&s->memory[index], OBJECT(m), memory_name,
                           UINT64_C(1) << 32);
        object_initialize_child(OBJECT(m), name, &s->soc[index],
                                TYPE_EFR32BG22_SOC);
        object_property_set_link(OBJECT(&s->soc[index]), "memory",
                                 OBJECT(&s->memory[index]), &error_fatal);
        object_property_set_link(OBJECT(&s->soc[index]), "rf-medium",
                                 OBJECT(s->rf_medium), &error_fatal);
        object_property_set_link(OBJECT(&s->soc[index]), "idau", OBJECT(m),
                                 &error_fatal);
        sysbus_realize(SYS_BUS_DEVICE(&s->soc[index]), &error_fatal);
        qemu_register_reset(efr32_cpu_reset, s->soc[index].armv7m.cpu);
        g_free(memory_name);
        g_free(name);
    }
    /*
     * The second CPU retains its complete private ARMv7M address space.  QTest
     * needs a system-bus route to configure its radio, so expose only aliases
     * of its native peripheral windows at non-overlapping test addresses.
     */
    memory_region_add_subregion(get_system_memory(), 0,
                                &s->soc[0].armv7m.container);
    memory_region_init_alias(&s->peer_periph, OBJECT(m), "peer-periph",
                             sysbus_mmio_get_region(
                                 SYS_BUS_DEVICE(&s->soc[1].periph), 0),
                             0, PERIPH_SIZE);
    memory_region_add_subregion(&s->soc[0].armv7m.container,
                                EFR32BG22_PEER_PERIPH_BASE, &s->peer_periph);
    memory_region_init_alias(&s->peer_radio, OBJECT(m), "peer-radio",
                             sysbus_mmio_get_region(
                                 SYS_BUS_DEVICE(&s->soc[1].periph), 1),
                             0, RADIO_SIZE);
    memory_region_add_subregion(&s->soc[0].armv7m.container,
                                EFR32BG22_PEER_RADIO_BASE, &s->peer_radio);
}

static void dual_mc(ObjectClass *o, void *d)
{
    MachineClass *m = MACHINE_CLASS(o);
    IDAUInterfaceClass *i = IDAU_INTERFACE_CLASS(o);
    static const char *const cpu[] = {ARM_CPU_TYPE_NAME("cortex-m33"), NULL};

    i->check = idau;
    m->desc = "EFR32BG22 dual-MCU RF test topology";
    m->init = dual_init;
    m->valid_cpu_types = cpu;
    m->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    m->default_ram_size = 2 * FLASH_SIZE;
    m->min_cpus = 2;
    m->max_cpus = 2;
    m->default_cpus = 2;
    m->ignore_memory_transaction_failures = true;
}

static const TypeInfo dual_machine_info = {
    .name = TYPE_EFR32BG22_DUAL_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Efr32DualState),
    .class_init = dual_mc,
    .interfaces = (InterfaceInfo[]){{TYPE_IDAU_INTERFACE}, {}}};
static void register_types(void)
{
    efr32_rf_medium_register_types();
    type_register_static(&periph_info);
    type_register_static(&efr32_soc_info);
    type_register_static(&machine_info);
    type_register_static(&dual_machine_info);
}
type_init(register_types)
