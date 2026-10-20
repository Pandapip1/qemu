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
#define PERIPH_NONSECURE_BASE 0x50000000u
#define PERIPH_NONSECURE_END 0x60000000u
#define PERIPH_NONSECURE_OFFSET (PERIPH_NONSECURE_BASE - PERIPH_BASE)
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
#define EFR32_PROTIMER_IF 0x064u
#define EFR32_PROTIMER_IEN 0x070u
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

/* EFR32BG22 DPLL0, used by Zephyr's CMU_DPLLLock() startup sequence. */
#define EFR32_DPLL0_BASE 0x4001c000u
#define EFR32_DPLL_EN 0x004u
#define EFR32_DPLL_CFG 0x008u
#define EFR32_DPLL_CFG1 0x00cu
#define EFR32_DPLL_IF 0x010u
#define EFR32_DPLL_IEN 0x014u
#define EFR32_DPLL_STATUS 0x018u
#define EFR32_DPLL_EN_EN (1u << 0)
#define EFR32_DPLL_IF_LOCK (1u << 0)
#define EFR32_DPLL_STATUS_RDY (1u << 0)
#define EFR32_DPLL_STATUS_ENS (1u << 1)
#define EFR32_DPLL_STATUS_LOCK (1u << 31)

#define EFR32_DCDC_BASE 0x40094000u
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

/* EFR32BG22 TIMER4: public Series-2 register layout, IRQ 11. */
#define EFR32_TIMER4_BASE 0x40058000u
#define EFR32_TIMER_EN 0x030u
#define EFR32_TIMER_CMD 0x00cu
#define EFR32_TIMER_STATUS 0x010u
#define EFR32_TIMER_IF 0x014u
#define EFR32_TIMER_IEN 0x018u
#define EFR32_TIMER_TOP 0x01cu
#define EFR32_TIMER_CNT 0x024u
#define EFR32_TIMER_CC0_OC 0x068u
#define EFR32_TIMER_EN_EN (1u << 0)
#define EFR32_TIMER_CMD_START (1u << 0)
#define EFR32_TIMER_CMD_STOP (1u << 1)
#define EFR32_TIMER_STATUS_RUNNING (1u << 0)
#define EFR32_TIMER_IF_CC0 (1u << 4)
#define EFR32_TIMER4_HZ 38400000ULL

/*
 * EFR32BG22 USART0/1.  This intentionally stops at the register-facing
 * transmit path used by Zephyr's polling console: bytes are accepted by the
 * peripheral, but are not forwarded to a host character device.
 */
#define EFR32_USART0_BASE 0x4005c000u
#define EFR32_USART1_BASE 0x40060000u
#define EFR32_USART_CMD 0x014u
#define EFR32_USART_STATUS 0x018u
#define EFR32_USART_TXDATA 0x03cu
#define EFR32_USART_CMD_TXEN (1u << 2)
#define EFR32_USART_CMD_TXDIS (1u << 3)
#define EFR32_USART_STATUS_TXENS (1u << 1)
#define EFR32_USART_STATUS_TXC (1u << 5)
#define EFR32_USART_STATUS_TXBL (1u << 6)

/* EFR32BG22 BURTC, as declared by efr32bg22_burtc.h. */
#define EFR32_BURTC_BASE 0x40064000u
#define EFR32_BURTC_EN 0x004u
#define EFR32_BURTC_CFG 0x008u
#define EFR32_BURTC_CMD 0x00cu
#define EFR32_BURTC_STATUS 0x010u
#define EFR32_BURTC_IF 0x014u
#define EFR32_BURTC_IEN 0x018u
#define EFR32_BURTC_PRECNT 0x01cu
#define EFR32_BURTC_CNT 0x020u
#define EFR32_BURTC_COMP 0x030u
#define EFR32_BURTC_EN_EN (1u << 0)
#define EFR32_BURTC_CMD_START (1u << 0)
#define EFR32_BURTC_CMD_STOP (1u << 1)
#define EFR32_BURTC_STATUS_RUNNING (1u << 0)

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
    MemoryRegion *memory;
    AddressSpace dma_as;
    bool dma_as_initialized;
    GHashTable *registers;
    uint32_t trng_state;
    uint32_t rac_state;
    bool hfxo_fsm_locked;
    bool rx_offer_pending;
    bool usart_tx_enabled[2];
    bool usart_tx_complete[2];
    Clock *protimer_clk;
    ptimer_state *protimer_counter;
    Clock *burtc_clk;
    ptimer_state *burtc_counter;
    bool burtc_running;
    QEMUTimer *burtc_compare_timer;
    uint32_t timer4_count;
    int64_t timer4_last_ns;
    bool timer4_running;
    QEMUTimer *timer4_compare_timer;
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

/*
 * The archived generic_phy FRC_IRQHandler reads PROTIMER IF and IEN, then
 * dispatches their intersection from the PROTIMER vector.  The individual
 * timer sources are not characterized yet, so this only models the observed
 * interrupt-bank contract; no counter or compare event raises a flag here.
 */
static void efr32_protimer_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s,
                                          EFR32_PROTIMER_BASE + EFR32_PROTIMER_IF);
    uint32_t enabled = efr32_register_get(s,
                                          EFR32_PROTIMER_BASE + EFR32_PROTIMER_IEN);

    qemu_set_irq(s->irq[36], !!(pending & enabled));
}

static void efr32_timer4_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_IEN);

    qemu_set_irq(s->irq[11], !!(pending & enabled));
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
        address_space_write(&s->dma_as, buffer_address + position,
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
        address_space_read(&s->dma_as, buffer_address + position,
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

        address_space_read(&s->dma_as, buffer_address + offset,
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

    /*
     * Series-2 exposes non-secure peripherals at 0x50000000..0x5fffffff;
     * these mirror the secure 0x40000000..0x4fffffff view.  Preserve the
     * 16 KiB normal/SET/CLR/TGL subwindows for the common register handler.
     */
    if (address >= PERIPH_NONSECURE_BASE &&
        address < PERIPH_NONSECURE_END) {
        address -= PERIPH_NONSECURE_OFFSET;
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

static bool efr32_dpll_enabled(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_DPLL0_BASE + EFR32_DPLL_EN) &
           EFR32_DPLL_EN_EN;
}

static int efr32_usart_index(uint32_t address)
{
    uint32_t block = address & ~0x3fffu;

    if (block == EFR32_USART0_BASE) {
        return 0;
    }
    if (block == EFR32_USART1_BASE) {
        return 1;
    }
    return -1;
}

static uint32_t efr32_usart_status(Efr32Periph *s, unsigned index)
{
    /*
     * The public register description resets TXBL high.  With no serial
     * timing or host endpoint in this model, accepted TXDATA completes at
     * the MMIO boundary, keeping the public TXC polling primitive
     * deterministic too.
     */
    return EFR32_USART_STATUS_TXBL |
           (s->usart_tx_complete[index] ? EFR32_USART_STATUS_TXC : 0) |
           (s->usart_tx_enabled[index] ? EFR32_USART_STATUS_TXENS : 0);
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

    efr32_rf_medium_complete_tx(s->rf_medium, EFR32_RF_RADIO(s));
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
    if ((address & ~0x3fffu) == EFR32_DPLL0_BASE)
    {
        switch (offset)
        {
        case EFR32_DPLL_EN:
            return value & EFR32_DPLL_EN_EN;
        case EFR32_DPLL_CFG:
            return value & 0x47;
        case EFR32_DPLL_CFG1:
            return value & 0x0fff0fff;
        case EFR32_DPLL_IF:
        case EFR32_DPLL_IEN:
            return value & 0x7;
        default:
            return value;
        }
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

static void efr32_free_running_counter_expired(void *opaque)
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

static void efr32_burtc_clock_update(void *opaque, ClockEvent event)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    ptimer_transaction_begin(s->burtc_counter);
    ptimer_set_period_from_clock(s->burtc_counter, s->burtc_clk, 1);
    ptimer_transaction_commit(s->burtc_counter);
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

static bool efr32_burtc_enabled(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_EN) &
           EFR32_BURTC_EN_EN;
}

static uint32_t efr32_burtc_counter(Efr32Periph *s)
{
    return UINT32_MAX - ptimer_get_count(s->burtc_counter);
}

static void efr32_burtc_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_IEN);

    /* xG22.dtsi and the part header both assign BURTC to external IRQ 18. */
    qemu_set_irq(s->irq[18], !!(pending & enabled));
}

static void efr32_burtc_schedule_compare(Efr32Periph *s)
{
    uint32_t ticks;
    int64_t deadline;

    timer_del(s->burtc_compare_timer);
    if (!s->burtc_running || !efr32_burtc_enabled(s)) {
        return;
    }

    /* CNT counts up modulo 2^32.  Schedule this COMP match once; IF latches
     * until the guest clears it, exactly as the one-shot Zephyr alarm uses it. */
    ticks = efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_COMP) -
            efr32_burtc_counter(s);
    if (ticks == 0) {
        ticks = 1;
    }
    deadline = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
               ((int64_t)ticks * NANOSECONDS_PER_SECOND + 32768 - 1) / 32768;
    timer_mod(s->burtc_compare_timer, deadline);
}

static void efr32_burtc_compare_expired(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    if (!s->burtc_running || !efr32_burtc_enabled(s)) {
        return;
    }
    efr32_register_put(s, EFR32_BURTC_BASE + EFR32_BURTC_IF,
                       efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_IF) |
                           (1u << 1)); /* BURTC_IF_COMP */
    efr32_burtc_update_irq(s);
}

static void efr32_burtc_update_running(Efr32Periph *s)
{
    ptimer_transaction_begin(s->burtc_counter);
    if (s->burtc_running && efr32_burtc_enabled(s)) {
        ptimer_run(s->burtc_counter, 0);
    } else {
        ptimer_stop(s->burtc_counter);
    }
    ptimer_transaction_commit(s->burtc_counter);
    efr32_burtc_schedule_compare(s);
}

static void efr32_burtc_set_counter(Efr32Periph *s, uint32_t value)
{
    bool run = s->burtc_running && efr32_burtc_enabled(s);

    ptimer_transaction_begin(s->burtc_counter);
    ptimer_stop(s->burtc_counter);
    ptimer_set_count(s->burtc_counter, UINT32_MAX - value);
    if (run) {
        ptimer_run(s->burtc_counter, 0);
    }
    ptimer_transaction_commit(s->burtc_counter);
    efr32_burtc_schedule_compare(s);
}

static bool efr32_timer4_enabled(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_EN) &
           EFR32_TIMER_EN_EN;
}

/* TIMER4 is a 16-bit up-counter, clocked from a dedicated 38.4 MHz source. */
static void efr32_timer4_update_counter(Efr32Periph *s)
{
    int64_t now;
    uint64_t ticks;
    uint32_t top;
    uint64_t period;

    if (!s->timer4_running || !efr32_timer4_enabled(s)) {
        return;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ticks = ((uint64_t)(now - s->timer4_last_ns) * EFR32_TIMER4_HZ) /
            NANOSECONDS_PER_SECOND;
    if (!ticks) {
        return;
    }

    top = efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_TOP) & 0xffff;
    period = (uint64_t)top + 1;
    s->timer4_count = (s->timer4_count + ticks) % period;
    s->timer4_last_ns += (ticks * NANOSECONDS_PER_SECOND) / EFR32_TIMER4_HZ;
}

static uint32_t efr32_timer4_counter(Efr32Periph *s)
{
    efr32_timer4_update_counter(s);
    return s->timer4_count;
}

static void efr32_timer4_schedule_compare(Efr32Periph *s)
{
    uint32_t top;
    uint32_t compare;
    uint64_t period;
    uint64_t ticks;
    int64_t delay_ns;

    timer_del(s->timer4_compare_timer);
    if (!s->timer4_running || !efr32_timer4_enabled(s)) {
        return;
    }

    efr32_timer4_update_counter(s);
    top = efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_TOP) & 0xffff;
    compare = efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_CC0_OC) & 0xffff;
    period = (uint64_t)top + 1;
    compare %= period;
    ticks = (compare + period - s->timer4_count) % period;
    if (!ticks) {
        ticks = period;
    }
    delay_ns = (ticks * NANOSECONDS_PER_SECOND + EFR32_TIMER4_HZ - 1) /
               EFR32_TIMER4_HZ;
    timer_mod(s->timer4_compare_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
}

static void efr32_timer4_compare_expired(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    if (!s->timer4_running || !efr32_timer4_enabled(s)) {
        return;
    }
    efr32_timer4_update_counter(s);
    efr32_register_put(s, EFR32_TIMER4_BASE + EFR32_TIMER_IF,
                       efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_IF) |
                           EFR32_TIMER_IF_CC0);
    efr32_timer4_update_irq(s);
    efr32_timer4_schedule_compare(s);
}

static void efr32_timer4_set_counter(Efr32Periph *s, uint32_t value)
{
    uint32_t top = efr32_register_get(s, EFR32_TIMER4_BASE + EFR32_TIMER_TOP) & 0xffff;

    s->timer4_count = value % ((uint64_t)top + 1);
    s->timer4_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    efr32_timer4_schedule_compare(s);
}

static uint32_t efr32_peripheral_read_word(Efr32Periph *s, uint32_t address)
{
    address = efr32_peripheral_canonical_address(address);
    uint32_t offset = address & 0xfff;
    unsigned channel;
    uint32_t register_offset;
    int usart_index = efr32_usart_index(address);

    if (usart_index >= 0 && offset == EFR32_USART_STATUS) {
        return efr32_usart_status(s, usart_index);
    }
    if (usart_index >= 0 && offset == EFR32_USART_CMD) {
        return 0; /* CMD is command strobes, not retained state. */
    }

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
    if (address == EFR32_DPLL0_BASE + EFR32_DPLL_STATUS)
    {
        return efr32_dpll_enabled(s) ? EFR32_DPLL_STATUS_RDY |
                                       EFR32_DPLL_STATUS_ENS |
                                       EFR32_DPLL_STATUS_LOCK : 0;
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
    if (address == EFR32_BURTC_BASE + EFR32_BURTC_CNT)
        return efr32_burtc_counter(s);
    if (address == EFR32_BURTC_BASE + EFR32_BURTC_STATUS)
        return s->burtc_running && efr32_burtc_enabled(s) ?
                   EFR32_BURTC_STATUS_RUNNING : 0;
    if (address == EFR32_TIMER4_BASE + EFR32_TIMER_CNT)
        return efr32_timer4_counter(s);
    if (address == EFR32_TIMER4_BASE + EFR32_TIMER_STATUS)
        return s->timer4_running && efr32_timer4_enabled(s) ?
                   EFR32_TIMER_STATUS_RUNNING : 0;
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
    bool dpll_was_enabled = efr32_dpll_enabled(s);
    int usart_index = efr32_usart_index(register_address);

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

    if (usart_index >= 0 && (register_address & 0xfff) == EFR32_USART_CMD)
    {
        /* CMD_SET is the form used by the public Zephyr/Silabs helpers. */
        if (next & EFR32_USART_CMD_TXEN) {
            s->usart_tx_enabled[usart_index] = true;
        }
        if (next & EFR32_USART_CMD_TXDIS) {
            s->usart_tx_enabled[usart_index] = false;
        }
        return;
    }

    if (usart_index >= 0 && (register_address & 0xfff) == EFR32_USART_TXDATA)
    {
        /*
         * Retain the public eight-bit data field; do not create host I/O.
         * The tracepoint is deliberately inside the native USART model, so a
         * bounded board smoke can observe its own console without a host
         * character-device shortcut or a guest-visible behaviour change.
         */
        efr32_register_put(s, register_address, next & 0xff);
        s->usart_tx_complete[usart_index] = true;
        trace_efr32bg22_usart_tx(usart_index, next & 0xff);
        return;
    }

    if ((register_address & ~0x3fffu) == EFR32_BURTC_BASE) {
        uint32_t burtc_offset = register_address & 0xfff;

        switch (burtc_offset) {
        case EFR32_BURTC_CMD:
            if (next & EFR32_BURTC_CMD_STOP) {
                s->burtc_running = false;
            } else if (next & EFR32_BURTC_CMD_START) {
                s->burtc_running = true;
            }
            efr32_burtc_update_running(s);
            return;
        case EFR32_BURTC_CNT:
            efr32_burtc_set_counter(s, next);
            return;
        case EFR32_BURTC_STATUS:
            return; /* status is generated from EN and CMD state. */
        case EFR32_BURTC_EN:
            next &= EFR32_BURTC_EN_EN;
            break;
        case EFR32_BURTC_CFG:
            next &= 0xf3;
            break;
        case EFR32_BURTC_IF:
        case EFR32_BURTC_IEN:
            next &= 0x3;
            break;
        case EFR32_BURTC_PRECNT:
            next &= 0x7fff;
            break;
        default:
            break;
        }
    }

    if ((register_address & ~0x3fffu) == EFR32_TIMER4_BASE) {
        uint32_t timer_offset = register_address & 0xfff;

        efr32_timer4_update_counter(s);
        switch (timer_offset) {
        case EFR32_TIMER_CMD:
            if (next & EFR32_TIMER_CMD_STOP) {
                s->timer4_running = false;
                timer_del(s->timer4_compare_timer);
            } else if (next & EFR32_TIMER_CMD_START) {
                s->timer4_running = true;
                s->timer4_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                efr32_timer4_schedule_compare(s);
            }
            return; /* CMD is write-only strobes. */
        case EFR32_TIMER_CNT:
            efr32_timer4_set_counter(s, next & 0xffff);
            return;
        case EFR32_TIMER_STATUS:
            return; /* status is generated from EN and CMD state. */
        case EFR32_TIMER_EN:
            next &= EFR32_TIMER_EN_EN;
            break;
        case EFR32_TIMER_TOP:
        case EFR32_TIMER_CC0_OC:
            next &= 0xffff;
            break;
        case EFR32_TIMER_IF:
        case EFR32_TIMER_IEN:
            next &= EFR32_TIMER_IF_CC0;
            break;
        default:
            break;
        }
    }

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CMD)
    {
        next &= 0xc000fdff;
        if (next & (EFR32_RAC_CMD_TXDIS | EFR32_RAC_CMD_RXDIS))
        {
            efr32_rac_cancel_tx_completion(s);
            timer_del(s->rac_rx_timer);
            /*
             * TXDIS aborts the in-flight model transaction as well as its
             * synthetic completion timer.  Do not let a TXEN mailbox offer
             * survive an explicit source abort and turn into a later RXDONE.
             */
            if (next & EFR32_RAC_CMD_TXDIS) {
                efr32_rf_medium_withdraw_active_tx(
                    s->rf_medium, EFR32_RF_RADIO(s));
            }
            efr32_rac_set_state(s, EFR32_RAC_STATE_OFF);
        }
        else if (next & EFR32_RAC_CMD_TXEN)
        {
            timer_del(s->rac_rx_timer);
            /*
             * This bounded model represents at most one active TXEN
             * transaction per radio.  A second TXEN replaces the existing
             * transaction, so withdraw its still-deferred mailbox offers
             * before publishing the new BUFC snapshot.  Otherwise the old
             * snapshot is freed below but its queued RXDONE can survive and
             * be delivered after the replacement transmission.
             */
            efr32_rf_medium_withdraw_active_tx(s->rf_medium,
                                                EFR32_RF_RADIO(s));
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

    if (register_address == EFR32_BURTC_BASE + EFR32_BURTC_EN) {
        efr32_burtc_update_running(s);
    }
    if (register_address == EFR32_BURTC_BASE + EFR32_BURTC_COMP) {
        efr32_burtc_schedule_compare(s);
    }
    if (register_address == EFR32_BURTC_BASE + EFR32_BURTC_IF ||
        register_address == EFR32_BURTC_BASE + EFR32_BURTC_IEN) {
        efr32_burtc_update_irq(s);
    }
    if (register_address == EFR32_TIMER4_BASE + EFR32_TIMER_EN) {
        if (efr32_timer4_enabled(s)) {
            s->timer4_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        efr32_timer4_schedule_compare(s);
    }
    if (register_address == EFR32_TIMER4_BASE + EFR32_TIMER_TOP ||
        register_address == EFR32_TIMER4_BASE + EFR32_TIMER_CC0_OC) {
        efr32_timer4_schedule_compare(s);
    }
    if (register_address == EFR32_TIMER4_BASE + EFR32_TIMER_IF ||
        register_address == EFR32_TIMER4_BASE + EFR32_TIMER_IEN) {
        efr32_timer4_update_irq(s);
    }

    /*
     * The public CMU_DPLLLock() path clears DPLL.IF then enables DPLL0 and
     * polls for LOCK or a LOCKFAIL flag.  This model has no analog loop or
     * reference-clock timing, so an enable transition completes immediately
     * and deterministically with the documented successful-lock state.
     */
    if (register_address == EFR32_DPLL0_BASE + EFR32_DPLL_EN &&
        !dpll_was_enabled && efr32_dpll_enabled(s))
    {
        efr32_register_put(s, EFR32_DPLL0_BASE + EFR32_DPLL_IF,
                           efr32_register_get(s, EFR32_DPLL0_BASE +
                                              EFR32_DPLL_IF) |
                               EFR32_DPLL_IF_LOCK);
    }

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
        /*
         * FORCEDISABLE is an explicit abort just like TXDIS.  Withdraw the
         * in-flight model transaction so a deferred peer handoff cannot turn
         * a cancelled transmission into RXDONE.
         */
        efr32_rf_medium_withdraw_active_tx(s->rf_medium, EFR32_RF_RADIO(s));
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
        /*
         * Just like TXDIS and FORCEDISABLE, changing the sequencer from TX
         * to RX aborts the synthetic transaction represented by the latest
         * TXEN snapshot.  Do not let its deferred peer offer escape after
         * this radio has stopped transmitting.
         */
        efr32_rf_medium_withdraw_active_tx(s->rf_medium, EFR32_RF_RADIO(s));
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

    if (register_address == EFR32_PROTIMER_BASE + EFR32_PROTIMER_IF ||
        register_address == EFR32_PROTIMER_BASE + EFR32_PROTIMER_IEN)
    {
        efr32_protimer_update_irq(s);
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

static void efr32_periph_realize(DeviceState *dev, Error **errp)
{
    Efr32Periph *s = EFR32_PERIPH(dev);

    if (!s->memory) {
        error_setg(errp, "memory property was not set");
        return;
    }

    address_space_init(&s->dma_as, s->memory, "efr32-bufc-dma");
    s->dma_as_initialized = true;
}

static void efr32_periph_init(Object *o)
{
    Efr32Periph *s = EFR32_PERIPH(o);
    s->registers = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->trng_state = 0x2545f491;
    s->rac_state = EFR32_RAC_STATE_OFF;
    s->hfxo_fsm_locked = true;
    s->rx_offer_pending = false;
    s->usart_tx_enabled[0] = false;
    s->usart_tx_enabled[1] = false;
    s->usart_tx_complete[0] = false;
    s->usart_tx_complete[1] = false;
    s->protimer_clk = qdev_init_clock_in(DEVICE(o), "protimerclk",
                                         efr32_protimer_clock_update, s,
                                         ClockUpdate);
    s->protimer_counter = ptimer_init(efr32_free_running_counter_expired, s,
                                      PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                                      PTIMER_POLICY_NO_COUNTER_ROUND_DOWN |
                                      PTIMER_POLICY_TRIGGER_ONLY_ON_DECREMENT);
    s->burtc_clk = qdev_init_clock_in(DEVICE(o), "burtcclk",
                                      efr32_burtc_clock_update, s,
                                      ClockUpdate);
    s->burtc_counter = ptimer_init(efr32_free_running_counter_expired, s,
                                   PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                                   PTIMER_POLICY_NO_COUNTER_ROUND_DOWN |
                                   PTIMER_POLICY_TRIGGER_ONLY_ON_DECREMENT);
    s->burtc_compare_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_burtc_compare_expired, s);
    s->timer4_compare_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_timer4_compare_expired, s);
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
    s->usart_tx_enabled[0] = false;
    s->usart_tx_enabled[1] = false;
    s->usart_tx_complete[0] = false;
    s->usart_tx_complete[1] = false;
    s->burtc_running = false;
    timer_del(s->burtc_compare_timer);
    s->timer4_running = false;
    s->timer4_count = 0;
    s->timer4_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_del(s->timer4_compare_timer);
    ptimer_transaction_begin(s->protimer_counter);
    ptimer_stop(s->protimer_counter);
    ptimer_set_period_from_clock(s->protimer_counter, s->protimer_clk, 1);
    ptimer_set_limit(s->protimer_counter, UINT32_MAX, 1);
    ptimer_run(s->protimer_counter, 0);
    ptimer_transaction_commit(s->protimer_counter);
    ptimer_transaction_begin(s->burtc_counter);
    ptimer_stop(s->burtc_counter);
    ptimer_set_period_from_clock(s->burtc_counter, s->burtc_clk, 1);
    ptimer_set_limit(s->burtc_counter, UINT32_MAX, 1);
    ptimer_set_count(s->burtc_counter, UINT32_MAX);
    ptimer_transaction_commit(s->burtc_counter);
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
    /*
     * Zephyr's alarm-mode counter driver reports COMP as its top value.
     * The LL ticker requires a free-running 32-bit top, so this machine
     * integration default is UINT32_MAX until guest software programs COMP.
     */
    efr32_register_put(s, EFR32_BURTC_BASE + EFR32_BURTC_COMP, UINT32_MAX);
    efr32_register_put(s, EFR32_TIMER4_BASE + EFR32_TIMER_TOP, 0xffff);
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

    if (s->dma_as_initialized) {
        address_space_destroy(&s->dma_as);
    }
    timer_free(s->rac_rx_timer);
    timer_free(s->rac_tx_timer);
    timer_free(s->burtc_compare_timer);
    timer_free(s->timer4_compare_timer);
    ptimer_free(s->protimer_counter);
    ptimer_free(s->burtc_counter);
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

static Property efr32_periph_properties[] = {
    DEFINE_PROP_LINK("memory", Efr32Periph, memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_END_OF_LIST(),
};

static void efr32_periph_class_init(ObjectClass *o, void *d)
{
    DeviceClass *dc = DEVICE_CLASS(o);
    Efr32RfRadioClass *radio_class = EFR32_RF_RADIO_CLASS(o);

    dc->realize = efr32_periph_realize;
    device_class_set_props(dc, efr32_periph_properties);
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
    Clock *burtc_clk;

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
    s->burtc_clk = clock_new(OBJECT(s), "BURTCCLK");
    clock_set_hz(s->burtc_clk, 32768);

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
    object_property_set_link(OBJECT(&s->periph), "memory", OBJECT(s->memory),
                             &error_abort);
    qdev_connect_clock_in(DEVICE(&s->periph), "protimerclk", s->protimer_clk);
    qdev_connect_clock_in(DEVICE(&s->periph), "burtcclk", s->burtc_clk);
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
