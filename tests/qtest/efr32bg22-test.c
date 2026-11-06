/*
 * QTest coverage for the EFR32BG22 native peripheral model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <sys/un.h>
#include "libqtest-single.h"
#include "qemu/bswap.h"

static char *tx_snapshot_trace_path;

#define FRC_BASE             0xa8004000
#define RADIO_IPVERSION      0x000
#define FRC_DATABUFFER       0x018
#define FRC_RXRAWDATA        0x090
#define FRC_STATUS           0x008
#define FRC_IF               0x068
#define FRC_IEN              0x06c
#define FRC_IF_TXDONE        (1u << 0)
#define FRC_IF_RXDONE        (1u << 4)
#define AGC_BASE             0xa800c000
#define AGC_RSSI             0x018
#define AGC_FRAMERSSI        0x01c
#define AGC_IF               0x044
#define AGC_IEN              0x048
#define AGC_CMD              0x04c
#define FRC_WHITEPOLY        0x034
#define FRC_CMD              0x02c
#define FRC_CMD_RXABORT      (1u << 0)
#define FRC_CMD_CONVINIT     (1u << 9)
#define FRC_BUFFERMODE       0x078
#define FRC_SNIFFCTRL        0x084
#define FRC_RAWCTRL          0x08c
#define FRC_TRAILRXDATA       0x054
#define FRC_RXCTRL            0x04c
#define FRC_SEQIF            0x0b4
#define FRC_SEQIEN           0x0b8
#define FRC_WCNTCMP0         0x020
#define MODEM_BASE           0xa8014000
#define MODEM_SHAPING1       0x098
#define MODEM_LONGRANGE5     0x14c
#define MODEM_IF             0x208
#define MODEM_IEN            0x20c
#define MODEM_IRCAL          0x228
#define MODEM_RAMPCTRL       0x0c4
#define MODEM_RAMPLEV        0x0cc
#define MODEM_SEQIF          0x24c
#define MODEM_IF_TXRAMPDONE  (1u << 3)
#define RFCRC_BASE           0xa8010000
#define RFCRC_EN             0x004
#define RFCRC_CTRL           0x008
#define RFCRC_CMD            0x010
#define RFCRC_CMD_INITIALIZE (1u << 0)
#define RFCRC_INIT           0x018
#define RFCRC_DATA           0x01c
#define PROTIMER_BASE         0xa801c000
#define PROTIMER_NS_BASE      0xb801c000
#define PROTIMER_EN           0x004
#define PROTIMER_CTRL         0x008
#define PROTIMER_CMD          0x00c
#define PROTIMER_COUNTER      0x020
#define PROTIMER_STATE        0x030
#define PROTIMER_PERIOD       0x038
#define PROTIMER_IF           0x064
#define PROTIMER_IEN          0x070
#define PROTIMER_CMP0         0x100
#define PROTIMER_CMP0_TIME    0x10c
#define SYNTH_BASE            0xa8018000
#define SYNTH_STATUS          0x008
#define SYNTH_STATUS_INLOCK   (1u << 0)
#define SYNTH_STATUS_SYNTHREADY (1u << 2)
#define SYNTH_FREQ            0x034
#define SYNTH_FREQ_2402_MHZ   0x01f46aabu
#define SYNTH_CMD             0x00c
#define SYNTH_CMD_SYNTHSTART  (1u << 0)
#define SYNTH_CMD_SYNTHSTOP   (1u << 1)
#define RAC_SYEN              0x15c
#define RAC_PAENCTRL          0x0b4
#define RAC_PAENCTRL_PARAMP   (1u << 8)
#define RAC_SEQIF_STATETXWARM (1u << 24)
#define RAC_SEQIF_STATETXPD   (1u << 26)
#define RAC_STATUS_STATE_TXPD (10u << 24)
#define RAC_BASE              0xa8020000
#define RAC_EN                0x004
#define RAC_STATUS            0x00c
#define RAC_CMD               0x010
#define RAC_CTRL              0x014
#define RAC_SR0               0x040
#define RAC_SR3               0x04c
#define RAC_STCTRL            0x050
/* STCAL[23:0] | STSKEW[24] -- the only two documented fields (RAC_S_field.py),
 * confirmed structurally identical in layout to Armv6-M SysTick.CALIB's own
 * TENMS[23:0]/SKEW[24] (see owo-open-fw docs/radio-tables.md SS7.9). */
#define RAC_STCTRL_WRITABLE_MASK 0x01ffffffu
/* The exact literal stock's sequencer reset handler writes to RAC.STCTRL,
 * independently measured live on stock hardware (docs/radio-tables.md
 * SS7.8/SS7.9). Not a derived or invented value. */
#define RAC_STCTRL_STOCK_VALUE   0x01061a7fu
#define RAC_RXENSRCEN         0x008
#define RAC_IEN               0x020
#define RAC_SEQIF             0x028
#define RAC_CMD_TXEN          (1u << 0)
#define RAC_CMD_TXDIS         (1u << 5)
#define RAC_CMD_RXDIS         (1u << 8)
#define RAC_CTRL_FORCEDISABLE (1u << 0)
#define RAC_CTRL_SEQRESET     (1u << 24)
#define RAC_CTRL_CPUWAITDIS   (1u << 26)
#define RAC_CTRL_ACTIVE_HIGH  (1u << 7)
#define RAC_CTRL_PAEN_HIGH    (1u << 8)
#define RAC_CTRL_LNAEN_HIGH   (1u << 9)
#define RAC_CTRL_BOARD_POLARITIES \
    (RAC_CTRL_ACTIVE_HIGH | RAC_CTRL_PAEN_HIGH | RAC_CTRL_LNAEN_HIGH)
#define THUMB16_LDR_R0_PC_8      0x4802u
#define THUMB16_LDR_R1_R0        0x6801u
#define THUMB16_ADDS_R1_1        0x3101u
#define THUMB16_STR_R1_R0        0x6001u
#define THUMB16_BRANCH_SELF      0xe7feu
#define THUMB16_PAIR(first, second) \
    ((uint32_t)(first) | ((uint32_t)(second) << 16))

/*
 * Small Thumb-16 (ARMv6-M) instruction encoders used to assemble one-off
 * probe snippets for the AP3 sequencer core at runtime, rather than hand-
 * computing and transcribing magic opcode words. Each encoder is named
 * after, and implements exactly, the architected encoding in the Armv6-M
 * ARM ARM for that mnemonic; none of this invents or approximates behavior.
 */
static uint16_t thumb_ldr_pc(unsigned rd, uint32_t insn_addr,
                             uint32_t literal_addr)
{
    uint32_t pc = (insn_addr + 4) & ~3u;
    uint32_t imm8;

    g_assert_cmpuint(literal_addr, >=, pc);
    g_assert_cmpuint((literal_addr - pc) % 4, ==, 0);
    imm8 = (literal_addr - pc) / 4;
    g_assert_cmpuint(imm8, <=, 0xff);
    return 0x4800u | (rd << 8) | imm8;
}

static uint16_t thumb_movs_imm8(unsigned rd, uint8_t imm8)
{
    return 0x2000u | (rd << 8) | imm8;
}

static uint16_t thumb_strb_imm(unsigned rt, unsigned rn, unsigned imm5)
{
    return 0x7000u | (imm5 << 6) | (rn << 3) | rt;
}

static uint16_t thumb_ldrb_imm(unsigned rt, unsigned rn, unsigned imm5)
{
    return 0x7800u | (imm5 << 6) | (rn << 3) | rt;
}

static uint16_t thumb_str_imm(unsigned rt, unsigned rn, unsigned imm5_words)
{
    return 0x6000u | (imm5_words << 6) | (rn << 3) | rt;
}

static uint16_t thumb_ldr_imm(unsigned rt, unsigned rn, unsigned imm5_words)
{
    return 0x6800u | (imm5_words << 6) | (rn << 3) | rt;
}

static uint16_t thumb_b(uint32_t insn_addr, uint32_t target_addr)
{
    int32_t offset = (int32_t)(target_addr - (insn_addr + 4));

    g_assert_cmpint(offset, >=, -2048);
    g_assert_cmpint(offset, <=, 2046);
    g_assert_true(offset % 2 == 0);
    return 0xe000u | (((uint32_t)offset >> 1) & 0x7ffu);
}
#define RAC_RXENSRCEN_SWRXEN  (1u << 1)
#define RAC_STATUS_STATE_MASK 0x0f000000
#define RAC_STATUS_STATE_RXSEARCH (2u << 24)
#define RAC_STATUS_STATE_TXWARM (8u << 24)
#define RAC_STATUS_STATE_TX   (9u << 24)
#define RAC_STATUS_STATE_SHUTDOWN (13u << 24)
#define RAC_STATUS_TXENS      (1u << 30)
#define RAC_STATUS_RXENS      (1u << 31)
#define RAC_SEQIF_STATEOFF    (1u << 16)
#define RAC_SEQIF_STATECHANGESEQ (1u << 0)
#define RAC_SEQIF_STATERXSEARCH (1u << 18)
#define RAC_SEQIF_STATETX     (1u << 25)
#define RAC_SEQIF_STATESHUTDOWN (1u << 29)
#define RAC_SEQIEN            0x02c
#define SEQRAM_NS_BASE         0xb0000000
#define SEQ_SPARE0             0x4fe0
#define DCDC_BASE             0x40094000
#define DCDC_NONSECURE_BASE   0x50094000
#define DCDC_CTRL             0x008
#define DCDC_IF               0x024
#define DCDC_STATUS           0x02c
#define DCDC_IF_REGULATION    (1u << 5)
#define EMU_BASE              0x40004000
#define EMU_STATUS            0x084
#define MSC_BASE              0x40030000
#define MSC_STATUS            0x01c
#define HFXO_BASE             0x4000c000
#define HFXO_NONSECURE_BASE   0x5000c000
#define HFXO_CTRL             0x028
#define HFXO_CMD              0x050
#define HFXO_STATUS           0x058
/* RDY, COREBIASOPTRDY, ENS, HWREQ: running stage 3 reads 0x40030003. */
#define HFXO_STATUS_READY     0x00030003
#define HFXO_STATUS_FSMLOCK   (1u << 30)
#define HFXO_CTRL_DISONDEMAND (1u << 1)
#define CMU_BASE              0x40008000
#define CMU_NONSECURE_BASE    0x50008000
#define CMU_SYSCLKCTRL        0x070
#define CMU_CLKEN1            0x068
#define CMU_RADIOCLKCTRL      0x280
#define CMU_RADIOCLKCTRL_EN   (1u << 0)
#define CMU_CLKEN1_RADIO_BLOCKS 0x87fu
#define CMU_CLKEN0            0x064
#define CMU_CLKEN0_SYSCFG     (1u << 16)
/* efr32bg22_cmu.h: TIMER0-3 are CLKEN0 bits 4-7, BURTC bit 29, TIMER4 CLKEN1 18. */
#define CMU_CLKEN0_TIMERS     (0xfu << 4)
#define CMU_CLKEN0_BURTC      (1u << 29)
#define CMU_CLKEN0_RTCC       (1u << 30)   /* RM 8.5.10 */
#define CMU_CLKEN1_TIMER4     (1u << 18)
#define CMU_CLKEN1_RDMAILBOX0 (1u << 8)
#define CMU_CLKEN1_RDMAILBOX1 (1u << 9)
#define SYSCFG_BASE           0x4007c000
#define SYSCFG_ROOTDATA1      0x604
#define CMU_CLKEN1_RAC        (1u << 5)
#define CMU_CLKEN1_BUFC       (1u << 11)
/* Unassigned radio window: a bus error whatever the enables (SS7.72). */
#define RADIO_HOLE_BASE       0xa8008000
#define RAC_STATUS_SEQSLEEPING (1u << 22)
#define THUMB16_WFI           0xbf30u
#define THUMB16_BRANCH_BACK_ONE 0xe7fdu
#define DPLL0_BASE            0x4001c000
#define DPLL0_NONSECURE_BASE  0x5001c000
#define DPLL_EN               0x004
#define DPLL_CFG              0x008
#define DPLL_CFG1             0x00c
#define DPLL_IF               0x010
#define DPLL_STATUS           0x018
#define DPLL_EN_EN            (1u << 0)
#define DPLL_IF_LOCK          (1u << 0)
#define DPLL_STATUS_RDY       (1u << 0)
#define DPLL_STATUS_ENS       (1u << 1)
#define HFXO_STATUS_RDY       (1u << 0)
#define HFXO_STATUS_COREBIASOPTRDY (1u << 1)
#define HFXO_STATUS_ENS       (1u << 16)
/* Cold start in FSRCO cycles at 19.807 MHz: ENS 25, RDY 5455, COREBIAS 5663. */
#define HFXO_ENS_NS           1263
#define HFXO_RDY_NS           275408
#define HFXO_COREBIASOPT_NS   285910
/* DPLL relock: 11588 HFXO cycles. */
#define DPLL_LOCK_NS          301771
#define LFXO_BASE             0x40020000
#define LFXO_STATUS           0x010
#define LFXO_CTRL             0x004
#define OSC_STATUS_RDY        (1u << 0)
#define OSC_STATUS_ENS        (1u << 16)
/* TIMEOUT = 4K LFXO cycles. */
#define LFXO_RDY_4K_NS        125000000
#define LFXO_CFG              0x008
#define LFXO_CFG_TIMEOUT_4K   (4u << 8)
#define USART0_BASE           0x4005c000
#define USART0_NONSECURE_BASE 0x5005c000
#define USART1_BASE           0x40060000
#define USART_CMD             0x014
#define USART_STATUS          0x018
#define USART_TXDATA          0x03c
#define USART_CMD_TXEN        (1u << 2)
#define USART_CMD_TXDIS       (1u << 3)
#define USART_STATUS_TXENS    (1u << 1)
#define USART_STATUS_TXC      (1u << 5)
#define USART_STATUS_TXBL     (1u << 6)
#define USART_STATUS_TXIDLE   (1u << 13)
#define BURTC_BASE             0x40064000
#define BURTC_NONSECURE_BASE   0x50064000
#define BURTC_EN               0x004
#define BURTC_CFG              0x008
#define BURTC_CMD              0x00c
#define BURTC_STATUS           0x010
#define BURTC_IF               0x014
#define BURTC_IEN              0x018
#define BURTC_CNT              0x020
#define BURTC_COMP             0x030
#define BURTC_IF_COMP          (1u << 1)
#define BURTC_EN_EN            (1u << 0)
#define BURTC_CMD_START        (1u << 0)
#define BURTC_CMD_STOP         (1u << 1)
#define BURTC_STATUS_RUNNING   (1u << 0)
#define TIMER4_BASE             0x40058000
#define TIMER4_NS_BASE          0x50058000
#define TIMER2_BASE             0x40050000
#define TIMER2_NS_BASE          0x50050000
#define TIMER1_BASE             0x4004c000
#define TIMER3_BASE             0x40054000
#define TIMER_EN                0x030
#define TIMER_CMD               0x00c
#define TIMER_STATUS            0x010
#define TIMER_IF                0x014
#define TIMER_IEN               0x018
#define TIMER_TOP               0x01c
#define TIMER_CNT               0x024
#define TIMER_CC0_OC            0x068
#define TIMER_EN_EN             (1u << 0)
#define TIMER_CMD_START          (1u << 0)
#define TIMER_CMD_STOP           (1u << 1)
#define TIMER_STATUS_RUNNING     (1u << 0)
#define TIMER_IF_CC0             (1u << 4)
#define TIMER0_BASE             0x40048000
#define TIMER0_NS_BASE          0x50048000
#define TIMER_CFG               0x004
#define TIMER_CFG_PRESC_SHIFT   18
#define TIMER_CC_CFG(n)         (0x060 + 0x20 * (n))
#define TIMER_CC_OC(n)          (0x068 + 0x20 * (n))
#define TIMER_CC_CFG_OUTPUTCOMPARE 0x2
#define TIMER_IF_OF             (1u << 0)
#define TIMER_IF_CC(n)          (1u << (4 + (n)))
#define SMU_BASE                0x44008000
#define SMU_PPUSATD1            0x064
#define SMU_PPUSATD1_AHBRADIO   (1u << 15)
#define CMU_BASE_ADDR           0x40008000
#define CMU_DPLLREFCLKCTRL      0x100
#define CMU_EM01GRPACLKCTRL     0x120
#define CMU_EM01GRPACLK_HFXO    2
#define CMU_EM01GRPACLK_FSRCO   3
#define DPLL0_BASE_ADDR         0x4001c000
#define DPLL_EN                 0x004
#define DPLL_CFG1               0x00c
#define RADIO_RAM_NS_BASE     0xb0000000
#define RADIO_RAM_RHCLKPRESC  0x4fe0
#define BUFC_BASE            0xaa000000
#define BUFC_EN              0x004
#define BUFC_CH0             0x00c
#define BUFC_CH1             0x04c
#define BUFC_CH2             0x08c
#define BUFC_IF              0x114
#define BUFC_IEN             0x118
#define BUFC_WRITEOFFSET     0x008
#define BUFC_READOFFSET      0x00c
#define BUFC_READDATA        0x014
#define BUFC_WRITEDATA       0x018
#define BUFC_STATUS          0x020
#define BUFC_THRESHOLD       0x024
#define BUFC_CMD             0x028
#define BUFC_TX_CHANNEL       BUFC_CH0
#define BUFC_RX_CHANNEL       BUFC_CH1

#define RFMAILBOX_B_BASE      0xa8028000
#define RFMAILBOX_C_BASE      0xa802c000
#define RFMAILBOX_NS_OFFSET   0x10000000
#define RFMAILBOX_DATA        0x004
#define RFMAILBOX_FLAGS       0x040

#define BUFC_SECURE_BASE     0xaa000000
#define RADIO_NONSECURE_ALIAS 0x10000000

#define REGISTER_SET_WINDOW  0x1000
#define REGISTER_CLEAR_WINDOW 0x2000

static void attribute_radio_nonsecure(void)
{
    writel(SMU_BASE + SMU_PPUSATD1 + REGISTER_CLEAR_WINDOW,
           SMU_PPUSATD1_AHBRADIO);
}

static void attribute_radio_secure(void)
{
    writel(SMU_BASE + SMU_PPUSATD1 + REGISTER_SET_WINDOW,
           SMU_PPUSATD1_AHBRADIO);
}
#define REGISTER_TOGGLE_WINDOW 0x3000
#define DEVINFO_BASE          0x0fe00000
#define DEVINFO_MODULENAME0   0x8130
#define DEVINFO_MODULEINFO    0x814c
/* DEVINFO INFO, PART, MEMINFO (RM SS6.4.2.1-3) of the kit's C224 die. */
#define DEVINFO_INFO          0x8000
#define DEVINFO_PART          0x8004
#define DEVINFO_MEMINFO       0x8008
#define KIT_DEVINFO_INFO      0x07090000 /* DEVINFOREV 7, PRODREV 9 */
#define KIT_DEVINFO_PART      0x021608b0 /* BG, family 22, device C224 */
#define KIT_DEVINFO_MEMINFO   0x00000003 /* 8 KiB flash pages */
/* OpenOCD's family byte (efm32.c): 128, Series 2, as on the suit. */
#define DEVINFO_SERIES2_ID    0x81fe
#define KIT_SERIES2_ID        0x80
#define SYSCFG_CHIPREV        0x014
#define SYSCFG_CHIPREV_FAMILY (0x3fu << 6)
#define SYSCFG_CHIPREV_BG22   (53u << 6)

static void qtest_seq_enable_radio_bus_clocks(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
                 CMU_RADIOCLKCTRL_EN);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
                 CMU_CLKEN1_RADIO_BLOCKS);
}

static void enable_radio_bus_clocks(void)
{
    writel(CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
           CMU_RADIOCLKCTRL_EN);
    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
           CMU_CLKEN1_RADIO_BLOCKS);
}

/*
 * Every radio block, BUFC and SYSCFG answer with a bus error while their CMU
 * clocks are off (owo-open-fw docs/radio-tables.md SS7.72), as on silicon.
 * Tests that are not about clock gating start from a reset with them on.
 */
static void reset_with_radio_clocks(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();
    /* TIMERn and BURTC fault while their CMU bus clock is off. */
    writel(CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
           CMU_CLKEN0_TIMERS | CMU_CLKEN0_BURTC);
    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW, CMU_CLKEN1_TIMER4);
}

static void test_bgm220_module_identity(void)
{
    const uint32_t module_name[] = {
        0x324d4742, 0x43503032, 0x4e483232, 0xffff3241,
        UINT32_MAX, UINT32_MAX, UINT32_MAX,
    };

    for (size_t i = 0; i < G_N_ELEMENTS(module_name); i++) {
        g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_MODULENAME0 + 4 * i),
                        ==, module_name[i]);
    }
    g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_MODULEINFO), ==, 0x00570100);
    /* What OpenOCD's efm32 driver decodes: "EFR32BG22 C224, rev 9". */
    g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_INFO), ==, KIT_DEVINFO_INFO);
    g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_PART), ==, KIT_DEVINFO_PART);
    g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_MEMINFO), ==,
                    KIT_DEVINFO_MEMINFO);
    g_assert_cmphex(readb(DEVINFO_BASE + DEVINFO_SERIES2_ID), ==,
                    KIT_SERIES2_ID);
    /* The family the boot code leaves in CHIPREV follows DEVINFO.PART. */
    writel(CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW, CMU_CLKEN0_SYSCFG);
    g_assert_cmphex(readl(SYSCFG_BASE + SYSCFG_CHIPREV) &
                    SYSCFG_CHIPREV_FAMILY, ==, SYSCFG_CHIPREV_BG22);
}

static void test_register_aliases(void)
{
    uint32_t address = FRC_BASE + FRC_WHITEPOLY;

    writel(address, 0);
    writel(address + REGISTER_SET_WINDOW, 0x12);
    g_assert_cmphex(readl(address), ==, 0x12);

    writel(address + REGISTER_CLEAR_WINDOW, 0x02);
    g_assert_cmphex(readl(address), ==, 0x10);

    writel(address + REGISTER_TOGGLE_WINDOW, 0x01);
    g_assert_cmphex(readl(address), ==, 0x11);
}

/* One BURTC clock period at the board's 32.768 kHz, rounded up. */
#define LF_PERIOD_NS          30518
/* Writes to CNT, COMP and CMD take effect at the third LF edge after them. */
#define BURTC_SYNC_EDGES      3
#define BURTC_SYNCBUSY        0x028
#define BURTC_SYNCBUSY_START  (1u << 0)
#define BURTC_SYNCBUSY_STOP   (1u << 1)
#define BURTC_SYNCBUSY_CNT    (1u << 3)
#define BURTC_SYNCBUSY_COMP   (1u << 4)
#define BURTC_SYNCBUSY_EN     (1u << 5)

static void clock_run(int64_t ns)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, ns);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
}

static void burtc_edges(int64_t edges)
{
    clock_run(edges * LF_PERIOD_NS);
}

/* Step to just after the next LF edge, so later steps count whole edges. */
static void burtc_align(void)
{
    uint64_t now = qtest_clock_step(global_qtest, 0);
    uint64_t next = (now * 32768 / 1000000000 + 1) * 1000000000 / 32768 + 1;

    clock_run(next - now);
}

static void test_burtc_counter_contract(void)
{
    uint32_t stopped;

    reset_with_radio_clocks();
    /* The LL ticker uses the driver's COMP-backed top-value API. */
    g_assert_cmphex(readl(BURTC_BASE + BURTC_COMP), ==, UINT32_MAX);

    writel(BURTC_BASE + BURTC_CFG, UINT32_MAX);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CFG), ==, 0xf3);
    writel(BURTC_BASE + BURTC_CFG + REGISTER_CLEAR_WINDOW, 0xf0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CFG), ==, 0x03);

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    burtc_align();
    writel(BURTC_BASE + BURTC_CNT, 0x11223344);
    /* CNT reads its old value until the write has synchronized. */
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_SYNCBUSY) & BURTC_SYNCBUSY_CNT,
                    ==, BURTC_SYNCBUSY_CNT);
    burtc_edges(BURTC_SYNC_EDGES - 1);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 0);
    burtc_edges(1);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 0x11223344);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_SYNCBUSY) & BURTC_SYNCBUSY_CNT,
                    ==, 0);

    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_SYNCBUSY), ==,
                    BURTC_SYNCBUSY_START | BURTC_SYNCBUSY_EN);
    burtc_edges(BURTC_SYNC_EDGES);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);
    /* START's own edge does not count; the next one does. */
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 0x11223344);
    burtc_edges(1);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 0x11223345);

    /* STOP: the counter still counts the three edges up to and with it. */
    writel(BURTC_BASE + BURTC_CMD + REGISTER_SET_WINDOW, BURTC_CMD_STOP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);
    burtc_edges(BURTC_SYNC_EDGES);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    stopped = readl(BURTC_BASE + BURTC_CNT);
    g_assert_cmphex(stopped, ==, 0x11223345 + BURTC_SYNC_EDGES);
    clock_run(1000000);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, stopped);

    writel(BURTC_BASE + BURTC_EN + REGISTER_CLEAR_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    burtc_edges(BURTC_SYNC_EDGES);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
}

/*
 * Series-2 SET/CLR/TGL pages are alternate write views of the normal
 * register page.  In particular, counter_silabs uses the alias forms for
 * EN, IEN, and IF.  Exercise the observable consequences too: EN gates the
 * free-running counter, and writes through the non-secure view share the
 * same canonical state.
 */
static void test_burtc_register_aliases(void)
{
    reset_with_radio_clocks();

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_EN), ==, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    burtc_edges(BURTC_SYNC_EDGES);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);

    writel(BURTC_BASE + BURTC_EN + REGISTER_TOGGLE_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_EN), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    writel(BURTC_BASE + BURTC_EN + REGISTER_TOGGLE_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_EN), ==, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);

    writel(BURTC_NONSECURE_BASE + BURTC_IEN + REGISTER_SET_WINDOW,
           BURTC_IF_COMP);
    writel(BURTC_NONSECURE_BASE + BURTC_IF + REGISTER_SET_WINDOW,
           BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF), ==, BURTC_IF_COMP);

    writel(BURTC_BASE + BURTC_IF + REGISTER_TOGGLE_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF), ==, 0);

    writel(BURTC_BASE + BURTC_IEN + REGISTER_TOGGLE_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, 0);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_TOGGLE_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_IF + REGISTER_SET_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, 0);
}

/* IF.COMP sets on the edge where CNT advances out of COMP. */
static void test_burtc_compare_pending(void)
{
    reset_with_radio_clocks();

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    burtc_align();
    writel(BURTC_BASE + BURTC_CNT, 10);
    writel(BURTC_BASE + BURTC_IF + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_COMP, 42);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_SET_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    /* Synced at edge 3 with CNT = 10; CNT leaves 42 at edge 3 + 33. */
    burtc_edges(BURTC_SYNC_EDGES + 32);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 42);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP, ==, 0);
    burtc_edges(1);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP, ==,
                    BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_IF + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP, ==, 0);
}

/*
 * COMP = CNT + d on a running counter fires for d >= 3 and waits for the
 * 2^32 wrap for d <= 2 (silicon, every LF phase).
 */
static bool burtc_compare_fires(uint32_t lead)
{
    uint32_t cnt;

    reset_with_radio_clocks();
    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    burtc_edges(BURTC_SYNC_EDGES + 5);
    clock_run(LF_PERIOD_NS / 2);
    cnt = readl(BURTC_BASE + BURTC_CNT);
    writel(BURTC_BASE + BURTC_COMP, cnt + lead);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_COMP), ==, UINT32_MAX);
    burtc_edges(lead + 8);
    return readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP;
}

static void test_burtc_compare_lead(void)
{
    g_assert_false(burtc_compare_fires(0));
    g_assert_false(burtc_compare_fires(1));
    g_assert_false(burtc_compare_fires(2));
    g_assert_true(burtc_compare_fires(3));
    g_assert_true(burtc_compare_fires(6));
}

/* A rewritten COMP still fires its old match up to two edges ahead. */
static bool burtc_retarget_cancels(uint32_t edges_ahead)
{
    uint32_t target;

    reset_with_radio_clocks();
    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    burtc_edges(BURTC_SYNC_EDGES + 2);
    clock_run(LF_PERIOD_NS / 2);
    target = readl(BURTC_BASE + BURTC_CNT) + 10;
    writel(BURTC_BASE + BURTC_COMP, target);
    while (readl(BURTC_BASE + BURTC_CNT) != target - edges_ahead) {
        burtc_edges(1);
    }
    writel(BURTC_BASE + BURTC_COMP, target + 1000);
    burtc_edges(10);
    return !(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP);
}

static void test_burtc_retarget_window(void)
{
    g_assert_false(burtc_retarget_cancels(0));
    g_assert_false(burtc_retarget_cancels(2));
    g_assert_true(burtc_retarget_cancels(3));
}

/* SYNCBUSY.EN: the sixth falling LF edge after EN = 1. */
static void test_burtc_en_sync(void)
{
    reset_with_radio_clocks();
    burtc_align();
    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_SYNCBUSY), ==,
                    BURTC_SYNCBUSY_EN);
    burtc_edges(5);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_SYNCBUSY), ==,
                    BURTC_SYNCBUSY_EN);
    burtc_edges(1);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_SYNCBUSY), ==, 0);
}


/* TIMER4 is Zephyr's 16-bit 38.4 MHz scheduling counter on this part. */
static void test_timer4_counter_and_aliases(void)
{
    uint32_t stopped;

    reset_with_radio_clocks();
    writel(TIMER4_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_TOP), ==, 0xffff);
    writel(TIMER4_NS_BASE + TIMER_TOP, 0xffff);
    writel(TIMER4_NS_BASE + TIMER_CNT, 0x1200);
    writel(TIMER4_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex((readl(TIMER4_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==,
                    TIMER_STATUS_RUNNING);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmpuint(readl(TIMER4_BASE + TIMER_CNT), >, 0x1200);

    writel(TIMER4_BASE + TIMER_CMD + REGISTER_SET_WINDOW, TIMER_CMD_STOP);
    g_assert_cmphex((readl(TIMER4_NS_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==, 0);
    stopped = readl(TIMER4_BASE + TIMER_CNT);
    qtest_clock_step(global_qtest, 1000);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_CNT), ==, stopped);

    writel(TIMER4_NS_BASE + TIMER_EN + REGISTER_CLEAR_WINDOW, TIMER_EN_EN);
    g_assert_cmphex((readl(TIMER4_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==, 0);
}

/* The retained stock image starts TIMER2 during application bring-up. */
static void test_timer2_counter_and_aliases(void)
{
    uint32_t stopped;

    reset_with_radio_clocks();
    writel(TIMER2_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER2_BASE + TIMER_TOP), ==, 0xffff);
    writel(TIMER2_NS_BASE + TIMER_TOP, 0xffff);
    writel(TIMER2_NS_BASE + TIMER_CNT, 0x1200);
    writel(TIMER2_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex((readl(TIMER2_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==,
                    TIMER_STATUS_RUNNING);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmpuint(readl(TIMER2_BASE + TIMER_CNT), >, 0x1200);

    writel(TIMER2_BASE + TIMER_CMD + REGISTER_SET_WINDOW, TIMER_CMD_STOP);
    g_assert_cmphex((readl(TIMER2_NS_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==, 0);
    stopped = readl(TIMER2_BASE + TIMER_CNT);
    qtest_clock_step(global_qtest, 1000);
    g_assert_cmphex(readl(TIMER2_BASE + TIMER_CNT), ==, stopped);

    writel(TIMER2_NS_BASE + TIMER_EN + REGISTER_CLEAR_WINDOW, TIMER_EN_EN);
    g_assert_cmphex((readl(TIMER2_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==, 0);
}

static void timer0_step(int64_t ns)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, ns);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
}

/* TIMER0 is 32-bit (TIMER0_CNTWIDTH = 0x20) and counts HFXO / (PRESC + 1). */
static void test_timer0_32bit_count_prescale_overflow(void)
{
    reset_with_radio_clocks();
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);

    writel(TIMER0_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_TOP), ==, 0xffff);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CNT, 0xffffff00);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex((readl(TIMER0_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==,
                    TIMER_STATUS_RUNNING);

    /* 10 us at 38.4 MHz is exactly 384 ticks: past the 32-bit wrap. */
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 0x80);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF) & TIMER_IF_OF, ==,
                    TIMER_IF_OF);

    /* PRESC = 3 divides by four from the current count; CFG needs EN = 0. */
    writel(TIMER0_BASE + TIMER_EN, 0);
    writel(TIMER0_BASE + TIMER_CFG, 3u << TIMER_CFG_PRESC_SHIFT);
    writel(TIMER0_BASE + TIMER_EN, TIMER_EN_EN);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 0x80 + 96);

    writel(TIMER0_BASE + TIMER_CMD + REGISTER_SET_WINDOW, TIMER_CMD_STOP);
    g_assert_cmphex((readl(TIMER0_NS_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==, 0);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 0x80 + 96);
}

/* Each output-compare channel flags only when CNT reaches its own OC. */
static void test_timer0_cc_output_compare(void)
{
    reset_with_radio_clocks();
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);

    writel(TIMER0_NS_BASE + TIMER_CC_CFG(0), TIMER_CC_CFG_OUTPUTCOMPARE);
    writel(TIMER0_NS_BASE + TIMER_CC_CFG(1), TIMER_CC_CFG_OUTPUTCOMPARE);
    writel(TIMER0_NS_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CC_OC(0), 1000);
    writel(TIMER0_NS_BASE + TIMER_CC_OC(1), 2000);
    /* CC2 is left in OFF mode: its OC value must not raise a flag. */
    writel(TIMER0_NS_BASE + TIMER_CC_OC(2), 500);
    writel(TIMER0_NS_BASE + TIMER_IEN, TIMER_IF_CC(0) | TIMER_IF_CC(1));
    writel(TIMER0_NS_BASE + TIMER_CNT, 0);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);

    /* CNT leaves 1000 after 1001 ticks, 26067.71 ns. */
    timer0_step(26067);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, 0);
    timer0_step(1);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, TIMER_IF_CC(0));

    writel(TIMER0_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC(0));
    timer0_step(26042);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, TIMER_IF_CC(1));

    /* A compare moved behind CNT waits for the counter to come around. */
    writel(TIMER0_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC(1));
    writel(TIMER0_BASE + TIMER_CC_OC(0), 10);
    timer0_step(100000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, 0);
}

/*
 * TIMER0 follows EM01GRPACLK.  The BG22 explorer kit runs it from HFRCODPLL
 * locked to HFXO with M = 1919, N = 3839 (xg22_explorer_kit.dtsi), i.e.
 * 38.4 MHz * 3840 / 1920 = 76.8 MHz.
 */
static void test_timer0_follows_em01grpaclk_dpll(void)
{
    reset_with_radio_clocks();

    writel(CMU_BASE_ADDR + CMU_DPLLREFCLKCTRL, 1); /* HFXO reference */
    writel(DPLL0_BASE_ADDR + DPLL_CFG1, (3839u << 16) | 1919u);
    writel(DPLL0_BASE_ADDR + DPLL_EN, 1);
    writel(TIMER0_NS_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CNT, 0);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 768);

    /* Switching EM01GRPACLK to HFXO halves the rate from the current count. */
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 768 + 384);

    /*
     * On FSRCO it counts at the measured 19.807 MHz (SS7.73), not the
     * nominal 20 MHz: 1980 counts in 100 us, where 20 MHz would give 2000.
     */
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_FSRCO);
    writel(TIMER0_NS_BASE + TIMER_CNT, 0);
    timer0_step(100000);
    g_assert_cmpuint(readl(TIMER0_BASE + TIMER_CNT), >=, 1980);
    g_assert_cmpuint(readl(TIMER0_BASE + TIMER_CNT), <=, 1981);
}

static void test_timer4_cc0_compare_pending(void)
{
    reset_with_radio_clocks();

    /* CC0 compares only in OUTPUTCOMPARE mode, set while EN = 0. */
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);
    writel(TIMER4_NS_BASE + TIMER_CC_CFG(0), TIMER_CC_CFG_OUTPUTCOMPARE);
    writel(TIMER4_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    writel(TIMER4_NS_BASE + TIMER_TOP, 0xffff);
    writel(TIMER4_NS_BASE + TIMER_CNT, 10);
    writel(TIMER4_NS_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC0);
    writel(TIMER4_NS_BASE + TIMER_CC0_OC, 48);
    writel(TIMER4_NS_BASE + TIMER_IEN + REGISTER_SET_WINDOW, TIMER_IF_CC0);
    writel(TIMER4_NS_BASE + TIMER_CMD, TIMER_CMD_START);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_IF) & TIMER_IF_CC0, ==,
                    TIMER_IF_CC0);
    writel(TIMER4_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC0);
    g_assert_cmphex(readl(TIMER4_NS_BASE + TIMER_IF) & TIMER_IF_CC0, ==, 0);
}

/*
 * A board reset is also the safe startup baseline for the radio port.  In
 * particular, it must discard both pending timer events and register-latched
 * interrupt enables before firmware begins a new LL schedule.
 */
static void test_system_reset_clears_radio_timer_state(void)
{
    reset_with_radio_clocks();

    writel(BURTC_BASE + BURTC_EN, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 10);
    writel(BURTC_BASE + BURTC_COMP, 42);
    writel(BURTC_BASE + BURTC_IEN, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);

    writel(TIMER4_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER4_BASE + TIMER_CNT, 10);
    writel(TIMER4_BASE + TIMER_CC0_OC, 48);
    writel(TIMER4_BASE + TIMER_IEN, TIMER_IF_CC0);
    writel(TIMER4_BASE + TIMER_CMD, TIMER_CMD_START);

    writel(PROTIMER_NS_BASE + PROTIMER_IF, 0x02000010);
    writel(PROTIMER_NS_BASE + PROTIMER_IEN, 0x00000010);

    reset_with_radio_clocks();
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2 * 1000 * 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, 0);
    g_assert_cmphex((readl(TIMER4_BASE + TIMER_STATUS) & TIMER_STATUS_RUNNING), ==, 0);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_IF), ==, 0);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_IEN), ==, 0);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IF), ==, 0);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IEN), ==, 0);
}

static void test_cmu_nonsecure_alias(void)
{
    reset_with_radio_clocks();
    g_assert_cmphex(readl(CMU_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);
    g_assert_cmphex(readl(CMU_NONSECURE_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);

    writel(CMU_NONSECURE_BASE + CMU_SYSCLKCTRL, 3);
    g_assert_cmphex(readl(CMU_BASE + CMU_SYSCLKCTRL) & 7, ==, 3);

    writel(CMU_BASE + CMU_SYSCLKCTRL, 1);
    g_assert_cmphex(readl(CMU_NONSECURE_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);
}

/* Zephyr CMU_DPLLLock() clears IF, enables DPLL0, then polls LOCK/FAIL. */
static void test_dpll_lock(void)
{
    reset_with_radio_clocks();
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_STATUS), ==, 0);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_IF) & 7, ==, 0);

    writel(DPLL0_NONSECURE_BASE + DPLL_CFG, UINT32_MAX);
    writel(DPLL0_NONSECURE_BASE + DPLL_CFG1, UINT32_MAX);
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_CFG), ==, 0x47);
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_CFG1), ==, 0x0fff0fff);

    /* HFXO already running, as for the measured relock. */
    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_SET_WINDOW, 1);
    timer0_step(HFXO_COREBIASOPT_NS);
    writel(CMU_BASE_ADDR + CMU_DPLLREFCLKCTRL, 1);
    writel(DPLL0_NONSECURE_BASE + DPLL_IF + REGISTER_CLEAR_WINDOW, 7);
    writel(DPLL0_NONSECURE_BASE + DPLL_EN + REGISTER_SET_WINDOW,
           DPLL_EN_EN);
    /* ENS at once, RDY and IF.LOCK after the lock; bit 31 stays clear. */
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_STATUS), ==, DPLL_STATUS_ENS);
    timer0_step(DPLL_LOCK_NS - 1);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_IF) & 7, ==, 0);
    timer0_step(1);
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_STATUS), ==,
                    DPLL_STATUS_ENS | DPLL_STATUS_RDY);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_IF) & 7, ==,
                    DPLL_IF_LOCK);

    /* ENS drops 41 HFXO cycles after EN = 0. */
    writel(DPLL0_BASE + DPLL_EN + REGISTER_CLEAR_WINDOW, DPLL_EN_EN);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_STATUS), ==,
                    DPLL_STATUS_ENS);
    timer0_step(1068);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_STATUS), ==, 0);
}

/*
 * Zephyr's USART_Tx() polls TXBL, then writes TXDATA through this sequence.
 * STATUS resets to TXIDLE | TXBL (RM 21.5.7); a written frame clears TXIDLE
 * while it is on the wire and sets TXC and TXIDLE at its stop bit. TXDATA is
 * write-only (RM 21.4).
 */
static void test_usart_tx_polling(void)
{
    /* One 8N1 frame at the reset rate takes well under this. */
    const int64_t frame_bound_ns = 1000000;

    reset_with_radio_clocks();

    g_assert_cmphex(readl(USART0_BASE + USART_STATUS), ==,
                    USART_STATUS_TXIDLE | USART_STATUS_TXBL);
    writel(USART0_NONSECURE_BASE + USART_CMD + REGISTER_SET_WINDOW,
           USART_CMD_TXEN);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS), ==,
                    USART_STATUS_TXIDLE | USART_STATUS_TXENS |
                    USART_STATUS_TXBL);

    writel(USART0_NONSECURE_BASE + USART_TXDATA, 0x1a5);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS) &
                    (USART_STATUS_TXIDLE | USART_STATUS_TXC), ==, 0);
    clock_run(frame_bound_ns);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS), ==,
                    USART_STATUS_TXIDLE | USART_STATUS_TXENS |
                    USART_STATUS_TXBL | USART_STATUS_TXC);

    writel(USART0_BASE + USART_CMD, USART_CMD_TXDIS);
    g_assert_cmphex(readl(USART0_NONSECURE_BASE + USART_STATUS), ==,
                    USART_STATUS_TXIDLE | USART_STATUS_TXBL |
                    USART_STATUS_TXC);
    g_assert_cmphex(readl(USART1_BASE + USART_STATUS), ==,
                    USART_STATUS_TXIDLE | USART_STATUS_TXBL);
}

/*
 * An 8N1 frame at PCLK = HFXO 38.4 MHz, OVS x16, CLKDIV 0 (2.4 Mbaud) lasts
 * 10 bit periods, 4.167 us (emlib's br = 256 fPCLK / (ovs (256 + DIV))).
 */
#define CMU_SYSCLKCTRL_HFXO   3
#define USART_FRAME_2M4_NS    4166

static void test_usart_frame_time(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel qtest");

    qtest_writel(qts, CMU_BASE + CMU_SYSCLKCTRL, CMU_SYSCLKCTRL_HFXO);
    qtest_writel(qts, USART0_BASE + USART_CMD, USART_CMD_TXEN);
    qtest_writel(qts, USART0_BASE + USART_TXDATA, 0x5a);
    qtest_clock_step(qts, USART_FRAME_2M4_NS - 1);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_STATUS) &
                    USART_STATUS_TXC, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_STATUS) &
                    USART_STATUS_TXC, ==, USART_STATUS_TXC);
    qtest_quit(qts);
}

/*
 * The USART timer as Zephyr's async driver sets it (RM Table 21.10,
 * receiver timeout): TIMECMP1 = TSTART RXEOF, TSTOP RXACT, RESTARTEN,
 * TCMPVAL 8 raises IF.TCMP1 8 bit times after a received frame ends.
 * A break from the host (telnet IAC BREAK on the chardev) is a frame of 0
 * with a framing error (RM SS21.3.2.13).  2.4 Mbaud: bit 416 ns.
 */
#define USART_RXDATAX         0x020
#define USART_IF              0x048
#define USART_TIMECMP1        0x064
#define USART_CMD_RXEN        (1u << 0)
#define USART_STATUS_RXDATAV  (1u << 7)
#define USART_IF_FERR         (1u << 9)
#define USART_IF_TCMP1        (1u << 15)
#define USART_RXDATAX_FERR    (1u << 15)
#define USART_BIT_NS          416
#define USART_TIMECMP1_RXTIMEOUT ((2u << 20) | (4u << 16) | (1u << 24) | 8)

static int unix_connect(const char *path)
{
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);

    g_assert_cmpint(fd, >=, 0);
    g_strlcpy(addr.sun_path, path, sizeof(addr.sun_path));
    g_assert_cmpint(connect(fd, (struct sockaddr *)&addr, sizeof(addr)), ==,
                    0);
    return fd;
}

static void usart_wait_rx(QTestState *qts)
{
    for (int i = 0; i < 1000 &&
                    !(qtest_readl(qts, USART0_BASE + USART_STATUS) &
                      USART_STATUS_RXDATAV); i++) {
        qtest_clock_step(qts, 100);
    }
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_STATUS) &
                    USART_STATUS_RXDATAV, ==, USART_STATUS_RXDATAV);
}

static void test_usart_timer_and_break(void)
{
    g_autofree char *path = g_strdup_printf("%s/efr32-usart-%d.sock",
                                            g_get_tmp_dir(), getpid());
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel qtest "
                                  "-chardev socket,id=soc.usart0,path=%s,"
                                  "server=on,wait=off,telnet=on", path);
    const uint8_t brk[2] = { 0xff, 0xf3 };
    int fd = unix_connect(path);

    qtest_writel(qts, CMU_BASE + CMU_SYSCLKCTRL, CMU_SYSCLKCTRL_HFXO);
    qtest_writel(qts, USART0_BASE + USART_TIMECMP1, USART_TIMECMP1_RXTIMEOUT);
    qtest_writel(qts, USART0_BASE + USART_CMD, USART_CMD_RXEN);
    g_assert_cmpint(write(fd, "A", 1), ==, 1);
    usart_wait_rx(qts);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_IF) &
                    USART_IF_TCMP1, ==, 0);
    qtest_clock_step(qts, 8 * USART_BIT_NS - 200);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_IF) &
                    USART_IF_TCMP1, ==, 0);
    qtest_clock_step(qts, 200);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_IF) &
                    USART_IF_TCMP1, ==, USART_IF_TCMP1);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_RXDATAX), ==, 'A');

    g_assert_cmpint(write(fd, brk, sizeof(brk)), ==, sizeof(brk));
    usart_wait_rx(qts);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_IF) & USART_IF_FERR,
                    ==, USART_IF_FERR);
    g_assert_cmphex(qtest_readl(qts, USART0_BASE + USART_RXDATAX), ==,
                    USART_RXDATAX_FERR);
    close(fd);
    unlink(path);
    qtest_quit(qts);
}

static void test_modem_masks(void)
{
    writel(MODEM_BASE + MODEM_SHAPING1, 0xdeadbeef);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_SHAPING1), ==, 0xdeadbeef);

    writel(MODEM_BASE + MODEM_LONGRANGE5, UINT32_MAX);
    writel(MODEM_BASE + MODEM_IF, UINT32_MAX);
    writel(MODEM_BASE + MODEM_IEN, UINT32_MAX);
    writel(MODEM_BASE + MODEM_IRCAL, UINT32_MAX);

    g_assert_cmphex(readl(MODEM_BASE + MODEM_LONGRANGE5), ==, 0x0fffffff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_IF), ==, 0x0007ffff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_IEN), ==, 0x0007ffff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_IRCAL), ==, 0x0000ffbf);
}

/*
 * MODEM.RAMPCTRL and RAMPLEV: the public revB2 reset values and writable
 * masks.  The open firmware's transmit power sets the 6 dBm PA's level in all
 * three RAMPLEV fields (owo-open-fw docs/tx-power.md); without it RAMPLEV
 * keeps 0x9f9f9f.
 */
static void test_modem_pa_ramp_registers(void)
{
    reset_with_radio_clocks();
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPCTRL), ==, 0x00000555);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPLEV), ==, 0x009f9f9f);

    writel(MODEM_BASE + MODEM_RAMPCTRL, UINT32_MAX);
    writel(MODEM_BASE + MODEM_RAMPLEV, UINT32_MAX);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPCTRL), ==, 0xff800fff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPLEV), ==, 0x00ffffff);

    writel(MODEM_BASE + MODEM_RAMPLEV + REGISTER_CLEAR_WINDOW, 0x00ffffff);
    writel(MODEM_BASE + MODEM_RAMPLEV + REGISTER_SET_WINDOW, 0x00202020);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPLEV), ==, 0x00202020);

    reset_with_radio_clocks();
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPCTRL), ==, 0x00000555);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_RAMPLEV), ==, 0x009f9f9f);
}

static void test_rfcrc_masks(void)
{
    enable_radio_bus_clocks();
    writel(RFCRC_BASE + RFCRC_EN, UINT32_MAX);
    writel(RFCRC_BASE + RFCRC_CTRL, UINT32_MAX);

    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_EN), ==, 1);
    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_CTRL), ==, 0x1fef);
}

/*
 * Real hardware (docs/radio-tables.md SS7.43): RFCRC.CMD.INITIALIZE loads
 * the live CRC accumulator (DATA) from the configured seed (INIT). CMD is
 * write-only and must not latch into its own register.
 */
static void test_rfcrc_cmd_initialize(void)
{
    enable_radio_bus_clocks();
    writel(RFCRC_BASE + RFCRC_INIT, 0x00abcdef);
    writel(RFCRC_BASE + RFCRC_DATA, 0);

    writel(RFCRC_BASE + RFCRC_CMD, RFCRC_CMD_INITIALIZE);

    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_DATA), ==, 0x00abcdef);
    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_CMD), ==, 0);

    writel(RFCRC_BASE + RFCRC_INIT, 0x00112233);
    writel(RFCRC_BASE + RFCRC_DATA, 0x00445566);
    writel(RFCRC_BASE + RFCRC_CMD, 0);

    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_DATA), ==, 0x00445566);
}

static void test_agc_rssi_status_registers(void)
{
    writel(AGC_BASE + AGC_RSSI, UINT32_MAX);
    writel(AGC_BASE + AGC_FRAMERSSI, UINT32_MAX);

    g_assert_cmphex(readl(AGC_BASE + AGC_RSSI), ==, 0x8000);
    g_assert_cmphex(readl(AGC_BASE + AGC_FRAMERSSI), ==, 0x8000);
}

static void test_agc_rssi_valid_latches(void)
{
    writel(AGC_BASE + AGC_IF, UINT32_MAX);
    writel(AGC_BASE + AGC_IEN, UINT32_MAX);

    g_assert_cmphex(readl(AGC_BASE + AGC_IF), ==, 0x37d);
    g_assert_cmphex(readl(AGC_BASE + AGC_IEN), ==, 0x37d);

    writel(AGC_BASE + REGISTER_CLEAR_WINDOW + AGC_IF, 1);
    g_assert_cmphex(readl(AGC_BASE + AGC_IF), ==, 0x37c);
}

static void test_agc_rssi_start_command(void)
{
    writel(AGC_BASE + AGC_CMD, 1);

    /* The public command register is write-only; no RF source is modeled. */
    g_assert_cmphex(readl(AGC_BASE + AGC_CMD), ==, 0);
    g_assert_cmphex(readl(AGC_BASE + AGC_IF) & 1, ==, 0);
}

static void test_frc_buffer_register_masks(void)
{
    writel(FRC_BASE + FRC_DATABUFFER, UINT32_MAX);
    writel(FRC_BASE + FRC_BUFFERMODE, UINT32_MAX);
    writel(FRC_BASE + FRC_TRAILRXDATA, UINT32_MAX);
    writel(FRC_BASE + FRC_RXRAWDATA, UINT32_MAX);

    g_assert_cmphex(readl(FRC_BASE + FRC_DATABUFFER), ==, 0xff);
    g_assert_cmphex(readl(FRC_BASE + FRC_BUFFERMODE), ==, 0xf);
    g_assert_cmphex(readl(FRC_BASE + FRC_TRAILRXDATA), ==, 0x3f);
    g_assert_cmphex(readl(FRC_BASE + FRC_RXRAWDATA), ==, 0);
}

static void test_frc_sniffctrl_mask(void)
{
    uint32_t address = FRC_BASE + FRC_SNIFFCTRL;

    writel(address, UINT32_MAX);
    g_assert_cmphex(readl(address), ==, 0x0003ffff);

    /* Stock clears this register then sets 0x17f8 through alias windows. */
    writel(address + REGISTER_CLEAR_WINDOW, UINT32_MAX);
    writel(address + REGISTER_SET_WINDOW, 0x17f8);
    g_assert_cmphex(readl(address), ==, 0x17f8);
}

static void test_frc_rawctrl_mask(void)
{
    writel(FRC_BASE + FRC_RAWCTRL, UINT32_MAX);

    g_assert_cmphex(readl(FRC_BASE + FRC_RAWCTRL), ==, 0x000021bf);
}

static void test_rail_initialization_latches(void)
{
    /* RAIL initializes convolution state while aborting stale receive work. */
    writel(FRC_BASE + FRC_CMD, FRC_CMD_RXABORT | FRC_CMD_CONVINIT);
    g_assert_cmphex(readl(FRC_BASE + FRC_CMD), ==, 0);

    /* Its RF initializer then writes RXCTRL=0x60 and clears RAC.SR0 bit 6. */
    writel(FRC_BASE + FRC_RXCTRL, 0x60);
    writel(RAC_BASE + RAC_SR0, UINT32_MAX);
    writel(RAC_BASE + RAC_SR0 + REGISTER_CLEAR_WINDOW, 1u << 6);

    g_assert_cmphex(readl(FRC_BASE + FRC_RXCTRL), ==, 0x60);
    g_assert_cmphex(readl(RAC_BASE + RAC_SR0) & (1u << 6), ==, 0);
}

static void test_rac_sr3_clear_alias(void)
{
    uint32_t address = RAC_BASE + RAC_SR3;

    writel(address, UINT32_MAX);
    writel(address + REGISTER_CLEAR_WINDOW, 0x40000000);

    /* Stock clears this exact bit through RAC.SR3's CLR window. */
    g_assert_cmphex(readl(address), ==, 0xbfffffff);
}

static void test_frc_interrupt_register_masks(void)
{
    const uint32_t registers[] = {
        FRC_IF,
        FRC_IEN,
        FRC_SEQIF,
        FRC_SEQIEN,
    };

    for (size_t i = 0; i < G_N_ELEMENTS(registers); i++) {
        writel(FRC_BASE + registers[i], UINT32_MAX);
        g_assert_cmphex(readl(FRC_BASE + registers[i]), ==, 0x7f7fffff);
    }
}

static void test_frc_completion_interrupt_enable(void)
{
    const uint32_t completion = FRC_IF_TXDONE | FRC_IF_RXDONE;

    writel(FRC_BASE + FRC_IEN + REGISTER_CLEAR_WINDOW, completion);
    writel(FRC_BASE + FRC_IEN + REGISTER_SET_WINDOW, completion);

    g_assert_cmphex(readl(FRC_BASE + FRC_IEN) & completion, ==, completion);
}

static void test_radio_enable_masks(void)
{
    const uint32_t blocks[] = {
        FRC_BASE,
        AGC_BASE,
        RFCRC_BASE,
        MODEM_BASE,
        SYNTH_BASE,
        RAC_BASE,
    };

    reset_with_radio_clocks();
    enable_radio_bus_clocks();
    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        writel(blocks[i] + 0x004, UINT32_MAX);
        g_assert_cmphex(readl(blocks[i] + 0x004), ==, 1);
    }
}

static void test_radio_enable_requires_bus_clocks(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(AGC_BASE + 0x004, 1);
    g_assert_cmphex(readl(AGC_BASE + 0x004), ==, 0);

    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW, 1u << 0);
    writel(AGC_BASE + 0x004, 1);
    g_assert_cmphex(readl(AGC_BASE + 0x004), ==, 0);

    writel(CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
           CMU_RADIOCLKCTRL_EN);
    writel(AGC_BASE + 0x004, 1);
    g_assert_cmphex(readl(AGC_BASE + 0x004), ==, 1);
}

static void test_radio_ipversions(void)
{
    static const struct {
        uint32_t base;
        uint32_t value;
    } blocks[] = {
        { FRC_BASE, 1 },
        { AGC_BASE, 1 },
        { RFCRC_BASE, 0 },
        { MODEM_BASE, 1 },
        { SYNTH_BASE, 1 },
        { RAC_BASE, 1 },
    };

    reset_with_radio_clocks();
    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        uint32_t secure = blocks[i].base + RADIO_IPVERSION;
        uint32_t nonsecure = secure + RADIO_NONSECURE_ALIAS;

        g_assert_cmphex(readl(secure), ==, blocks[i].value);
        g_assert_cmphex(readl(nonsecure), ==, 0);
        writel(secure, UINT32_MAX);
        g_assert_cmphex(readl(secure), ==, blocks[i].value);
    }

    attribute_radio_nonsecure();
    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        uint32_t secure = blocks[i].base + RADIO_IPVERSION;
        uint32_t nonsecure = secure + RADIO_NONSECURE_ALIAS;

        g_assert_cmphex(readl(secure), ==, 0);
        g_assert_cmphex(readl(nonsecure), ==, blocks[i].value);
    }
}

/*
 * FRC.STATUS is hardware read-only (public revB2 FRC_S_register.py); no
 * modelled command path sets ACTIVETXFCD or any other bit
 * (docs/radio-tables.md SS7.4 records that open question separately). This
 * only confirms the model cannot be made to disagree with that read-only
 * contract through either address view; it is not a claim about what value
 * real hardware would show at this point.
 */
static void test_frc_status_is_read_only(void)
{
    uint32_t secure = FRC_BASE + FRC_STATUS;
    uint32_t nonsecure = secure + RADIO_NONSECURE_ALIAS;

    reset_with_radio_clocks();
    g_assert_cmphex(readl(secure), ==, 0);
    writel(secure, UINT32_MAX);
    g_assert_cmphex(readl(secure), ==, 0);

    attribute_radio_nonsecure();
    g_assert_cmphex(readl(nonsecure), ==, 0);
    writel(nonsecure, UINT32_MAX);
    g_assert_cmphex(readl(nonsecure), ==, 0);
    attribute_radio_secure();
}

static void test_radio_nonsecure_alias(void)
{
    uint32_t secure_address = FRC_BASE + FRC_WHITEPOLY;
    uint32_t nonsecure_address = secure_address + RADIO_NONSECURE_ALIAS;

    reset_with_radio_clocks();
    writel(secure_address, 0x1234);
    writel(nonsecure_address, UINT32_MAX); /* inactive alias is WI */
    g_assert_cmphex(readl(secure_address), ==, 0x1234);
    g_assert_cmphex(readl(nonsecure_address), ==, 0); /* inactive alias is RAZ */

    attribute_radio_nonsecure();
    g_assert_cmphex(readl(secure_address), ==, 0);
    g_assert_cmphex(readl(nonsecure_address), ==, 0x1234);
    writel(nonsecure_address, UINT32_MAX);
    g_assert_cmphex(readl(nonsecure_address), ==, 0xffff);

    attribute_radio_secure();
    g_assert_cmphex(readl(secure_address), ==, 0xffff);
    g_assert_cmphex(readl(nonsecure_address), ==, 0);
}

static void test_stock_setup_aliases(void)
{
    /* Stock writes one at +0x004 through these two radio windows. */
    enable_radio_bus_clocks();
    attribute_radio_nonsecure();
    writel(BUFC_BASE + RADIO_NONSECURE_ALIAS + BUFC_EN, 1);
    writel(PROTIMER_NS_BASE + PROTIMER_EN, 1);

    g_assert_cmphex(readl(BUFC_BASE + RADIO_NONSECURE_ALIAS + BUFC_EN), ==, 1);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_EN), ==, 1);
    g_assert_cmphex(readl(BUFC_BASE + BUFC_EN), ==, 0);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_EN), ==, 0);
}

static void test_rac_enable_requires_dependency_order(void)
{
    reset_with_radio_clocks();
    enable_radio_bus_clocks();

    writel(RAC_BASE + RAC_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_SHUTDOWN);
    writel(BUFC_BASE + BUFC_EN, 1);
    writel(PROTIMER_BASE + PROTIMER_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_SHUTDOWN);

    reset_with_radio_clocks();
    enable_radio_bus_clocks();
    writel(BUFC_BASE + BUFC_EN, 1);
    writel(PROTIMER_BASE + PROTIMER_EN, 1);
    writel(RAC_BASE + RAC_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, 0);

    /* PROTIMER.EN is not needed at all (silicon, SS7.71). */
    reset_with_radio_clocks();
    enable_radio_bus_clocks();
    writel(BUFC_BASE + BUFC_EN, 1);
    writel(RAC_BASE + RAC_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, 0);
}

static void test_dcdc_regulation(void)
{
    reset_with_radio_clocks();
    g_assert_cmphex(readl(DCDC_NONSECURE_BASE + DCDC_IF) & DCDC_IF_REGULATION,
                    ==, 0);

    writel(DCDC_NONSECURE_BASE + DCDC_CTRL + REGISTER_SET_WINDOW, 1);

    g_assert_cmphex(readl(DCDC_NONSECURE_BASE + DCDC_IF) & DCDC_IF_REGULATION,
                    ==,
                    DCDC_IF_REGULATION);
    g_assert_cmphex(readl(DCDC_BASE + DCDC_IF) & DCDC_IF_REGULATION, ==,
                    DCDC_IF_REGULATION);
    g_assert_cmphex(readl(DCDC_BASE + DCDC_STATUS), ==, 6);
}

static void test_emu_status_reset(void)
{
    g_assert_cmphex(readl(EMU_BASE + EMU_STATUS), ==, 0x80);
}

static void test_msc_status_reset(void)
{
    g_assert_cmphex(readl(MSC_BASE + MSC_STATUS), ==, 0x08000008);
}

static void test_hfxo_ready_status(void)
{
    reset_with_radio_clocks();
    g_assert_cmphex(readl(CMU_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_CTRL), ==, HFXO_CTRL_DISONDEMAND);
    g_assert_cmphex(readl(HFXO_NONSECURE_BASE + HFXO_STATUS), ==, 0);

    /* FORCEEN alone: ENS, RDY, COREBIASOPTRDY in turn, no HWREQ. */
    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_SET_WINDOW, 1);
    g_assert_cmphex(readl(HFXO_NONSECURE_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_FSMLOCK);
    timer0_step(HFXO_ENS_NS);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_ENS | HFXO_STATUS_FSMLOCK);
    timer0_step(HFXO_RDY_NS - HFXO_ENS_NS);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_ENS | HFXO_STATUS_RDY | HFXO_STATUS_FSMLOCK);
    timer0_step(HFXO_COREBIASOPT_NS - HFXO_RDY_NS);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_ENS | HFXO_STATUS_RDY |
                    HFXO_STATUS_COREBIASOPTRDY | HFXO_STATUS_FSMLOCK);

    writel(HFXO_NONSECURE_BASE + HFXO_CMD, 1u << 1);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS) & HFXO_STATUS_FSMLOCK, ==,
                    0);

    /* A new request restarts both the oscillator and the FSM lock. */
    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_CLEAR_WINDOW, 1);
    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_SET_WINDOW, 1);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==, HFXO_STATUS_FSMLOCK);

    /* A hardware consumer (SYSCLK) reads HWREQ. */
    writel(HFXO_BASE + HFXO_CTRL + REGISTER_CLEAR_WINDOW, 1);
    writel(CMU_BASE + CMU_SYSCLKCTRL, 3);
    timer0_step(HFXO_COREBIASOPT_NS);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_READY | HFXO_STATUS_FSMLOCK);

    reset_with_radio_clocks();
    g_assert_cmphex(readl(HFXO_BASE + HFXO_CTRL), ==, HFXO_CTRL_DISONDEMAND);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==, 0);
}

/* LFXO: RDY follows ENS by CFG.TIMEOUT LFXO cycles. */
static void test_lfxo_ready_timeout(void)
{
    reset_with_radio_clocks();
    writel(LFXO_BASE + LFXO_CFG, LFXO_CFG_TIMEOUT_4K | 1);
    g_assert_cmphex(readl(LFXO_BASE + LFXO_STATUS), ==, 0);
    writel(LFXO_BASE + LFXO_CTRL + REGISTER_SET_WINDOW, 1);
    g_assert_cmphex(readl(LFXO_BASE + LFXO_STATUS), ==, OSC_STATUS_ENS);
    timer0_step(LFXO_RDY_4K_NS - 1);
    g_assert_cmphex(readl(LFXO_BASE + LFXO_STATUS), ==, OSC_STATUS_ENS);
    timer0_step(1);
    g_assert_cmphex(readl(LFXO_BASE + LFXO_STATUS), ==,
                    OSC_STATUS_ENS | OSC_STATUS_RDY);
}

static void test_bufc_data_port(void)
{
    uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    uint32_t sequencer_channel = BUFC_SECURE_BASE + BUFC_CH1;
    uint32_t nonsecure_rx_channel = BUFC_BASE + BUFC_CH1;
    const uint32_t rx_buffer = 0x20002000;

    writel(tx_channel + BUFC_WRITEDATA, 0x5a);
    writel(tx_channel + BUFC_WRITEDATA, 0xa5);
    writel(tx_channel + BUFC_READOFFSET, 0);

    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0x5a);
    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0xa5);

    /* RAIL permits TX FIFO contents to be preloaded in its backing RAM. */
    writel(tx_channel + 0x004, 0x20001000);
    qtest_writeb(global_qtest, 0x20001000, 0x6b);
    writel(tx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0x6b);

    /*
     * The recovered sequencer image uses 32-bit STR instructions here, but
     * WRITEDATA is an 8-bit field. Only the low octet is consumed per write.
     */
    writel(sequencer_channel + BUFC_CMD, 1);
    writel(sequencer_channel + 0x004, rx_buffer);
    writel(sequencer_channel + BUFC_WRITEDATA, 0xa5b63c);
    writel(nonsecure_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(readl(nonsecure_rx_channel + BUFC_READDATA), ==, 0x3c);
    g_assert_cmphex(qtest_readb(global_qtest, rx_buffer), ==, 0x3c);
    writel(sequencer_channel + BUFC_CMD, 1);
}

static void test_bufc_packet_channels_are_independent(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_TX_CHANNEL;
    const uint32_t rx_channel = BUFC_BASE + BUFC_RX_CHANNEL;
    const uint32_t tx_buffer = 0x20001000;
    const uint32_t rx_buffer = 0x20002000;

    /* RAIL's recovered FIFO helpers use channel 0 for TX and channel 1 for RX. */
    writel(tx_channel + BUFC_CMD, 1);
    writel(rx_channel + BUFC_CMD, 1);
    writel(tx_channel, 3);
    writel(rx_channel, 3);
    writel(tx_channel + 0x004, tx_buffer);
    writel(rx_channel + 0x004, rx_buffer);

    writel(tx_channel + BUFC_WRITEDATA, 0x31);
    writel(rx_channel + BUFC_WRITEDATA, 0x72);

    g_assert_cmphex(qtest_readb(global_qtest, tx_buffer), ==, 0x31);
    g_assert_cmphex(qtest_readb(global_qtest, rx_buffer), ==, 0x72);
    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0x31);
    g_assert_cmphex(readl(rx_channel + BUFC_READDATA), ==, 0x72);
}

static void test_bufc_tx_fifo_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH0;

    /* A vendor 512-byte FIFO encodes as log2(512) - 6 = 3. */
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20001000);
    writel(channel + BUFC_WRITEOFFSET, 18);
    writel(channel + BUFC_CMD, 2);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20001000);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 2);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, 18 | (1u << 20));
}

static void test_bufc_backing_memory_wrap(void)
{
    const uint32_t channel = BUFC_BASE + BUFC_CH0;
    const uint32_t buffer = 0x20001000;

    writel(channel + BUFC_CMD, 1);
    writel(channel, 3); /* 512-byte FIFO */
    writel(channel + 0x004, buffer);
    writel(channel + BUFC_WRITEOFFSET, 511);
    writel(channel + BUFC_WRITEDATA, 0x31);
    writel(channel + BUFC_WRITEDATA, 0x32);

    g_assert_cmphex(qtest_readb(global_qtest, buffer + 511), ==, 0x31);
    g_assert_cmphex(qtest_readb(global_qtest, buffer), ==, 0x32);
    g_assert_cmphex(readl(channel + BUFC_WRITEOFFSET), ==, 513);
}

static void test_bufc_overflow_latch(void)
{
    const uint32_t channel = BUFC_BASE + BUFC_CH0;

    writel(channel + BUFC_CMD, 1);
    writel(channel, 0); /* 64-byte FIFO */
    for (unsigned i = 0; i <= 64; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & 1, ==, 1);
}

static void test_bufc_descriptor_ring_overflow_latch(void)
{
    const uint32_t channel = BUFC_BASE + BUFC_CH2;
    const uint32_t overflow_flag = 1u << 16;

    /*
     * RAIL's archive-backed class-4 producer starts from BUF2OF.  The BUFC
     * block supplies that channel-2 condition; software, not this generic
     * FIFO model, decides whether and how to publish the eight-byte record.
     */
    writel(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW, overflow_flag);
    writel(channel + BUFC_CMD, 1);
    writel(channel, 0); /* 64-byte descriptor-ring backing FIFO */
    for (unsigned i = 0; i <= 64; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & overflow_flag, ==,
                    overflow_flag);
}

static void test_bufc_rx_fifo_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH1;

    /* RAIL configures the RX FIFO as a 512-byte channel-1 buffer. */
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20002000);
    writel(channel + BUFC_THRESHOLD, 0x0fff);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20002000);
    g_assert_cmphex(readl(channel + BUFC_THRESHOLD), ==, 0x0fff);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 1);
}

static void test_bufc_rx_descriptor_ring_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH2;

    /*
     * RAIL configures BUFC channel 2 as its eight-byte RX descriptor ring:
     * CMD=1, capacity encoding, backing address, and threshold=7.  This is
     * distinct from the channel-1 sequencer payload-byte FIFO.
     */
    writel(BUFC_BASE + BUFC_IF, 0x00050000);
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20002400);
    writel(channel + BUFC_THRESHOLD, 7);
    writel(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW, 0x00050000);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20002400);
    g_assert_cmphex(readl(channel + BUFC_THRESHOLD), ==, 7);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 1);
    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & 0x00050000, ==, 0);
}

static void test_bufc_interrupt_register_masks(void)
{
    writel(BUFC_BASE + BUFC_IF, UINT32_MAX);
    writel(BUFC_BASE + BUFC_IEN, UINT32_MAX);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF), ==, 0x8f0f0f0f);
    g_assert_cmphex(readl(BUFC_BASE + BUFC_IEN), ==, 0x8f0f0f0f);
}

static void test_bufc_channel_register_masks(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH2;

    writel(channel + BUFC_CMD, UINT32_MAX);
    writel(channel, UINT32_MAX);
    writel(channel + BUFC_WRITEOFFSET, UINT32_MAX);
    writel(channel + BUFC_READOFFSET, UINT32_MAX);
    writel(channel + BUFC_THRESHOLD, UINT32_MAX);

    g_assert_cmphex(readl(channel), ==, 0x7);
    g_assert_cmphex(readl(channel + BUFC_WRITEOFFSET), ==, 0x1fff);
    g_assert_cmphex(readl(channel + BUFC_READOFFSET), ==, 0x1fff);
    g_assert_cmphex(readl(channel + BUFC_THRESHOLD), ==, 0x2fff);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 0xf);
}

static void test_bufc_sparse_threshold_flag(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH1;
    uint32_t threshold_flag = 1u << 10;

    writel(BUFC_BASE + BUFC_IEN, threshold_flag);
    /* Channel 1's sparse threshold source is independent from BUF2THR. */
    writel(channel + BUFC_THRESHOLD, 7);
    for (unsigned int i = 0; i < 7; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==, 0);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, 7);

    writel(channel + BUFC_WRITEDATA, 7);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==,
                    threshold_flag);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, (1u << 20) | 8u);
}

static void test_bufc_descriptor_threshold_flag(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH2;
    uint32_t threshold_flag = 1u << 18;

    /*
     * The archived RAIL descriptor-ring setup uses buffer 2, whose global
     * threshold source is bit 18 (not a compact third threshold bit).  This
     * checks only BUFC's documented counter/flag presentation; it does not
     * claim that this CPU data-port write is a hardware descriptor producer.
     */
    writel(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW, threshold_flag);
    writel(BUFC_BASE + BUFC_IEN, threshold_flag);
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20002400);
    writel(channel + BUFC_THRESHOLD, 7);
    for (unsigned int i = 0; i < 7; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==, 0);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, 7);

    writel(channel + BUFC_WRITEDATA, 7);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==,
                    threshold_flag);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, (1u << 20) | 8u);
    g_assert_cmphex(readl(0x20002400), ==, 0x03020100);
    g_assert_cmphex(readl(0x20002404), ==, 0x07060504);
}

static void test_protimer_trace_latches(void)
{
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);
    writel(PROTIMER_BASE + PROTIMER_STATE, 0x00001233);
    writel(PROTIMER_BASE + PROTIMER_PERIOD, 0x80015558);
    writel(PROTIMER_BASE + PROTIMER_CMP0, 0x01000003);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CTRL), ==, 0x00011102);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_STATE), ==, 0x00001233);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_PERIOD), ==, 0x80015558);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CMP0), ==, 0x01000003);
}

static void test_protimer_secure_nonsecure_alias(void)
{
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);
    attribute_radio_nonsecure();
    /* RAIL enables PROTIMER through the attributed 0xb801c000 window. */
    writel(PROTIMER_NS_BASE + PROTIMER_CTRL, 0x00011102);
    writel(PROTIMER_NS_BASE + PROTIMER_CMD, 1);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CTRL), ==, 0);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CTRL), ==, 0x00011102);
}

/*
 * generic_phy.o:FRC_IRQHandler reads PROTIMER.IF (+0x064) and IEN (+0x070)
 * through the non-secure window.  Keep those banks as aliasable latches; a
 * counter or compare match deliberately does not manufacture a pending bit.
 */
static void test_protimer_interrupt_banks_are_aliasable(void)
{
    attribute_radio_nonsecure();
    writel(PROTIMER_NS_BASE + PROTIMER_IF, 0x02000010);
    writel(PROTIMER_NS_BASE + PROTIMER_IEN + REGISTER_SET_WINDOW, 0x00000010);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IF), ==, 0);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IEN), ==, 0x00000010);

    writel(PROTIMER_NS_BASE + PROTIMER_IF + REGISTER_CLEAR_WINDOW, 0x10);
    writel(PROTIMER_NS_BASE + PROTIMER_IEN + REGISTER_TOGGLE_WINDOW, 0x10);

    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IF), ==, 0x02000000);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IEN), ==, 0);
}

/*
 * rfhal_protimer.o:0x3c140 writes 0x11 to the first indexed slot and, on
 * one branch, clears a bit through 0xb801e100. This only preserves the
 * observed normal/CLR alias access; it does not assign compare semantics.
 */
static void test_protimer_compare_slot_clear_alias(void)
{
    attribute_radio_nonsecure();
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0, 0x11);
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0_TIME, 0x12345678);
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0 + REGISTER_CLEAR_WINDOW, 1);

    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0), ==, 0x10);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0_TIME), ==, 0x12345678);

    writel(PROTIMER_NS_BASE + PROTIMER_CMP0 + REGISTER_CLEAR_WINDOW, 0x10);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0), ==, 0);
}

/*
 * PROTIMER's counter is frozen at 0 until CMD bit 0 starts it, even with
 * EN = 1 and stock's CTRL value (silicon, SS7.32, SS7.33, SS7.71).
 */
#define PROTIMER_CMD_START 1u

static void protimer_start(void)
{
    writel(PROTIMER_BASE + PROTIMER_EN, 1);
    writel(PROTIMER_BASE + PROTIMER_CMD, PROTIMER_CMD_START);
}

static void test_protimer_counter(void)
{
    uint32_t before;

    reset_with_radio_clocks();
    writel(PROTIMER_BASE + PROTIMER_EN, 1);
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, 0);
    protimer_start();
    before = readl(PROTIMER_BASE + PROTIMER_COUNTER);

    attribute_radio_nonsecure();
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before);
    writel(PROTIMER_NS_BASE + PROTIMER_COUNTER, UINT32_MAX);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    /* Step past the second 500 ns edge so QEMU dispatches both ptimer ticks. */
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before + 2);

    reset_with_radio_clocks();
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, 0);
}

static void test_protimer_counter_is_independent_from_radio_setup(void)
{
    reset_with_radio_clocks();

    /*
     * Stock setup derives state from these inputs, but does not establish
     * them as the RAIL_GetTime counter's source or divisor. Keep the model's
     * documented 2 MHz clock independent until that relationship is known.
     */
    writel(CMU_BASE + CMU_SYSCLKCTRL, 3);
    writel(RADIO_RAM_NS_BASE + RADIO_RAM_RHCLKPRESC, 1u << 16);
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);
    protimer_start();

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, 2);
}

static void test_protimer_counter_is_independent_from_burtc(void)
{
    uint32_t before;

    reset_with_radio_clocks();
    protimer_start();
    before = readl(PROTIMER_BASE + PROTIMER_COUNTER);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, before + 2);

    reset_with_radio_clocks();
    protimer_start();
    writel(BURTC_BASE + BURTC_EN, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 0x12345678);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    burtc_edges(BURTC_SYNC_EDGES);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);

    before = readl(PROTIMER_BASE + PROTIMER_COUNTER);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    /* BURTC's separate 32.768 kHz clock must not replace PROTIMERCLK. */
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, before + 2);
}

static void test_rac_tx_commands(void)
{
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);
    /*
     * TXENS does NOT set here, matching real silicon: this exact
     * TXEN sequence measures RAC.STATUS == 0x09400000 on real hardware
     * (owo-open-fw docs/radio-tables.md SS7.21/SS7.25-27), i.e. STATE=TX
     * with TXENS=0. Confirmed repeatedly, not a simplification gap.
     */
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) &
                        (RAC_STATUS_TXENS | RAC_STATUS_RXENS),
                    ==, 0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATETX, ==,
                    RAC_SEQIF_STATETX);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATECHANGESEQ, ==,
                    RAC_SEQIF_STATECHANGESEQ);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATECHANGESEQ, ==,
                    RAC_SEQIF_STATECHANGESEQ);
}

static void test_frc_wcntcmp0_dynamic_length(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;

    /*
     * Two independent real-hardware measurements, both from a live
     * execution trace of the actual staged packet bytes (owo-open-fw
     * docs/radio-tables.md SS7.38, superseding SS7.29's assumed values):
     * a real BLE Length header byte of 9 measures WCNTCMP0 == 0xA, and
     * growing the advertising payload by 3 bytes (Length becomes 12)
     * measures WCNTCMP0 == 0xD. Both follow
     * WCNTCMP0 == <Length header byte> + MINLENGTH(1).
     */
    /* FRC.DFLCTRL: DFLMODE=1, DFLOFFSET=1, DFLBITS=8, MINLENGTH=1 --
     * exactly the one configuration radio.c's efr32_ble_1m_base_table
     * applies for every PHY variant this project supports. */
    writel(FRC_BASE + 0x00c, 0x00018101);
    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel + BUFC_WRITEDATA, 0x00); /* header byte 0 */
    writel(tx_channel + BUFC_WRITEDATA, 9);    /* header byte 1: length */
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(FRC_BASE + FRC_WCNTCMP0), ==, 0xa);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel + BUFC_WRITEDATA, 0x00); /* header byte 0 */
    writel(tx_channel + BUFC_WRITEDATA, 12);   /* header byte 1: length */
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(FRC_BASE + FRC_WCNTCMP0), ==, 0xd);
}

/*
 * A frame's TX lasts its airtime plus the measured anchors (owo-open-fw
 * docs/radio-tables.md SS7.71/SS7.72): 0.65 us from TX entry to the first
 * preamble bit, 40 us of preamble and access address, 8 us per PDU and CRC
 * octet, and 0.9 us from the last bit to leaving TX.
 */
#define TX_TIMING_PDU_OCTETS 2u
#define TX_TIMING_NS (650 + 40000 + (TX_TIMING_PDU_OCTETS + 3) * 8000 + 900)

static void test_rac_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(BUFC_BASE + BUFC_CH0 + BUFC_CMD, 1);
    for (unsigned i = 0; i < TX_TIMING_PDU_OCTETS; i++) {
        writel(BUFC_BASE + BUFC_CH0 + BUFC_WRITEDATA, i);
    }
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(TX_TIMING_NS - 1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    /* The frame is not over before its airtime and anchors have passed. */
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==,
                    FRC_IF_TXDONE);
}

/* TXDIS cancels QEMU's synthetic completion; it is not a hardware timing claim. */
static void test_rac_tx_disable_cancels_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
}

/*
 * RX supersedes QEMU's synthetic TX operation, so it cannot later raise
 * TXDONE.
 */
static void test_rac_rx_enable_cancels_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);

    writel(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
           RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_RXSEARCH);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_RXSEARCH);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
}

static void test_system_reset_cancels_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);

    reset_with_radio_clocks();
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
}

static void test_rac_tx_snapshot_captures_staged_bufc(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    gchar *trace;

    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel + BUFC_WRITEDATA, 0x91);
    writel(tx_channel + BUFC_WRITEDATA, 0x72);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* This later staging write must not change the first TXEN snapshot. */
    writel(tx_channel + BUFC_WRITEDATA, 0xe3);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=2 first=0x00007291"));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=3 first=0x00e37291"));
    g_free(trace);
}

static void test_rac_tx_snapshot_captures_guest_backed_fifo(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t tx_buffer = 0x20001000;
    gchar *trace;

    /* Match the RAIL SetTxFifo staging shape used by the Zephyr HAL. */
    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel, 3);
    writel(tx_channel + 0x004, tx_buffer);
    qtest_writeb(global_qtest, tx_buffer, 0x8d);
    qtest_writeb(global_qtest, tx_buffer + 1, 0x5b);
    qtest_writeb(global_qtest, tx_buffer + 2, 0xe1);
    writel(tx_channel + BUFC_WRITEOFFSET, 3);
    writel(tx_channel + BUFC_CMD, 2);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=3 first=0x00e15b8d"));
    g_free(trace);
}

static void test_rac_tx_snapshot_wraps_guest_backed_fifo(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t tx_buffer = 0x20001000;
    gchar *trace;

    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel, 3); /* 512-byte circular FIFO */
    writel(tx_channel + 0x004, tx_buffer);
    qtest_writeb(global_qtest, tx_buffer + 511, 0x2a);
    qtest_writeb(global_qtest, tx_buffer, 0x4c);
    qtest_writeb(global_qtest, tx_buffer + 1, 0x7e);
    writel(tx_channel + BUFC_READOFFSET, 511);
    writel(tx_channel + BUFC_WRITEOFFSET, 514);
    writel(tx_channel + BUFC_CMD, 2);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=3 first=0x007e4c2a"));
    g_free(trace);
}

static void test_rac_tx_snapshot_has_no_local_offers(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    gchar *trace;

    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel + BUFC_WRITEDATA, 0x4d);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1, "RX mailbox offers=0"));
    g_free(trace);
}

static void test_rac_software_rx_enable(void)
{
    writel(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
           RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_RXSEARCH);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) &
                        (RAC_STATUS_TXENS | RAC_STATUS_RXENS),
                    ==, RAC_STATUS_RXENS);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATERXSEARCH, ==,
                    RAC_SEQIF_STATERXSEARCH);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_RXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);
}

static void test_rac_masks(void)
{
    writel(RAC_BASE + RAC_RXENSRCEN, UINT32_MAX);
    writel(RAC_BASE + RAC_IEN, UINT32_MAX);

    g_assert_cmphex(readl(RAC_BASE + RAC_RXENSRCEN), ==, 0x00003fff);
    g_assert_cmphex(readl(RAC_BASE + RAC_IEN), ==, 0x00ff000f);
}

/*
 * RDMAILBOX IF is a latch with the usual SET/CLR aliases: the stock M0+
 * posts on mailbox 0 with IF_SET and waits for the M33 to clear the flag,
 * and the M33 posts on mailbox 1 for the M0+ (hw/arm/efr32bg22.c).  Message
 * pointers keep their value.
 */
static void test_rfmailbox_flags_latch(void)
{
    attribute_radio_nonsecure();
    /* Each mailbox endpoint has its own CMU bus clock (SS7.72). */
    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
           CMU_CLKEN1_RDMAILBOX0 | CMU_CLKEN1_RDMAILBOX1);
    writel(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET + RFMAILBOX_DATA, 0x3412);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET +
                          RFMAILBOX_DATA), ==, 0x3412);

    writel(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET + RFMAILBOX_FLAGS +
               REGISTER_SET_WINDOW,
           1u << 1);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET +
                          RFMAILBOX_FLAGS), ==, 1u << 1);
    writel(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET + RFMAILBOX_FLAGS +
               REGISTER_CLEAR_WINDOW,
           1u << 1);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET +
                          RFMAILBOX_FLAGS), ==, 0);
    attribute_radio_secure();
}

/*
 * With its CMU bus clock off, a radio-subsystem mailbox, scratchpad, PRORTC
 * or SYSCFG access is a bus error for reads and a dropped write: measured on
 * silicon by gating each CLKEN bit alone (owo-open-fw docs/radio-tables.md
 * SS7.72).  The M33 below loads SYSCFG.ROOTDATA1 with CLKEN0.SYSCFG clear and
 * must take the (escalated) HardFault; the dropped write is checked by
 * re-enabling the clock.
 */
/*
 * Boot an image-less M33 that loads one word from @address and stores it to
 * SRAM, or stores a marker from its HardFault handler.  @setup runs after the
 * reset, before the core starts.  Returns true if the load faulted.
 */
static bool m33_load_faults_on(const char *machine, uint32_t address,
                               void (*setup)(QTestState *qts))
{
    const uint32_t code = 0x100, handler = 0x180, result = 0x20000000;
    const uint32_t fault_marker = 0xfa0170ffu, loaded_marker_bit = 1u << 31;
    uint16_t insn[8], hinsn[6];
    QTestState *qts = qtest_initf("-machine %s -accel tcg -S", machine);
    uint32_t outcome;
    size_t i;

    /* Vector table: SP, reset, NMI, HardFault. */
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, code | 1);
    qtest_writel(qts, 0x00000008, handler | 1);
    qtest_writel(qts, 0x0000000c, handler | 1);
    /* r0 = address; r1 = *r0; r2 = result; r3 = marker bit; str r3; b . */
    insn[0] = thumb_ldr_pc(0, code + 0, code + 16);
    insn[1] = thumb_ldr_imm(1, 0, 0);
    insn[2] = thumb_ldr_pc(2, code + 4, code + 20);
    insn[3] = thumb_ldr_pc(3, code + 6, code + 24);
    insn[4] = thumb_str_imm(3, 2, 0);
    insn[5] = THUMB16_BRANCH_SELF;
    insn[6] = THUMB16_BRANCH_SELF;
    insn[7] = THUMB16_BRANCH_SELF;
    for (i = 0; i < 8; i += 2) {
        qtest_writel(qts, code + 2 * i, THUMB16_PAIR(insn[i], insn[i + 1]));
    }
    qtest_writel(qts, code + 16, address);
    qtest_writel(qts, code + 20, result);
    qtest_writel(qts, code + 24, loaded_marker_bit);
    /* Handler: r2 = result; r1 = marker; *r2 = r1; b . */
    hinsn[0] = thumb_ldr_pc(2, handler + 0, handler + 12);
    hinsn[1] = thumb_ldr_pc(1, handler + 2, handler + 16);
    hinsn[2] = thumb_str_imm(1, 2, 0);
    hinsn[3] = THUMB16_BRANCH_SELF;
    hinsn[4] = THUMB16_BRANCH_SELF;
    hinsn[5] = THUMB16_BRANCH_SELF;
    for (i = 0; i < 6; i += 2) {
        qtest_writel(qts, handler + 2 * i,
                     THUMB16_PAIR(hinsn[i], hinsn[i + 1]));
    }
    qtest_writel(qts, handler + 12, result);
    qtest_writel(qts, handler + 16, fault_marker);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_writel(qts, result, 0);
    setup(qts);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, result)) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    outcome = qtest_readl(qts, result);
    qtest_quit(qts);
    g_assert(outcome == fault_marker || outcome == loaded_marker_bit);
    return outcome == fault_marker;
}

static bool m33_load_faults(uint32_t address,
                            void (*setup)(QTestState *qts))
{
    return m33_load_faults_on("efr32bg22", address, setup);
}

static void setup_no_clocks(QTestState *qts)
{
}

/*
 * Part identity from DEVINFO/USERDATA page images: an EFR32BG22C222F352GM40
 * (public part header: FLASH_SIZE 0x58000) in the BGM220PC22WGA2 module
 * (public datasheet: 352 KB flash, 32 KB RAM).  The pages are synthetic:
 * only the public identity words are set.
 */
#define PART_DEVINFO_PART       0x004
#define PART_DEVINFO_MSIZE      0x00c
#define PART_C222F352_PART      0x021608ae /* BG, family 22, device 2222 */
#define PART_C222F352_MSIZE     0x00200160 /* 32 KB SRAM, 352 KB flash */
#define PART_C222F352_FLASH_END 0x58000
#define DEVINFO_OFFSET          0x8000
#define INFO_PAGE_SIZE          0x400
#define MSC_WRITECTRL           0x00c
#define MSC_WRITECTRL_WREN      (1u << 0)
#define MSC_WRITECMD            0x010
#define MSC_WRITECMD_ERASEPAGE  (1u << 1)
#define MSC_ADDRB               0x014
#define MSC_WDATA               0x018
#define MSC_STATUS_LOCKED       (1u << 1)
#define MSC_STATUS_INVADDR      (1u << 2)
#define MSC_IF                  0x020
#define MSC_IF_ERASE            (1u << 0)
#define MSC_IF_WRITE            (1u << 1)
#define MSC_MISCLOCKWORD        0x040
#define MSC_MISCLOCKWORD_UDLOCK (1u << 4)
#define MSC_PAGELOCK1           0x124
#define FLASH_PAGE_SIZE         0x2000

static gchar *part_devinfo_path;
static gchar *part_userdata_path;

static void part_write_pages(void)
{
    uint8_t devinfo[INFO_PAGE_SIZE], userdata[INFO_PAGE_SIZE / 4];

    memset(devinfo, 0xff, sizeof(devinfo));
    stl_le_p(devinfo + PART_DEVINFO_PART, PART_C222F352_PART);
    stl_le_p(devinfo + PART_DEVINFO_MSIZE, PART_C222F352_MSIZE);
    memcpy(devinfo + DEVINFO_MODULENAME0 - DEVINFO_OFFSET, "BGM220PC22WGA2",
           14);
    for (size_t i = 0; i < sizeof(userdata); i++) {
        userdata[i] = i;
    }
    part_devinfo_path = g_strdup_printf("%s/efr32bg22-devinfo-%u.bin",
                                        g_get_tmp_dir(), g_random_int());
    part_userdata_path = g_strdup_printf("%s/efr32bg22-userdata-%u.bin",
                                         g_get_tmp_dir(), g_random_int());
    g_assert(g_file_set_contents(part_devinfo_path, (char *)devinfo,
                                 sizeof(devinfo), NULL));
    g_assert(g_file_set_contents(part_userdata_path, (char *)userdata,
                                 sizeof(userdata), NULL));
}

static gchar *part_machine(void)
{
    return g_strdup_printf("efr32bg22,devinfo=%s,userdata=%s",
                           part_devinfo_path, part_userdata_path);
}

/* The 352 KB part: identity pages, flash end, MSC and CHIPREV. */
static void test_part_352k_identity(void)
{
    g_autofree gchar *machine = part_machine();
    QTestState *qts = qtest_initf("-machine %s -accel tcg -S", machine);
    uint32_t last = PART_C222F352_FLASH_END - 4;

    g_assert_cmphex(qtest_readl(qts, DEVINFO_BASE + DEVINFO_OFFSET +
                                         PART_DEVINFO_MSIZE),
                    ==, PART_C222F352_MSIZE);
    g_assert_cmphex(qtest_readl(qts, DEVINFO_BASE + DEVINFO_MODULENAME0),
                    ==, 0x324d4742); /* "BGM2" */
    /* USERDATA: the image, erased beyond its end. */
    g_assert_cmphex(qtest_readl(qts, DEVINFO_BASE + 4), ==, 0x07060504);
    g_assert_cmphex(qtest_readl(qts, DEVINFO_BASE + INFO_PAGE_SIZE / 4), ==,
                    UINT32_MAX);

    /* The family the boot code leaves in CHIPREV follows DEVINFO.PART. */
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_SYSCFG);
    g_assert_cmphex(qtest_readl(qts, SYSCFG_BASE + SYSCFG_CHIPREV), ==,
                    SYSCFG_CHIPREV_BG22);

    /* MSC: ADDRB past the flash end is INVADDR; the last page is valid. */
    qtest_writel(qts, MSC_BASE + MSC_WRITECTRL, MSC_WRITECTRL_WREN);
    qtest_writel(qts, MSC_BASE + MSC_ADDRB, PART_C222F352_FLASH_END);
    g_assert_cmphex(qtest_readl(qts, MSC_BASE + MSC_STATUS) &
                    MSC_STATUS_INVADDR, ==, MSC_STATUS_INVADDR);
    qtest_writel(qts, MSC_BASE + MSC_ADDRB, last - 4);
    g_assert_cmphex(qtest_readl(qts, MSC_BASE + MSC_STATUS) &
                    MSC_STATUS_INVADDR, ==, 0);
    /* Program: only 1 -> 0, ADDRB advances by a word. */
    qtest_writel(qts, MSC_BASE + MSC_WRITECMD, MSC_WRITECMD_ERASEPAGE);
    g_assert_cmphex(qtest_readl(qts, last), ==, UINT32_MAX);
    qtest_writel(qts, MSC_BASE + MSC_WDATA, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, MSC_BASE + MSC_ADDRB), ==, last);
    qtest_writel(qts, MSC_BASE + MSC_WDATA, 0xff00ff00);
    g_assert_cmphex(qtest_readl(qts, last - 4), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, last), ==, 0xff00ff00);
    g_assert_cmphex(qtest_readl(qts, MSC_BASE + MSC_IF) &
                    (MSC_IF_ERASE | MSC_IF_WRITE), ==,
                    MSC_IF_ERASE | MSC_IF_WRITE);
    /* A locked page is refused with STATUS.LOCKED. */
    qtest_writel(qts, MSC_BASE + MSC_PAGELOCK1,
                 1u << (last / FLASH_PAGE_SIZE - 32));
    qtest_writel(qts, MSC_BASE + MSC_ADDRB, last);
    qtest_writel(qts, MSC_BASE + MSC_WRITECMD, MSC_WRITECMD_ERASEPAGE);
    g_assert_cmphex(qtest_readl(qts, last), ==, 0xff00ff00);
    g_assert_cmphex(qtest_readl(qts, MSC_BASE + MSC_STATUS) &
                    MSC_STATUS_LOCKED, ==, MSC_STATUS_LOCKED);
    /* USERDATA needs MISCLOCKWORD.UDLOCKBIT clear (reset value 1). */
    qtest_writel(qts, MSC_BASE + MSC_ADDRB, DEVINFO_BASE);
    qtest_writel(qts, MSC_BASE + MSC_WDATA, 0);
    g_assert_cmphex(qtest_readl(qts, DEVINFO_BASE), ==, 0x03020100);
    qtest_writel(qts, MSC_BASE + MSC_MISCLOCKWORD + REGISTER_CLEAR_WINDOW,
                 MSC_MISCLOCKWORD_UDLOCK);
    qtest_writel(qts, MSC_BASE + MSC_ADDRB, DEVINFO_BASE);
    qtest_writel(qts, MSC_BASE + MSC_WDATA, 0);
    g_assert_cmphex(qtest_readl(qts, DEVINFO_BASE), ==, 0);
    qtest_quit(qts);
}

/* RM SS4.2.1.2: an access to an unassigned address is a bus fault. */
static void test_part_352k_flash_end_faults(void)
{
    g_autofree gchar *machine = part_machine();

    g_assert_false(m33_load_faults_on(machine, PART_C222F352_FLASH_END - 4,
                                      setup_no_clocks));
    g_assert_true(m33_load_faults_on(machine, PART_C222F352_FLASH_END,
                                     setup_no_clocks));
    /* The default module keeps its 512 KB. */
    g_assert_false(m33_load_faults(PART_C222F352_FLASH_END, setup_no_clocks));
}

/*
 * RTCC (RM SS15.3): CMD.START runs CNT at the LF clock (LFRCO, 32.768 kHz
 * on this board, CNTPRESC DIV1); an output-compare channel sets IF.CCn when
 * CNT reaches OCVALUE.
 */
#define RTCC_BASE             0x48000000
#define RTCC_EN               0x004
#define RTCC_CMD              0x00c
#define RTCC_CMD_START        (1u << 0)
#define RTCC_STATUS           0x010
#define RTCC_IF               0x014
#define RTCC_IF_CC1           (1u << 6)
#define RTCC_CNT              0x020
#define RTCC_CC1_CTRL         0x03c
#define RTCC_CC1_OCVALUE      0x040
#define RTCC_CC_OUTPUTCOMPARE 0x2

static void test_rtcc_counts_and_compares(void)
{
    uint32_t cnt;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    writel(CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW, CMU_CLKEN0_RTCC);
    writel(RTCC_BASE + RTCC_EN, 1);
    writel(RTCC_BASE + RTCC_CMD, RTCC_CMD_START);
    g_assert_cmphex(readl(RTCC_BASE + RTCC_STATUS), ==, 1);
    clock_run(100 * LF_PERIOD_NS);
    cnt = readl(RTCC_BASE + RTCC_CNT);
    g_assert_cmpuint(cnt, >=, 99);
    g_assert_cmpuint(cnt, <=, 101);
    writel(RTCC_BASE + RTCC_CC1_CTRL, RTCC_CC_OUTPUTCOMPARE);
    writel(RTCC_BASE + RTCC_CC1_OCVALUE, cnt + 10);
    clock_run(8 * LF_PERIOD_NS);
    g_assert_cmphex(readl(RTCC_BASE + RTCC_IF) & RTCC_IF_CC1, ==, 0);
    clock_run(3 * LF_PERIOD_NS);
    g_assert_cmphex(readl(RTCC_BASE + RTCC_IF) & RTCC_IF_CC1, ==,
                    RTCC_IF_CC1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();
}

/*
 * PRORTC: an RTCC_TypeDef at 0xA8000000 (device header) on its own clock
 * (CMU.PRORTCCLKCTRL, reset LFRCO), separate from the RTCC.
 */
#define PRORTC_BASE           0xa8000000
#define CMU_CLKEN1_PRORTC     (1u << 10)

static void test_prortc_counts_and_compares(void)
{
    uint32_t cnt;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();
    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW, CMU_CLKEN1_PRORTC);
    writel(PRORTC_BASE + RTCC_EN, 1);
    writel(PRORTC_BASE + RTCC_CMD, RTCC_CMD_START);
    g_assert_cmphex(readl(PRORTC_BASE + RTCC_STATUS), ==, 1);
    clock_run(100 * LF_PERIOD_NS);
    cnt = readl(PRORTC_BASE + RTCC_CNT);
    g_assert_cmpuint(cnt, >=, 99);
    g_assert_cmpuint(cnt, <=, 101);
    writel(PRORTC_BASE + RTCC_CC1_CTRL, RTCC_CC_OUTPUTCOMPARE);
    writel(PRORTC_BASE + RTCC_CC1_OCVALUE, cnt + 10);
    clock_run(8 * LF_PERIOD_NS);
    g_assert_cmphex(readl(PRORTC_BASE + RTCC_IF) & RTCC_IF_CC1, ==, 0);
    clock_run(3 * LF_PERIOD_NS);
    g_assert_cmphex(readl(PRORTC_BASE + RTCC_IF) & RTCC_IF_CC1, ==,
                    RTCC_IF_CC1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();
}

/*
 * IADC single conversion of a board input (machine "analog" property):
 * PD02 at 968 mV against VDDX (3300 mV), gain 1, 12-bit right-aligned.
 */
#define IADC_BASE             0x4a004000
#define IADC_CMD              0x00c
#define IADC_STATUS           0x014
#define IADC_STATUS_FIFODV    (1u << 8)
#define IADC_IF               0x024
#define IADC_IF_SINGLEDONE    (1u << 9)
#define IADC_CFG0             0x048
#define IADC_CFG0_VDDX_GAIN1  0x00032000
#define IADC_SINGLEFIFODATA   0x074
#define IADC_SINGLE           0x098
#define IADC_SINGLE_PD02_GND  0x0000b200

/*
 * Timing (RM SS24.3.3, measured on silicon: docs/radio-tables.md SS7.89): at
 * reset the IADC clock is EM01GRPACLK = HFRCODPLL's 19 MHz; TIMEBASE 18
 * makes a 1 us interval, PRESCALE 1 a 9.5 MHz ADC_CLK.  WARMUPMODE NORMAL:
 * 5 intervals of warm-up then the conversion, every time; KEEPWARM: the
 * warm-up once, and again after EN is cleared.  A request's first result
 * takes (4 * OSR 2 + 2) = 10 ADC_CLK cycles plus the pipeline latency of
 * 9 ADC_CLK + 18 CLK_SRC_ADC cycles + 440 ns.  STATUS.ADCWARM is set from
 * the warm-up's end until a NORMAL conversion ends.
 */
#define IADC_EN               0x004
#define IADC_CTRL             0x008
#define IADC_CTRL_TIMEBASE_1US (18u << 16)
#define IADC_CTRL_KEEPWARM    (2u << 4)
#define IADC_SCHED0           0x054
#define IADC_STATUS_CONVERTING (1u << 6)
#define IADC_STATUS_ADCWARM   (1u << 30)
#define IADC_WARMUP_NS        (5 * 19 * 1000000000ull / 19000000)
#define IADC_CONV_NS          ((10 + 9) * 1000000000ull * 2 / 19000000 + \
                               18 * 1000000000ull / 19000000 + 440)

static void iadc_convert_after(QTestState *qts, uint64_t ns)
{
    qtest_writel(qts, IADC_BASE + IADC_CMD, 1);
    qtest_clock_step(qts, ns - 1);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_CONVERTING, ==, IADC_STATUS_CONVERTING);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    (IADC_STATUS_CONVERTING | IADC_STATUS_FIFODV), ==,
                    IADC_STATUS_FIFODV);
}

static void test_iadc_single_conversion(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22,analog=PD02=968 "
                                  "-accel qtest");

    qtest_writel(qts, IADC_BASE + IADC_CTRL, IADC_CTRL_TIMEBASE_1US);
    qtest_writel(qts, IADC_BASE + IADC_SCHED0, 1);
    qtest_writel(qts, IADC_BASE + IADC_CFG0, IADC_CFG0_VDDX_GAIN1);
    qtest_writel(qts, IADC_BASE + IADC_SINGLE, IADC_SINGLE_PD02_GND);
    iadc_convert_after(qts, IADC_WARMUP_NS + IADC_CONV_NS);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_ADCWARM, ==, 0);
    /* NORMAL: powered down again, so the next one warms up again. */
    qtest_writel(qts, IADC_BASE + IADC_CMD, 1);
    qtest_clock_step(qts, IADC_WARMUP_NS - 1);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_ADCWARM, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_ADCWARM, ==, IADC_STATUS_ADCWARM);
    qtest_clock_step(qts, IADC_CONV_NS);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    (IADC_STATUS_FIFODV | IADC_STATUS_ADCWARM), ==,
                    IADC_STATUS_FIFODV);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     1201);
    qtest_writel(qts, IADC_BASE + IADC_CTRL,
                 IADC_CTRL_TIMEBASE_1US | IADC_CTRL_KEEPWARM);
    iadc_convert_after(qts, IADC_WARMUP_NS + IADC_CONV_NS);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     1201);
    iadc_convert_after(qts, IADC_CONV_NS);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     1201);
    /* Disabled and enabled again, it warms up again. */
    qtest_writel(qts, IADC_BASE + IADC_EN, 0);
    qtest_writel(qts, IADC_BASE + IADC_EN, 1);
    iadc_convert_after(qts, IADC_WARMUP_NS + IADC_CONV_NS);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     1201);
    iadc_convert_after(qts, IADC_CONV_NS);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     1201);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_FIFODV, ==, IADC_STATUS_FIFODV);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_IF) &
                    IADC_IF_SINGLEDONE, ==, IADC_IF_SINGLEDONE);
    /* 968 / 3300 * 4095 = 1201.2 */
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     1201);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_FIFODV, ==, 0);
    qtest_quit(qts);
}

/*
 * Output resolution 11 + log2(OSR x DIGAVG) bits (RM SS24.3.7.4): OSR 4 and
 * 4-sample averaging give 15 bits, 968 mV of 3.3 V = 9612, read RIGHT16 as
 * 9612 << 1 = 19224 (its 12 MSBs are still 1201); the window comparator
 * (SS24.3.8) flags it inside ADGT 0x4000..ADLT 0x5000, not outside it.
 */
#define IADC_CMPTHR           0x01c
#define IADC_SINGLEFIFOCFG    0x070
#define IADC_CFG_OSR4         (1u << 2)
#define IADC_CFG_AVG4         (2u << 21)
#define IADC_ALIGN_RIGHT16    1u
#define IADC_SINGLE_CMP       (1u << 17)
#define IADC_IF_SINGLECMP     (1u << 2)

static void test_iadc_resolution_and_window(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22,analog=PD02=968 "
                                  "-accel qtest");

    qtest_writel(qts, IADC_BASE + IADC_CTRL, IADC_CTRL_TIMEBASE_1US);
    qtest_writel(qts, IADC_BASE + IADC_SCHED0, 1);
    qtest_writel(qts, IADC_BASE + IADC_CFG0, IADC_CFG0_VDDX_GAIN1 |
                                                 IADC_CFG_OSR4 | IADC_CFG_AVG4);
    qtest_writel(qts, IADC_BASE + IADC_SINGLEFIFOCFG, IADC_ALIGN_RIGHT16);
    qtest_writel(qts, IADC_BASE + IADC_SINGLE,
                 IADC_SINGLE_PD02_GND | IADC_SINGLE_CMP);
    qtest_writel(qts, IADC_BASE + IADC_CMPTHR, (0x4000u << 16) | 0x5000u);
    qtest_writel(qts, IADC_BASE + IADC_CMD, 1);
    qtest_clock_step(qts, 20000);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     19224);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_IF) & IADC_IF_SINGLECMP,
                    ==, IADC_IF_SINGLECMP);
    qtest_writel(qts, IADC_BASE + IADC_IF + REGISTER_CLEAR_WINDOW,
                 IADC_IF_SINGLECMP);
    qtest_writel(qts, IADC_BASE + IADC_CMPTHR, (0x5000u << 16) | 0x4000u);
    qtest_writel(qts, IADC_BASE + IADC_CMD, 1);
    qtest_clock_step(qts, 20000);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFODATA), ==,
                     19224);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_IF) & IADC_IF_SINGLECMP,
                    ==, 0);
    qtest_quit(qts);
}

/* EMU.RSTCAUSE: POR after power-up, cleared by EMU_CMD.RSTCAUSECLR. */
#define EMU_CMD               0x070
#define EMU_CMD_RSTCAUSECLR   (1u << 17)
#define EMU_RSTCAUSE          0x094
#define EMU_RSTCAUSE_POR      (1u << 0)

static void test_emu_rstcause(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    g_assert_cmphex(qtest_readl(qts, EMU_BASE + EMU_RSTCAUSE), ==,
                    EMU_RSTCAUSE_POR);
    qtest_writel(qts, EMU_BASE + EMU_CMD, EMU_CMD_RSTCAUSECLR);
    g_assert_cmphex(qtest_readl(qts, EMU_BASE + EMU_RSTCAUSE), ==, 0);
    qtest_quit(qts);
}

static void setup_radio_clocks(QTestState *qts)
{
    qtest_seq_enable_radio_bus_clocks(qts);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_SYSCFG);
}

static void setup_radio_clocks_but_rac(QTestState *qts)
{
    setup_radio_clocks(qts);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN1 + REGISTER_CLEAR_WINDOW,
                 CMU_CLKEN1_RAC);
}

static void setup_radio_clocks_but_bufc(QTestState *qts)
{
    setup_radio_clocks(qts);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN1 + REGISTER_CLEAR_WINDOW,
                 CMU_CLKEN1_BUFC);
}

/*
 * Bus clock gating, as measured on silicon by gating one CMU enable at a time
 * from the debugger (owo-open-fw docs/radio-tables.md SS7.72): a read of a
 * gated SYSCFG or radio block is a bus error, which the M33 takes as an
 * (escalated) HardFault; a gated write is dropped; CLKEN1.BUFC gates nothing
 * visible; the window at 0xa8008000 faults whatever the enables.
 */
static void test_gated_bus_clock_faults_and_drops(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_SYSCFG);
    qtest_writel(qts, SYSCFG_BASE + SYSCFG_ROOTDATA1, 0x12345678);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_CLEAR_WINDOW,
                 CMU_CLKEN0_SYSCFG);
    qtest_writel(qts, SYSCFG_BASE + SYSCFG_ROOTDATA1, 0xcafef00d);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_SYSCFG);
    g_assert_cmphex(qtest_readl(qts, SYSCFG_BASE + SYSCFG_ROOTDATA1), ==,
                    0x12345678);
    qtest_quit(qts);

    g_assert_true(m33_load_faults(SYSCFG_BASE + SYSCFG_ROOTDATA1,
                                  setup_no_clocks));
    g_assert_false(m33_load_faults(SYSCFG_BASE + SYSCFG_ROOTDATA1,
                                   setup_radio_clocks));
    g_assert_true(m33_load_faults(RAC_BASE + RAC_STATUS,
                                  setup_radio_clocks_but_rac));
    g_assert_false(m33_load_faults(RAC_BASE + RAC_STATUS,
                                   setup_radio_clocks));
    g_assert_false(m33_load_faults(BUFC_BASE + BUFC_CH0,
                                   setup_radio_clocks_but_bufc));
    g_assert_true(m33_load_faults(BUFC_BASE + BUFC_CH0, setup_no_clocks));
    g_assert_true(m33_load_faults(RADIO_HOLE_BASE, setup_radio_clocks));
}

/*
 * An output compare fires on the tick where CNT advances out of OC, so OC
 * equal to the count at START fires after one tick, not after a wrap.
 */
static void test_timer0_compare_equal_fires_next_tick(void)
{
    reset_with_radio_clocks();
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);
    writel(TIMER0_NS_BASE + TIMER_CC_CFG(0), TIMER_CC_CFG_OUTPUTCOMPARE);
    writel(TIMER0_NS_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CNT, 10);
    writel(TIMER0_NS_BASE + TIMER_CC_OC(0), 10);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    /* One tick at 38.4 MHz is 26.04 ns. */
    timer0_step(26);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, 0);
    timer0_step(1);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, TIMER_IF_CC(0));
}

#define TIMER_STATUS_SYNCBUSY (1u << 6)
#define TIMER_CMD_SYNC_NS     495 /* 38 HCLK cycles at 76.8 MHz */

static void test_timer_cmd_syncbusy(void)
{
    reset_with_radio_clocks();
    writel(TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER1_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex(readl(TIMER1_BASE + TIMER_STATUS), ==,
                    TIMER_STATUS_RUNNING | TIMER_STATUS_SYNCBUSY);
    timer0_step(TIMER_CMD_SYNC_NS);
    g_assert_cmphex(readl(TIMER1_BASE + TIMER_STATUS), ==,
                    TIMER_STATUS_RUNNING);
}

/* EN = 0 reads SYNC registers as zero and keeps their values. */
static void test_timer_sync_registers_read_zero_when_disabled(void)
{
    reset_with_radio_clocks();
    g_assert_cmphex(readl(TIMER3_BASE + TIMER_TOP), ==, 0);
    writel(TIMER3_BASE + TIMER_EN, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER3_BASE + TIMER_TOP), ==, 0xffff);
    writel(TIMER3_BASE + TIMER_TOP, 0x1234);
    writel(TIMER3_BASE + TIMER_EN, 0);
    g_assert_cmphex(readl(TIMER3_BASE + TIMER_TOP), ==, 0);
    writel(TIMER3_BASE + TIMER_EN, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER3_BASE + TIMER_TOP), ==, 0x1234);
}

static void setup_timer_clocks(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_TIMERS | CMU_CLKEN0_BURTC);
    qtest_writel(qts, CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
                 CMU_CLKEN1_TIMER4);
}

static void setup_rtcc_clock(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_RTCC);
}

static void setup_timer_clocks_enabled_timer1(QTestState *qts)
{
    setup_timer_clocks(qts);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
}

/* A store that should fault: r0 = address; str r1, [r0]; marker. */
static bool m33_store_faults(uint32_t address, void (*setup)(QTestState *qts))
{
    const uint32_t code = 0x100, handler = 0x180, result = 0x20000000;
    const uint32_t fault_marker = 0xfa0170ffu, stored_marker_bit = 1u << 31;
    uint16_t insn[8], hinsn[6];
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");
    uint32_t outcome;
    size_t i;

    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, code | 1);
    qtest_writel(qts, 0x00000008, handler | 1);
    qtest_writel(qts, 0x0000000c, handler | 1);
    insn[0] = thumb_ldr_pc(0, code + 0, code + 16);
    insn[1] = thumb_str_imm(1, 0, 0);
    insn[2] = thumb_ldr_pc(2, code + 4, code + 20);
    insn[3] = thumb_ldr_pc(3, code + 6, code + 24);
    insn[4] = thumb_str_imm(3, 2, 0);
    insn[5] = THUMB16_BRANCH_SELF;
    insn[6] = THUMB16_BRANCH_SELF;
    insn[7] = THUMB16_BRANCH_SELF;
    for (i = 0; i < 8; i += 2) {
        qtest_writel(qts, code + 2 * i, THUMB16_PAIR(insn[i], insn[i + 1]));
    }
    qtest_writel(qts, code + 16, address);
    qtest_writel(qts, code + 20, result);
    qtest_writel(qts, code + 24, stored_marker_bit);
    hinsn[0] = thumb_ldr_pc(2, handler + 0, handler + 12);
    hinsn[1] = thumb_ldr_pc(1, handler + 2, handler + 16);
    hinsn[2] = thumb_str_imm(1, 2, 0);
    hinsn[3] = THUMB16_BRANCH_SELF;
    hinsn[4] = THUMB16_BRANCH_SELF;
    hinsn[5] = THUMB16_BRANCH_SELF;
    for (i = 0; i < 6; i += 2) {
        qtest_writel(qts, handler + 2 * i,
                     THUMB16_PAIR(hinsn[i], hinsn[i + 1]));
    }
    qtest_writel(qts, handler + 12, result);
    qtest_writel(qts, handler + 16, fault_marker);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_writel(qts, result, 0);
    setup(qts);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, result)) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    outcome = qtest_readl(qts, result);
    qtest_quit(qts);
    g_assert(outcome == fault_marker || outcome == stored_marker_bit);
    return outcome == fault_marker;
}

/*
 * TIMERn and BURTC fault on any access while their CMU clock is off; a
 * TIMER faults SYNC-register writes while EN = 0 and CONFIG-register writes
 * while EN = 1.
 */
static void test_timer_bus_faults(void)
{
    g_assert_true(m33_load_faults(TIMER0_BASE + TIMER_CFG, setup_no_clocks));
    g_assert_true(m33_load_faults(TIMER4_BASE + TIMER_CFG, setup_no_clocks));
    g_assert_true(m33_load_faults(BURTC_BASE + BURTC_CNT, setup_no_clocks));
    g_assert_false(m33_load_faults(TIMER0_BASE + TIMER_CFG,
                                   setup_timer_clocks));
    g_assert_false(m33_load_faults(BURTC_BASE + BURTC_CNT,
                                   setup_timer_clocks));
    /* RTCC likewise with CLKEN0.RTCC (silicon, SS7.73). */
    g_assert_true(m33_load_faults(RTCC_BASE + RTCC_CNT, setup_no_clocks));
    g_assert_false(m33_load_faults(RTCC_BASE + RTCC_CNT, setup_rtcc_clock));
    g_assert_true(m33_store_faults(TIMER2_BASE + TIMER_CFG, setup_no_clocks));
    g_assert_true(m33_store_faults(TIMER1_BASE + TIMER_CMD,
                                   setup_timer_clocks));
    g_assert_false(m33_store_faults(TIMER1_BASE + TIMER_CFG,
                                    setup_timer_clocks));
    g_assert_true(m33_store_faults(TIMER1_BASE + TIMER_CFG,
                                   setup_timer_clocks_enabled_timer1));
    g_assert_false(m33_store_faults(TIMER1_BASE + TIMER_CMD,
                                    setup_timer_clocks_enabled_timer1));
}


static QTestState *seq_release_with_program(const uint16_t *insns,
                                            size_t n_insns,
                                            const uint32_t *literals,
                                            size_t n_literals);
static void seq_load_and_release(QTestState *qts, const uint16_t *insns,
                                 size_t n_insns, const uint32_t *literals,
                                 size_t n_literals);

/*
 * Virtual-time checkpoint for TCG tests: under -icount sleep=off virtual time
 * advances only with executed instructions, so a fixed M33 countdown marks
 * the same virtual instant on every run, whatever the host load.  The M33
 * counts SEQ_CHECKPOINT_LOOPS two-instruction iterations (about 0.5-1 ms of
 * virtual time at HCLK) and then stores 1 to SEQ_CHECKPOINT_MARKER.
 */
#define SEQ_CHECKPOINT_LOOPS   10000u
#define SEQ_CHECKPOINT_MARKER  0x20001000u
#define SEQ_CHECKPOINT_CODE    0x00000100u
#define SEQ_THUMB16_SUBS_R3_1  0x3b01u
#define SEQ_THUMB16_BNE_BACK_ONE 0xd1fdu

static QTestState *seq_release_at_checkpoint(const uint16_t *insns,
                                             size_t n_insns)
{
    const uint32_t code = SEQ_CHECKPOINT_CODE;
    const uint32_t literals = code + 16;
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg "
                                  "-icount shift=5,sleep=off -S");
    uint16_t m33[8];
    size_t i;

    m33[0] = thumb_ldr_pc(0, code + 0, literals + 0);  /* marker address */
    m33[1] = thumb_ldr_pc(3, code + 2, literals + 4);  /* loop count */
    m33[2] = SEQ_THUMB16_SUBS_R3_1;
    m33[3] = SEQ_THUMB16_BNE_BACK_ONE;
    m33[4] = thumb_movs_imm8(1, 1);
    m33[5] = thumb_str_imm(1, 0, 0);
    m33[6] = THUMB16_BRANCH_SELF;
    m33[7] = THUMB16_BRANCH_SELF;
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, code | 1);
    for (i = 0; i < G_N_ELEMENTS(m33); i += 2) {
        qtest_writel(qts, code + 2 * i, THUMB16_PAIR(m33[i], m33[i + 1]));
    }
    qtest_writel(qts, literals + 0, SEQ_CHECKPOINT_MARKER);
    qtest_writel(qts, literals + 4, SEQ_CHECKPOINT_LOOPS);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_writel(qts, SEQ_CHECKPOINT_MARKER, 0);
    seq_load_and_release(qts, insns, n_insns, NULL, 0);
    /* Real time only bounds a hung QEMU; the outcome is fixed by icount. */
    for (unsigned attempt = 0; attempt < 60000; attempt++) {
        if (qtest_readl(qts, SEQ_CHECKPOINT_MARKER)) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmpuint(qtest_readl(qts, SEQ_CHECKPOINT_MARKER), ==, 1);
    return qts;
}

/*
 * RAC.STATUS.SEQSLEEPING is set while the sequencer core sleeps in WFI and
 * clear while it runs (silicon: halted or in a handler, SS7.72).  Read at a
 * virtual-time checkpoint, not after a real-time sleep: a loaded host used
 * to read it before the M0+ had reached its WFI.
 */
static void test_seq_sleeping_follows_wfi(void)
{
    static const uint16_t sleeper[] = { THUMB16_WFI, THUMB16_BRANCH_BACK_ONE };
    static const uint16_t spinner[] = { THUMB16_BRANCH_SELF,
                                        THUMB16_BRANCH_SELF };
    QTestState *qts = seq_release_at_checkpoint(sleeper, 2);

    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_SEQSLEEPING, ==, RAC_STATUS_SEQSLEEPING);
    qtest_quit(qts);

    qts = seq_release_at_checkpoint(spinner, 2);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_SEQSLEEPING, ==, 0);
    qtest_quit(qts);
}

/*
 * Lockup and EMU_RSTCTRL (RM 12.5.14, 12.5.15; owo-open-fw
 * docs/radio-tables.md SS7.58).  Each program counts its runs in a RAM word
 * and then faults (or requests a reset); the HardFault vector points back
 * at the program, so its second fault, inside HardFault, is a Lockup.
 */
#define LOCKUP_COUNTER_M33     0x20001000u
#define LOCKUP_EMU_RSTCTRL     (EMU_BASE + 0x090)
#define LOCKUP_RSTCTRL_RESET   0x00010405u
#define LOCKUP_RSTCTRL_SYSRMODE (1u << 2)
#define LOCKUP_RSTCTRL_LOCKUPRMODE (1u << 3)
#define LOCKUP_RSTCAUSE        (EMU_BASE + 0x094)
#define LOCKUP_RSTCAUSE_LOCKUP (1u << 5)
#define LOCKUP_RSTCAUSE_SYSREQ (1u << 6)

/* ldr r0,=ctr; ldr r1,[r0]; adds r1,#1; str r1,[r0]; ldr r2,=bad; ldr r3,[r2] */
static const uint32_t lockup_program[] = {
    0x68014803, 0x60013101, 0x68134a02, 0xbf00e7fe,
};
/* ...; ldr r2,=AIRCR; ldr r3,=VECTKEY|SYSRESETREQ; str r3,[r2]; b . */
static const uint32_t sysreq_program[] = {
    0x68014803, 0x60013101, 0x4b034a02, 0xe7fe6013,
};

static QTestState *m33_run(const uint32_t *code, size_t n, uint32_t lit1,
                           uint32_t lit2, uint32_t rstctrl)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    qtest_writel(qts, 0x0, 0x20008000);
    qtest_writel(qts, 0x4, 0x101);
    qtest_writel(qts, 0xc, 0x101);  /* HardFault: the program again */
    for (size_t i = 0; i < n; i++) {
        qtest_writel(qts, 0x100 + 4 * i, code[i]);
    }
    qtest_writel(qts, 0x110, LOCKUP_COUNTER_M33);
    qtest_writel(qts, 0x114, lit1);
    qtest_writel(qts, 0x118, lit2);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_writel(qts, LOCKUP_COUNTER_M33, 0);
    qtest_writel(qts, LOCKUP_EMU_RSTCTRL, rstctrl);
    qtest_writel(qts, EMU_BASE + EMU_CMD, EMU_CMD_RSTCAUSECLR);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    return qts;
}

static uint32_t wait_counter(QTestState *qts, uint32_t addr, uint32_t min)
{
    for (unsigned i = 0; i < 5000 && qtest_readl(qts, addr) < min; i++) {
        g_usleep(1000);
    }
    return qtest_readl(qts, addr);
}

/*
 * An M33 Lockup with LOCKUPRMODE 0 (the reset value) leaves the core locked
 * and QEMU running; with LOCKUPRMODE 1 it resets the device, RSTCAUSE.LOCKUP.
 */
static void test_m33_lockup(void)
{
    const uint32_t unmapped = 0x30000000;
    QTestState *qts = m33_run(lockup_program, 4, unmapped, 0,
                              LOCKUP_RSTCTRL_RESET);

    g_assert_cmpuint(wait_counter(qts, LOCKUP_COUNTER_M33, 2), ==, 2);
    g_usleep(50000);
    g_assert_cmpuint(qtest_readl(qts, LOCKUP_COUNTER_M33), ==, 2);
    g_assert_cmphex(qtest_readl(qts, LOCKUP_RSTCAUSE) &
                    LOCKUP_RSTCAUSE_LOCKUP, ==, 0);
    g_assert_cmphex(qtest_readl(qts, LOCKUP_EMU_RSTCTRL), ==,
                    LOCKUP_RSTCTRL_RESET);
    qtest_quit(qts);

    qts = m33_run(lockup_program, 4, unmapped, 0,
                  LOCKUP_RSTCTRL_RESET | LOCKUP_RSTCTRL_LOCKUPRMODE);
    g_assert_cmpuint(wait_counter(qts, LOCKUP_COUNTER_M33, 6), >=, 6);
    g_assert_cmphex(qtest_readl(qts, LOCKUP_RSTCAUSE) &
                    LOCKUP_RSTCAUSE_LOCKUP, ==, LOCKUP_RSTCAUSE_LOCKUP);
    /* RSTCTRL survives the soft resets it allowed */
    g_assert_cmphex(qtest_readl(qts, LOCKUP_EMU_RSTCTRL), ==,
                    LOCKUP_RSTCTRL_RESET | LOCKUP_RSTCTRL_LOCKUPRMODE);
    qtest_quit(qts);
}

/* SYSRESETREQ resets with SYSRMODE 1 (reset value) and is blocked with 0. */
static void test_rstctrl_blocks_sysresetreq(void)
{
    const uint32_t aircr = 0xe000ed0c, sysresetreq = 0x05fa0004;
    QTestState *qts = m33_run(sysreq_program, 4, aircr, sysresetreq,
                              LOCKUP_RSTCTRL_RESET);

    g_assert_cmpuint(wait_counter(qts, LOCKUP_COUNTER_M33, 4), >=, 4);
    g_assert_cmphex(qtest_readl(qts, LOCKUP_RSTCAUSE) &
                    LOCKUP_RSTCAUSE_SYSREQ, ==, LOCKUP_RSTCAUSE_SYSREQ);
    qtest_quit(qts);

    qts = m33_run(sysreq_program, 4, aircr, sysresetreq,
                  LOCKUP_RSTCTRL_RESET & ~LOCKUP_RSTCTRL_SYSRMODE);
    g_assert_cmpuint(wait_counter(qts, LOCKUP_COUNTER_M33, 1), ==, 1);
    g_usleep(50000);
    g_assert_cmpuint(qtest_readl(qts, LOCKUP_COUNTER_M33), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LOCKUP_RSTCAUSE) &
                    LOCKUP_RSTCAUSE_SYSREQ, ==, 0);
    qtest_quit(qts);
}

/*
 * The radio M0+ (Armv6-M) locks up on a fault inside HardFault and stays
 * locked -- no reset follows -- until SEQRESET (silicon, SS7.58).  Its
 * load from the M33-only non-secure radio alias 0xb8000000 faults.
 */
static void seq_release_lockup_program(QTestState *qts)
{
    static const uint32_t code[] = {
        0x68014803, 0x60013101, 0x68134a02, 0xbf00e7fe,
        SEQ_SPARE0, 0xb8000000,
    };

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x00c, 0x00000101);
    for (size_t i = 0; i < G_N_ELEMENTS(code); i++) {
        qtest_writel(qts, SEQRAM_NS_BASE + 0x100 + 4 * i, code[i]);
    }
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
}

static void test_seq_lockup_until_seqreset(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");
    const uint32_t counter = SEQRAM_NS_BASE + SEQ_SPARE0;

    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_seq_enable_radio_bus_clocks(qts);
    qtest_writel(qts, counter, 0);
    seq_release_lockup_program(qts);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_assert_cmpuint(wait_counter(qts, counter, 2), ==, 2);
    g_usleep(50000);
    g_assert_cmpuint(qtest_readl(qts, counter), ==, 2);

    /* SEQRESET and a new release run it again: 3, then 4 and Lockup. */
    seq_release_lockup_program(qts);
    g_assert_cmpuint(wait_counter(qts, counter, 4), ==, 4);
    qtest_quit(qts);
}

/* RAIL_StartTx clears RAC.SR0 bit 5 through the Series-2 CLR alias. */
static void test_rac_sr0_clear_alias(void)
{
    writel(RAC_BASE + RAC_SR0, UINT32_MAX);
    writel(RAC_BASE + RAC_SR0 + REGISTER_CLEAR_WINDOW, 1u << 5);

    g_assert_cmphex(readl(RAC_BASE + RAC_SR0), ==, UINT32_MAX & ~(1u << 5));
}

static void test_rac_force_disable(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_TX);

    writel(RAC_BASE + RAC_CTRL, RAC_CTRL_FORCEDISABLE);
    g_assert_cmphex(readl(RAC_BASE + RAC_CTRL), ==, RAC_CTRL_FORCEDISABLE);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_SHUTDOWN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);

    writel(RAC_BASE + RAC_CTRL, 0);
    g_assert_cmphex(readl(RAC_BASE + RAC_CTRL), ==, 0);
}

/*
 * RAC.STCTRL's only two documented fields -- STCAL[23:0] and STSKEW[24]
 * (RAC_S_field.py) -- are the whole writable mask; everything else must be
 * dropped. Checked against the real literal value (0x01061a7f) stock's own
 * sequencer reset handler writes (docs/radio-tables.md SS7.8/SS7.9): that
 * value already fits the mask exactly, so this also pins the mask against
 * silently regressing to something that would truncate it.
 */
static void test_rac_stctrl_mask(void)
{
    writel(RAC_BASE + RAC_STCTRL, RAC_STCTRL_STOCK_VALUE);
    g_assert_cmphex(readl(RAC_BASE + RAC_STCTRL), ==, RAC_STCTRL_STOCK_VALUE);

    writel(RAC_BASE + RAC_STCTRL, UINT32_MAX);
    g_assert_cmphex(readl(RAC_BASE + RAC_STCTRL), ==,
                    RAC_STCTRL_WRITABLE_MASK);
}

/*
 * Release the AP3 sequencer core exactly as
 * test_radio_sequencer_executes_open_code does, then load an arbitrary
 * caller-assembled Thumb-16 program (instruction count must be even)
 * immediately followed by its literal pool, both at the same
 * SEQRAM_NS_BASE + 0x100 the other sequencer probes use. SEQ.SPARE0 and
 * the seven words after it are zeroed first and are the program's
 * exfiltration channel back to the qtest side. Returns with the core
 * already running ('cont' issued); callers stop/cont it themselves.
 */
static QTestState *seq_release_with_program(const uint16_t *insns,
                                            size_t n_insns,
                                            const uint32_t *literals,
                                            size_t n_literals)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    /* Keep the image-less application M33 in a valid idle loop while TCG
     * executes the independently released auxiliary core. */
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    seq_load_and_release(qts, insns, n_insns, literals, n_literals);
    return qts;
}

/* Load the sequencer program after the M33 reset, release AP3, 'cont'. */
static void seq_load_and_release(QTestState *qts, const uint16_t *insns,
                                 size_t n_insns, const uint32_t *literals,
                                 size_t n_literals)
{
    const uint32_t code_base = SEQRAM_NS_BASE + 0x100;
    size_t i;

    g_assert_cmpuint(n_insns % 2, ==, 0);
    qtest_seq_enable_radio_bus_clocks(qts);

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);

    for (i = 0; i < n_insns; i += 2) {
        qtest_writel(qts, code_base + 2 * i,
                    THUMB16_PAIR(insns[i], insns[i + 1]));
    }
    for (i = 0; i < n_literals; i++) {
        qtest_writel(qts, code_base + 2 * n_insns + 4 * i, literals[i]);
    }
    for (i = 0; i < 8; i++) {
        qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 4 * i, 0);
    }

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
}

/*
 * IRQ8/IRQ9 are the two interrupts this project's radio_seq.S actually
 * services (TXWARM and TX). Stock's own sequencer reset handler sets both
 * to priority 1 through NVIC.IPR's byte-addressable aliases specifically
 * so a single `strb` touches only its own IRQ's priority byte, never a
 * neighboring one packed into the same 32-bit IPR word
 * (docs/radio-tables.md SS7.9). This proves that behavior for AP3's own
 * private NVIC instance -- not the M33's, which qtest's own memory
 * access path always reaches first -- by having AP3 itself write and
 * read back IRQ8..IRQ11's priority bytes (IRQ10/IRQ11 sharing NVIC.IPR2
 * with IRQ8/IRQ9) and publish the results to shared RAM.
 *
 * It also pins down that AP3's NVIC is genuinely Armv6-M (2 priority bits
 * implemented, not the M33's 8): every byte written here truncates to one
 * of only four possible stored values (0x00/0x40/0x80/0xc0), which is
 * itself a real, previously-unrecorded consequence of stock writing the
 * literal priority value 1 -- it quantizes away to the same priority 0
 * the core already resets to.
 */
static void test_seq_nvic_ipr_byte_isolation(void)
{
    static const uint32_t ipr2_addr = 0xe000e408; /* NVIC.IPR2: IRQ8..11 */
    const uint32_t code_base = SEQRAM_NS_BASE + 0x100;
    uint16_t insn[22];
    uint32_t literals[3];
    QTestState *qts;
    unsigned n = 0;

    insn[n++] = 0; /* [0] LDR r0, [PC, #...] -> ipr2_addr (patched below) */
    insn[n++] = 0; /* [1] LDR r1, [PC, #...] -> SEQ_SPARE0 (patched below) */
    insn[n++] = thumb_movs_imm8(2, 0x80);        /* [2] sentinel for IRQ10 */
    insn[n++] = thumb_strb_imm(2, 0, 2);         /* [3] */
    insn[n++] = thumb_movs_imm8(2, 0x00);        /* [4] sentinel for IRQ11 */
    insn[n++] = thumb_strb_imm(2, 0, 3);         /* [5] */
    insn[n++] = thumb_movs_imm8(2, 0x40);        /* [6] IRQ8 priority byte */
    insn[n++] = thumb_strb_imm(2, 0, 0);         /* [7] */
    insn[n++] = thumb_movs_imm8(2, 0xc0);        /* [8] IRQ9 priority byte */
    insn[n++] = thumb_strb_imm(2, 0, 1);         /* [9] */
    insn[n++] = thumb_ldrb_imm(3, 0, 0);         /* [10] read IRQ8 back */
    insn[n++] = thumb_str_imm(3, 1, 0);          /* [11] -> SPARE0+0 */
    insn[n++] = thumb_ldrb_imm(3, 0, 1);         /* [12] read IRQ9 back */
    insn[n++] = thumb_str_imm(3, 1, 1);          /* [13] -> SPARE0+4 */
    insn[n++] = thumb_ldrb_imm(3, 0, 2);         /* [14] read IRQ10 back */
    insn[n++] = thumb_str_imm(3, 1, 2);          /* [15] -> SPARE0+8 */
    insn[n++] = thumb_ldrb_imm(3, 0, 3);         /* [16] read IRQ11 back */
    insn[n++] = thumb_str_imm(3, 1, 3);          /* [17] -> SPARE0+12 */
    insn[n++] = 0; /* [18] LDR r4, [PC, #...] -> done marker (patched) */
    insn[n++] = thumb_str_imm(4, 1, 4);          /* [19] -> SPARE0+16 */
    insn[n++] = THUMB16_BRANCH_SELF;             /* [20] park here, done */
    insn[n++] = THUMB16_BRANCH_SELF;             /* [21] padding */
    g_assert_cmpuint(n, ==, 22);

    literals[0] = ipr2_addr;
    literals[1] = SEQ_SPARE0;
    literals[2] = 0xabcd1234u; /* arbitrary completion marker */
    insn[0] = thumb_ldr_pc(0, code_base + 0, code_base + 2 * n);
    insn[1] = thumb_ldr_pc(1, code_base + 2, code_base + 2 * n + 4);
    insn[18] = thumb_ldr_pc(4, code_base + 2 * 18, code_base + 2 * n + 8);

    qts = seq_release_with_program(insn, n, literals, 3);

    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 16) ==
            0xabcd1234u) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 16), ==,
                    0xabcd1234u);
    /* IRQ8: wrote 0x40 (priority bits 01) -- untruncated, since the two
     * implemented priority bits are exactly its top two. */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 0), ==,
                    0x40);
    /* IRQ9: wrote 0xc0 (priority bits 11). */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 4), ==,
                    0xc0);
    /* IRQ10: never touched after its 0x80 sentinel -- a word-wide or
     * neighbor-clobbering write would corrupt this. */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 8), ==,
                    0x80);
    /* IRQ11: never touched after its 0x00 sentinel. */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 12), ==,
                    0x00);
    qtest_quit(qts);
}

/*
 * This core's own private Armv6-M SysTick (0xE000E010) is separate from
 * the M33's own SysTick -- stock's reset handler configures it (from
 * SysTick.CALIB, enabled from the processor clock, no interrupt) right
 * after the NVIC.IPR loop above (docs/radio-tables.md SS7.9). This test
 * reads CALIB purely for the record: this model reports NOREF here
 * because no refclk is wired into either core's SysTick, and this
 * project has no confirmed real-hardware CALIB value to assert against
 * (STCAL/CALIB is explicitly recorded as "an opaque empirical value this
 * project has no theory for" -- not invented here either). The actual
 * counting proof below instead drives RELOAD/CSR from a fixed,
 * deliberately-arbitrary test constant, so it is testing the real
 * Armv6-M counting mechanism for AP3's own SysTick instance, not a claimed
 * real-hardware timing value.
 */
static void test_seq_systick_counts(void)
{
    static const uint32_t systick_base = 0xe000e010;
    static const uint32_t test_reload = 0x00ffffffu;
    const uint32_t code_base = SEQRAM_NS_BASE + 0x100;
    uint16_t insn[12];
    uint32_t literals[3];
    QTestState *qts;
    uint32_t val1, val2;

    insn[0] = 0; /* LDR r0, [PC, #...] -> systick_base (patched below) */
    insn[1] = 0; /* LDR r1, [PC, #...] -> SEQ_SPARE0 (patched below) */
    insn[2] = thumb_ldr_imm(4, 0, 3);    /* r4 = CALIB (base+12) */
    insn[3] = thumb_str_imm(4, 1, 4);    /* SPARE0+16 = CALIB, for the record */
    insn[4] = 0; /* LDR r5, [PC, #...] -> test_reload (patched below) */
    insn[5] = thumb_str_imm(5, 0, 1);    /* RELOAD (base+4) = test_reload */
    insn[6] = thumb_movs_imm8(6, 5);     /* r6 = ENABLE | CLKSOURCE */
    insn[7] = thumb_str_imm(6, 0, 0);    /* CSR (base+0) = r6 */
    insn[8] = thumb_ldr_imm(2, 0, 2);    /* loop: r2 = CURRENT (base+8) */
    insn[9] = thumb_str_imm(2, 1, 0);    /* SPARE0 = r2 */
    insn[10] = thumb_b(code_base + 2 * 10, code_base + 2 * 8); /* -> loop */
    insn[11] = THUMB16_BRANCH_SELF;      /* padding, never reached */

    literals[0] = systick_base;
    literals[1] = SEQ_SPARE0;
    literals[2] = test_reload;
    insn[0] = thumb_ldr_pc(0, code_base + 0, code_base + 24);
    insn[1] = thumb_ldr_pc(1, code_base + 2, code_base + 28);
    insn[4] = thumb_ldr_pc(5, code_base + 8, code_base + 32);

    qts = seq_release_with_program(insn, 12, literals, 3);

    /* The virtual clock only advances while the VM is actually running;
     * stop before reading so the two samples below can't race the loop
     * still rewriting SPARE0. */
    g_usleep(5000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    val1 = qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0);

    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(5000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    val2 = qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0);

    g_assert_cmpuint(val1, <=, test_reload);
    g_assert_cmpuint(val2, <=, test_reload);
    g_assert_cmphex(val1, !=, val2);

    qtest_quit(qts);
}

/*
 * Per-CPU clocks under -icount (owo-open-fw docs/radio-tables.md SS7.73).
 * On silicon the M33 runs at HCLK and the M0+ at SYSCLK / (RHCLKPRESC + 1),
 * each at its own rate while the other runs; firmware code averaged 1.84
 * M33 cycles per instruction and M0+ code follows the Armv6-M timings, and
 * a radio-register access costs extra cycles.  Each core here times a loop
 * with its own SysTick (core cycles) and with RAC.STIMER (4.8 ticks/us of
 * virtual time), both cores at once.
 */
#define CORE_CLOCK_SYSTICK_BASE      0xe000e010u
#define CORE_CLOCK_SYSTICK_FULL      0x00ffffffu
#define CORE_CLOCK_SYSTICK_ON_CORE   5u    /* ENABLE | CLKSOURCE */
#define CORE_CLOCK_STIMER            (RAC_BASE + 0x030)
#define CORE_CLOCK_STIMER_PER_US     4.8
#define CORE_CLOCK_LOOPS             4800u
#define CORE_CLOCK_M33_RESULT        0x20001000u
#define CORE_CLOCK_SEQ_RESULT_LOCAL  0x00003000u
#define CORE_CLOCK_SEQ_RESULT        (SEQRAM_NS_BASE + 0x3000)
#define CORE_CLOCK_CMU_SYSCLK_HFXO   3u
#define CORE_CLOCK_CMU_RHCLK_DIV2    (1u << 16)
#define CORE_CLOCK_HFXO_HZ           38400000.0
#define CORE_CLOCK_M33_CPI           1.84
#define CORE_CLOCK_SEQ_LOOP_CYCLES   3.0
#define THUMB16_SUBS_R3_1            0x3b01u
#define THUMB16_BNE_BACK_ONE         0xd1fdu  /* bne to the insn before */

/*
 * Write a probe at @code (address as the core sees it: @local) that stores
 * {SysTick before, SysTick after, STIMER before, STIMER after} around
 * CORE_CLOCK_LOOPS iterations of a two-instruction loop.  Between the
 * SysTick samples it executes 2 * CORE_CLOCK_LOOPS + 2 instructions, one
 * of them a radio-register load.
 */
static void core_clock_write_probe(QTestState *qts, uint32_t code,
                                   uint32_t local, uint32_t result)
{
    uint16_t insn[20];
    uint32_t lit = local + 2 * G_N_ELEMENTS(insn);
    uint32_t literals[] = { CORE_CLOCK_SYSTICK_BASE, CORE_CLOCK_SYSTICK_FULL,
                            CORE_CLOCK_STIMER, CORE_CLOCK_LOOPS, result };
    size_t i;

    insn[0] = thumb_ldr_pc(0, local + 0, lit + 0);
    insn[1] = thumb_ldr_pc(1, local + 2, lit + 4);
    insn[2] = thumb_str_imm(1, 0, 1);          /* RELOAD */
    insn[3] = thumb_movs_imm8(1, CORE_CLOCK_SYSTICK_ON_CORE);
    insn[4] = thumb_str_imm(1, 0, 0);          /* CSR */
    insn[5] = thumb_ldr_pc(6, local + 10, lit + 8);
    insn[6] = thumb_ldr_pc(3, local + 12, lit + 12);
    insn[7] = thumb_ldr_imm(2, 0, 2);          /* SysTick before */
    insn[8] = thumb_ldr_imm(7, 6, 0);          /* STIMER before */
    insn[9] = THUMB16_SUBS_R3_1;
    insn[10] = THUMB16_BNE_BACK_ONE;
    insn[11] = thumb_ldr_imm(4, 0, 2);         /* SysTick after */
    insn[12] = thumb_ldr_imm(5, 6, 0);         /* STIMER after */
    insn[13] = thumb_ldr_pc(1, local + 26, lit + 16);
    insn[14] = thumb_str_imm(2, 1, 0);
    insn[15] = thumb_str_imm(4, 1, 1);
    insn[16] = thumb_str_imm(7, 1, 2);
    insn[17] = thumb_str_imm(5, 1, 3);
    insn[18] = THUMB16_BRANCH_SELF;
    insn[19] = THUMB16_BRANCH_SELF;
    for (i = 0; i < G_N_ELEMENTS(insn); i += 2) {
        qtest_writel(qts, code + 2 * i, THUMB16_PAIR(insn[i], insn[i + 1]));
    }
    for (i = 0; i < G_N_ELEMENTS(literals); i++) {
        qtest_writel(qts, code + 2 * G_N_ELEMENTS(insn) + 4 * i, literals[i]);
    }
    for (i = 0; i < 4; i++) {
        qtest_writel(qts, result == CORE_CLOCK_SEQ_RESULT_LOCAL ?
                     CORE_CLOCK_SEQ_RESULT + 4 * i : result + 4 * i, 0);
    }
}

static void core_clock_check(QTestState *qts, uint32_t result,
                             double expected_cycles, double hz)
{
    uint32_t c0 = 0, c1, s0, s1;
    double cycles, us;

    for (unsigned attempt = 0; attempt < 5000; attempt++) {
        if (qtest_readl(qts, result + 12)) {
            break;
        }
        g_usleep(1000);
    }
    c0 = qtest_readl(qts, result);
    c1 = qtest_readl(qts, result + 4);
    s0 = qtest_readl(qts, result + 8);
    s1 = qtest_readl(qts, result + 12);
    g_assert_cmpuint(s1, !=, 0);
    cycles = (double)((c0 - c1) & CORE_CLOCK_SYSTICK_FULL);
    us = (double)((s1 - s0) & 0xffffu) / CORE_CLOCK_STIMER_PER_US;
    g_assert_cmpfloat(cycles, >=, expected_cycles * 0.99);
    g_assert_cmpfloat(cycles, <=, expected_cycles * 1.01);
    /* Core clock against virtual time: within STIMER's resolution. */
    g_assert_cmpfloat(cycles / us * 1e6, >=, hz * 0.98);
    g_assert_cmpfloat(cycles / us * 1e6, <=, hz * 1.02);
}

static QTestState *core_clock_start(const char *machine_options)
{
    QTestState *qts = qtest_initf("-machine efr32bg22%s -accel tcg "
                                  "-icount shift=5,sleep=off -S",
                                  machine_options);
    const uint32_t m33_code = 0x100, seq_code = 0x100;

    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, m33_code | 1);
    core_clock_write_probe(qts, m33_code, m33_code, CORE_CLOCK_M33_RESULT);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_seq_enable_radio_bus_clocks(qts);
    /* SYSCLK on HFXO, the radio HCLK prescaler at /2. */
    qtest_writel(qts, CMU_BASE + CMU_SYSCLKCTRL,
                 CORE_CLOCK_CMU_SYSCLK_HFXO | CORE_CLOCK_CMU_RHCLK_DIV2);
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, seq_code | 1);
    core_clock_write_probe(qts, SEQRAM_NS_BASE + seq_code, seq_code,
                           CORE_CLOCK_SEQ_RESULT_LOCAL);
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    return qts;
}

/* Both cores at once, each at its own rate: M33 at HCLK = SYSCLK = 38.4 MHz,
 * the M0+ at SYSCLK / 2 = 19.2 MHz. */
static void test_core_clocks_per_cpu_rates(void)
{
    QTestState *qts = core_clock_start("");

    /* M33: the measured firmware average, 1.84 cycles per instruction. */
    core_clock_check(qts, CORE_CLOCK_M33_RESULT,
                     CORE_CLOCK_M33_CPI * (2.0 * CORE_CLOCK_LOOPS + 2),
                     CORE_CLOCK_HFXO_HZ);
    /* M0+: Armv6-M timings, SUBS 1 + taken BNE 2 per iteration. */
    core_clock_check(qts, CORE_CLOCK_SEQ_RESULT,
                     CORE_CLOCK_SEQ_LOOP_CYCLES * CORE_CLOCK_LOOPS,
                     CORE_CLOCK_HFXO_HZ / 2);
    qtest_quit(qts);
}

/*
 * RADIOCLKCTRL.EN clear stops the M0+ with its state kept (silicon,
 * SS7.73); setting it again resumes it.  The sequencer counts in SEQRAM.
 */
static void test_core_clocks_radio_clock_stops_seq(void)
{
    static const uint16_t counter[] = {
        0, /* ldr r0, =SEQ_SPARE0 (patched below) */
        THUMB16_LDR_R1_R0, THUMB16_ADDS_R1_1, THUMB16_STR_R1_R0,
        0xe7fbu, /* b to the load */
        THUMB16_BRANCH_SELF,
    };
    uint16_t insn[G_N_ELEMENTS(counter)];
    uint32_t literal = SEQ_SPARE0;
    QTestState *qts;
    uint32_t a, b;

    memcpy(insn, counter, sizeof(insn));
    insn[0] = thumb_ldr_pc(0, 0x100, 0x100 + 2 * G_N_ELEMENTS(insn));
    qts = qtest_initf("-machine efr32bg22 -accel tcg -icount shift=5,sleep=off "
                      "-S");
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_seq_enable_radio_bus_clocks(qts);
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);
    for (size_t i = 0; i < G_N_ELEMENTS(insn); i += 2) {
        qtest_writel(qts, SEQRAM_NS_BASE + 0x100 + 2 * i,
                     THUMB16_PAIR(insn[i], insn[i + 1]));
    }
    qtest_writel(qts, SEQRAM_NS_BASE + 0x100 + 2 * G_N_ELEMENTS(insn),
                 literal);
    qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0, 0);
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(20000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmpuint(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), !=, 0);

    qtest_writel(qts, CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_CLEAR_WINDOW,
                 CMU_RADIOCLKCTRL_EN);
    a = qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(20000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    b = qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0);
    g_assert_cmpuint(a, ==, b);

    qtest_writel(qts, CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
                 CMU_RADIOCLKCTRL_EN);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(20000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmpuint(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), >, b);
    qtest_quit(qts);
}

/*
 * A cleared state event releases the next RAC transition ~1.09 us later
 * (owo-open-fw docs/radio-tables.md SS7.72); these TCG tests run in real
 * time, so wait for the state rather than read it once.
 */
static void seq_wait_state(QTestState *qts, uint32_t expected)
{
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if ((qtest_readl(qts, RAC_BASE + RAC_STATUS) &
             RAC_STATUS_STATE_MASK) == expected) {
            break;
        }
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, expected);
}

/*
 * A released sequencer gates RAC on every state event it enables: on silicon
 * a TXEN issued in SHUTDOWN was held while SEQIF.STATESHUTDOWN stayed set
 * with SEQIEN enabling it, and clearing that bit alone released the
 * transmission (owo-open-fw docs/radio-tables.md SS7.69).
 */
static void test_seq_unacked_state_event_holds_txen(void)
{
    static const uint16_t idle[] = { THUMB16_BRANCH_SELF, THUMB16_BRANCH_SELF };
    QTestState *qts = seq_release_with_program(idle, 2, NULL, 0);

    qtest_writel(qts, RAC_BASE + RAC_SEQIEN + REGISTER_SET_WINDOW,
                 RAC_SEQIF_STATESHUTDOWN);
    qtest_writel(qts, RAC_BASE + RAC_CTRL + REGISTER_SET_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
    qtest_writel(qts, RAC_BASE + RAC_CTRL + REGISTER_CLEAR_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, RAC_STATUS_STATE_SHUTDOWN);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_SEQIF) &
                        RAC_SEQIF_STATESHUTDOWN, ==, RAC_SEQIF_STATESHUTDOWN);

    qtest_writel(qts, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, RAC_STATUS_STATE_SHUTDOWN);

    /* Clearing an event that is not pending changes nothing. */
    qtest_writel(qts, RAC_BASE + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATETX);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, RAC_STATUS_STATE_SHUTDOWN);

    qtest_writel(qts, RAC_BASE + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATESHUTDOWN);
    seq_wait_state(qts, RAC_STATUS_STATE_TXWARM);

    qtest_quit(qts);
}

/*
 * A PA ramp completes some time after RAC.PAENCTRL.PARAMP changes.  The TXPD
 * ramp-down drops PARAMP, then clears MODEM.SEQIF.TXRAMPDONE, then polls it
 * (owo-open-fw docs/radio-tables.md SS7.62); that clear must not discard the
 * ramp still in progress, or the poll never sees completion.
 */
static void test_pa_ramp_survives_clear_after_paramp(void)
{
    static const uint16_t idle[] = { THUMB16_BRANCH_SELF, THUMB16_BRANCH_SELF };
    QTestState *qts = seq_release_with_program(idle, 2, NULL, 0);

    qtest_writel(qts, MODEM_BASE + MODEM_SEQIF + REGISTER_CLEAR_WINDOW,
                 MODEM_IF_TXRAMPDONE);
    qtest_writel(qts, RAC_BASE + RAC_PAENCTRL + REGISTER_SET_WINDOW,
                 RAC_PAENCTRL_PARAMP);
    g_assert_cmphex(qtest_readl(qts, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, MODEM_IF_TXRAMPDONE);

    qtest_writel(qts, RAC_BASE + RAC_PAENCTRL + REGISTER_CLEAR_WINDOW,
                 RAC_PAENCTRL_PARAMP);
    qtest_writel(qts, MODEM_BASE + MODEM_SEQIF + REGISTER_CLEAR_WINDOW,
                 MODEM_IF_TXRAMPDONE);
    g_assert_cmphex(qtest_readl(qts, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, MODEM_IF_TXRAMPDONE);
    g_assert_cmphex(qtest_readl(qts, MODEM_BASE + MODEM_IF) &
                        MODEM_IF_TXRAMPDONE, ==, MODEM_IF_TXRAMPDONE);

    /* Without a PARAMP change, a clear stays cleared. */
    qtest_writel(qts, MODEM_BASE + MODEM_SEQIF + REGISTER_CLEAR_WINDOW,
                 MODEM_IF_TXRAMPDONE);
    g_assert_cmphex(qtest_readl(qts, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, 0);

    qtest_quit(qts);
}

/*
 * Once sequencer firmware is released, a TX frame only drains while the
 * synthesizer runs (owo-open-fw docs/radio-tables.md SS7.60/SS7.61): on
 * silicon, RAC sat in TX with FRC stalled for as long as RAC.SYEN was clear
 * and SYNTH.CMD.SYNTHSTART had not been issued, and writing SYEN then
 * SYNTHSTART on that stuck radio drained the frame immediately.  SYNTHSTART
 * alone did nothing.  The idle program stands in for the auxiliary core; the
 * qtest side performs its STATETXWARM/STATETX acknowledgements.
 */
static void test_seq_tx_waits_for_synthesizer(void)
{
    static const uint16_t idle[] = { THUMB16_BRANCH_SELF, THUMB16_BRANCH_SELF };
    /* Stock's own RAC.SYEN value while transmitting (SS7.60). */
    static const uint32_t syen_stock_tx = 0x00001fbf;
    QTestState *qts = seq_release_with_program(idle, 2, NULL, 0);

    /* A carrier the synthesizer reaches (SS7.85): 2402 MHz. */
    qtest_writel(qts, SYNTH_BASE + SYNTH_FREQ, SYNTH_FREQ_2402_MHZ);
    qtest_writel(qts, FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    qtest_writel(qts, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, RAC_STATUS_STATE_TXWARM);
    qtest_writel(qts, RAC_BASE + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATETXWARM);
    seq_wait_state(qts, RAC_STATUS_STATE_TX);
    qtest_writel(qts, RAC_BASE + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATETX);

    /* Far longer than the model's completion delay: still parked in TX. */
    g_usleep(20000);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, RAC_STATUS_STATE_TX);
    g_assert_cmphex(qtest_readl(qts, FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
    /* SS7.42/SS7.60: 0x4, SYNTHREADY without INLOCK, no SYNTH.CMD written */
    g_assert_cmphex(qtest_readl(qts, SYNTH_BASE + SYNTH_STATUS), ==,
                    SYNTH_STATUS_SYNTHREADY);

    /* SYNTHSTART with the VCO unpowered starts nothing. */
    qtest_writel(qts, SYNTH_BASE + SYNTH_CMD, SYNTH_CMD_SYNTHSTART);
    g_usleep(20000);
    g_assert_cmphex(qtest_readl(qts, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, 0);
    g_assert_cmphex(qtest_readl(qts, FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);

    /* Software cannot fake lock. */
    qtest_writel(qts, SYNTH_BASE + SYNTH_STATUS, SYNTH_STATUS_INLOCK);
    g_assert_cmphex(qtest_readl(qts, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, 0);

    qtest_writel(qts, RAC_BASE + RAC_SYEN, syen_stock_tx);
    qtest_writel(qts, SYNTH_BASE + SYNTH_CMD, SYNTH_CMD_SYNTHSTART);
    /* Lock follows SYNTHSTART by ~8.9 us (SS7.72). */
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SYNTH_BASE + SYNTH_STATUS) & SYNTH_STATUS_INLOCK) {
            break;
        }
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(qts, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, SYNTH_STATUS_INLOCK);
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, FRC_BASE + FRC_IF) & FRC_IF_TXDONE) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==,
                    FRC_IF_TXDONE);
    /* The frame ends in TXPD, not OFF, until STATETXPD is acknowledged. */
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK, ==, RAC_STATUS_STATE_TXPD);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_SEQIF) &
                        RAC_SEQIF_STATETXPD, ==, RAC_SEQIF_STATETXPD);
    qtest_writel(qts, RAC_BASE + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATETXPD);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    seq_wait_state(qts, 0);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");

    /* SYNTHSTOP drops lock again. */
    qtest_writel(qts, SYNTH_BASE + SYNTH_CMD, SYNTH_CMD_SYNTHSTOP);
    g_assert_cmphex(qtest_readl(qts, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, 0);
    qtest_quit(qts);
}

static void test_radio_sequencer_executes_open_code(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    /* Keep the image-less application M33 in a valid idle loop while TCG
     * executes the independently released auxiliary core. */
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_seq_enable_radio_bus_clocks(qts);

    /* Model the real stage-3 boundary: SEQRAM may retain an unrelated image
     * from an earlier boot.  The M33 asserts the hardware-proven base-register
     * SEQRESET request, overwrites that image, and only then replaces CTRL
     * with CPUWAITDIS.  The replacement is important for the explicit-reset
     * path; RAIL's SET-alias release is a warm path that did not set reset. */
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0xdeadbeef);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0xcafebabe);
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);

    /* Same independent four-instruction ARMv6-M proof used on silicon:
     * load the local SEQ.SPARE0 address/value, store it, then loop. */
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x100, 0x49024801);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x104, 0xe7fe6001);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x108, 0x00004fe0);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x10c, 0x12345678);
    qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0, 0);

    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + 0x000), ==,
                    0x00003e00);
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + 0x004), ==,
                    0x00000101);

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0) == 0x12345678) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), ==,
                    0x12345678);

    /* This probe never acknowledges the TXWARM state interrupt.  Merely
     * releasing a private core and issuing TXEN must therefore leave RAC at
     * the handoff boundary; the old eager model incorrectly published and
     * completed a frame from the staged FIFO here. */
    qtest_writel(qts, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_TXWARM);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(10000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_TXWARM);
    qtest_quit(qts);
}

static void test_radio_sequencer_release_is_idempotent(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_seq_enable_radio_bus_clocks(qts);

    /* Increment SPARE0 once at reset entry, then loop.  A second write of
     * CPUWAITDIS must not restart the already-released sequencer. */
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x100,
                 THUMB16_PAIR(THUMB16_LDR_R0_PC_8, THUMB16_LDR_R1_R0));
    qtest_writel(qts, SEQRAM_NS_BASE + 0x104,
                 THUMB16_PAIR(THUMB16_ADDS_R1_1, THUMB16_STR_R1_R0));
    qtest_writel(qts, SEQRAM_NS_BASE + 0x108, THUMB16_BRANCH_SELF);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x10c, 0x00004fe0);
    qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0, 0);

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0) == 1) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), ==, 1);

    qtest_writel(qts, RAC_BASE + RAC_CTRL,
                 RAC_CTRL_CPUWAITDIS | RAC_CTRL_BOARD_POLARITIES);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_CTRL), ==,
                    RAC_CTRL_CPUWAITDIS | RAC_CTRL_BOARD_POLARITIES);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(10000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), ==, 1);
    qtest_quit(qts);
}

/*
 * RADIOAES (RM 11.4; descriptor format of Silicon Labs'
 * sli_protocol_crypto_radioaes.c): one AES-ECB block through the
 * scatter-gather DMA.  Run time on silicon (owo-open-fw docs/crypto.md SS4,
 * HCLK 76.8 MHz): ECB 1 block 133 cycles = 80 + 55 per block, so the DMA
 * reports busy for 1042 + 716 ns.  The engine has 128- and 256-bit keys
 * only (BA411E_HW_CFG_1.g_Keysize = 5 read on silicon): a 192-bit operation
 * is reported, not computed.
 */
#define RADIOAES_BASE          0x44000000
#define RADIOAES_FETCHADDR     0x000
#define RADIOAES_PUSHADDR      0x010
#define RADIOAES_IF            0x028
#define RADIOAES_CTRL          0x034
#define RADIOAES_CMD           0x038
#define RADIOAES_STATUS        0x03c
#define RADIOAES_HW_CFG_1      0x404
#define RADIOAES_CTRL_SG       0x3        /* fetcher + pusher scatter-gather */
#define RADIOAES_CMD_START     0x3        /* start fetcher + pusher */
#define RADIOAES_STATUS_BUSY   0x3
#define RADIOAES_IF_STOPPED    0x12       /* FETCHERSTOPPED | PUSHERSTOPPED */
#define RADIOAES_DESC_STOP     0x1
#define RADIOAES_TAG_CONFIG    0x11       /* BA411E, configuration, offset 0 */
#define RADIOAES_TAG_LASTDATA  0x21       /* BA411E, last data */
#define RADIOAES_MODE_ECB      0x100
#define RADIOAES_MODE_AES192   0x8
#define RADIOAES_ECB_1BLOCK_NS (1042 + 716)
#define AES_SRAM               0x20001000
#define AES_CFG                (AES_SRAM + 0x000) /* mode, 0, key */
#define AES_IN                 (AES_SRAM + 0x040)
#define AES_OUT                (AES_SRAM + 0x080)
#define AES_FETCH              (AES_SRAM + 0x100) /* two descriptors */
#define AES_PUSH               (AES_SRAM + 0x120)

static void radioaes_ecb(QTestState *qts, uint32_t mode, const uint8_t *key,
                         size_t keylen, const uint8_t *in)
{
    qtest_writel(qts, AES_CFG, mode);
    qtest_writel(qts, AES_CFG + 4, 0);
    qtest_memwrite(qts, AES_CFG + 8, key, keylen);
    qtest_memwrite(qts, AES_IN, in, 16);
    qtest_memset(qts, AES_OUT, 0xa5, 16);
    /* Fetch: the configuration, then the block; push: the block. */
    qtest_writel(qts, AES_FETCH + 0x0, AES_CFG);
    qtest_writel(qts, AES_FETCH + 0x4, AES_FETCH + 0x10);
    qtest_writel(qts, AES_FETCH + 0x8, 8 + keylen);
    qtest_writel(qts, AES_FETCH + 0xc, RADIOAES_TAG_CONFIG);
    qtest_writel(qts, AES_FETCH + 0x10, AES_IN);
    qtest_writel(qts, AES_FETCH + 0x14, RADIOAES_DESC_STOP);
    qtest_writel(qts, AES_FETCH + 0x18, 16);
    qtest_writel(qts, AES_FETCH + 0x1c, RADIOAES_TAG_LASTDATA);
    qtest_writel(qts, AES_PUSH + 0x0, AES_OUT);
    qtest_writel(qts, AES_PUSH + 0x4, RADIOAES_DESC_STOP);
    qtest_writel(qts, AES_PUSH + 0x8, 16);
    qtest_writel(qts, AES_PUSH + 0xc, 0);
    qtest_writel(qts, RADIOAES_BASE + RADIOAES_FETCHADDR, AES_FETCH);
    qtest_writel(qts, RADIOAES_BASE + RADIOAES_PUSHADDR, AES_PUSH);
    qtest_writel(qts, RADIOAES_BASE + RADIOAES_CTRL, RADIOAES_CTRL_SG);
    qtest_writel(qts, RADIOAES_BASE + RADIOAES_CMD, RADIOAES_CMD_START);
}

/*
 * RADIOAES needs CMU_CLKEN0.RADIOAES and CMU_RADIOCLKCTRL.EN (RM 8.5,
 * 8.5.25; what sli_radioaes_acquire and the open firmware set): a gated
 * access faults.
 */
#define CMU_CLKEN0_RADIOAES    (1u << 2)

static void setup_radioaes_clocks(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_RADIOAES);
    qtest_writel(qts, CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
                 CMU_RADIOCLKCTRL_EN);
}

static void setup_radioaes_bus_clock_only(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_RADIOAES);
}

static void test_radioaes_clock_gate(void)
{
    g_assert_true(m33_load_faults(RADIOAES_BASE + RADIOAES_STATUS,
                                  setup_no_clocks));
    g_assert_true(m33_load_faults(RADIOAES_BASE + RADIOAES_STATUS,
                                  setup_radioaes_bus_clock_only));
    g_assert_false(m33_load_faults(RADIOAES_BASE + RADIOAES_STATUS,
                                   setup_radioaes_clocks));
}

static void test_radioaes_ecb_and_key_sizes(void)
{
    /* FIPS-197 Appendix C.1 (AES-128) and C.2 (AES-192). */
    static const uint8_t key[24] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };
    static const uint8_t plain[16] = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    };
    static const uint8_t cipher128[16] = {
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
        0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
    };
    static const uint8_t cipher192[16] = {
        0xdd, 0xa9, 0x7c, 0xa4, 0x86, 0x4c, 0xdf, 0xe0,
        0x6e, 0xaf, 0x70, 0xa0, 0xec, 0x0d, 0x71, 0x91,
    };
    static const uint8_t zero[16];
    QTestState *qts = qtest_init("-machine efr32bg22 -accel qtest");
    uint8_t out[16];

    setup_radioaes_clocks(qts);
    g_assert_cmphex(qtest_readl(qts, RADIOAES_BASE + RADIOAES_HW_CFG_1), ==,
                    0x05010127);
    radioaes_ecb(qts, RADIOAES_MODE_ECB, key, 16, plain);
    g_assert_cmphex(qtest_readl(qts, RADIOAES_BASE + RADIOAES_STATUS), ==,
                    RADIOAES_STATUS_BUSY);
    qtest_clock_step(qts, RADIOAES_ECB_1BLOCK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, RADIOAES_BASE + RADIOAES_STATUS), ==,
                    RADIOAES_STATUS_BUSY);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, RADIOAES_BASE + RADIOAES_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RADIOAES_BASE + RADIOAES_IF) &
                    RADIOAES_IF_STOPPED, ==, RADIOAES_IF_STOPPED);
    qtest_memread(qts, AES_OUT, out, sizeof(out));
    g_assert_cmpmem(out, sizeof(out), cipher128, sizeof(cipher128));

    /* No 192-bit keys: nothing computed (the push delivers zeros). */
    radioaes_ecb(qts, RADIOAES_MODE_ECB | RADIOAES_MODE_AES192, key, 24,
                 plain);
    qtest_clock_step(qts, RADIOAES_ECB_1BLOCK_NS);
    qtest_memread(qts, AES_OUT, out, sizeof(out));
    g_assert(memcmp(out, cipher192, sizeof(out)) != 0);
    g_assert_cmpmem(out, sizeof(out), zero, sizeof(zero));
    qtest_quit(qts);
}

/*
 * CRYPTOACC TRNG (RM SS11.5): RNGCTRL.ENABLE starts the noise source; after
 * the start-up (INITWAITVAL HCLK cycles, the 512-sample NIST start-up test
 * and a 512-sample first block, measured on silicon: docs/radio-tables.md
 * SS7.89) 128-bit blocks fill the 64-word FIFO;
 * a full FIFO stops the rings (STATE FIFOFULLOFF, FULLIF); each read of the
 * FIFO window pops a word and the rings restart.  Bus clock CLKEN1 bit 13.
 */
#define TRNG_CTRL              0x4c021000
#define TRNG_FIFOLEVEL         0x004
#define TRNG_FIFODEPTH         0x00c
#define TRNG_RNGSTATUS         0x030
#define TRNG_INITWAITVAL       0x034
#define TRNG_FIFO              0x4c024000
#define TRNG_ENABLE            0x00040001 /* NB128BITBLOCKS 4 | ENABLE */
#define TRNG_STATUS_FULLIF     (1u << 7)
#define TRNG_STATUS_STATE(v)   (((v) >> 1) & 7u)
#define TRNG_STATE_STARTUP     1
#define TRNG_STATE_FIFOFULLOFF 3
#define TRNG_STATE_RUNNING     4
#define CMU_CLKEN1_CRYPTOACC   (1u << 13)

static void test_trng_fifo(void)
{
    QTestState *qts = qtest_init("-machine efr32bg22 -accel qtest");
    uint32_t words[8];
    bool differ = false;

    qtest_writel(qts, CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
                 CMU_CLKEN1_CRYPTOACC);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_FIFODEPTH), ==, 64);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_INITWAITVAL), ==,
                    0xffff);
    qtest_writel(qts, TRNG_CTRL, TRNG_ENABLE);
    g_assert_cmphex(TRNG_STATUS_STATE(qtest_readl(qts, TRNG_CTRL +
                                                  TRNG_RNGSTATUS)), ==,
                    TRNG_STATE_STARTUP);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_FIFOLEVEL), ==, 0);
    qtest_clock_step(qts, 20 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_FIFOLEVEL), ==, 64);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_RNGSTATUS) &
                    TRNG_STATUS_FULLIF, ==, TRNG_STATUS_FULLIF);
    g_assert_cmphex(TRNG_STATUS_STATE(qtest_readl(qts, TRNG_CTRL +
                                                  TRNG_RNGSTATUS)), ==,
                    TRNG_STATE_FIFOFULLOFF);
    for (size_t i = 0; i < G_N_ELEMENTS(words); i++) {
        words[i] = qtest_readl(qts, TRNG_FIFO + 4 * i);
        differ |= i && words[i] != words[0];
    }
    g_assert_true(differ);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_FIFOLEVEL), ==, 56);
    g_assert_cmphex(TRNG_STATUS_STATE(qtest_readl(qts, TRNG_CTRL +
                                                  TRNG_RNGSTATUS)), ==,
                    TRNG_STATE_RUNNING);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_FIFOLEVEL), ==, 64);
    qtest_quit(qts);
}

/*
 * TRNG test mode: TESTDATA through the conditioning function, AES-128
 * CBC-MAC under KEY0-3. Key and blocks of NIST SP 800-38A F.2.1 (the first
 * block XORed with that example's IV), so the MAC is its last ciphertext
 * block 3ff1caa1 681fac09 120eca30 7586e1a7; words little-endian.
 */
#define TRNG_KEY0              0x010
#define TRNG_TESTDATA          0x020
#define TRNG_TESTEN_BYPASSES   0x00003004 /* TESTEN | BYPNIST | BYPAIS31 */

static void test_trng_conditioning_kat(void)
{
    static const uint32_t key[4] = {
        0x16157e2b, 0xa6d2ae28, 0x8815f7ab, 0x3c4fcf09,
    };
    static const uint32_t data[16] = {
        0xe1bcc06b, 0x9199452a, 0x1a7434e1, 0x25199e7f,
        0x578a2dae, 0x9cac031e, 0xac6fb79e, 0x518eaf45,
        0x461cc830, 0x11e45ca3, 0x19c1fbe5, 0xef520a1a,
        0x45249ff6, 0x179b4fdf, 0x7b412bad, 0x10376ce6,
    };
    static const uint32_t mac[4] = {
        0xa1caf13f, 0x09ac1f68, 0x30ca0e12, 0xa7e18675,
    };
    QTestState *qts = qtest_init("-machine efr32bg22 -accel qtest");

    qtest_writel(qts, CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
                 CMU_CLKEN1_CRYPTOACC);
    qtest_writel(qts, TRNG_CTRL, TRNG_TESTEN_BYPASSES);
    for (size_t i = 0; i < G_N_ELEMENTS(key); i++) {
        qtest_writel(qts, TRNG_CTRL + TRNG_KEY0 + 4 * i, key[i]);
    }
    for (size_t i = 0; i < G_N_ELEMENTS(data); i++) {
        g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_RNGSTATUS) & 1, ==,
                        0);
        qtest_writel(qts, TRNG_CTRL + TRNG_TESTDATA, data[i]);
        g_assert_cmphex(qtest_readl(qts, TRNG_CTRL + TRNG_FIFOLEVEL), ==,
                        i == G_N_ELEMENTS(data) - 1 ? 4 : 0);
    }
    qtest_writel(qts, TRNG_CTRL, 0);
    for (size_t i = 0; i < G_N_ELEMENTS(mac); i++) {
        g_assert_cmphex(qtest_readl(qts, TRNG_FIFO), ==, mac[i]);
    }
    qtest_quit(qts);
}

/*
 * GPCRC (RM chapter 14) through the register sequence of sl_hal_gpcrc.c and
 * Zephyr's crc_silabs_gpcrc.c, on the check string "123456789":
 * CRC-32/IEEE 0xCBF43926 (BITREVERSE 0, DATA, xorout ~0, written as words
 * then a byte), CRC-16/CCITT-FALSE 0x29B1 (POLY 0x8408 reversed, BITREVERSE
 * 1, DATAREV, half-words then a byte), CRC-16/KERMIT 0x2189 (BITREVERSE 0,
 * DATA); BYTEREVERSE of a big-endian word gives the same CRC-32.
 */
#define GPCRC_BASE             0x40088000
#define GPCRC_EN               0x004
#define GPCRC_CTRL             0x008
#define GPCRC_CMD              0x00c
#define GPCRC_INIT             0x010
#define GPCRC_POLY             0x014
#define GPCRC_INPUTDATA        0x018
#define GPCRC_INPUTDATAHWORD   0x01c
#define GPCRC_INPUTDATABYTE    0x020
#define GPCRC_DATA             0x024
#define GPCRC_DATAREV          0x028
#define GPCRC_CTRL_CRC16       (1u << 4)
#define GPCRC_CTRL_BITREVERSE  (1u << 9)
#define GPCRC_CTRL_BYTEREVERSE (1u << 10)
#define CMU_CLKEN0_GPCRC       (1u << 3)

static void gpcrc_start(QTestState *qts, uint32_t ctrl, uint32_t poly,
                        uint32_t init)
{
    qtest_writel(qts, GPCRC_BASE + GPCRC_EN, 0);
    qtest_writel(qts, GPCRC_BASE + GPCRC_CTRL, ctrl);
    qtest_writel(qts, GPCRC_BASE + GPCRC_POLY, poly);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INIT, init);
    qtest_writel(qts, GPCRC_BASE + GPCRC_EN, 1);
    qtest_writel(qts, GPCRC_BASE + GPCRC_CMD, 1);
}

static void test_gpcrc_crc_vectors(void)
{
    QTestState *qts = qtest_init("-machine efr32bg22 -accel qtest");

    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_GPCRC);
    /* "1234" "5678" little-endian words, then '9'. */
    gpcrc_start(qts, 0, 0, 0xffffffff);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATA, 0x34333231);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATA, 0x38373635);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATABYTE, '9');
    g_assert_cmphex(qtest_readl(qts, GPCRC_BASE + GPCRC_DATA) ^ 0xffffffff,
                    ==, 0xcbf43926);
    /* The same words big-endian, byte-reversed by the GPCRC. */
    gpcrc_start(qts, GPCRC_CTRL_BYTEREVERSE, 0, 0xffffffff);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATA, 0x31323334);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATA, 0x35363738);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATABYTE, '9');
    g_assert_cmphex(qtest_readl(qts, GPCRC_BASE + GPCRC_DATA) ^ 0xffffffff,
                    ==, 0xcbf43926);
    /* CRC-16/CCITT-FALSE: MSB-first, so BITREVERSE and DATAREV. */
    gpcrc_start(qts, GPCRC_CTRL_CRC16 | GPCRC_CTRL_BITREVERSE, 0x8408,
                0xffff);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATAHWORD, 0x3231);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATAHWORD, 0x3433);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATAHWORD, 0x3635);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATAHWORD, 0x3837);
    qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATABYTE, '9');
    g_assert_cmphex(qtest_readl(qts, GPCRC_BASE + GPCRC_DATAREV), ==, 0x29b1);
    /* CRC-16/KERMIT: reflected, DATA. */
    gpcrc_start(qts, GPCRC_CTRL_CRC16, 0x8408, 0);
    for (const char *c = "123456789"; *c; c++) {
        qtest_writel(qts, GPCRC_BASE + GPCRC_INPUTDATABYTE, *c);
    }
    g_assert_cmphex(qtest_readl(qts, GPCRC_BASE + GPCRC_DATA), ==, 0x2189);
    qtest_quit(qts);
}

/*
 * PRS (RM ch. 13) and the TIMER input logic it feeds (RM SS19.3.3).
 * TIMER1 runs from EM01GRPACLK, set to the HFXO's 38.4 MHz here.
 */
#define PRS_BASE                0x40038000
#define PRS_IPVERSION           0x000
#define PRS_ASYNC_SWPULSE       0x008
#define PRS_ASYNC_SWLEVEL       0x00c
#define PRS_ASYNC_PEEK          0x010
#define PRS_SYNC_PEEK           0x014
#define PRS_ASYNC_CH_CTRL(n)    (0x018 + 4 * (n))
#define PRS_SYNC_CH_CTRL(n)     (0x048 + 4 * (n))
#define PRS_CH_SIGSEL(n)        (n)
#define PRS_CH_SOURCESEL(n)     ((n) << 8)
#define PRS_CH_FNSEL(n)         ((n) << 16)
#define PRS_CH_AUXSEL(n)        ((n) << 24)
#define PRS_FNSEL_A             0xc
#define PRS_SRC_PRSL            0x0a
#define PRS_SRC_TIMER0          0x22
#define PRS_SYNC_SRC_TIMER0     0x01
#define PRS_SIG_TIMER_OF        1
#define PRS_CONSUMER_TIMER1_CC(n) (0x124 + 4 * (n))
#define PRS_CONSUMER_RTCC_CC(n) (0x0e8 + 4 * (n))
#define PRS_CONSUMER_IADC0_SINGLETRIGGER 0x068
#define PRS_CONSUMER_LETIMER0_START 0x078
#define PRS_CONSUMER_SPRSSEL(n) ((n) << 8)
#define CMU_CLKEN0_TIMER0       (1u << 4)
#define CMU_CLKEN0_TIMER1       (1u << 5)
#define CMU_CLKEN0_IADC0        (1u << 10)
#define CMU_CLKEN0_PRS          (1u << 27)
#define TIMER_CTRL              0x008
#define TIMER_CTRL_RISEA_RELOADSTART 3
#define TIMER_CFG_OSMEN         (1u << 4)
#define TIMER_CC_CTRL(n)        (0x064 + 0x20 * (n))
#define TIMER_CC_CTRL_ICEDGE_BOTH (2u << 24)
#define TIMER_CC_ICF(n)         (0x074 + 0x20 * (n))
#define TIMER_CC_ICOF(n)        (0x078 + 0x20 * (n))
#define TIMER_CC_CFG_INPUTCAPTURE 0x1
#define TIMER_CC_CFG_PRSSYNC    (1u << 17)
#define TIMER_CC_CFG_PRSASYNCLEVEL (2u << 17)
#define TIMER_CC_CFG_PRSASYNCPULSE (3u << 17)
#define TIMER_CC_CFG_FILT       (1u << 20)
#define TIMER_IF_ICFOF(n)       (1u << (20 + (n)))
#define TIMER_IF_ICFUF(n)       (1u << (24 + (n)))
#define TIMER_STATUS_ICFEMPTY(n) (1u << (16 + (n)))
#define TIMER_STATUS_CCPOL(n)   (1u << (24 + (n)))

static QTestState *prs_start(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel qtest");

    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_PRS | CMU_CLKEN0_TIMER0 | CMU_CLKEN0_TIMER1 |
                     CMU_CLKEN0_RTCC | CMU_CLKEN0_IADC0);
    qtest_writel(qts, CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL,
                 CMU_EM01GRPACLK_HFXO);
    return qts;
}

/* TIMER1 CCn in input capture from PRS channel @ch, input @insel. */
static void prs_timer1_capture(QTestState *qts, unsigned n, unsigned ch,
                               uint32_t insel)
{
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_TIMER1_CC(n), ch);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, 0);
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CFG(n),
                 TIMER_CC_CFG_INPUTCAPTURE | insel);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
}

/*
 * Every FNSEL against every A/B pair (RM Table 13.1: output FNSEL[2A + B]),
 * with A and B from other channels' SWLEVEL through the PRSL producer and
 * AUXSEL; then SWLEVEL inverting a channel's own output (SS13.3.1).
 */
static void test_prs_logic_table(void)
{
    QTestState *qts = prs_start();

    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_IPVERSION), ==, 1);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_CH_CTRL(0)), ==,
                    PRS_CH_FNSEL(PRS_FNSEL_A));
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK), ==, 0);
    for (unsigned fnsel = 0; fnsel < 16; fnsel++) {
        qtest_writel(qts, PRS_BASE + PRS_ASYNC_CH_CTRL(0),
                     PRS_CH_SOURCESEL(PRS_SRC_PRSL) | PRS_CH_SIGSEL(1) |
                         PRS_CH_FNSEL(fnsel) | PRS_CH_AUXSEL(2));
        for (unsigned a = 0; a < 2; a++) {
            for (unsigned b = 0; b < 2; b++) {
                qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWLEVEL,
                             (a << 1) | (b << 2));
                g_assert_cmpuint(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK) &
                                 1, ==, (fnsel >> (2 * a + b)) & 1);
            }
        }
    }
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_CH_CTRL(0),
                 PRS_CH_FNSEL(PRS_FNSEL_A));
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWLEVEL, 0x801);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK), ==, 0x801);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_SWPULSE), ==, 0);
    qtest_quit(qts);
}

/* Reads and writes fault while CMU_CLKEN0.PRS is clear (SS7.71, SS7.73). */
static void setup_prs_clock(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_PRS);
}

static void test_prs_bus_gate(void)
{
    g_assert_true(m33_load_faults(PRS_BASE + PRS_ASYNC_PEEK,
                                  setup_no_clocks));
    g_assert_true(m33_store_faults(PRS_BASE + PRS_ASYNC_SWLEVEL,
                                   setup_no_clocks));
    g_assert_false(m33_load_faults(PRS_BASE + PRS_ASYNC_PEEK,
                                   setup_prs_clock));
    g_assert_false(m33_store_faults(PRS_BASE + PRS_ASYNC_SWLEVEL,
                                    setup_prs_clock));
}

/*
 * SWPULSE into TIMER1 input capture, INSEL = PRSASYNCPULSE: each pulse
 * captures CNT; one store pulsing two channels is captured on the same
 * tick by both (silicon, SS7.73); the 2-entry FIFO, ICOF and the overflow
 * and underflow flags (RM SS19.3.3.3).
 */
static void test_prs_swpulse_timer_capture(void)
{
    QTestState *qts = prs_start();
    uint32_t first, second;

    prs_timer1_capture(qts, 0, 9, TIMER_CC_CFG_PRSASYNCPULSE);
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_TIMER1_CC(1), 10);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, 0);
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CFG(1),
                 TIMER_CC_CFG_INPUTCAPTURE | TIMER_CC_CFG_PRSASYNCPULSE);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_STATUS) &
                    (TIMER_STATUS_ICFEMPTY(0) | TIMER_STATUS_ICFEMPTY(1)), ==,
                    TIMER_STATUS_ICFEMPTY(0) | TIMER_STATUS_ICFEMPTY(1));
    qtest_writel(qts, TIMER1_BASE + TIMER_CMD, TIMER_CMD_START);
    qtest_clock_step(qts, 10000);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, (1u << 9) | (1u << 10));
    qtest_clock_step(qts, 10000);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 9);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_STATUS) &
                    TIMER_STATUS_ICFEMPTY(0), ==, 0);
    first = qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0));
    g_assert_cmpuint(qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(1)), ==,
                     first);
    g_assert_cmpuint(abs((int)first - 384), <=, 1);
    second = qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0));
    g_assert_cmpuint(abs((int)(second - first) - 384), <=, 1);
    g_assert_cmpuint(qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICOF(0)), ==,
                     second);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_IF) &
                    (TIMER_IF_CC(0) | TIMER_IF_CC(1)), ==,
                    TIMER_IF_CC(0) | TIMER_IF_CC(1));
    /* Empty: an underflow; three more pulses: an overflow into ICOF. */
    qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0));
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_IF) &
                    TIMER_IF_ICFUF(0), ==, TIMER_IF_ICFUF(0));
    for (unsigned i = 0; i < 3; i++) {
        qtest_clock_step(qts, 1000);
        qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 9);
    }
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_IF) &
                    TIMER_IF_ICFOF(0), ==, TIMER_IF_ICFOF(0));
    first = qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0));
    second = qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0));
    g_assert_cmpuint(abs((int)(qtest_readl(qts, TIMER1_BASE +
                                                TIMER_CC_ICOF(0)) -
                               second) - 38), <=, 1);
    g_assert_cmpuint(abs((int)(second - first) - 38), <=, 1);
    qtest_quit(qts);
}

/*
 * INSEL = PRSASYNCLEVEL qualifies the level for 2 HFPERCLK cycles (7 with
 * FILT): a pulse is never captured -- silicon's "PRSASYNCLEVEL captured
 * nothing" for the MODEM events (SS7.71) -- while a SWLEVEL step is, 2
 * cycles late.  INSEL = PRSSYNC follows the synchronous channel.
 */
static void test_prs_timer_input_qualification(void)
{
    QTestState *qts = prs_start();
    uint32_t value;

    prs_timer1_capture(qts, 0, 3, TIMER_CC_CFG_PRSASYNCLEVEL);
    qtest_writel(qts, TIMER1_BASE + TIMER_CMD, TIMER_CMD_START);
    qtest_clock_step(qts, 10000);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 3);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_IF) &
                    TIMER_IF_CC(0), ==, 0);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWLEVEL, 1u << 3);
    value = qtest_readl(qts, TIMER1_BASE + TIMER_CNT);
    qtest_clock_step(qts, 2 * 27);
    value = qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0)) - value;
    g_assert_cmpuint(value, >=, 2);
    g_assert_cmpuint(value, <=, 3);
    /* FILT: 2 + 5 cycles, and both edges with ICEDGE = BOTH. */
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CTRL(0),
                 TIMER_CC_CTRL_ICEDGE_BOTH);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, 0);
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CFG(0),
                 TIMER_CC_CFG_INPUTCAPTURE | TIMER_CC_CFG_PRSASYNCLEVEL |
                     TIMER_CC_CFG_FILT);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
    qtest_writel(qts, TIMER1_BASE + TIMER_CMD, TIMER_CMD_START);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWLEVEL, 0);
    value = qtest_readl(qts, TIMER1_BASE + TIMER_CNT);
    qtest_clock_step(qts, 7 * 27);
    value = qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0)) - value;
    g_assert_cmpuint(value, >=, 7);
    g_assert_cmpuint(value, <=, 8);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_STATUS) &
                    TIMER_STATUS_ICFEMPTY(0), ==, TIMER_STATUS_ICFEMPTY(0));

    /* PRSSYNC: TIMER0 OF through synchronous channel 2, TOP = 99. */
    qtest_writel(qts, PRS_BASE + PRS_SYNC_CH_CTRL(2),
                 PRS_CH_SOURCESEL(PRS_SYNC_SRC_TIMER0) |
                     PRS_CH_SIGSEL(PRS_SIG_TIMER_OF));
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_TIMER1_CC(2),
                 PRS_CONSUMER_SPRSSEL(2));
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, 0);
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CFG(2),
                 TIMER_CC_CFG_INPUTCAPTURE | TIMER_CC_CFG_PRSSYNC);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
    qtest_writel(qts, TIMER1_BASE + TIMER_CMD, TIMER_CMD_START);
    qtest_writel(qts, TIMER0_BASE + TIMER_EN, TIMER_EN_EN);
    qtest_writel(qts, TIMER0_BASE + TIMER_TOP, 99);
    qtest_writel(qts, TIMER0_BASE + TIMER_CMD, TIMER_CMD_START);
    value = qtest_readl(qts, TIMER1_BASE + TIMER_CNT);
    qtest_clock_step(qts, 2 * 100 * 27);
    g_assert_cmpuint(abs((int)(qtest_readl(qts, TIMER1_BASE +
                                                TIMER_CC_ICF(2)) - value) -
                         100), <=, 1);
    g_assert_cmpuint(abs((int)(qtest_readl(qts, TIMER1_BASE +
                                                TIMER_CC_ICF(2)) - value) -
                         200), <=, 1);
    qtest_quit(qts);
}

/*
 * The probe route of scripts/bg22-tifs-probe.tcl (SS7.71): CC0's PRS edge
 * reload-starts a one-shot TIMER1 (CTRL.RISEA = RELOADSTART, CFG.OSMEN)
 * that stops at TOP; captures after that read TOP.
 */
static void test_prs_timer_reloadstart_oneshot(void)
{
    QTestState *qts = prs_start();

    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_TIMER1_CC(0), 9);
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_TIMER1_CC(1), 10);
    qtest_writel(qts, TIMER1_BASE + TIMER_CFG, TIMER_CFG_OSMEN);
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CFG(0),
                 TIMER_CC_CFG_INPUTCAPTURE | TIMER_CC_CFG_PRSASYNCPULSE);
    qtest_writel(qts, TIMER1_BASE + TIMER_CC_CFG(1),
                 TIMER_CC_CFG_INPUTCAPTURE | TIMER_CC_CFG_PRSASYNCPULSE);
    qtest_writel(qts, TIMER1_BASE + TIMER_EN, TIMER_EN_EN);
    qtest_writel(qts, TIMER1_BASE + TIMER_TOP, 999);
    qtest_writel(qts, TIMER1_BASE + TIMER_CTRL, TIMER_CTRL_RISEA_RELOADSTART);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_STATUS) &
                    TIMER_STATUS_RUNNING, ==, 0);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 9);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_STATUS) &
                    TIMER_STATUS_RUNNING, ==, TIMER_STATUS_RUNNING);
    qtest_clock_step(qts, 5000);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 10);
    g_assert_cmpuint(abs((int)qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(1)) -
                         192), <=, 1);
    qtest_clock_step(qts, 50000);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_STATUS) &
                    TIMER_STATUS_RUNNING, ==, 0);
    g_assert_cmpuint(qtest_readl(qts, TIMER1_BASE + TIMER_CNT), ==, 999);
    g_assert_cmphex(qtest_readl(qts, TIMER1_BASE + TIMER_IF) & TIMER_IF_OF,
                    ==, TIMER_IF_OF);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 10);
    g_assert_cmpuint(qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(1)), ==, 999);
    /* The CC0 capture of the start edge read 0 counts. */
    g_assert_cmpuint(qtest_readl(qts, TIMER1_BASE + TIMER_CC_ICF(0)), ==, 0);
    qtest_quit(qts);
}

/*
 * RTCC input capture from the PRS (RM SS15.3.2): an edge on the selected
 * channel captures CNT into CCn_ICVALUE and sets IF.CCn.
 */
#define RTCC_CC_CTRL(n)         (0x030 + 0xc * (n))
#define RTCC_CC_ICVALUE(n)      (0x038 + 0xc * (n))
#define RTCC_CC_INPUTCAPTURE    0x1
#define RTCC_IF_CC(n)           (1u << (4 + 2 * (n)))

static void test_prs_rtcc_capture(void)
{
    QTestState *qts = prs_start();
    uint32_t cnt;

    qtest_writel(qts, RTCC_BASE + RTCC_EN, 1);
    qtest_writel(qts, RTCC_BASE + RTCC_CC_CTRL(2), RTCC_CC_INPUTCAPTURE);
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_RTCC_CC(2), 5);
    qtest_writel(qts, RTCC_BASE + RTCC_CMD, RTCC_CMD_START);
    qtest_clock_step(qts, 100 * LF_PERIOD_NS);
    cnt = qtest_readl(qts, RTCC_BASE + RTCC_CNT);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 5);
    g_assert_cmpuint(qtest_readl(qts, RTCC_BASE + RTCC_CC_ICVALUE(2)), ==,
                     cnt);
    g_assert_cmphex(qtest_readl(qts, RTCC_BASE + RTCC_IF) & RTCC_IF_CC(2), ==,
                    RTCC_IF_CC(2));
    qtest_quit(qts);
}

/*
 * IADC triggers (RM SS24.3.3.3): SINGLESTART only enables the queue for a
 * PRSPOS trigger, which converts after the 2 CLK_SRC_ADC synchronisation
 * cycles; the local timer triggers every IADC_TIMER CLK_SRC_ADC cycles.
 * CLK_SRC_ADC is EM01GRPACLK, HFRCODPLL's 19 MHz at reset.
 */
#define IADC_TIMER            0x010
#define IADC_TRIGGER          0x02c
#define IADC_TRIGGER_SINGLE_TIMER  (1u << 8)
#define IADC_TRIGGER_SINGLE_PRSPOS (3u << 8)
#define IADC_CMD_SINGLESTART  (1u << 0)
#define IADC_CMD_SINGLESTOP   (1u << 1)
#define IADC_CMD_TIMEREN      (1u << 16)
#define IADC_STATUS_SINGLEQEN (1u << 0)
#define IADC_STATUS_TIMERACTIVE (1u << 16)
#define IADC_SINGLEFIFOSTAT   0x078
#define IADC_PRS_SYNC_NS      (2 * 1000000000ull / 19000000 + 1)
#define IADC_TIMER_PERIOD_NS  100000ull

static void test_iadc_triggers(void)
{
    QTestState *qts = prs_start();

    qtest_writel(qts, CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, 1);
    qtest_writel(qts, IADC_BASE + IADC_CTRL,
                 IADC_CTRL_TIMEBASE_1US | IADC_CTRL_KEEPWARM);
    qtest_writel(qts, IADC_BASE + IADC_SCHED0, 1);
    qtest_writel(qts, IADC_BASE + IADC_CFG0, IADC_CFG0_VDDX_GAIN1);
    qtest_writel(qts, IADC_BASE + IADC_SINGLE, IADC_SINGLE_PD02_GND);
    qtest_writel(qts, IADC_BASE + IADC_TRIGGER, IADC_TRIGGER_SINGLE_PRSPOS);
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_IADC0_SINGLETRIGGER, 4);
    qtest_writel(qts, IADC_BASE + IADC_CMD, IADC_CMD_SINGLESTART);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    (IADC_STATUS_SINGLEQEN | IADC_STATUS_CONVERTING), ==,
                    IADC_STATUS_SINGLEQEN);
    qtest_clock_step(qts, 100000);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFOSTAT), ==, 0);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 4);
    qtest_clock_step(qts, IADC_PRS_SYNC_NS + IADC_WARMUP_NS + IADC_CONV_NS -
                              1);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFOSTAT), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFOSTAT), ==, 1);
    qtest_writel(qts, IADC_BASE + IADC_CMD, IADC_CMD_SINGLESTOP);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 4);
    qtest_clock_step(qts, 100000);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFOSTAT), ==, 1);

    /* Local timer, 1900 cycles = 100 us; warm already (KEEPWARM). */
    qtest_writel(qts, IADC_BASE + IADC_TRIGGER, IADC_TRIGGER_SINGLE_TIMER);
    qtest_writel(qts, IADC_BASE + IADC_TIMER, 1900);
    qtest_writel(qts, IADC_BASE + IADC_CMD,
                 IADC_CMD_SINGLESTART | IADC_CMD_TIMEREN);
    g_assert_cmphex(qtest_readl(qts, IADC_BASE + IADC_STATUS) &
                    IADC_STATUS_TIMERACTIVE, ==, IADC_STATUS_TIMERACTIVE);
    qtest_clock_step(qts, IADC_TIMER_PERIOD_NS + IADC_CONV_NS);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFOSTAT), ==, 2);
    qtest_clock_step(qts, IADC_TIMER_PERIOD_NS);
    g_assert_cmpuint(qtest_readl(qts, IADC_BASE + IADC_SINGLEFIFOSTAT), ==, 3);
    qtest_quit(qts);
}

/*
 * BURAM (RM ch. 17): 32 retention words with SET/CLR/TGL aliases, byte
 * lanes, kept through resets other than power-on; gated by CLKEN0.BURAM.
 */
#define BURAM_BASE              0x40080000
#define BURAM_NS_BASE           0x50080000
#define CMU_CLKEN0_BURAM        (1u << 28)

static void setup_buram_clock(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_BURAM);
}

static void test_buram_retention(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel qtest -S");

    setup_buram_clock(qts);
    for (unsigned n = 0; n < 32; n++) {
        g_assert_cmphex(qtest_readl(qts, BURAM_BASE + 4 * n), ==, 0);
        qtest_writel(qts, BURAM_BASE + 4 * n, 0x01010101u * n);
    }
    qtest_writeb(qts, BURAM_NS_BASE + 4 * 3 + 2, 0xa5);
    g_assert_cmphex(qtest_readl(qts, BURAM_BASE + 4 * 3), ==, 0x03a50303);
    qtest_writel(qts, BURAM_BASE + 4 * 5 + REGISTER_SET_WINDOW, 0xf0000000);
    qtest_writel(qts, BURAM_BASE + 4 * 5 + REGISTER_CLEAR_WINDOW, 0x5);
    g_assert_cmphex(qtest_readl(qts, BURAM_BASE + 4 * 5), ==, 0xf5050500);
    g_assert_cmphex(qtest_readw(qts, BURAM_BASE + 4 * 31 + 2), ==, 0x1f1f);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_qmp_eventwait(qts, "RESET");
    setup_buram_clock(qts);
    g_assert_cmphex(qtest_readl(qts, BURAM_BASE + 4 * 3), ==, 0x03a50303);
    g_assert_cmphex(qtest_readl(qts, BURAM_BASE + 4 * 31), ==, 0x1f1f1f1f);
    qtest_quit(qts);
    g_assert_true(m33_load_faults(BURAM_BASE, setup_no_clocks));
    g_assert_false(m33_load_faults(BURAM_BASE, setup_buram_clock));
}

/*
 * LETIMER0 (RM ch. 18) at EM23GRPACLK = LFRCO, 32.768 kHz: LFSYNC writes
 * land at the third LF edge (SYNCBUSY meanwhile); the start edge loads
 * TOP; UF when the counter reaches 0, the toggle output with it (seen
 * through PRS producer LETIMER0 CH0); one-shot stops after REP0
 * underflows; PRS START.
 */
#define LETIMER0_BASE           0x4a000000
#define LT_EN                   0x004
#define LT_CTRL                 0x008
#define LT_CMD                  0x00c
#define LT_STATUS               0x010
#define LT_CNT                  0x018
#define LT_COMP0                0x01c
#define LT_TOP                  0x024
#define LT_REP0                 0x02c
#define LT_IF                   0x034
#define LT_SYNCBUSY             0x040
#define LT_PRSMODE              0x050
#define LT_CTRL_ONESHOT         0x1
#define LT_CTRL_UFOA0_TOGGLE    (1u << 2)
#define LT_CTRL_UFOA0_PWM       (3u << 2)
#define LT_CTRL_CNTTOPEN        (1u << 9)
#define LT_CMD_START            (1u << 0)
#define LT_IF_COMP0             (1u << 0)
#define LT_IF_UF                (1u << 2)
#define LT_IF_REP0              (1u << 3)
#define LT_SYNCBUSY_TOP         (1u << 2)
#define LT_SYNCBUSY_START       (1u << 5)
#define LT_PRSMODE_START_RISING (1u << 18)
#define CMU_CLKEN0_LETIMER0     (1u << 12)
#define PRS_SRC_LETIMER0        0x02

static uint64_t lf_edge_ns(uint64_t edge)
{
    return (edge * 1000000000ull + 32767) / 32768;
}

/* Step @qts to just after LF edge number @edge. */
static void lf_step_to(QTestState *qts, uint64_t edge)
{
    uint64_t now = qtest_clock_step(qts, 0);

    qtest_clock_step(qts, lf_edge_ns(edge) + 1 - now);
}

static uint64_t lf_now(QTestState *qts)
{
    return qtest_clock_step(qts, 0) * 32768 / 1000000000ull;
}

static QTestState *letimer_start(void)
{
    QTestState *qts = prs_start();

    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_LETIMER0);
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_CH_CTRL(0),
                 PRS_CH_SOURCESEL(PRS_SRC_LETIMER0) | PRS_CH_SIGSEL(0) |
                     PRS_CH_FNSEL(PRS_FNSEL_A));
    lf_step_to(qts, lf_now(qts) + 1);
    return qts;
}

static void test_letimer_free_running_toggle(void)
{
    QTestState *qts = letimer_start();
    uint64_t e0 = lf_now(qts);

    qtest_writel(qts, LETIMER0_BASE + LT_EN, 1);
    qtest_writel(qts, LETIMER0_BASE + LT_CTRL,
                 LT_CTRL_CNTTOPEN | LT_CTRL_UFOA0_TOGGLE);
    qtest_writel(qts, LETIMER0_BASE + LT_TOP, 3);
    qtest_writel(qts, LETIMER0_BASE + LT_CMD, LT_CMD_START);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_SYNCBUSY), ==,
                    LT_SYNCBUSY_TOP | LT_SYNCBUSY_START);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_TOP), ==, 3);
    lf_step_to(qts, e0 + 2);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_SYNCBUSY), ==,
                    LT_SYNCBUSY_TOP | LT_SYNCBUSY_START);
    lf_step_to(qts, e0 + 3);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_SYNCBUSY), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 3);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_STATUS), ==, 1);
    lf_step_to(qts, e0 + 5);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK) & 1, ==, 0);
    lf_step_to(qts, e0 + 6);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 0);
    /* COMP0 and COMP1 are 0: they match with the underflow. */
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF), ==,
                    LT_IF_UF | LT_IF_COMP0 | (LT_IF_COMP0 << 1));
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK) & 1, ==, 1);
    lf_step_to(qts, e0 + 7);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 3);
    /* INTFLAG: a plain write does nothing, the CLR alias clears. */
    qtest_writel(qts, LETIMER0_BASE + LT_IF, 0);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF) & LT_IF_UF, ==,
                    LT_IF_UF);
    qtest_writel(qts, LETIMER0_BASE + LT_IF + REGISTER_CLEAR_WINDOW, LT_IF_UF);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF) & LT_IF_UF, ==, 0);
    lf_step_to(qts, e0 + 10);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF) & LT_IF_UF, ==,
                    LT_IF_UF);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK) & 1, ==, 0);
    lf_step_to(qts, e0 + 10 + 4 * 1000);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 0);
    qtest_quit(qts);
}

/* PWM (Table 18.2): idle at underflow, active at the COMP0 match. */
static void test_letimer_oneshot_pwm(void)
{
    QTestState *qts = letimer_start();
    uint64_t e0 = lf_now(qts);

    qtest_writel(qts, LETIMER0_BASE + LT_EN, 1);
    qtest_writel(qts, LETIMER0_BASE + LT_CTRL,
                 LT_CTRL_CNTTOPEN | LT_CTRL_UFOA0_PWM | LT_CTRL_ONESHOT);
    qtest_writel(qts, LETIMER0_BASE + LT_TOP, 3);
    qtest_writel(qts, LETIMER0_BASE + LT_COMP0, 2);
    qtest_writel(qts, LETIMER0_BASE + LT_REP0, 3);
    qtest_writel(qts, LETIMER0_BASE + LT_PRSMODE, LT_PRSMODE_START_RISING);
    qtest_writel(qts, PRS_BASE + PRS_CONSUMER_LETIMER0_START, 5);
    lf_step_to(qts, e0 + 3);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_STATUS), ==, 0);
    /* The PRS START acts at the next edge: e0 + 4 loads TOP = 3. */
    qtest_writel(qts, PRS_BASE + PRS_ASYNC_SWPULSE, 1u << 5);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_STATUS), ==, 1);
    lf_step_to(qts, e0 + 4);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 3);
    lf_step_to(qts, e0 + 5);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF), ==, LT_IF_COMP0);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK) & 1, ==, 1);
    lf_step_to(qts, e0 + 7);
    g_assert_cmphex(qtest_readl(qts, PRS_BASE + PRS_ASYNC_PEEK) & 1, ==, 0);
    /* Three underflows (e0 + 7, 11, 15), then the stop at e0 + 16. */
    lf_step_to(qts, e0 + 15);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_REP0), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_STATUS), ==, 1);
    lf_step_to(qts, e0 + 16);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_STATUS), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_REP0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LETIMER0_BASE + LT_IF) & LT_IF_REP0, ==,
                    LT_IF_REP0);
    lf_step_to(qts, e0 + 40);
    g_assert_cmpuint(qtest_readl(qts, LETIMER0_BASE + LT_CNT), ==, 0);
    qtest_quit(qts);
    /* A gated LETIMER0 read faults (silicon, SS7.73). */
    g_assert_true(m33_load_faults(LETIMER0_BASE + LT_CNT, setup_no_clocks));
}

/*
 * EUART0 (RM ch. 22) on EM01GRPACLK = HFXO, 38.4 MHz, 8N1 at CLKDIV 5072
 * (115.3 kbaud with OVS x16): one frame is 10 x 16 x (256 + 5072) /
 * (256 x 38.4 MHz) = 86.72 us.  CMD is LFSYNC (SYNCBUSY for 3 kernel
 * clocks), CFG0 a CONFIG register (faults with EN set), loopback, the FIFO
 * level flags, INTFLAG IF; the chardev line side both ways.
 */
#define EUART0_BASE             0x4a030000
#define EU_EN                   0x004
#define EU_CFG0                 0x008
#define EU_CLKDIV               0x028
#define EU_CMD                  0x030
#define EU_RXDATA               0x034
#define EU_TXDATA               0x03c
#define EU_STATUS               0x040
#define EU_IF                   0x044
#define EU_SYNCBUSY             0x04c
#define EU_CFG0_LOOPBK          (1u << 1)
#define EU_CMD_RXEN             (1u << 0)
#define EU_CMD_TXEN             (1u << 2)
#define EU_STATUS_RXENS         (1u << 0)
#define EU_STATUS_TXENS         (1u << 1)
#define EU_STATUS_TXC           (1u << 5)
#define EU_STATUS_TXFL          (1u << 6)
#define EU_STATUS_RXFL          (1u << 7)
#define EU_STATUS_TXIDLE        (1u << 13)
#define EU_IF_TXC               (1u << 0)
#define EU_IF_RXFL              (1u << 2)
#define EU_IF_RXUF              (1u << 5)
#define EU_CLKDIV_115200        5072
#define EU_FRAME_NS             86718
#define CMU_CLKEN0_EUART0       (1u << 24)

static void setup_euart_enabled(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_EUART0);
    qtest_writel(qts, EUART0_BASE + EU_EN, 1);
}

static void setup_euart_clock(QTestState *qts)
{
    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_EUART0);
}

static QTestState *euart_start(const char *extra, bool loopback)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel qtest %s",
                                  extra);

    setup_euart_clock(qts);
    qtest_writel(qts, CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL,
                 CMU_EM01GRPACLK_HFXO);
    qtest_writel(qts, EUART0_BASE + EU_CFG0, loopback ? EU_CFG0_LOOPBK : 0);
    qtest_writel(qts, EUART0_BASE + EU_CLKDIV, EU_CLKDIV_115200);
    qtest_writel(qts, EUART0_BASE + EU_EN, 1);
    qtest_writel(qts, EUART0_BASE + EU_CMD, EU_CMD_RXEN | EU_CMD_TXEN);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_SYNCBUSY), ==,
                    (EU_CMD_RXEN | EU_CMD_TXEN) << 3);
    qtest_clock_step(qts, 3 * 27);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_SYNCBUSY), ==, 0);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_STATUS) &
                    (EU_STATUS_RXENS | EU_STATUS_TXENS), ==,
                    EU_STATUS_RXENS | EU_STATUS_TXENS);
    return qts;
}

static void test_euart_loopback(void)
{
    QTestState *qts = euart_start("", true);

    qtest_writel(qts, EUART0_BASE + EU_TXDATA, 0x55);
    qtest_writel(qts, EUART0_BASE + EU_TXDATA, 0xaa);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_STATUS) &
                    (EU_STATUS_TXC | EU_STATUS_TXIDLE | EU_STATUS_RXFL), ==,
                    0);
    qtest_clock_step(qts, 2 * EU_FRAME_NS - 100);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_STATUS) &
                    (EU_STATUS_TXC | EU_STATUS_RXFL), ==, EU_STATUS_RXFL);
    qtest_clock_step(qts, 100);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_STATUS) &
                    (EU_STATUS_TXC | EU_STATUS_TXFL | EU_STATUS_TXIDLE), ==,
                    EU_STATUS_TXC | EU_STATUS_TXFL | EU_STATUS_TXIDLE);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_IF) &
                    (EU_IF_TXC | EU_IF_RXFL), ==, EU_IF_TXC | EU_IF_RXFL);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 0x55);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 0xaa);
    /* RXFL is a level flag: cleared, it stays clear with the FIFO empty. */
    qtest_writel(qts, EUART0_BASE + EU_IF + REGISTER_CLEAR_WINDOW, EU_IF_RXFL);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_IF) & EU_IF_RXFL, ==,
                    0);
    qtest_readl(qts, EUART0_BASE + EU_RXDATA);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_IF) & EU_IF_RXUF, ==,
                    EU_IF_RXUF);
    qtest_quit(qts);
    /* CONFIG: CFG0 written with EN set is a bus fault (RM SS4.2.4.4.2). */
    g_assert_true(m33_store_faults(EUART0_BASE + EU_CFG0,
                                   setup_euart_enabled));
    g_assert_false(m33_store_faults(EUART0_BASE + EU_CFG0,
                                    setup_euart_clock));
}

static void test_euart_chardev(void)
{
    g_autofree char *path = g_strdup_printf("%s/efr32-euart-%d.sock",
                                            g_get_tmp_dir(), getpid());
    g_autofree char *args = g_strdup_printf(
        "-chardev socket,id=soc.euart0,path=%s", path);
    int server = qtest_socket_server(path);
    QTestState *qts;
    int fd;
    char byte = 0;

    qts = euart_start(args, false);
    fd = accept(server, NULL, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(write(fd, "Q", 1), ==, 1);
    /* The byte is on the wire for one frame time. */
    for (int i = 0; i < 100 &&
                    !(qtest_readl(qts, EUART0_BASE + EU_STATUS) &
                      EU_STATUS_RXFL); i++) {
        qtest_clock_step(qts, EU_FRAME_NS);
    }
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 'Q');
    qtest_writel(qts, EUART0_BASE + EU_TXDATA, 'z');
    qtest_clock_step(qts, EU_FRAME_NS);
    g_assert_cmpint(read(fd, &byte, 1), ==, 1);
    g_assert_cmpint(byte, ==, 'z');
    close(fd);
    close(server);
    unlink(path);
    qtest_quit(qts);
}

/*
 * LDMA (RM ch. 26): a software-requested memory copy, a linked list loaded
 * with LINKLOAD (XFER, relative link, WRI), a peripheral-paced transfer
 * (EUART0 TXFL into TXDATA, looped back into the RX FIFO), and the bus
 * error of a radio-register source (silicon, SS7.74).
 */
#define LDMA_BASE               0x40040000
#define LDMAXBAR_BASE           0x40044000
#define LD_STATUS               0x00c
#define LD_CHEN                 0x024
#define LD_CHSTATUS             0x02c
#define LD_CHDONE               0x034
#define LD_SWREQ                0x03c
#define LD_LINKLOAD             0x048
#define LD_IF                   0x050
#define LD_CH(n)                (0x058 + 0x30 * (n))
#define LD_CH_CTRL              0x0c
#define LD_CH_SRC               0x10
#define LD_CH_DST               0x14
#define LD_CH_LINK              0x18
#define LD_IF_ERROR             (1u << 31)
#define LD_CTRL_XFERCNT(n)      (((n) - 1) << 4)
#define LD_CTRL_STRUCTREQ       (1u << 3)
#define LD_CTRL_BLOCK_ALL       (15u << 16)
#define LD_CTRL_DONEIEN         (1u << 20)
#define LD_CTRL_REQMODE_ALL     (1u << 21)
#define LD_CTRL_SRCINC_NONE     (3u << 24)
#define LD_CTRL_SIZE_WORD       (2u << 26)
#define LD_CTRL_DSTINC_NONE     (3u << 28)
#define LD_STRUCT_WRI           2u
#define LD_LINK_REL_NEXT        (0x10 | 0x2 | 0x1)
#define CMU_CLKEN0_LDMA         ((1u << 0) | (1u << 1))
#define LD_SRAM                 0x20004000

static QTestState *ldma_start(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel qtest");

    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_LDMA | CMU_CLKEN0_EUART0);
    return qts;
}

static void test_ldma_memory_copy(void)
{
    QTestState *qts = ldma_start();

    for (unsigned i = 0; i < 16; i++) {
        qtest_writeb(qts, LD_SRAM + i, 0x30 + i);
    }
    qtest_writel(qts, LDMA_BASE + LD_CH(0) + LD_CH_CTRL,
                 LD_CTRL_XFERCNT(16) | LD_CTRL_BLOCK_ALL |
                     LD_CTRL_REQMODE_ALL | LD_CTRL_DONEIEN);
    qtest_writel(qts, LDMA_BASE + LD_CH(0) + LD_CH_SRC, LD_SRAM);
    qtest_writel(qts, LDMA_BASE + LD_CH(0) + LD_CH_DST, LD_SRAM + 0x100);
    qtest_writel(qts, LDMA_BASE + LD_CHEN, 1);
    qtest_writel(qts, LDMA_BASE + LD_SWREQ, 1);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CHDONE), ==, 0);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CHDONE), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_IF), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CHSTATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LD_SRAM + 0x10c), ==, 0x3f3e3d3c);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CH(0) + LD_CH_SRC), ==,
                    LD_SRAM + 16);

    /* LINKLOAD a two-entry list: 4 words, then a write-immediate. */
    qtest_writel(qts, LD_SRAM + 0x200, LD_CTRL_XFERCNT(4) | LD_CTRL_BLOCK_ALL |
                                           LD_CTRL_REQMODE_ALL |
                                           LD_CTRL_SIZE_WORD |
                                           LD_CTRL_STRUCTREQ);
    qtest_writel(qts, LD_SRAM + 0x204, LD_SRAM);
    qtest_writel(qts, LD_SRAM + 0x208, LD_SRAM + 0x300);
    qtest_writel(qts, LD_SRAM + 0x20c, LD_LINK_REL_NEXT);
    qtest_writel(qts, LD_SRAM + 0x210, LD_STRUCT_WRI | LD_CTRL_DONEIEN);
    qtest_writel(qts, LD_SRAM + 0x214, 0xcafef00d);
    qtest_writel(qts, LD_SRAM + 0x218, LD_SRAM + 0x320);
    qtest_writel(qts, LD_SRAM + 0x21c, 0);
    qtest_writel(qts, LDMA_BASE + LD_IF + REGISTER_CLEAR_WINDOW, ~0u);
    qtest_writel(qts, LDMA_BASE + LD_CHDONE, 0);
    qtest_writel(qts, LDMA_BASE + LD_CH(2) + LD_CH_LINK, LD_SRAM + 0x200);
    qtest_writel(qts, LDMA_BASE + LD_LINKLOAD, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CHSTATUS), ==, 1u << 2);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, LD_SRAM + 0x30c), ==, 0x3f3e3d3c);
    g_assert_cmphex(qtest_readl(qts, LD_SRAM + 0x320), ==, 0xcafef00d);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CHDONE), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_IF), ==, 1u << 2);

    /* A radio register source: IF.ERROR, STATUS.CHERROR = 3. */
    qtest_writel(qts, LDMA_BASE + LD_CH(3) + LD_CH_CTRL,
                 LD_CTRL_XFERCNT(1) | LD_CTRL_SIZE_WORD);
    qtest_writel(qts, LDMA_BASE + LD_CH(3) + LD_CH_SRC, RAC_BASE + RAC_STATUS);
    qtest_writel(qts, LDMA_BASE + LD_CH(3) + LD_CH_DST, LD_SRAM + 0x400);
    qtest_writel(qts, LDMA_BASE + LD_CHEN, 1u << 3);
    qtest_writel(qts, LDMA_BASE + LD_SWREQ, 1u << 3);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_IF) & LD_IF_ERROR, ==,
                    LD_IF_ERROR);
    g_assert_cmphex((qtest_readl(qts, LDMA_BASE + LD_STATUS) >> 8) & 0x1f,
                    ==, 3);
    qtest_quit(qts);
}

static void test_ldma_peripheral_request(void)
{
    QTestState *qts = euart_start("", true);

    qtest_writel(qts, CMU_BASE + CMU_CLKEN0 + REGISTER_SET_WINDOW,
                 CMU_CLKEN0_LDMA);
    qtest_writel(qts, LD_SRAM, 0x44332211);
    /* EUART0 TXFL: SOURCESEL 0x10, SIGSEL 1; one byte per request. */
    qtest_writel(qts, LDMAXBAR_BASE + 4 * 1, (0x10u << 16) | 1);
    qtest_writel(qts, LDMA_BASE + LD_CH(1) + LD_CH_CTRL,
                 LD_CTRL_XFERCNT(4) | LD_CTRL_DSTINC_NONE | LD_CTRL_DONEIEN);
    qtest_writel(qts, LDMA_BASE + LD_CH(1) + LD_CH_SRC, LD_SRAM);
    qtest_writel(qts, LDMA_BASE + LD_CH(1) + LD_CH_DST,
                 EUART0_BASE + EU_TXDATA);
    qtest_writel(qts, LDMA_BASE + LD_CHEN, 1u << 1);
    qtest_clock_step(qts, 4 * EU_FRAME_NS + 1000);
    g_assert_cmphex(qtest_readl(qts, LDMA_BASE + LD_CHDONE), ==, 1u << 1);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 0x11);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 0x22);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 0x33);
    g_assert_cmphex(qtest_readl(qts, EUART0_BASE + EU_RXDATA), ==, 0x44);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    int result;
    gchar *qemu_args;

    g_test_init(&argc, &argv, NULL);
    part_write_pages();
    tx_snapshot_trace_path = g_strdup_printf(
        "%s/efr32bg22-tx-snapshot-%u.trace", g_get_tmp_dir(), g_random_int());
    qemu_args = g_strdup_printf(
        "-machine efr32bg22 -S -trace enable=efr32bg22_*,file=%s",
        tx_snapshot_trace_path);
    qtest_start(qemu_args);
    enable_radio_bus_clocks();
    g_free(qemu_args);


    qtest_add_func("/efr32bg22/register-aliases", test_register_aliases);
    qtest_add_func("/efr32bg22/bgm220-module-identity",
                   test_bgm220_module_identity);
    qtest_add_func("/efr32bg22/burtc-counter-contract",
                   test_burtc_counter_contract);
    qtest_add_func("/efr32bg22/burtc-register-aliases",
                   test_burtc_register_aliases);
    qtest_add_func("/efr32bg22/burtc-compare-pending",
                   test_burtc_compare_pending);
    qtest_add_func("/efr32bg22/burtc-compare-lead", test_burtc_compare_lead);
    qtest_add_func("/efr32bg22/burtc-retarget-window",
                   test_burtc_retarget_window);
    qtest_add_func("/efr32bg22/burtc-en-sync", test_burtc_en_sync);
    qtest_add_func("/efr32bg22/timer0-compare-equal-fires-next-tick",
                   test_timer0_compare_equal_fires_next_tick);
    qtest_add_func("/efr32bg22/timer-cmd-syncbusy", test_timer_cmd_syncbusy);
    qtest_add_func("/efr32bg22/timer-sync-registers-read-zero-when-disabled",
                   test_timer_sync_registers_read_zero_when_disabled);
    qtest_add_func("/efr32bg22/timer-bus-faults", test_timer_bus_faults);
    qtest_add_func("/efr32bg22/prs-logic-table", test_prs_logic_table);
    qtest_add_func("/efr32bg22/prs-bus-gate", test_prs_bus_gate);
    qtest_add_func("/efr32bg22/prs-swpulse-timer-capture",
                   test_prs_swpulse_timer_capture);
    qtest_add_func("/efr32bg22/prs-timer-input-qualification",
                   test_prs_timer_input_qualification);
    qtest_add_func("/efr32bg22/prs-timer-reloadstart-oneshot",
                   test_prs_timer_reloadstart_oneshot);
    qtest_add_func("/efr32bg22/prs-rtcc-capture", test_prs_rtcc_capture);
    qtest_add_func("/efr32bg22/iadc-triggers", test_iadc_triggers);
    qtest_add_func("/efr32bg22/iadc-resolution-and-window",
                   test_iadc_resolution_and_window);
    qtest_add_func("/efr32bg22/buram-retention", test_buram_retention);
    qtest_add_func("/efr32bg22/letimer-free-running-toggle",
                   test_letimer_free_running_toggle);
    qtest_add_func("/efr32bg22/letimer-oneshot-pwm", test_letimer_oneshot_pwm);
    qtest_add_func("/efr32bg22/euart-loopback", test_euart_loopback);
    qtest_add_func("/efr32bg22/euart-chardev", test_euart_chardev);
    qtest_add_func("/efr32bg22/ldma-memory-copy", test_ldma_memory_copy);
    qtest_add_func("/efr32bg22/ldma-peripheral-request",
                   test_ldma_peripheral_request);
    qtest_add_func("/efr32bg22/trng-fifo", test_trng_fifo);
    qtest_add_func("/efr32bg22/trng-conditioning-kat",
                   test_trng_conditioning_kat);
    qtest_add_func("/efr32bg22/gpcrc-crc-vectors", test_gpcrc_crc_vectors);
    qtest_add_func("/efr32bg22/radioaes-ecb-and-key-sizes",
                   test_radioaes_ecb_and_key_sizes);
    qtest_add_func("/efr32bg22/radioaes-clock-gate", test_radioaes_clock_gate);
    qtest_add_func("/efr32bg22/lfxo-ready-timeout", test_lfxo_ready_timeout);
    qtest_add_func("/efr32bg22/timer4-counter-and-aliases",
                   test_timer4_counter_and_aliases);
    qtest_add_func("/efr32bg22/timer2-counter-and-aliases",
                   test_timer2_counter_and_aliases);
    qtest_add_func("/efr32bg22/timer4-cc0-compare-pending",
                   test_timer4_cc0_compare_pending);
    qtest_add_func("/efr32bg22/timer0-32bit-count-prescale-overflow",
                   test_timer0_32bit_count_prescale_overflow);
    qtest_add_func("/efr32bg22/timer0-cc-output-compare",
                   test_timer0_cc_output_compare);
    qtest_add_func("/efr32bg22/timer0-follows-em01grpaclk-dpll",
                   test_timer0_follows_em01grpaclk_dpll);
    qtest_add_func("/efr32bg22/system-reset-clears-radio-timer-state",
                   test_system_reset_clears_radio_timer_state);
    qtest_add_func("/efr32bg22/cmu-nonsecure-alias", test_cmu_nonsecure_alias);
    qtest_add_func("/efr32bg22/dpll-lock", test_dpll_lock);
    qtest_add_func("/efr32bg22/usart-tx-polling", test_usart_tx_polling);
    qtest_add_func("/efr32bg22/usart-frame-time", test_usart_frame_time);
    qtest_add_func("/efr32bg22/usart-timer-and-break",
                   test_usart_timer_and_break);
    qtest_add_func("/efr32bg22/modem-masks", test_modem_masks);
    qtest_add_func("/efr32bg22/modem-pa-ramp-registers",
                   test_modem_pa_ramp_registers);
    qtest_add_func("/efr32bg22/rfcrc-masks", test_rfcrc_masks);
    qtest_add_func("/efr32bg22/rfcrc-cmd-initialize",
                   test_rfcrc_cmd_initialize);
    qtest_add_func("/efr32bg22/agc-rssi-status-registers",
                   test_agc_rssi_status_registers);
    qtest_add_func("/efr32bg22/agc-rssi-valid-latches",
                   test_agc_rssi_valid_latches);
    qtest_add_func("/efr32bg22/agc-rssi-start-command",
                   test_agc_rssi_start_command);
    qtest_add_func("/efr32bg22/frc-buffer-register-masks",
                   test_frc_buffer_register_masks);
    qtest_add_func("/efr32bg22/frc-sniffctrl-mask", test_frc_sniffctrl_mask);
    qtest_add_func("/efr32bg22/frc-rawctrl-mask", test_frc_rawctrl_mask);
    qtest_add_func("/efr32bg22/frc-status-is-read-only",
                   test_frc_status_is_read_only);
    qtest_add_func("/efr32bg22/rail-initialization-latches",
                   test_rail_initialization_latches);
    qtest_add_func("/efr32bg22/rac-sr3-clear-alias", test_rac_sr3_clear_alias);
    qtest_add_func("/efr32bg22/frc-interrupt-register-masks",
                   test_frc_interrupt_register_masks);
    qtest_add_func("/efr32bg22/frc-completion-interrupt-enable",
                   test_frc_completion_interrupt_enable);
    qtest_add_func("/efr32bg22/radio-enable-masks", test_radio_enable_masks);
    qtest_add_func("/efr32bg22/radio-enable-requires-bus-clocks",
                   test_radio_enable_requires_bus_clocks);
    qtest_add_func("/efr32bg22/radio-ipversions", test_radio_ipversions);
    qtest_add_func("/efr32bg22/radio-nonsecure-alias",
                   test_radio_nonsecure_alias);
    qtest_add_func("/efr32bg22/stock-setup-aliases",
                   test_stock_setup_aliases);
    qtest_add_func("/efr32bg22/rac-enable-requires-dependency-order",
                   test_rac_enable_requires_dependency_order);
    qtest_add_func("/efr32bg22/dcdc-regulation", test_dcdc_regulation);
    qtest_add_func("/efr32bg22/emu-status-reset", test_emu_status_reset);
    qtest_add_func("/efr32bg22/msc-status-reset", test_msc_status_reset);
    qtest_add_func("/efr32bg22/part-352k-identity", test_part_352k_identity);
    qtest_add_func("/efr32bg22/part-352k-flash-end-faults",
                   test_part_352k_flash_end_faults);
    qtest_add_func("/efr32bg22/rtcc-counts-and-compares",
                   test_rtcc_counts_and_compares);
    qtest_add_func("/efr32bg22/prortc-counts-and-compares",
                   test_prortc_counts_and_compares);
    qtest_add_func("/efr32bg22/iadc-single-conversion",
                   test_iadc_single_conversion);
    qtest_add_func("/efr32bg22/emu-rstcause", test_emu_rstcause);
    qtest_add_func("/efr32bg22/hfxo-ready-status", test_hfxo_ready_status);
    qtest_add_func("/efr32bg22/bufc-data-port", test_bufc_data_port);
    qtest_add_func("/efr32bg22/bufc-packet-channels-are-independent",
                   test_bufc_packet_channels_are_independent);
    qtest_add_func("/efr32bg22/bufc-tx-fifo-setup-latches",
                   test_bufc_tx_fifo_setup_latches);
    qtest_add_func("/efr32bg22/bufc-backing-memory-wrap",
                   test_bufc_backing_memory_wrap);
    qtest_add_func("/efr32bg22/bufc-overflow-latch",
                   test_bufc_overflow_latch);
    qtest_add_func("/efr32bg22/bufc-descriptor-ring-overflow-latch",
                   test_bufc_descriptor_ring_overflow_latch);
    qtest_add_func("/efr32bg22/bufc-rx-fifo-setup-latches",
                   test_bufc_rx_fifo_setup_latches);
    qtest_add_func("/efr32bg22/bufc-rx-descriptor-ring-setup-latches",
                   test_bufc_rx_descriptor_ring_setup_latches);
    qtest_add_func("/efr32bg22/bufc-interrupt-register-masks",
                   test_bufc_interrupt_register_masks);
    qtest_add_func("/efr32bg22/bufc-channel-register-masks",
                   test_bufc_channel_register_masks);
    qtest_add_func("/efr32bg22/bufc-sparse-threshold-flag",
                   test_bufc_sparse_threshold_flag);
    qtest_add_func("/efr32bg22/bufc-descriptor-threshold-flag",
                   test_bufc_descriptor_threshold_flag);
    qtest_add_func("/efr32bg22/protimer-trace-latches",
                   test_protimer_trace_latches);
    qtest_add_func("/efr32bg22/protimer-secure-nonsecure-alias",
                   test_protimer_secure_nonsecure_alias);
    qtest_add_func("/efr32bg22/protimer-interrupt-banks-are-aliasable",
                   test_protimer_interrupt_banks_are_aliasable);
    qtest_add_func("/efr32bg22/protimer-compare-slot-clear-alias",
                   test_protimer_compare_slot_clear_alias);
    qtest_add_func("/efr32bg22/protimer-counter", test_protimer_counter);
    qtest_add_func("/efr32bg22/protimer-counter-is-independent-from-radio-setup",
                   test_protimer_counter_is_independent_from_radio_setup);
    qtest_add_func("/efr32bg22/protimer-counter-is-independent-from-burtc",
                   test_protimer_counter_is_independent_from_burtc);
    qtest_add_func("/efr32bg22/rac-tx-commands", test_rac_tx_commands);
    qtest_add_func("/efr32bg22/seq-tx-waits-for-synthesizer",
                   test_seq_tx_waits_for_synthesizer);
    qtest_add_func("/efr32bg22/pa-ramp-survives-clear-after-paramp",
                   test_pa_ramp_survives_clear_after_paramp);
    qtest_add_func("/efr32bg22/seq-unacked-state-event-holds-txen",
                   test_seq_unacked_state_event_holds_txen);
    qtest_add_func("/efr32bg22/frc-wcntcmp0-dynamic-length",
                   test_frc_wcntcmp0_dynamic_length);
    qtest_add_func("/efr32bg22/rac-tx-completion", test_rac_tx_completion);
    qtest_add_func("/efr32bg22/rac-tx-disable-cancels-completion",
                   test_rac_tx_disable_cancels_completion);
    qtest_add_func("/efr32bg22/rac-rx-enable-cancels-tx-completion",
                   test_rac_rx_enable_cancels_tx_completion);
    qtest_add_func("/efr32bg22/system-reset-cancels-tx-completion",
                   test_system_reset_cancels_tx_completion);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-captures-staged-bufc",
                   test_rac_tx_snapshot_captures_staged_bufc);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-captures-guest-backed-fifo",
                   test_rac_tx_snapshot_captures_guest_backed_fifo);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-wraps-guest-backed-fifo",
                   test_rac_tx_snapshot_wraps_guest_backed_fifo);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-has-no-local-offers",
                   test_rac_tx_snapshot_has_no_local_offers);
    qtest_add_func("/efr32bg22/rac-software-rx-enable",
                   test_rac_software_rx_enable);
    qtest_add_func("/efr32bg22/rac-masks", test_rac_masks);
    qtest_add_func("/efr32bg22/rfmailbox-flags-latch",
                   test_rfmailbox_flags_latch);
    qtest_add_func("/efr32bg22/gated-bus-clock-faults-and-drops",
                   test_gated_bus_clock_faults_and_drops);
    qtest_add_func("/efr32bg22/seq-sleeping-follows-wfi",
                   test_seq_sleeping_follows_wfi);
    qtest_add_func("/efr32bg22/m33-lockup", test_m33_lockup);
    qtest_add_func("/efr32bg22/rstctrl-blocks-sysresetreq",
                   test_rstctrl_blocks_sysresetreq);
    qtest_add_func("/efr32bg22/seq-lockup-until-seqreset",
                   test_seq_lockup_until_seqreset);
    qtest_add_func("/efr32bg22/rac-sr0-clear-alias",
                   test_rac_sr0_clear_alias);
    qtest_add_func("/efr32bg22/rac-force-disable", test_rac_force_disable);
    qtest_add_func("/efr32bg22/rac-stctrl-mask", test_rac_stctrl_mask);
    qtest_add_func("/efr32bg22/seq-nvic-ipr-byte-isolation",
                   test_seq_nvic_ipr_byte_isolation);
    qtest_add_func("/efr32bg22/seq-systick-counts", test_seq_systick_counts);
    qtest_add_func("/efr32bg22/core-clocks-per-cpu-rates",
                   test_core_clocks_per_cpu_rates);
    qtest_add_func("/efr32bg22/core-clocks-radio-clock-stops-seq",
                   test_core_clocks_radio_clock_stops_seq);
    qtest_add_func("/efr32bg22/radio-sequencer-executes-open-code",
                   test_radio_sequencer_executes_open_code);
    qtest_add_func("/efr32bg22/radio-sequencer-release-is-idempotent",
                   test_radio_sequencer_release_is_idempotent);

    result = g_test_run();
    qtest_end();
    g_unlink(tx_snapshot_trace_path);
    g_unlink(part_devinfo_path);
    g_unlink(part_userdata_path);
    g_free(tx_snapshot_trace_path);
    return result;
}
