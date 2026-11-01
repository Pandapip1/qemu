/*
 * End-to-end QTest coverage for the EFR32BG22 in-process RF medium.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

static char *rf_trace_path;

#define EFR32BG22_PEER_PERIPH_BASE 0x60000000
#define EFR32BG22_PEER_RADIO_BASE  0x80000000

#define RADIO_BASE               0xa8000000
#define FRC_BASE                 0xa8004000
#define FRC_IF                   0x068
#define FRC_RXRAWDATA            0x090
#define FRC_IF_TXDONE            (1u << 0)
#define FRC_IF_RXDONE            (1u << 4)
#define RAC_BASE                 0xa8020000
#define RAC_CMD                  0x010
#define RAC_CTRL                 0x014
#define RAC_RXENSRCEN            0x008
#define RAC_CMD_TXEN             (1u << 0)
#define RAC_CMD_TXDIS            (1u << 5)
#define RAC_CMD_RXDIS            (1u << 8)
#define RAC_CTRL_FORCEDISABLE    (1u << 0)
#define RAC_RXENSRCEN_SWRXEN     (1u << 1)
#define RAC_STATUS               0x00c
#define RAC_STATUS_STATE_MASK     0x0f000000
#define RAC_STATUS_STATE_RXSEARCH (2u << 24)
#define SYNTH_BASE               0xa8018000
#define SYNTH_CHCTRL             0x040
#define SYNTH_FREQ               0x034
#define SYNTH_FREQ_2402_MHZ      0x01f46aabu
#define SYNTH_FREQ_2404_MHZ      0x01f4d555u
#define SYNTH_CHSP               0x044
#define BUFC_BASE                0xaa000000
#define BUFC_CH0                 0x00c
#define BUFC_CH1                 0x04c
#define BUFC_CH2                 0x08c
#define BUFC_ADDR                0x004
#define BUFC_WRITEOFFSET         0x008
#define BUFC_READOFFSET          0x00c
#define BUFC_READDATA            0x014
#define BUFC_WRITEDATA           0x018
#define BUFC_STATUS              0x020
#define BUFC_THRESHOLD           0x024
#define BUFC_CMD                 0x028
#define BUFC_IF                  0x114
#define BUFC_IEN                 0x118
#define REGISTER_SET_WINDOW      0x1000
#define REGISTER_CLEAR_WINDOW    0x2000

/* Released-sequencer receive and turnaround (docs/radio-tables.md SS7.63-69). */
#define RAC_SEQIF                0x028
#define RAC_SEQIEN               0x02c
#define RAC_STIMER               0x030
#define RAC_SYEN                 0x15c
#define RAC_IFADCDBG             0x0e0
#define RAC_IFADCDBG_IFADCENCLK  (1u << 1)
#define RAC_RFPATHEN             0x140
#define RAC_RFPATHEN_LNAMIXEN    (1u << 1)
#define RAC_CTRL_CPUWAITDIS      (1u << 26)
#define RAC_STATUS_STATE_SHIFT   24
#define RAC_STATUS_SEQACTIVE     (1u << 28)
#define RAC_STATUS_TXENS         (1u << 30)
#define RAC_STATUS_RXENS         (1u << 31)
#define RAC_STATUS_RXMASK_SWRXEN (1u << 1)
#define RAC_STATE_OFF            0u
#define RAC_STATE_RXSEARCH       2u
#define RAC_STATE_RXFRAME        3u
#define RAC_STATE_RXPD           4u
#define RAC_STATE_RX2RX          5u
#define RAC_STATE_RX2TX          7u
#define RAC_STATE_TXWARM         8u
#define RAC_STATE_TX             9u
#define RAC_STATE_TXPD           10u
#define RAC_STATE_TX2RX          11u
#define RAC_SEQIF_STATE(state)   (1u << (16 + (state)))
/* Stock's RAC.SYEN while transmitting (SS7.60). */
#define RAC_SYEN_STOCK_TX        0x00001fbfu
#define SYNTH_CMD                0x00c
#define SYNTH_CMD_SYNTHSTART     (1u << 0)
#define FRC_TRAILRXDATA          0x054
#define FRC_TRAILRXDATA_CRCOK    (1u << 1)
/* [I] Trailer octet bit 7: CRC passed (SS7.66); the medium has no errors. */
#define FRC_TRAILER_CRCOK        0x80u
/*
 * Model timing (SS7.71/SS7.72): TX entry to exit is 0.65 us + preamble and
 * access address + (PDU + CRC) octets at 8 us + 0.9 us; a peer's EOF is
 * 9.45 us after the transmitter's last bit; frame detection 3.5 us after the
 * access address arrives; RXFRAME exit 3.3 us after EOF; transitions follow
 * a cleared state event by 1.09 us.
 */
#define OCTET_NS                 8000
#define TX_ENTRY_TO_AIR_NS       650
#define SYNC_NS                  (5 * OCTET_NS)
#define CRC_OCTETS               3
#define FRAMESENT_TO_TX_EXIT_NS  900
#define FRAMESENT_TO_PEER_EOF_NS 9450
#define SYNC_TO_FRAMEDET_NS      3500
#define EOF_TO_RX_EXIT_NS        3300
#define ACK_TO_TRANSITION_NS     1090
#define MODEL_TX_NS(pdu)         (TX_ENTRY_TO_AIR_NS + SYNC_NS + \
                                  ((pdu) + CRC_OCTETS) * OCTET_NS + \
                                  FRAMESENT_TO_TX_EXIT_NS)
/* Legacy TXEN: TX entry at TXEN. */
#define MODEL_RX_DETECT_NS       (TX_ENTRY_TO_AIR_NS + SYNC_NS + \
                                  FRAMESENT_TO_PEER_EOF_NS + \
                                  SYNC_TO_FRAMEDET_NS)
#define MODEL_RX_EOF_NS(pdu)     (TX_ENTRY_TO_AIR_NS + SYNC_NS + \
                                  ((pdu) + CRC_OCTETS) * OCTET_NS + \
                                  FRAMESENT_TO_PEER_EOF_NS)
#define MODEL_LONG_WAIT_NS       1000000
#define STATUS_MEASURED_BITS     (RAC_STATUS_RXENS | RAC_STATUS_TXENS | \
                                  RAC_STATUS_DEMODENS | RAC_STATUS_SEQACTIVE | \
                                  RAC_STATUS_RXMASK_SWRXEN)
#define RAC_STATUS_DEMODENS      (1u << 29)
#define RAC_STATE_RXWARM         1u
#define RAC_STATE_SHUTDOWN       13u
#define RAC_SR0                  0x040
#define CMU_CLKEN1_RAC           (1u << 5)
#define CMU_CLKEN1_BUFC          (1u << 11)
/* RAC.STIMER: ~4.8 ticks/us (SS7.66). */
#define STIMER_TEST_INTERVAL_NS  10000
#define SYNTH_STATUS             0x008
#define SYNTH_STATUS_INLOCK      (1u << 0)
#define SYNTH_STATUS_IFFREQEN    (1u << 1)
#define SYNTH_CMD_SYNTHSTOP      (1u << 1)
#define SYNTH_CMD_ENABLEIF       (1u << 2)
#define SYNTH_CMD_DISABLEIF      (1u << 3)
#define SYNTH_LOCK_NS            8900
#define MODEM_BASE               0xa8014000
#define MODEM_SEQIF              0x24c
#define MODEM_IF_TXRAMPDONE      (1u << 3)
#define RAC_PAENCTRL             0x0b4
#define RAC_PAENCTRL_PARAMP      (1u << 8)
#define PA_RAMP_UP_NS            440
#define PA_RAMP_DOWN_NS          3960
#define STIMER_TICKS_IN_INTERVAL 48

static uint64_t peer(uint32_t address)
{
    if (address >= FRC_BASE) {
        return EFR32BG22_PEER_RADIO_BASE + (address - RADIO_BASE);
    }
    return EFR32BG22_PEER_PERIPH_BASE + (address - 0x40000000);
}

/*
 * Every radio block and BUFC answers with a bus error while its CMU clocks
 * are off (owo-open-fw docs/radio-tables.md SS7.72); start both SoCs with
 * them on.
 */
#define CMU_BASE                 0x40008000
#define CMU_CLKEN1               0x068
#define CMU_RADIOCLKCTRL         0x280
#define CMU_RADIOCLKCTRL_EN      (1u << 0)
#define CMU_CLKEN1_RADIO_BLOCKS  0x7ffu

static void rf_enable_radio_clocks(void)
{
    const uint64_t cmu[] = { CMU_BASE, peer(CMU_BASE) };

    for (size_t i = 0; i < G_N_ELEMENTS(cmu); i++) {
        qtest_writel(global_qtest, cmu[i] + CMU_RADIOCLKCTRL + 0x1000,
                     CMU_RADIOCLKCTRL_EN);
        qtest_writel(global_qtest, cmu[i] + CMU_CLKEN1 + 0x1000,
                     CMU_CLKEN1_RADIO_BLOCKS);
    }
}

static void rf_power_up(void)
{
    rf_enable_radio_clocks();
    /*
     * Both radios on the same reachable carrier, 2402 MHz: SYNTH.FREQ resets to 0, which the
     * synthesizer does not reach and the medium never matches (SS7.85).
     */
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, SYNTH_FREQ_2402_MHZ);
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_FREQ), SYNTH_FREQ_2402_MHZ);
}

static void rf_reset_with_clocks(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    rf_power_up();
}

/*
 * The test board intentionally gives both SoCs RAM at 0x20000000.  BUFC is a
 * bus master, so its guest backing store must resolve through the owning
 * SoC's address space, not the global qtest map (which is SoC 0's map).
 */
static void test_peer_bufc_dma_uses_peer_address_space(void)
{
    const uint32_t shared_guest_address = 0x20000100;
    const uint64_t peer_tx_channel = peer(BUFC_BASE + BUFC_CH0);

    qtest_writeb(global_qtest, shared_guest_address, 0xa5);
    qtest_writel(global_qtest, peer_tx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_tx_channel + BUFC_ADDR,
                 shared_guest_address);
    qtest_writel(global_qtest, peer_tx_channel, 0); /* 64-byte peer FIFO */
    qtest_writel(global_qtest, peer_tx_channel + BUFC_WRITEDATA, 0x5a);

    g_assert_cmphex(qtest_readb(global_qtest, shared_guest_address), ==, 0xa5);
    qtest_writel(global_qtest, peer_tx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_tx_channel + BUFC_READDATA), ==, 0x5a);
}

/*
 * This fixture has two native EFR32 SoCs.  qtest only performs their normal
 * MMIO staging/configuration: the source's guest-backed BUFC channel 0 holds
 * the TX bytes and the receiver is placed in RXSEARCH with an independent,
 * same-address guest-backed BUFC FIFO. No QMP, host Bluetooth, or packet
 * injection path exists in this test.
 */
static void test_tx_offer_reaches_peer_rx_fifo(void)
{
    const uint8_t payload[] = { 0x91, 0x72, 0xe3 };
    const uint32_t fifo_guest_address = 0x20000180;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_RXENSRCEN)) &
                        RAC_RXENSRCEN_SWRXEN,
                    ==, RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_STATUS)) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_RXSEARCH);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_RXRAWDATA)),
                    ==, payload[G_N_ELEMENTS(payload) - 1]);
    /* An unconfigured channel 2 must not acquire a synthetic record. */
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer(BUFC_BASE + BUFC_CH2 + BUFC_STATUS)),
                    ==, 0);
    /*
     * RAIL's RX-FIFO reader consumes channel 1 using STATUS's low 13-bit
     * count together with READOFFSET.  The medium is deliberately a
     * synthetic producer, but its byte transport must retain that FIFO
     * consumer contract rather than presenting a fabricated PDU record.
     */
    g_assert_cmphex(qtest_readl(global_qtest, peer_rx_channel + BUFC_STATUS) &
                        0x1fff,
                    ==, G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, payload[index]);
    }
    g_assert_cmphex(qtest_readl(global_qtest, peer_rx_channel + BUFC_STATUS) &
                        0x1fff,
                    ==, 0);
}

/*
 * The RF medium is still a raw-byte mailbox, but it must not hand a staged
 * transmission to an RXSEARCH peer tuned to another physical channel.  The
 * matching peer may enter RXSEARCH while the TX transaction is active and
 * receive that same snapshot once.
 */
static void test_tx_offer_requires_matching_synth_channel(void)
{
    const uint8_t payload[] = { 0x6e, 0xb4 };
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    rf_reset_with_clocks();
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    /* The carrier is SYNTH.FREQ (f * 2^19 / 38.4 MHz; SS7.80) with CHCTRL zero, as
     * silicon's working path programs it; CHSP alone does not move it. */
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, SYNTH_FREQ_2402_MHZ);
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_FREQ), SYNTH_FREQ_2404_MHZ);
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_CHSP, 10);
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_CHSP), 10);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);

    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_FREQ), SYNTH_FREQ_2402_MHZ);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, payload[index]);
    }

    rf_reset_with_clocks();
}

static void rf_step(int64_t ns);

/* SYNTH.FREQ for a carrier in kHz: round(f x 2^19 / 38.4 MHz) (SS7.80). */
static uint32_t synth_freq_word(uint64_t khz)
{
    return (uint32_t)(((khz * 1000u << 19) + 19200000u) / 38400000u);
}

#define MODEM_CTRL0              0x020
#define MODEM_CTRL0_MODFORMAT_OOKASK (6u << 6)
#define MODEM_TXBR               0x058
/* TXBRNUM 24000, TXBRDEN 1: 38.4 MHz / (8 x 24000) = 200 Bd. */
#define MODEM_TXBR_200BD         ((1u << 16) | 24000u)
#define MODEM_CTRL1              0x024
#define MODEM_CTRL1_SYNCBITS_32  31u

/*
 * Transmit two octets on @p tx_khz with the peer searching on @p rx_khz; return whether the
 * peer received them. With @p ook the source modem is set to on-off keying.
 */
static bool rf_delivered(uint64_t tx_khz, uint64_t rx_khz, bool ook)
{
    const uint8_t payload[] = { 0x6e, 0xb4 };
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    bool delivered;

    rf_reset_with_clocks();
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, synth_freq_word(tx_khz));
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_FREQ), synth_freq_word(rx_khz));
    if (ook) {
        qtest_writel(global_qtest, MODEM_BASE + MODEM_CTRL0, MODEM_CTRL0_MODFORMAT_OOKASK);
        qtest_writel(global_qtest, MODEM_BASE + MODEM_TXBR, MODEM_TXBR_200BD);
        qtest_writel(global_qtest, MODEM_BASE + MODEM_CTRL1, MODEM_CTRL1_SYNCBITS_32);
    }
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA, payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    delivered = (qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) & FRC_IF_RXDONE) != 0;
    rf_reset_with_clocks();
    return delivered;
}

/*
 * Amateur channel plans (owo-open-fw docs/amateur-radio.md) put carriers on any whole MHz: the
 * medium matches carriers on a 1 MHz raster, so 2403 MHz reaches only a receiver on 2403 MHz,
 * not one on a Bluetooth channel either side.
 */
static void test_carrier_matches_on_1mhz_raster(void)
{
    g_assert_true(rf_delivered(2403000, 2403000, false));
    g_assert_false(rf_delivered(2403000, 2402000, false));
    g_assert_false(rf_delivered(2403000, 2404000, false));
    g_assert_true(rf_delivered(2447000, 2447000, false));
    g_assert_true(rf_delivered(2402000, 2402000, false));
    /* 13 cm below the ISM band: measured reachable (SS7.85). */
    g_assert_true(rf_delivered(2304000, 2304000, false));
    g_assert_true(rf_delivered(2395000, 2395000, false));
}

/*
 * A carrier the synthesizer does not reach (EFR32_SYNTH_LOCK_MIN_HZ..MAX_HZ, SS7.85) reaches no
 * receiver, even one tuned to the same word.
 */
static void test_carrier_outside_synth_range_not_delivered(void)
{
    g_assert_false(rf_delivered(2289000, 2289000, false));
    g_assert_false(rf_delivered(2280000, 2280000, false));
    g_assert_false(rf_delivered(2501000, 2501000, false));
}

/* Nor does the synthesizer report lock there (SYNTH.STATUS.INLOCK, SS7.72, SS7.85). */
static void test_synth_never_locks_outside_range(void)
{
    rf_reset_with_clocks();
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, synth_freq_word(2280000));
    qtest_writel(global_qtest, RAC_BASE + RAC_SYEN, RAC_SYEN_STOCK_TX);
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_CMD,
                 SYNTH_CMD_DISABLEIF | SYNTH_CMD_SYNTHSTART);
    rf_step(SYNTH_LOCK_NS * 10);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, 0);
    /* Retuned to a carrier it reaches, it is locked. */
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, SYNTH_FREQ_2402_MHZ);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, SYNTH_STATUS_INLOCK);
    rf_reset_with_clocks();
}

/* An on-off keyed frame (MODEM.CTRL0.MODFORMAT = OOK/ASK) is no Bluetooth packet. */
static void test_ook_frame_not_delivered_to_ble_receiver(void)
{
    g_assert_false(rf_delivered(2427000, 2427000, true));
}

/*
 * The OOK frame lasts its symbols at the TXBR rate: 32 sync + 16 data symbols at 200 Bd is
 * 240 ms; TXDONE is not set before.
 */
static void test_ook_frame_airtime(void)
{
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const int64_t airtime_ns = (32 + 16) * 5000000ll;

    rf_reset_with_clocks();
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, synth_freq_word(2427000));
    qtest_writel(global_qtest, MODEM_BASE + MODEM_CTRL0, MODEM_CTRL0_MODFORMAT_OOKASK);
    qtest_writel(global_qtest, MODEM_BASE + MODEM_TXBR, MODEM_TXBR_200BD);
    qtest_writel(global_qtest, MODEM_BASE + MODEM_CTRL1, MODEM_CTRL1_SYNCBITS_32);
    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA, 0xff);
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA, 0x0f);
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(airtime_ns - 1000000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(2000000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==,
                    FRC_IF_TXDONE);
    rf_reset_with_clocks();
}

/*
 * The in-process medium has a deliberately narrow, synthetic receive seam:
 * it writes raw bytes to BUFC channel 1 and raises FRC.RXDONE.  When the
 * guest configures channel 2, QEMU additionally publishes its explicit
 * simulator-only successful-RX record.  Stock RAIL instead publishes
 * RX_PACKET_RECEIVED after a hardware-produced channel-2 descriptor crosses
 * BUF2THR; this test does not claim to reproduce that silicon producer.
 */
static void test_tx_offer_enqueues_synthetic_rx_descriptor(void)
{
    const uint8_t payload[] = { 0x2b, 0x6c, 0xf0 };
    const uint32_t fifo_guest_address = 0x20000480;
    const uint32_t descriptor_guest_address = 0x20000500;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint64_t peer_descriptor_channel = peer(BUFC_BASE + BUFC_CH2);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t descriptor_threshold = 1u << 18;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW),
                 descriptor_threshold);
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IEN), descriptor_threshold);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_ADDR,
                 descriptor_guest_address);
    qtest_writel(global_qtest, peer_descriptor_channel, 3);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_THRESHOLD, 7);

    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    /*
     * This record is deliberately a QEMU-only seam.  Its endpoint is the
     * channel-1 write offset (three bytes), followed by the archive-observed
     * received-packet class 0x06 and zero metadata/reserved bytes.  The
     * channel-2 threshold must be visible by the time RXDONE is observed.
     */
    g_assert_cmphex(qtest_readl(global_qtest, peer_descriptor_channel +
                                BUFC_STATUS),
                    ==, (1u << 20) | 8u);
    g_assert_cmphex(qtest_readl(global_qtest, peer(BUFC_BASE + BUFC_IF)) &
                        descriptor_threshold,
                    ==, descriptor_threshold);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_descriptor_channel + BUFC_READDATA),
                    ==, 0x03);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_descriptor_channel + BUFC_READDATA),
                    ==, 0x00);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_descriptor_channel + BUFC_READDATA),
                    ==, 0x06);
    for (size_t index = 3; index < 8; index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_descriptor_channel + BUFC_READDATA),
                        ==, 0);
    }
    g_assert_cmphex(qtest_readl(global_qtest, peer_descriptor_channel +
                                BUFC_STATUS) & 0x1fff,
                    ==, 0);
}

static void test_tx_offer_drops_descriptor_ring_overflow(void)
{
    const uint8_t payload[] = { 0x2b, 0x6c, 0xf0 };
    const uint32_t fifo_guest_address = 0x20000700;
    const uint32_t descriptor_guest_address = 0x20000500;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint64_t peer_descriptor_channel = peer(BUFC_BASE + BUFC_CH2);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t descriptor_threshold = 1u << 18;
    const uint32_t descriptor_overflow = 1u << 16;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IF +
                                    REGISTER_CLEAR_WINDOW),
                 descriptor_threshold | descriptor_overflow);
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IEN),
                 descriptor_threshold | descriptor_overflow);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_ADDR,
                 descriptor_guest_address);
    qtest_writel(global_qtest, peer_descriptor_channel, 3);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_THRESHOLD, 7);

    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 3); /* 512-byte RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    for (unsigned int record = 0; record < 65; record++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
        for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
            qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                         payload[index]);
        }
        qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

        qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
        clock_step(1);
        qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

        if (record < 64) {
            g_assert_cmphex(qtest_readl(global_qtest,
                                        peer(FRC_BASE + FRC_IF)) &
                                FRC_IF_RXDONE,
                            ==, FRC_IF_RXDONE);
            qtest_writel(global_qtest,
                         peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                         FRC_IF_RXDONE);
        } else {
            /* A full ring must not turn an old descriptor into RXDONE. */
            g_assert_cmphex(qtest_readl(global_qtest,
                                        peer(FRC_BASE + FRC_IF)) &
                                FRC_IF_RXDONE,
                            ==, 0);
            g_assert_cmphex(qtest_readl(global_qtest,
                                        peer_rx_channel + BUFC_WRITEOFFSET),
                            ==, 192);
        }

        qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
        clock_step(150001);
        qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    }

    g_assert_cmphex(qtest_readl(global_qtest, peer_descriptor_channel +
                                BUFC_STATUS) & 0x1fff,
                    ==, 512);
    g_assert_cmphex(qtest_readl(global_qtest, peer(BUFC_BASE + BUFC_IF)) &
                        descriptor_overflow,
                    ==, descriptor_overflow);

    /*
     * Consume the 64 queued records and clear the explicit overflow.  The
     * next accepted offer must use only its own channel-1 interval.
     */
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_READOFFSET, 0);
    for (unsigned int byte = 0; byte < 64 * 8; byte++) {
        (void)qtest_readl(global_qtest,
                          peer_descriptor_channel + BUFC_READDATA);
    }
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IF +
                                    REGISTER_CLEAR_WINDOW),
                 descriptor_threshold | descriptor_overflow);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer(FRC_BASE + FRC_IF)) & FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);

    /* The post-overflow record is at logical offset 512, physical slot zero. */
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_descriptor_channel + BUFC_READDATA),
                    ==, 0xc3);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_descriptor_channel + BUFC_READDATA),
                    ==, 0x00);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_descriptor_channel + BUFC_READDATA),
                    ==, 0x06);
}

/*
 * Consuming every descriptor before the next offer must permit the synthetic
 * ring to wrap without raising overflow or replaying an old record.
 */
static void test_descriptor_ring_reuse_after_each_delivery(void)
{
    const uint8_t payload[] = { 0x41, 0x9c, 0x07 };
    const uint32_t fifo_guest_address = 0x20000a00;
    const uint32_t descriptor_guest_address = 0x20000c00;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint64_t peer_descriptor_channel = peer(BUFC_BASE + BUFC_CH2);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t descriptor_threshold = 1u << 18;

    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW),
                 descriptor_threshold);
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_IEN), descriptor_threshold);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_ADDR,
                 descriptor_guest_address);
    qtest_writel(global_qtest, peer_descriptor_channel, 3);
    qtest_writel(global_qtest, peer_descriptor_channel + BUFC_THRESHOLD, 7);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 3); /* 512-byte RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    for (unsigned int record = 0; record < 65; record++) {
        qtest_writel(global_qtest,
                     peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                     FRC_IF_RXDONE);
        qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
        for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
            qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                         payload[index]);
        }
        qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
        qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
        clock_step(1);
        qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

        g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                            FRC_IF_RXDONE,
                        ==, FRC_IF_RXDONE);
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_descriptor_channel + BUFC_STATUS) &
                            0x1fff,
                        ==, 8);
        qtest_writel(global_qtest, peer_descriptor_channel + BUFC_READOFFSET,
                     (record + 1) * 8);
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_descriptor_channel + BUFC_STATUS) &
                            0x1fff,
                        ==, 0);
    }

    g_assert_cmphex(qtest_readl(global_qtest, peer(BUFC_BASE + BUFC_IF)) &
                        (1u << 16),
                    ==, 0);
}

/*
 * The medium's synthetic TX transaction remains active until its completion
 * timer. A peer that reaches RXSEARCH in that bounded interval must receive
 * the existing TX snapshot, rather than requiring an artificial second TXEN.
 */
static void test_peer_rxsearch_during_active_tx_gets_offer(void)
{
    const uint8_t payload[] = { 0x55, 0xa9, 0x06 };
    const uint32_t fifo_guest_address = 0x200001c0;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* Enter RXSEARCH before the source's synthetic completion callback. */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_STATUS)) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_RXSEARCH);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, payload[index]);
    }
}

/*
 * Once the synthetic TX completion boundary retires a snapshot, a receiver
 * entering RXSEARCH later must not manufacture a receive from that completed
 * transaction.  This is a QEMU mailbox-lifetime contract, not a silicon
 * airtime or completion-source claim.
 */
static void test_completed_tx_is_not_offered_to_late_rxsearch(void)
{
    const uint8_t payload[] = { 0x62, 0x1f };
    const uint32_t fifo_guest_address = 0x20000500;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    /* Leave the peer's state from the preceding in-process test behind. */
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* Retire the model transaction before the peer begins RXSEARCH. */
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_WRITEOFFSET), ==, 0);
}

/*
 * A receiver may leave RXSEARCH after an offer has been queued but before
 * the deferred handoff callback runs.  Completion must retire that queued
 * PDU too; otherwise a later RXSEARCH entry can replay a completed packet.
 */
static void test_completed_tx_withdraws_deferred_offer(void)
{
    const uint8_t payload[] = { 0x38, 0x9e };
    const uint32_t fifo_guest_address = 0x20000580;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* Cancel only the receiver's deferred callback, leaving its offer queued. */
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_TXDONE,
                    ==, FRC_IF_TXDONE);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);

    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_WRITEOFFSET), ==, 0);
}

/*
 * RX offers are deferred out of RAC.CMD.TXEN.  If the receiver leaves
 * RXSEARCH before that virtual-clock callback runs, it must not observe a
 * spurious RXDONE or partially written BUFC channel.  The offer is retained
 * and is delivered only after the normal SWRXEN path enters RXSEARCH again.
 */
static void test_tx_offer_waits_for_receiver_rxsearch(void)
{
    const uint8_t payload[] = { 0x4d, 0xb2 };
    const uint32_t fifo_guest_address = 0x20000200;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* Leave RXSEARCH before the deferred handoff gets a virtual-clock turn. */
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA),
                    ==, 0);

    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, payload[index]);
    }
}

/*
 * Two chips on a bench reset independently: a WDOG0 timeout in the second
 * SoC (CFG 0: PERSEL 0, 2^3 + 1 LF cycles, RM SS27.3.2) resets that SoC
 * alone, RSTCAUSE.WDOG0 (RM SS12.5.15); the first keeps its clocks and its
 * cleared RSTCAUSE.  (The machine reset, QMP system_reset, still resets both.)
 */
#define EMU_BASE                 0x40004000
#define EMU_CMD                  0x070
#define EMU_CMD_RSTCAUSECLR      (1u << 17)
#define EMU_RSTCAUSE             0x094
#define EMU_RSTCAUSE_WDOG0       (1u << 3)
#define WDOG0_BASE               0x4a018000
#define WDOG_EN                  0x04
#define WDOG_CFG                 0x08
#define WDOG_TIMEOUT_NS          (UINT64_C(9) * 1000000000 / 32768 + 1)

static void test_wdog_resets_one_soc(void)
{
    rf_reset_with_clocks();
    qtest_writel(global_qtest, EMU_BASE + EMU_CMD, EMU_CMD_RSTCAUSECLR);
    qtest_writel(global_qtest, peer(EMU_BASE + EMU_CMD), EMU_CMD_RSTCAUSECLR);
    qtest_writel(global_qtest, peer(WDOG0_BASE + WDOG_CFG), 0);
    qtest_writel(global_qtest, peer(WDOG0_BASE + WDOG_EN), 1);
    /* Virtual-clock timers run only while the machine runs. */
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2 * WDOG_TIMEOUT_NS);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(EMU_BASE + EMU_RSTCAUSE)),
                    ==, EMU_RSTCAUSE_WDOG0);
    g_assert_cmphex(qtest_readl(global_qtest, peer(WDOG0_BASE + WDOG_EN)), ==,
                    0);
    g_assert_cmphex(qtest_readl(global_qtest, peer(CMU_BASE + CMU_CLKEN1)),
                    ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest, EMU_BASE + EMU_RSTCAUSE), ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest, CMU_BASE + CMU_CLKEN1) &
                    CMU_CLKEN1_RADIO_BLOCKS, ==, CMU_CLKEN1_RADIO_BLOCKS);
    rf_reset_with_clocks();
}

/*
 * A board reset must retire both the deferred handoff timer and its medium
 * mailbox.  Re-entering RXSEARCH after reset must not replay the pre-reset
 * offer as a synthetic receive.
 */
static void test_system_reset_with_deferred_offer(void)
{
    const uint8_t payload[] = { 0x7a, 0x31 };
    const uint32_t fifo_guest_address = 0x20000800;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* Reset before the deferred receiver handoff gets a virtual-clock turn. */
    rf_reset_with_clocks();

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_WRITEOFFSET), ==, 0);
}

/*
 * TXEN snapshots staged BUFC data, but a subsequent TXDIS cancels that model
 * transaction.  In particular, an offer already deferred for a peer must not
 * leak through when the peer later re-enters RXSEARCH.
 */
static void test_tx_disable_withdraws_deferred_offer(void)
{
    const uint8_t payload[] = { 0x6e, 0x51 };
    const uint32_t fifo_guest_address = 0x20000280;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0);
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA), ==, 0);
}

/*
 * This model treats either RAC.CMD disable bit as ending the current RAC
 * operation.  RXDIS issued while its synthetic transmitter is active must
 * therefore retire that transmission's deferred peer offer too.  Otherwise
 * the source is OFF while the peer can still receive a stale RXDONE.
 */
static void test_rx_disable_withdraws_active_tx_offer(void)
{
    const uint8_t payload[] = { 0x87, 0x14 };
    const uint32_t fifo_guest_address = 0x200002c0;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_RXDIS);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA), ==, 0);
}

/*
 * RAC.CTRL.FORCEDISABLE also aborts an in-flight TX operation.  Like TXDIS,
 * it must withdraw the deferred mailbox offer before a peer can observe it.
 */
static void test_force_disable_withdraws_deferred_offer(void)
{
    const uint8_t payload[] = { 0x79, 0x2c };
    const uint32_t fifo_guest_address = 0x20000300;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0);
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest, RAC_BASE + RAC_CTRL, RAC_CTRL_FORCEDISABLE);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA), ==, 0);
}

/*
 * SWRXEN supersedes this model's TX operation just as TXDIS does.  A TXEN
 * offer already queued for a peer must therefore be withdrawn rather than
 * appearing as a receive after the source has entered RXSEARCH.
 */
static void test_rx_enable_withdraws_deferred_offer(void)
{
    const uint8_t payload[] = { 0x3a, 0xc7 };
    const uint32_t fifo_guest_address = 0x20000380;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0);
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest,
                 RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA), ==, 0);
}

/*
 * The native model defines TXEN as replacing its one active synthetic TX
 * transaction.  A second TXEN must therefore discard an undelivered offer
 * from the first snapshot rather than write both packets into the peer FIFO.
 */
static void test_tx_enable_replaces_deferred_offer(void)
{
    const uint8_t first_payload[] = { 0x19, 0x82 };
    const uint8_t replacement_payload[] = { 0xa4, 0x3d };
    const uint32_t fifo_guest_address = 0x20000400;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(first_payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     first_payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(replacement_payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     replacement_payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(replacement_payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, replacement_payload[index]);
    }
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA), ==, 0);
}

/*
 * Exercise the lower RX-to-TX boundary using only the native QEMU model:
 * receive one staged packet, disable RX, stage a response in BUFC channel 0,
 * and issue RAC.CMD.TXEN.  The response must be a fresh medium snapshot,
 * complete once through the QEMU virtual timer, and not be replayed while
 * the peer remains in RXSEARCH.  This is a simulator contract; it does not
 * claim a silicon PHY timing or TX completion-source implementation.
 */
static void test_bidirectional_rac_txen_after_rx(void)
{
    const uint8_t request[] = { 0x05, 0x0a, 0x11, 0x22, 0x33 };
    const uint8_t response[] = { 0x02, 0x03, 0xa5, 0x5a };
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;
    const uint64_t source_rx_channel = BUFC_BASE + BUFC_CH1;
    const uint64_t peer_tx_channel = peer(BUFC_BASE + BUFC_CH0);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    /* Keep this bilateral transaction independent from preceding qtests. */
    rf_reset_with_clocks();

    qtest_writel(global_qtest, FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW,
                 FRC_IF_TXDONE | FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_TXDONE | FRC_IF_RXDONE);

    /* The peer starts as the receiver and must be in RXSEARCH first. */
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_STATUS)) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_RXSEARCH);

    /* Source A performs the first staged transmission. */
    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(request); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     request[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(request); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, request[index]);
    }
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);

    /* The first TX completion is separate from the RX handoff. */
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK,
                    ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_TXDONE,
                    ==, FRC_IF_TXDONE);
    qtest_writel(global_qtest, FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW,
                 FRC_IF_TXDONE);

    /* Model the scheduler-owned RX-to-TX turn-around at the peer. */
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_writel(global_qtest, source_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest,
                 RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_RXSEARCH);

    /* Source B stages exactly one response and starts a fresh TX snapshot. */
    qtest_writel(global_qtest, peer_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(response); index++) {
        qtest_writel(global_qtest, peer_tx_channel + BUFC_WRITEDATA,
                     response[index]);
    }
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_TXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, source_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(response); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    source_rx_channel + BUFC_READDATA),
                        ==, response[index]);
    }

    /* A second virtual-clock turn cannot replay the same active snapshot. */
    qtest_writel(global_qtest, FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW,
                 FRC_IF_RXDONE);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_RXDONE,
                    ==, 0);

    /* The response's own completion must turn B OFF and raise TXDONE once. */
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_STATUS)) &
                        RAC_STATUS_STATE_MASK,
                    ==, 0);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_TXDONE,
                    ==, FRC_IF_TXDONE);

    /* Endpoint IDs distinguish the two same-address SoCs in this exchange. */
    gchar *trace;

    /* The simple trace backend buffers records until explicitly flushed. */
    g_free(qtest_hmp(global_qtest, "trace-file flush"));
    g_assert(g_file_get_contents(rf_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1, "endpoint=0 TX snapshot"));
    g_assert_nonnull(g_strstr_len(trace, -1, "endpoint=1 TX snapshot"));
    g_assert_true(g_regex_match_simple("endpoint=0 now=[0-9]+ RAC\\.CMD=",
                                       trace, 0, 0));
    g_assert_true(g_regex_match_simple("endpoint=1 now=[0-9]+ RAC\\.CMD=",
                                       trace, 0, 0));
    g_free(trace);
}

static void rf_step(int64_t ns)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(ns);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
}

static uint32_t rf_status(uint64_t rac)
{
    return qtest_readl(global_qtest, rac + RAC_STATUS);
}

static uint32_t rf_state(uint64_t rac)
{
    return (rf_status(rac) & RAC_STATUS_STATE_MASK) >> RAC_STATUS_STATE_SHIFT;
}

/* Wait out the ~1.09 us between clearing a state event and the transition. */
static void rf_ack(uint64_t rac, uint32_t state)
{
    qtest_writel(global_qtest, rac + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATE(state));
    rf_step(ACK_TO_TRANSITION_NS);
}

/*
 * Release one SoC's sequencer core and start its synthesizer, as the open
 * M0+ image does on TXWARM (SS7.61).  Under the qtest accelerator the core
 * never executes; qtest performs its acknowledgements.
 */
static void rf_release_sequencer(uint64_t rac, uint64_t synth)
{
    qtest_writel(global_qtest, rac + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_writel(global_qtest, rac + RAC_SYEN, RAC_SYEN_STOCK_TX);
    qtest_writel(global_qtest, synth + SYNTH_CMD, SYNTH_CMD_SYNTHSTART);
}

static void rf_stage_tx(uint64_t tx_channel, const uint8_t *bytes,
                        size_t length)
{
    qtest_writel(global_qtest, tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < length; index++) {
        qtest_writel(global_qtest, tx_channel + BUFC_WRITEDATA, bytes[index]);
    }
}

/*
 * From OFF to RXSEARCH through RXWARM, as on silicon (SS7.72), with the
 * receive chain up as the open sequencer's RXWARM leaves it (SS7.80): the
 * synthesizer started in IF mode, the LNA/mixer on the RF path, the IF ADC
 * clocked.  A receiver searching without them detects nothing.
 */
static void rf_start_rx(uint64_t rac)
{
    const uint64_t synth = rac - RAC_BASE + SYNTH_BASE;

    qtest_writel(global_qtest, rac + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXWARM);
    qtest_writel(global_qtest, rac + RAC_SYEN, RAC_SYEN_STOCK_TX);
    qtest_writel(global_qtest, synth + SYNTH_CMD,
                 SYNTH_CMD_SYNTHSTOP | SYNTH_CMD_ENABLEIF);
    qtest_writel(global_qtest, synth + SYNTH_CMD, SYNTH_CMD_SYNTHSTART);
    qtest_writel(global_qtest, rac + RAC_RFPATHEN + REGISTER_SET_WINDOW,
                 RAC_RFPATHEN_LNAMIXEN);
    qtest_writel(global_qtest, rac + RAC_IFADCDBG + REGISTER_SET_WINDOW,
                 RAC_IFADCDBG_IFADCENCLK);
    rf_ack(rac, RAC_STATE_RXWARM);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXSEARCH);
}

/*
 * Receive-side state machine, as measured from the debugger with the M0+
 * parked (SS7.72): SWRXEN takes OFF to RXWARM and each state waits for its
 * own event to be cleared, whether or not SEQIEN enables it; RXWARM goes on
 * to RXSEARCH even if the source was withdrawn; withdrawing SWRXEN alone ends
 * RXSEARCH in RXPD; RXDIS does too, even with SWRXEN still asserted, after
 * which OFF re-enters RXWARM; RXDIS in RXWARM or RXPD does nothing.
 */
static void test_released_rx_state_machine(void)
{
    const uint64_t rac = RAC_BASE;

    rf_reset_with_clocks();
    rf_release_sequencer(rac, SYNTH_BASE);

    qtest_writel(global_qtest, rac + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXWARM);
    /* Stock's 0x91400002 less SEQSLEEPING: RXENS, SEQACTIVE, RXMASK. */
    g_assert_cmphex(rf_status(rac) & STATUS_MEASURED_BITS, ==,
                    RAC_STATUS_RXENS | RAC_STATUS_SEQACTIVE |
                    RAC_STATUS_RXMASK_SWRXEN);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_RXDIS);
    qtest_writel(global_qtest, rac + RAC_RXENSRCEN + REGISTER_CLEAR_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    rf_step(MODEL_LONG_WAIT_NS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXWARM);
    rf_ack(rac, RAC_STATE_RXWARM);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXSEARCH);
    /* 0x32400000 less SEQSLEEPING: DEMODENS, SEQACTIVE. */
    g_assert_cmphex(rf_status(rac) & STATUS_MEASURED_BITS, ==,
                    RAC_STATUS_DEMODENS | RAC_STATUS_SEQACTIVE);
    rf_ack(rac, RAC_STATE_RXSEARCH);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXPD);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_RXDIS);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXPD);
    rf_ack(rac, RAC_STATE_RXPD);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_OFF);
    g_assert_cmphex(rf_status(rac) & RAC_STATUS_SEQACTIVE, ==,
                    RAC_STATUS_SEQACTIVE);
    rf_ack(rac, RAC_STATE_OFF);
    g_assert_cmphex(rf_status(rac) & RAC_STATUS_SEQACTIVE, ==, 0);

    /* RXDIS with the source still asserted: RXPD, then back into RX. */
    rf_start_rx(rac);
    rf_ack(rac, RAC_STATE_RXSEARCH);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_RXDIS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXPD);
    rf_ack(rac, RAC_STATE_RXPD);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXWARM);

    /* Holding needs no SEQIEN enable. */
    qtest_writel(global_qtest, rac + RAC_SEQIEN, 0);
    rf_step(MODEL_LONG_WAIT_NS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXWARM);

    rf_reset_with_clocks();
}

/*
 * TX-side commands (SS7.72): TXEN latches TXENS (0x58400000 in TXWARM);
 * TXDIS in TXWARM only withdraws it, so the acknowledgement leads to TXPD;
 * TXDIS in TX ends it at once even with STATETX uncleared, into TX2RX when
 * SWRXEN is asserted; FORCEDISABLE goes to SHUTDOWN at once, whose
 * acknowledgement reaches OFF, where the radio stays until it is released.
 */
static void test_released_tx_disables(void)
{
    const uint64_t rac = RAC_BASE;

    rf_reset_with_clocks();
    rf_release_sequencer(rac, SYNTH_BASE);

    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TXWARM);
    g_assert_cmphex(rf_status(rac) & STATUS_MEASURED_BITS, ==,
                    RAC_STATUS_TXENS | RAC_STATUS_SEQACTIVE);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TXWARM);
    g_assert_cmphex(rf_status(rac) & RAC_STATUS_TXENS, ==, 0);
    rf_ack(rac, RAC_STATE_TXWARM);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TXPD);
    rf_ack(rac, RAC_STATE_TXPD);
    rf_ack(rac, RAC_STATE_OFF);

    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXEN);
    rf_ack(rac, RAC_STATE_TXWARM);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TX);
    g_assert_cmphex(rf_status(rac) & RAC_STATUS_TXENS, ==, 0);
    /* RXDIS in TX does nothing. */
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_RXDIS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TX);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TXPD);
    rf_ack(rac, RAC_STATE_TX);
    rf_ack(rac, RAC_STATE_TXPD);
    rf_ack(rac, RAC_STATE_OFF);

    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest, rac + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    rf_ack(rac, RAC_STATE_TXWARM);
    rf_ack(rac, RAC_STATE_TX);
    qtest_writel(global_qtest, rac + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_TX2RX);
    rf_ack(rac, RAC_STATE_TX2RX);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXSEARCH);
    rf_ack(rac, RAC_STATE_RXSEARCH);

    qtest_writel(global_qtest, rac + RAC_CTRL + REGISTER_SET_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_SHUTDOWN);
    rf_ack(rac, RAC_STATE_SHUTDOWN);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_OFF);
    rf_ack(rac, RAC_STATE_OFF);
    rf_step(MODEL_LONG_WAIT_NS);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_OFF);
    qtest_writel(global_qtest, rac + RAC_CTRL + REGISTER_CLEAR_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
    g_assert_cmpuint(rf_state(rac), ==, RAC_STATE_RXWARM);

    rf_reset_with_clocks();
}

/*
 * A released sequencer's TX that ends with an RX source asserted turns around
 * through TX2RX into RXSEARCH (SS7.63).  The frame lasts the measured
 * TX-entry-to-exit time for its length (SS7.71/SS7.72).
 */
static void test_released_tx_turns_around_to_rx(void)
{
    const uint8_t frame[] = { 0x40, 0x01, 0x5a };

    rf_reset_with_clocks();
    rf_release_sequencer(RAC_BASE, SYNTH_BASE);
    qtest_writel(global_qtest, FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW,
                 FRC_IF_TXDONE);
    rf_stage_tx(BUFC_BASE + BUFC_CH0, frame, sizeof(frame));

    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest, RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmpuint(rf_state(RAC_BASE), ==, RAC_STATE_TXWARM);
    rf_ack(RAC_BASE, RAC_STATE_TXWARM);
    g_assert_cmpuint(rf_state(RAC_BASE), ==, RAC_STATE_TX);
    g_assert_cmphex(rf_status(RAC_BASE) &
                        (RAC_STATUS_RXENS | RAC_STATUS_RXMASK_SWRXEN |
                         RAC_STATUS_TXENS | RAC_STATUS_SEQACTIVE),
                    ==, RAC_STATUS_RXENS | RAC_STATUS_RXMASK_SWRXEN |
                        RAC_STATUS_SEQACTIVE);
    rf_ack(RAC_BASE, RAC_STATE_TX);

    rf_step(MODEL_TX_NS(sizeof(frame)) - ACK_TO_TRANSITION_NS - 1);
    g_assert_cmpuint(rf_state(RAC_BASE), ==, RAC_STATE_TX);
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_TXDONE, ==, 0);
    rf_step(1);
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_TXDONE, ==, FRC_IF_TXDONE);
    g_assert_cmpuint(rf_state(RAC_BASE), ==, RAC_STATE_TX2RX);
    rf_step(MODEL_LONG_WAIT_NS);
    g_assert_cmpuint(rf_state(RAC_BASE), ==, RAC_STATE_TX2RX);
    rf_ack(RAC_BASE, RAC_STATE_TX2RX);
    g_assert_cmpuint(rf_state(RAC_BASE), ==, RAC_STATE_RXSEARCH);

    rf_reset_with_clocks();
}

/*
 * The receive half of a scan exchange on a released sequencer (SS7.65-7.72):
 * the frame is detected 3.5 us after its access address arrives ([I]) and the
 * receiver enters RXFRAME; the sequencer withdraws SWRXEN and issues TXEN
 * (TXENS without RXENS, stock's 0x63400000); EOF comes the measured 9.45 us
 * after the transmitter's last bit, with header, length, payload and the
 * TRAILRXDATA.CRCOK trailer in BUFC channel 1 and RXDONE raised; 3.3 us
 * later the RAC is in RX2TX, held until STATERX2TX is cleared.  RAC.STIMER
 * counts ~4.8 ticks/us from the RX2TX entry; TX follows the clear by the
 * ack latency, without TXWARM.
 */
/*
 * SYNTH.LOCNTSTATUS.BUSY after LOCNTCTRL.RUN lasts 2^(NUMCYCLE + 1) HFXO
 * periods (fitted to the silicon calibration stamps, SS7.80): 64 periods,
 * 1.667 us, at NUMCYCLE 5 and 8 periods, 0.208 us, at NUMCYCLE 2.
 */
#define SYNTH_LOCNTCTRL          0x088
#define SYNTH_LOCNTCTRL_RUN      (1u << 2)
#define SYNTH_LOCNTSTATUS        0x08c
#define SYNTH_LOCNTSTATUS_BUSY   (1u << 19)

static void rf_locnt_busy_window(uint32_t numcycle, int64_t busy_ns)
{
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_LOCNTCTRL,
                 (numcycle << 4) | SYNTH_LOCNTCTRL_RUN);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_LOCNTSTATUS) &
                    SYNTH_LOCNTSTATUS_BUSY, ==, SYNTH_LOCNTSTATUS_BUSY);
    rf_step(busy_ns - 10);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_LOCNTSTATUS) &
                    SYNTH_LOCNTSTATUS_BUSY, ==, SYNTH_LOCNTSTATUS_BUSY);
    rf_step(20);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_LOCNTSTATUS) &
                    SYNTH_LOCNTSTATUS_BUSY, ==, 0);
}

static void test_locnt_busy_window(void)
{
    rf_reset_with_clocks();
    rf_locnt_busy_window(5, 1667);
    rf_locnt_busy_window(2, 208);
}

/*
 * On silicon a receiver opened from OFF without the RXWARM bring-up reached
 * RXSEARCH with the synthesizer stopped and the receive chain off, and never
 * detected a frame (SS7.80): RXSEARCH alone does not receive.
 */
static void test_released_rx_chain_off_detects_nothing(void)
{
    const uint8_t request[] = { 0xc3, 0x02, 0x11, 0x22 };
    const uint64_t peer_rac = peer(RAC_BASE);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    rf_reset_with_clocks();
    qtest_writel(global_qtest, peer_rac + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE | FRC_IF_TXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest, peer_rac + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    rf_ack(peer_rac, RAC_STATE_RXWARM);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXSEARCH);

    rf_stage_tx(BUFC_BASE + BUFC_CH0, request, sizeof(request));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    rf_step(MODEL_RX_EOF_NS(sizeof(request)) + MODEL_RX_DETECT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXSEARCH);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                    FRC_IF_RXDONE, ==, 0);
}

/*
 * A received frame that does not fit the RX FIFO (SS7.80, silicon): BUFC.IF
 * channel 1 overflow, no RXDONE, the RAC held in RXOVERFLOW with no state
 * event latched until FORCEDISABLE, which takes it to SHUTDOWN.
 */
#define BUFC_IF_OFFSET           0x114
#define BUFC_IF_BUF1OF           (1u << 8)
#define RAC_STATE_RXOVERFLOW     6

static void test_released_rx_fifo_overflow(void)
{
    const uint8_t request[] = { 0xc3, 0x02, 0x11, 0x22 };
    const uint64_t peer_rac = peer(RAC_BASE);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    rf_reset_with_clocks();
    rf_release_sequencer(peer_rac, peer(SYNTH_BASE));
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE | FRC_IF_TXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    /* 62 of the default 64 octets already held: the 4-octet frame cannot fit. */
    for (int i = 0; i < 62; i++) {
        qtest_writel(global_qtest, peer_rx_channel + BUFC_WRITEDATA, 0);
    }
    rf_start_rx(peer_rac);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);

    rf_stage_tx(BUFC_BASE + BUFC_CH0, request, sizeof(request));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    rf_step(MODEL_RX_DETECT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    qtest_writel(global_qtest, peer_rac + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 UINT32_MAX);
    rf_step(MODEL_RX_EOF_NS(sizeof(request)) - MODEL_RX_DETECT_NS + 1);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXOVERFLOW);
    g_assert_cmphex(qtest_readl(global_qtest, peer(BUFC_BASE + BUFC_IF_OFFSET)) &
                    BUFC_IF_BUF1OF, ==, BUFC_IF_BUF1OF);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                    FRC_IF_RXDONE, ==, 0);
    g_assert_cmphex(rf_status(peer_rac) & RAC_STATUS_SEQACTIVE, ==, 0);

    /* RXDIS and withdrawing SWRXEN leave it there. */
    qtest_writel(global_qtest, peer_rac + RAC_CMD, RAC_CMD_RXDIS);
    qtest_writel(global_qtest, peer_rac + RAC_RXENSRCEN + REGISTER_CLEAR_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    rf_step(ACK_TO_TRANSITION_NS * 4);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXOVERFLOW);

    qtest_writel(global_qtest, peer_rac + RAC_CTRL + REGISTER_SET_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_SHUTDOWN);
}

/*
 * Disabling a receiver in the middle of a frame (silicon, SS7.80): RXDIS --
 * with SWRXEN withdrawn, as the scanner's abort does -- does not end RXFRAME;
 * the RAC finishes the frame (EOF, RXDONE) and only then leaves for RXPD.
 * FORCEDISABLE ends RXFRAME at once: SHUTDOWN, and the abandoned frame never
 * reaches BUFC or raises RXDONE.
 */
static void rf_peer_in_rxframe(const uint8_t *frame, size_t length)
{
    const uint64_t peer_rac = peer(RAC_BASE);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    rf_reset_with_clocks();
    rf_release_sequencer(peer_rac, peer(SYNTH_BASE));
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE | FRC_IF_TXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    rf_start_rx(peer_rac);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);
    rf_stage_tx(BUFC_BASE + BUFC_CH0, frame, length);
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    rf_step(MODEL_RX_DETECT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    qtest_writel(global_qtest, peer_rac + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 UINT32_MAX);
}

static void test_released_rx_frame_disables(void)
{
    const uint8_t request[] = { 0xc3, 0x02, 0x11, 0x22 };
    const uint64_t peer_rac = peer(RAC_BASE);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    /* RXDIS: the frame is finished. */
    rf_peer_in_rxframe(request, sizeof(request));
    qtest_writel(global_qtest, peer_rac + RAC_RXENSRCEN + REGISTER_CLEAR_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    qtest_writel(global_qtest, peer_rac + RAC_CMD, RAC_CMD_RXDIS);
    rf_step(MODEL_RX_EOF_NS(sizeof(request)) - MODEL_RX_DETECT_NS - 1);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                    FRC_IF_RXDONE, ==, 0);
    rf_step(2);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                    FRC_IF_RXDONE, ==, FRC_IF_RXDONE);
    g_assert_cmpuint(qtest_readl(global_qtest,
                                 peer_rx_channel + BUFC_WRITEOFFSET), !=, 0);
    rf_step(EOF_TO_RX_EXIT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXPD);

    /* FORCEDISABLE: RXFRAME ends at once and the frame is abandoned. */
    rf_peer_in_rxframe(request, sizeof(request));
    qtest_writel(global_qtest, peer_rac + RAC_CTRL + REGISTER_SET_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_SHUTDOWN);
    rf_step(MODEL_RX_EOF_NS(sizeof(request)) + MODEL_LONG_WAIT_NS);
    g_assert_cmpuint(rf_state(peer_rac), !=, RAC_STATE_RXFRAME);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                    FRC_IF_RXDONE, ==, 0);
    g_assert_cmpuint(qtest_readl(global_qtest,
                                 peer_rx_channel + BUFC_WRITEOFFSET), ==, 0);
    qtest_writel(global_qtest, peer_rac + RAC_CTRL + REGISTER_CLEAR_WINDOW,
                 RAC_CTRL_FORCEDISABLE);
}

static void test_released_rx_frame_turns_around_to_tx(void)
{
    const uint8_t request[] = { 0xc3, 0x02, 0x11, 0x22 };
    const uint8_t response[] = { 0x44, 0x01, 0x99 };
    const uint64_t peer_rac = peer(RAC_BASE);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint64_t source_rx_channel = BUFC_BASE + BUFC_CH1;

    rf_reset_with_clocks();
    rf_release_sequencer(peer_rac, peer(SYNTH_BASE));
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE | FRC_IF_TXDONE);
    qtest_writel(global_qtest, FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW,
                 FRC_IF_RXDONE | FRC_IF_TXDONE);
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_TRAILRXDATA),
                 FRC_TRAILRXDATA_CRCOK);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    rf_start_rx(peer_rac);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);

    /* The legacy source transmits the request at once. */
    rf_stage_tx(BUFC_BASE + BUFC_CH0, request, sizeof(request));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    rf_step(MODEL_RX_DETECT_NS - 1);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXSEARCH);
    rf_step(1);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);

    /* The sequencer's RXFRAME handler requests the response. */
    qtest_writel(global_qtest, peer_rac + RAC_RXENSRCEN + REGISTER_CLEAR_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    qtest_writel(global_qtest, peer_rac + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    g_assert_cmphex(rf_status(peer_rac) & STATUS_MEASURED_BITS, ==,
                    RAC_STATUS_TXENS | RAC_STATUS_DEMODENS |
                    RAC_STATUS_SEQACTIVE);
    rf_ack(peer_rac, RAC_STATE_RXFRAME);
    g_assert_cmphex(rf_status(peer_rac) & STATUS_MEASURED_BITS, ==,
                    RAC_STATUS_TXENS | RAC_STATUS_DEMODENS);
    rf_stage_tx(peer(BUFC_BASE + BUFC_CH0), response, sizeof(response));
    qtest_writel(global_qtest, source_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, source_rx_channel, 0);

    rf_step(MODEL_RX_EOF_NS(sizeof(request)) - MODEL_RX_DETECT_NS -
            ACK_TO_TRANSITION_NS - 1);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE, ==, 0);
    rf_step(1);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE, ==, FRC_IF_RXDONE);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    g_assert_cmphex(qtest_readl(global_qtest, peer_rx_channel + BUFC_STATUS) &
                        0x1fff, ==, sizeof(request) + 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < sizeof(request); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, request[index]);
    }
    g_assert_cmphex(qtest_readl(global_qtest, peer_rx_channel + BUFC_READDATA),
                    ==, FRC_TRAILER_CRCOK);
    rf_step(EOF_TO_RX_EXIT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RX2TX);

    /* Held in RX2TX, counting STIMER from its entry. */
    rf_step(STIMER_TEST_INTERVAL_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RX2TX);
    g_assert_cmpuint(qtest_readl(global_qtest, peer_rac + RAC_STIMER), ==,
                     STIMER_TICKS_IN_INTERVAL);
    qtest_writel(global_qtest, RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    qtest_writel(global_qtest, peer_rac + RAC_SEQIF + REGISTER_CLEAR_WINDOW,
                 RAC_SEQIF_STATE(RAC_STATE_RX2TX));
    rf_step(ACK_TO_TRANSITION_NS - 1);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RX2TX);
    rf_step(1);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_TX);
    g_assert_cmphex(rf_status(peer_rac) & RAC_STATUS_TXENS, ==, 0);
    g_assert_cmpuint(qtest_readl(global_qtest, peer_rac + RAC_STIMER), ==, 0);

    /* The response reaches the legacy source through its instant path. */
    rf_step(1);
    g_assert_cmphex(qtest_readl(global_qtest, FRC_BASE + FRC_IF) &
                        FRC_IF_RXDONE, ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, source_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < sizeof(response); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    source_rx_channel + BUFC_READDATA),
                        ==, response[index]);
    }

    rf_ack(peer_rac, RAC_STATE_TX);
    rf_step(MODEL_TX_NS(sizeof(response)));
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_TXDONE, ==, FRC_IF_TXDONE);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_TXPD);
    rf_ack(peer_rac, RAC_STATE_TXPD);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_OFF);

    rf_reset_with_clocks();
}

/*
 * The demodulator rate follows the channel (owo-open-fw SS7.83, silicon): on
 * 2426 MHz the sample-rate converter must carry that channel's ratios
 * (stock's MODEM.SRCCHF there, 0x8C2F2891); the 2402 MHz PHY-table value
 * (0x8C2BE890) is 1 % off and the frame fails its CRC.  1M PHY settings:
 * SYNTH.IFFREQ 9557 high side, IF ADC in half mode.
 */
#define SYNTH_IFFREQ             0x038
#define SYNTH_FREQ_2426MHZ       0x01f96aabu
#define SYNTH_IFFREQ_BLE_1M      0x00102555u
#define RAC_IFADCTRIM0           0x0e4
#define RAC_IFADCTRIM0_BLE_1M    0x13512c6eu
#define MODEM_SRCCHF             (MODEM_BASE + 0x0ec)
#define MODEM_SRCCHF_2426MHZ     0x8c2f2891u
#define MODEM_SRCCHF_2402MHZ     0x8c2be890u

static uint32_t rf_receive_with_srcchf(uint32_t srcchf)
{
    const uint8_t frame[] = { 0x00, 0x0e, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                              13, 14 };
    const uint64_t peer_rac = peer(RAC_BASE);
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);

    rf_reset_with_clocks();
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, SYNTH_FREQ_2426MHZ);
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_FREQ), SYNTH_FREQ_2426MHZ);
    qtest_writel(global_qtest, peer(SYNTH_BASE + SYNTH_IFFREQ),
                 SYNTH_IFFREQ_BLE_1M);
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_IFADCTRIM0),
                 RAC_IFADCTRIM0_BLE_1M);
    qtest_writel(global_qtest, peer(MODEM_SRCCHF), srcchf);
    rf_release_sequencer(peer_rac, peer(SYNTH_BASE));
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE | FRC_IF_TXDONE);
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_TRAILRXDATA),
                 FRC_TRAILRXDATA_CRCOK);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    rf_start_rx(peer_rac);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);

    rf_stage_tx(BUFC_BASE + BUFC_CH0, frame, sizeof(frame));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    rf_step(MODEL_RX_DETECT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    rf_step(MODEL_RX_EOF_NS(sizeof(frame)) - MODEL_RX_DETECT_NS);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                    FRC_IF_RXDONE, ==, FRC_IF_RXDONE);
    g_assert_cmphex(qtest_readl(global_qtest, peer_rx_channel + BUFC_STATUS) &
                    0x1fff, ==, sizeof(frame) + 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < sizeof(frame); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, frame[index]);
    }
    return qtest_readl(global_qtest, peer_rx_channel + BUFC_READDATA);
}

static void test_released_rx_srcchf_follows_channel(void)
{
    g_assert_cmphex(rf_receive_with_srcchf(MODEM_SRCCHF_2426MHZ), ==,
                    FRC_TRAILER_CRCOK);
    g_assert_cmphex(rf_receive_with_srcchf(MODEM_SRCCHF_2402MHZ) &
                    FRC_TRAILER_CRCOK, ==, 0);
    rf_reset_with_clocks();
}

/* Deliver one legacy-source frame to a released receiver in RXSEARCH. */
static void rf_offer_frame_to_released_peer(void)
{
    const uint8_t frame[] = { 0x02, 0x00 };
    const uint64_t peer_rac = peer(RAC_BASE);

    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_CH1 + BUFC_CMD), 1);
    qtest_writel(global_qtest, peer(BUFC_BASE + BUFC_CH1), 0);
    rf_start_rx(peer_rac);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);
    rf_stage_tx(BUFC_BASE + BUFC_CH0, frame, sizeof(frame));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    rf_step(MODEL_RX_DETECT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
}

/*
 * Frame-end alternatives on a released receiver.  A TXEN latched while SWRXEN
 * is still asserted does not turn the RAC around -- it returns to receive
 * through RX2RX (SS7.65, SS7.69); with no request at all it powers down.  The
 * frame-driven exit waits for RXFRAME's own event, though RXDONE does not.
 */
static void test_released_rx_frame_end_alternatives(void)
{
    const uint64_t peer_rac = peer(RAC_BASE);

    rf_reset_with_clocks();
    rf_release_sequencer(peer_rac, peer(SYNTH_BASE));

    rf_offer_frame_to_released_peer();
    qtest_writel(global_qtest, peer_rac + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(rf_status(peer_rac) & RAC_STATUS_TXENS, ==,
                    RAC_STATUS_TXENS);
    rf_ack(peer_rac, RAC_STATE_RXFRAME);
    rf_step(MODEL_LONG_WAIT_NS);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RX2RX);
    g_assert_cmphex(rf_status(peer_rac) & RAC_STATUS_TXENS, ==, 0);
    rf_ack(peer_rac, RAC_STATE_RX2RX);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXSEARCH);
    rf_ack(peer_rac, RAC_STATE_RXSEARCH);
    qtest_writel(global_qtest, peer_rac + RAC_RXENSRCEN + REGISTER_CLEAR_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXPD);
    rf_ack(peer_rac, RAC_STATE_RXPD);
    rf_ack(peer_rac, RAC_STATE_OFF);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_OFF);
    rf_step(MODEL_LONG_WAIT_NS);

    /* No request, and STATERXFRAME left uncleared: the exit is held. */
    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    rf_offer_frame_to_released_peer();
    qtest_writel(global_qtest, peer_rac + RAC_RXENSRCEN + REGISTER_CLEAR_WINDOW,
                 RAC_RXENSRCEN_SWRXEN);
    rf_step(MODEL_LONG_WAIT_NS);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE, ==, FRC_IF_RXDONE);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXFRAME);
    rf_ack(peer_rac, RAC_STATE_RXFRAME);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_RXPD);
    rf_ack(peer_rac, RAC_STATE_RXPD);
    g_assert_cmpuint(rf_state(peer_rac), ==, RAC_STATE_OFF);

    rf_reset_with_clocks();
}

/*
 * Bus clock gating (SS7.72): each CLKEN1 bit gates exactly one radio block
 * and CLKEN1.BUFC gates nothing visible; a gated write is dropped.
 */
static void test_radio_block_clock_gates(void)
{
    rf_reset_with_clocks();
    qtest_writel(global_qtest, RAC_BASE + RAC_SR0, 0x11111111);
    qtest_writel(global_qtest, CMU_BASE + CMU_CLKEN1 + REGISTER_CLEAR_WINDOW,
                 CMU_CLKEN1_RAC);
    qtest_writel(global_qtest, RAC_BASE + RAC_SR0, 0x22222222);
    /* BUFC stays accessible with CLKEN1.BUFC clear. */
    qtest_writel(global_qtest, CMU_BASE + CMU_CLKEN1 + REGISTER_CLEAR_WINDOW,
                 CMU_CLKEN1_BUFC);
    qtest_writel(global_qtest, BUFC_BASE + BUFC_CH0 + BUFC_ADDR, 0x20000040);
    g_assert_cmphex(qtest_readl(global_qtest, BUFC_BASE + BUFC_CH0 + BUFC_ADDR),
                    ==, 0x20000040);
    qtest_writel(global_qtest, CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
                 CMU_CLKEN1_RAC);
    g_assert_cmphex(qtest_readl(global_qtest, RAC_BASE + RAC_SR0), ==,
                    0x11111111);
    rf_reset_with_clocks();
}

/*
 * RADIOCLKCTRL.EN clear makes the M33's SEQRAM window (0xb0000000) fault
 * like the radio windows and BUFC (SS7.72): a read is a bus error, a write
 * is dropped, and the RAM keeps its contents.
 */
#define SEQRAM_NS_WORD           0xb0001000u

static void test_seqram_radio_clock_gate(void)
{
    rf_reset_with_clocks();
    qtest_writel(global_qtest, SEQRAM_NS_WORD, 0x5eed0001);
    qtest_writel(global_qtest, CMU_BASE + CMU_RADIOCLKCTRL +
                 REGISTER_CLEAR_WINDOW, CMU_RADIOCLKCTRL_EN);
    g_assert_cmphex(qtest_readl(global_qtest, SEQRAM_NS_WORD), ==, 0);
    qtest_writel(global_qtest, SEQRAM_NS_WORD, 0x5eed0002);
    qtest_writel(global_qtest, CMU_BASE + CMU_RADIOCLKCTRL +
                 REGISTER_SET_WINDOW, CMU_RADIOCLKCTRL_EN);
    g_assert_cmphex(qtest_readl(global_qtest, SEQRAM_NS_WORD), ==, 0x5eed0001);
    rf_reset_with_clocks();
}

/*
 * Measured analog latencies (SS7.72): SYNTH.STATUS.INLOCK 8.9 us after
 * SYNTHSTART, never with the IF path enabled (IFFREQEN instead); MODEM
 * TXRAMPDONE 0.44 us after PARAMP is set and 3.96 us after it is cleared.
 */
static void test_synth_lock_and_pa_ramp_timing(void)
{
    rf_reset_with_clocks();
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_FREQ, SYNTH_FREQ_2402_MHZ);
    qtest_writel(global_qtest, RAC_BASE + RAC_SYEN, RAC_SYEN_STOCK_TX);
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_CMD,
                 SYNTH_CMD_DISABLEIF | SYNTH_CMD_SYNTHSTART);
    rf_step(SYNTH_LOCK_NS - 1);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, 0);
    rf_step(1);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_STATUS) &
                        SYNTH_STATUS_INLOCK, ==, SYNTH_STATUS_INLOCK);
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_CMD,
                 SYNTH_CMD_SYNTHSTOP | SYNTH_CMD_ENABLEIF);
    qtest_writel(global_qtest, SYNTH_BASE + SYNTH_CMD, SYNTH_CMD_SYNTHSTART);
    rf_step(SYNTH_LOCK_NS * 2);
    g_assert_cmphex(qtest_readl(global_qtest, SYNTH_BASE + SYNTH_STATUS) &
                        (SYNTH_STATUS_INLOCK | SYNTH_STATUS_IFFREQEN), ==,
                    SYNTH_STATUS_IFFREQEN);

    qtest_writel(global_qtest, MODEM_BASE + MODEM_SEQIF + REGISTER_CLEAR_WINDOW,
                 MODEM_IF_TXRAMPDONE);
    qtest_writel(global_qtest, RAC_BASE + RAC_PAENCTRL + REGISTER_SET_WINDOW,
                 RAC_PAENCTRL_PARAMP);
    rf_step(PA_RAMP_UP_NS - 1);
    g_assert_cmphex(qtest_readl(global_qtest, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, 0);
    rf_step(1);
    g_assert_cmphex(qtest_readl(global_qtest, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, MODEM_IF_TXRAMPDONE);
    qtest_writel(global_qtest, RAC_BASE + RAC_PAENCTRL + REGISTER_CLEAR_WINDOW,
                 RAC_PAENCTRL_PARAMP);
    qtest_writel(global_qtest, MODEM_BASE + MODEM_SEQIF + REGISTER_CLEAR_WINDOW,
                 MODEM_IF_TXRAMPDONE);
    rf_step(PA_RAMP_DOWN_NS - 1);
    g_assert_cmphex(qtest_readl(global_qtest, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, 0);
    rf_step(1);
    g_assert_cmphex(qtest_readl(global_qtest, MODEM_BASE + MODEM_SEQIF) &
                        MODEM_IF_TXRAMPDONE, ==, MODEM_IF_TXRAMPDONE);
    rf_reset_with_clocks();
}

int main(int argc, char **argv)
{

    int result;
    gchar *qemu_args;

    g_test_init(&argc, &argv, NULL);
    rf_trace_path = g_strdup_printf("%s/efr32bg22-rf-%u.trace",
                                    g_get_tmp_dir(), g_random_int());
    qemu_args = g_strdup_printf("-machine efr32bg22-dual -S "
                                "-trace enable=efr32bg22_*,file=%s",
                                rf_trace_path);
    qtest_start(qemu_args);
    rf_power_up();
    g_free(qemu_args);
    qtest_add_func("/efr32bg22/rf/peer-bufc-dma-address-space",
                   test_peer_bufc_dma_uses_peer_address_space);
    qtest_add_func("/efr32bg22/rf/tx-offer-reaches-peer-rx-fifo",
                   test_tx_offer_reaches_peer_rx_fifo);
    qtest_add_func("/efr32bg22/rf/tx-offer-requires-matching-synth-channel",
                   test_tx_offer_requires_matching_synth_channel);
    qtest_add_func("/efr32bg22/rf/carrier-matches-on-1mhz-raster",
                   test_carrier_matches_on_1mhz_raster);
    qtest_add_func("/efr32bg22/rf/carrier-outside-synth-range-not-delivered",
                   test_carrier_outside_synth_range_not_delivered);
    qtest_add_func("/efr32bg22/rf/ook-frame-not-delivered-to-ble-receiver",
                   test_ook_frame_not_delivered_to_ble_receiver);
    qtest_add_func("/efr32bg22/rf/ook-frame-airtime", test_ook_frame_airtime);
    qtest_add_func("/efr32bg22/rf/synth-never-locks-outside-range",
                   test_synth_never_locks_outside_range);
    qtest_add_func("/efr32bg22/rf/tx-offer-enqueues-synthetic-rx-descriptor",
                   test_tx_offer_enqueues_synthetic_rx_descriptor);
    qtest_add_func("/efr32bg22/rf/tx-offer-drops-descriptor-ring-overflow",
                   test_tx_offer_drops_descriptor_ring_overflow);
    qtest_add_func("/efr32bg22/rf/descriptor-ring-reuse-after-each-delivery",
                   test_descriptor_ring_reuse_after_each_delivery);
    qtest_add_func("/efr32bg22/rf/peer-rxsearch-during-active-tx-gets-offer",
                   test_peer_rxsearch_during_active_tx_gets_offer);
    qtest_add_func("/efr32bg22/rf/completed-tx-is-not-offered-to-late-rxsearch",
                   test_completed_tx_is_not_offered_to_late_rxsearch);
    qtest_add_func("/efr32bg22/rf/completed-tx-withdraws-deferred-offer",
                   test_completed_tx_withdraws_deferred_offer);
    qtest_add_func("/efr32bg22/rf/tx-offer-waits-for-receiver-rxsearch",
                   test_tx_offer_waits_for_receiver_rxsearch);
    qtest_add_func("/efr32bg22/rf/system-reset-with-deferred-offer",
                   test_system_reset_with_deferred_offer);
    qtest_add_func("/efr32bg22/rf/wdog-resets-one-soc",
                   test_wdog_resets_one_soc);
    qtest_add_func("/efr32bg22/rf/tx-disable-withdraws-deferred-offer",
                   test_tx_disable_withdraws_deferred_offer);
    qtest_add_func("/efr32bg22/rf/rx-disable-withdraws-active-tx-offer",
                   test_rx_disable_withdraws_active_tx_offer);
    qtest_add_func("/efr32bg22/rf/force-disable-withdraws-deferred-offer",
                   test_force_disable_withdraws_deferred_offer);
    qtest_add_func("/efr32bg22/rf/rx-enable-withdraws-deferred-offer",
                   test_rx_enable_withdraws_deferred_offer);
    qtest_add_func("/efr32bg22/rf/tx-enable-replaces-deferred-offer",
                   test_tx_enable_replaces_deferred_offer);
    qtest_add_func("/efr32bg22/rf/bidirectional-rac-txen-after-rx",
                   test_bidirectional_rac_txen_after_rx);
    qtest_add_func("/efr32bg22/rf/released-tx-turns-around-to-rx",
                   test_released_tx_turns_around_to_rx);
    qtest_add_func("/efr32bg22/rf/locnt-busy-window", test_locnt_busy_window);
    qtest_add_func("/efr32bg22/rf/released-rx-srcchf-follows-channel",
                   test_released_rx_srcchf_follows_channel);
    qtest_add_func("/efr32bg22/rf/released-rx-fifo-overflow",
                   test_released_rx_fifo_overflow);
    qtest_add_func("/efr32bg22/rf/released-rx-frame-disables",
                   test_released_rx_frame_disables);
    qtest_add_func("/efr32bg22/rf/released-rx-chain-off-detects-nothing",
                   test_released_rx_chain_off_detects_nothing);
    qtest_add_func("/efr32bg22/rf/released-rx-frame-turns-around-to-tx",
                   test_released_rx_frame_turns_around_to_tx);
    qtest_add_func("/efr32bg22/rf/released-rx-frame-end-alternatives",
                   test_released_rx_frame_end_alternatives);
    qtest_add_func("/efr32bg22/rf/released-rx-state-machine",
                   test_released_rx_state_machine);
    qtest_add_func("/efr32bg22/rf/released-tx-disables",
                   test_released_tx_disables);
    qtest_add_func("/efr32bg22/rf/radio-block-clock-gates",
                   test_radio_block_clock_gates);
    qtest_add_func("/efr32bg22/rf/seqram-radio-clock-gate",
                   test_seqram_radio_clock_gate);
    qtest_add_func("/efr32bg22/rf/synth-lock-and-pa-ramp-timing",
                   test_synth_lock_and_pa_ramp_timing);
    result = g_test_run();
    qtest_end();
    g_unlink(rf_trace_path);
    g_free(rf_trace_path);
    return result;
}
