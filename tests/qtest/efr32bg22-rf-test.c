/*
 * End-to-end QTest coverage for the EFR32BG22 in-process RF medium.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

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

static uint64_t peer(uint32_t address)
{
    if (address >= FRC_BASE) {
        return EFR32BG22_PEER_RADIO_BASE + (address - RADIO_BASE);
    }
    return EFR32BG22_PEER_PERIPH_BASE + (address - 0x40000000);
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

int main(int argc, char **argv)
{
    int result;

    g_test_init(&argc, &argv, NULL);
    qtest_start("-machine efr32bg22-dual -S");
    qtest_add_func("/efr32bg22/rf/peer-bufc-dma-address-space",
                   test_peer_bufc_dma_uses_peer_address_space);
    qtest_add_func("/efr32bg22/rf/tx-offer-reaches-peer-rx-fifo",
                   test_tx_offer_reaches_peer_rx_fifo);
    qtest_add_func("/efr32bg22/rf/tx-offer-enqueues-synthetic-rx-descriptor",
                   test_tx_offer_enqueues_synthetic_rx_descriptor);
    qtest_add_func("/efr32bg22/rf/tx-offer-drops-descriptor-ring-overflow",
                   test_tx_offer_drops_descriptor_ring_overflow);
    qtest_add_func("/efr32bg22/rf/peer-rxsearch-during-active-tx-gets-offer",
                   test_peer_rxsearch_during_active_tx_gets_offer);
    qtest_add_func("/efr32bg22/rf/completed-tx-is-not-offered-to-late-rxsearch",
                   test_completed_tx_is_not_offered_to_late_rxsearch);
    qtest_add_func("/efr32bg22/rf/completed-tx-withdraws-deferred-offer",
                   test_completed_tx_withdraws_deferred_offer);
    qtest_add_func("/efr32bg22/rf/tx-offer-waits-for-receiver-rxsearch",
                   test_tx_offer_waits_for_receiver_rxsearch);
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
    result = g_test_run();
    qtest_end();
    return result;
}
