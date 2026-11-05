/*
 * Silicon Labs EFR32BG22 native QEMU machine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <math.h>
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/timer.h"
#include "hw/boards.h"
#include "hw/arm/efr32-rf-medium.h"
#include "hw/arm/efr32-usart.h"
#include "hw/arm/efr32-gpio.h"
#include "hw/arm/efr32-i2c.h"
#include "hw/arm/efr32-radioaes.h"
#include "hw/arm/efr32-trng.h"
#include "hw/arm/efr32-gpcrc.h"
#include "hw/arm/efr32-interconnect.h"
#include "hw/arm/efr32-prs.h"
#include "hw/arm/efr32-buram.h"
#include "hw/arm/efr32-letimer.h"
#include "hw/arm/efr32-eusart.h"
#include "hw/arm/efr32-ldma.h"
#include "hw/arm/efr32-soc-link.h"
#include "hw/misc/analog-source.h"
#include "hw/misc/owo-suit.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/efr32bg22-debug.h"
#include "hw/arm/efr32bg22.h"
#include "hw/core/resetcontainer.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "sysemu/cpu-timers.h"
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

/* Radio block addresses and source-backed register offsets. */
#define EFR32_FRC_BASE 0xa8004000u
#define EFR32_AGC_BASE 0xa800c000u
#define EFR32_RFCRC_BASE 0xa8010000u
#define EFR32_RFCRC_CMD 0x010u
#define EFR32_RFCRC_CMD_INITIALIZE (1u << 0)
#define EFR32_RFCRC_INIT 0x018u
#define EFR32_RFCRC_DATA 0x01cu
#define EFR32_MODEM_BASE 0xa8014000u
#define EFR32_SYNTH_BASE 0xa8018000u
#define EFR32_SYNTH_CHCTRL 0x040u
#define EFR32_SYNTH_CHSP 0x044u
#define EFR32_SYNTH_CHSP_CHSP_MASK 0x3ffffu
#define EFR32_SYNTH_CHCTRL_CHNO_MASK 0x3fu
#define EFR32_SYNTH_CALOFFSET 0x048u
#define EFR32_SYNTH_CALOFFSET_MASK 0x7fffu
#define EFR32_SYNTH_CALOFFSET_SIGN 0x4000u
/*
 * The carrier.  SYNTH.FREQ is in units of f_HFXO / 2^19: the three words
 * captured on silicon for the advertising channels are round(f * 2^19 /
 * 38.4 MHz) exactly (owo-open-fw docs/radio-tables.md SS7.80), and stock
 * leaves CHCTRL zero.  CHCTRL.CHNO x CHSP (6 and 18 bits, public field
 * widths) and CALOFFSET (15-bit two's complement) add to FREQ in the same
 * units: measured on silicon from the LO count, CHNO 1..3 x CHSP 13653 moved
 * the LO by 1.00 MHz per channel and CALOFFSET 0x1000 / 0x4000 / 0x7fff by
 * +300 kHz / -1.2 MHz / 0 (SS7.90).
 */
#define EFR32_SYNTH_FREQ 0x034u
#define EFR32_SYNTH_FREQ_MASK 0x0fffffffu
#define EFR32_SYNTH_FREQ_FRACTION_BITS 19
#define EFR32_RF_RASTER0_HZ EFR32_SYNTH_LOCK_MIN_HZ
#define EFR32_RF_RASTER_HZ 1000000ull
/*
 * Carriers the synthesizer calibrates and locks to (owo-open-fw docs/radio-tables.md SS7.90,
 * measured on a BGM220P, receive only): with the sequencer's calibration (VCOTUNING.VCAPSEL
 * 16) the LO count read the programmed LO within one count (4.7 kHz) for every carrier from
 * 2158 to 2644 MHz in 2 MHz steps; at 2156 and 2646 MHz the calibration code sat at its end
 * (2047, 0) and the LO stopped short (pinned near 2156.5 and 2647.6 MHz LO). VCAPSEL 31 and
 * 0 move the range to 2134-2596 and 2192-2702 MHz; the open sequencer always uses 16.
 * Transmit lock was checked at 2304, 2395 and 2445 MHz only (SS7.85). Beyond the range the
 * model reaches nothing (the datasheet specifies 2400-2483.5 MHz, Table 4.5 FRANGE).
 */
#define EFR32_SYNTH_LOCK_MIN_HZ 2158000000ull
#define EFR32_SYNTH_LOCK_MAX_HZ 2644000000ull
/*
 * The frame path needs a running synthesizer (owo-open-fw
 * docs/radio-tables.md SS7.60/SS7.61, measured on silicon): with RAC.SYEN
 * clear and no SYNTH.CMD.SYNTHSTART, RAC sits in TX indefinitely and FRC
 * never sets ACTIVETXFCD; writing SYEN and then SYNTHSTART on that stuck
 * radio immediately drains the frame.  SYNTH.STATUS read 0x4 (SYNTHREADY
 * only) throughout the stalled TX and reports INLOCK once started.  The
 * register itself is read-only (public SYNTH_S_register.py).
 */
#define EFR32_SYNTH_STATUS 0x008u
#define EFR32_SYNTH_STATUS_INLOCK (1u << 0)
#define EFR32_SYNTH_STATUS_SYNTHREADY (1u << 2)
#define EFR32_SYNTH_STATUS_IFFREQEN (1u << 1)
#define EFR32_SYNTH_CMD_ENABLEIF (1u << 2)
#define EFR32_SYNTH_CMD_DISABLEIF (1u << 3)
/*
 * SYNTH.STATUS.INLOCK follows SYNTH.CMD.SYNTHSTART by 7.6-9.95 us (median
 * 8.9 us, n = 216 TX starts on running stage 3, timed by the M0+ SysTick at
 * 38.4 MHz, owo-open-fw docs/radio-tables.md SS7.72).  With the IF path
 * enabled (receive), STATUS read 0x6 -- IFFREQEN and SYNTHREADY, never
 * INLOCK -- on stock and stage 3 alike (SS7.64, SS7.72).
 */
#define EFR32_SYNTH_LOCK_NS 8900
/*
 * MODEM TXRAMPDONE follows a RAC.PAENCTRL.PARAMP change: 0.44 us for the
 * ramp up (one M0+ polling step, n = 110) and a median 3.96 us for the ramp
 * down (0.81-3.96 us, n = 216), M0+ SysTick on running stage 3 (SS7.72).
 */
#define EFR32_PA_RAMP_UP_NS 440
#define EFR32_PA_RAMP_DOWN_NS 3960
#define EFR32_SYNTH_CMD 0x00cu
#define EFR32_SYNTH_CMD_SYNTHSTART (1u << 0)
#define EFR32_SYNTH_CMD_SYNTHSTOP (1u << 1)
/* Hardware-derived from the programmed channel; read-only in the public
 * model, so software writes must not reach them. */
#define EFR32_SYNTH_LOCNTSTATUS 0x08cu
#define EFR32_SYNTH_LOCNTTARGET 0x090u
/*
 * LO-count measurement time.  On silicon (owo-open-fw docs/radio-tables.md
 * SS7.80) the open sequencer's VCO calibration, stamped with the M0+ SysTick
 * (38.4 MHz) from just after LOCNTCTRL.RUN to the first poll that reads
 * LOCNTSTATUS.BUSY clear, took 70 cycles at NUMCYCLE 5, 40 at 4 and 20 at 3
 * and 2 (20 being the instrumented poll's floor), n = 200 calibrations each.
 * [I] BUSY lasting 2^(NUMCYCLE + 1) HFXO periods from RUN reproduces all four
 * after the poll's granularity; the same probe image in this model then reads
 * the same cycle counts (SS7.80).
 */
#define EFR32_SYNTH_LOCNTCTRL 0x088u
#define EFR32_SYNTH_LOCNTCTRL_RUN (1u << 2)
#define EFR32_SYNTH_LOCNTCTRL_NUMCYCLE_SHIFT 4
#define EFR32_SYNTH_LOCNTCTRL_NUMCYCLE_MASK (0xfu << 4)
#define EFR32_SYNTH_LOCNTSTATUS_BUSY (1u << 19)
/*
 * RAC.SYEN's VCO supply enables (public RAC_S_field.py).  [I] Taken as the
 * condition for SYNTHSTART to start the VCO: SYEN=0 never started it on
 * silicon, while 0x1FBF and 0x7FBF (both containing these three) did.
 */
#define EFR32_RAC_SYEN 0x15cu
/*
 * PA ramp completion.  Silicon latches MODEM.IF/SEQIF.TXRAMPDONE (public
 * MODEM_S_field.py, bit 3) around a transmission; the sticky MODEM.IF of a
 * live stage-3 board reads 0x0004000F, TXRAMPDONE included (owo-open-fw
 * docs/radio-tables.md SS7.63).  Stock's and the open sequencer's TXPD
 * handlers clear SEQIF.TXRAMPDONE, drop RAC.PAENCTRL.PARAMP and wait for it.
 * Those handlers clear the flag *after* changing PARAMP, then poll: on
 * silicon the ramp is still running at the clear.  The model has no measured
 * ramp duration, so a PARAMP change leaves the ramp pending and it completes
 * at the first later read of MODEM.IF or MODEM.SEQIF; a clear issued in
 * between cannot discard a ramp that has not finished.
 */
#define EFR32_RAC_PAENCTRL 0x0b4u
#define EFR32_RAC_PAENCTRL_PARAMP (1u << 8)
#define EFR32_MODEM_IF 0x208u
/*
 * PA ramp control and ramp levels: public revB2 MODEM_S_register.py offsets
 * 0x0c4/0x0cc, writable masks 0xff800fff/0x00ffffff, reset values 0x555
 * (RAMPRATE0..2 = 5) and 0x9f9f9f (RAMPLEV0..2 = 159).  Latches only: the
 * medium carries no transmit power.
 */
#define EFR32_MODEM_RAMPCTRL 0x0c4u
#define EFR32_MODEM_RAMPCTRL_RESET 0x00000555u
#define EFR32_MODEM_RAMPLEV 0x0ccu
#define EFR32_MODEM_RAMPLEV_RESET 0x009f9f9fu
#define EFR32_MODEM_SEQIF 0x24cu
#define EFR32_MODEM_IF_TXRAMPDONE (1u << 3)
#define EFR32_RAC_SYEN_VCO_POWER ((1u << 7) | (1u << 8) | (1u << 9))
#define EFR32_PROTIMER_BASE 0xa801c000u
#define EFR32_PROTIMER_NS_BASE 0xb801c000u
#define EFR32_RAC_BASE 0xa8020000u

#define EFR32_RADIO_IPVERSION 0x000u
#define EFR32_RADIO_EN 0x004u

/*
 * Read-only (public revB2 FRC_S_register.py); software never sets ACTIVETXFCD
 * or any other bit here. QEMU does not yet know what hardware condition sets
 * ACTIVETXFCD (docs/radio-tables.md SS7.4), so this only enforces the
 * read-only contract and the public mask; it does not synthesize a value.
 */
#define EFR32_FRC_STATUS 0x008u
#define EFR32_FRC_STATUS_MASK 0x01ffffffu
#define EFR32_FRC_IF 0x068u
#define EFR32_FRC_IEN 0x06cu
#define EFR32_FRC_CMD 0x02cu
#define EFR32_FRC_RXRAWDATA 0x090u
#define EFR32_FRC_CMD_RXABORT (1u << 0)
#define EFR32_FRC_CMD_CONVINIT (1u << 9)
#define EFR32_FRC_IF_TXDONE (1u << 0)
#define EFR32_FRC_IF_RXDONE (1u << 4)

/*
 * DFLCTRL (0xa8004000+0x00c, public FRC_S_register.py) and WCNTCMP0
 * (+0x020). Unlike ACTIVETXFCD above, this one IS confirmed: real hardware
 * measures WCNTCMP0 == <the extracted length-field byte> + 1 for two
 * independent real BLE PDU Length header values (9 and 12, captured via a
 * live execution trace of the real staged packet bytes, not assumed --
 * owo-open-fw docs/radio-tables.md SS7.38). `+1` happens to equal this
 * project's only-ever-configured MINLENGTH value, so the implementation
 * below uses `field + MINLENGTH` -- a named field, not a bare increment --
 * but only one MINLENGTH value has ever been tested, so this is "the
 * simplest formula consistent with the real evidence, expressed in named
 * terms," not an independently confirmed general law. (SS7.29's original
 * fix used `DFLOFFSET + field + 3` instead, which happened to satisfy
 * that commit's own self-constructed qtest because the test used assumed,
 * not real, header byte values -- retracted in SS7.38 once a live trace
 * showed the assumption was wrong.) Only DFLMODE==1 -- the only value
 * this project's firmware ever configures (every PHY variant shares one
 * DFLCTRL write in radio.c's base config table) -- is implemented;
 * DFLMODE's other enum values have no public description text in the
 * generated field model and no real-hardware confirmation here, so they
 * are deliberately left alone (WCNTCMP0 keeps whatever software wrote).
 */
#define EFR32_FRC_DFLCTRL 0x00cu
#define EFR32_FRC_DFLCTRL_DFLMODE_MASK 0x7u
#define EFR32_FRC_DFLCTRL_DFLMODE_LENGTH_FIELD 0x1u
#define EFR32_FRC_DFLCTRL_DFLBITORDER (1u << 3)
#define EFR32_FRC_DFLCTRL_DFLSHIFT_SHIFT 4u
#define EFR32_FRC_DFLCTRL_DFLSHIFT_MASK 0x7u
#define EFR32_FRC_DFLCTRL_DFLOFFSET_SHIFT 8u
#define EFR32_FRC_DFLCTRL_DFLOFFSET_MASK 0xfu
#define EFR32_FRC_DFLCTRL_DFLBITS_SHIFT 12u
#define EFR32_FRC_DFLCTRL_DFLBITS_MASK 0xfu
#define EFR32_FRC_DFLCTRL_MINLENGTH_SHIFT 16u
#define EFR32_FRC_DFLCTRL_MINLENGTH_MASK 0xfu
#define EFR32_FRC_WCNTCMP0 0x020u

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
#define EFR32_RAC_CTRL_SEQRESET (1u << 24)
#define EFR32_RAC_CTRL_CPUWAITDIS (1u << 26)
#define EFR32_RAC_RXENSRCEN_SWRXEN (1u << 1)
#define EFR32_RAC_STATE_OFF 0u
#define EFR32_RAC_STATE_RXSEARCH 2u
/*
 * Receive-side and turnaround states (public RAC_S STATE encoding; the
 * sequencer IRQ of each is IRQ<state>).  Silicon with a released sequencer
 * ends a TX frame in TX2RX when an RX source is asserted, then reaches
 * RXSEARCH; RXDIS from RXSEARCH passes RXPD before OFF (owo-open-fw
 * docs/radio-tables.md SS7.63).  A TXEN issued during RXFRAME with SWRXEN
 * withdrawn turns the RAC into RX2TX at frame end; with SWRXEN still asserted
 * it returns to receive instead (SS7.65), and the RAC can rest in RX2RX
 * (SS7.69).  RX2TX is held until STATERX2TX is acknowledged and TX begins
 * shortly after that acknowledgement (SS7.67, SS7.68).
 */
#define EFR32_RAC_STATE_RXWARM 1u
#define EFR32_RAC_STATE_RXFRAME 3u
#define EFR32_RAC_STATE_RXPD 4u
#define EFR32_RAC_STATE_RX2RX 5u
#define EFR32_RAC_STATE_RXOVERFLOW 6u
#define EFR32_RAC_STATE_RX2TX 7u
#define EFR32_RAC_STATE_TX2RX 11u
#define EFR32_RAC_STATE_TXWARM 8u
#define EFR32_RAC_STATE_TX 9u
/*
 * With sequencer firmware released, silicon ends a frame in TXPD, raises
 * SEQIF.STATETXPD on the sequencer's IRQ10, and reaches OFF only once that
 * is acknowledged; a sequencer that never services IRQ10 leaves the RAC
 * parked in TXPD (owo-open-fw docs/radio-tables.md SS7.61/SS7.62).
 */
#define EFR32_RAC_STATE_TXPD 10u
#define EFR32_RAC_STATE_SHUTDOWN 13u
/*
 * State event n is RAC.SEQIF bit n + 16 (public RAC_S_field.py STATEOFF ..
 * STATESHUTDOWN) and arrives on the sequencer's IRQn (owo-open-fw
 * docs/radio-tables.md SS7.63).
 */
#define EFR32_RAC_SEQ_STATE_EVENTS 14u
#define EFR32_RAC_SEQIF_STATE_SHIFT 16u
#define EFR32_RAC_SEQIF_STATE_EVENTS \
    (((1u << EFR32_RAC_SEQ_STATE_EVENTS) - 1) << EFR32_RAC_SEQIF_STATE_SHIFT)
#define efr32_rac_seq_state_flag(state) \
    (1u << (EFR32_RAC_SEQIF_STATE_SHIFT + (state)))

/*
 * Airtime.  On silicon MODEM.FRAMESENT - MODEMH.SYNCSENT is exactly the
 * PDU + CRC octet count at 8 us per octet (LE 1M), for 9-, 24- and 35-octet
 * payloads (owo-open-fw docs/radio-tables.md SS7.71).  The preamble and
 * access address ahead of the sync point are the LE 1M packet format's one
 * and four octets.  The BUFC TX snapshot holds header, length and payload.
 * Only LE 1M is modelled.
 */
#define EFR32_RF_LE1M_NS_PER_OCTET 8000
#define EFR32_RF_LE1M_PREAMBLE_OCTETS 1
#define EFR32_RF_LE_ACCESS_ADDRESS_OCTETS 4
#define EFR32_RF_LE_CRC_OCTETS 3
#define EFR32_RF_LE1M_SYNC_NS \
    ((EFR32_RF_LE1M_PREAMBLE_OCTETS + EFR32_RF_LE_ACCESS_ADDRESS_OCTETS) * \
     EFR32_RF_LE1M_NS_PER_OCTET)
#define efr32_rf_le1m_airtime_ns(pdu_octets) \
    (EFR32_RF_LE1M_SYNC_NS + \
     ((int64_t)(pdu_octets) + EFR32_RF_LE_CRC_OCTETS) * \
         EFR32_RF_LE1M_NS_PER_OCTET)
/*
 * Frame timing anchors, measured on running stage 3 with PRS routing RACL and
 * MODEM signals into TIMER1 input capture at 76.8 MHz (owo-open-fw
 * docs/radio-tables.md SS7.72, n > 300 each).  The model's air_start_ns is
 * the transmitter modem's first preamble bit: MODEMH.SYNCSENT minus the
 * preamble and access address ([I] SS7.71: SYNCSENT marks the end of the
 * access address).
 *  - RACL.TX rise (TX entry) -> SYNCSENT: 40.6-40.7 us;
 *  - MODEM.FRAMESENT -> RACL.TX fall (TX exit): 0.8-1.0 us;
 *  - MODEMH.EOF -> RACL.RX fall (receiver leaves RXFRAME): 3.2-3.4 us;
 *  - a peer's FRAMESENT -> this receiver's EOF: 9.2-9.7 us, the
 *    d_tx + d_rx of SS7.71, from two independent peers.
 * [I] RXFRAME entry: MODEML.FRAMEDET came 131.5-133.2 us before EOF on
 * frames assumed to be 14-octet SCAN_REQs (136 us from the access address
 * end), so frame detection is taken as 3.5 us after the access address
 * arrives; it is assumed that RXFRAME begins at FRAMEDET.
 */
#define EFR32_RAC_TX_ENTRY_TO_SYNCSENT_NS 40650
#define EFR32_RAC_TX_ENTRY_TO_AIR_NS \
    (EFR32_RAC_TX_ENTRY_TO_SYNCSENT_NS - EFR32_RF_LE1M_SYNC_NS)
#define EFR32_RAC_FRAMESENT_TO_TX_EXIT_NS 900
#define EFR32_RAC_EOF_TO_RX_EXIT_NS 3300
#define EFR32_RF_FRAMESENT_TO_PEER_EOF_NS 9450
#define EFR32_RAC_SYNC_TO_FRAMEDET_NS 3500
/*
 * A cleared state event releases the next transition 1.09 us later: M0+
 * SysTick from its SEQIF clear to a changed RAC.STATUS, the same for TX2RX,
 * RXPD, TXPD and RX2TX (n > 100 each, 1.09-1.22 us including one polling
 * step; SS7.72).  It replaces SS7.68's ~3 us, which also counted the
 * sequencer's exception exit and entry.
 */
#define EFR32_RAC_ACK_TO_TRANSITION_NS 1090
/* RAC.STATUS.STATE is four bits wide (public RAC_S.STATUS). */
#define EFR32_RAC_STATE_COUNT 16u

/*
 * RAC.STIMER: read-only, 16 bits wide (public RAC_S.STIMER).  Measured at
 * ~4.8 ticks/us against BURTC (SS7.66), restarting on every RAC state change
 * (SS7.68).  The prescaler (RAC.PRESC.STIMER) is not modelled; the rate is
 * the one measured with the open sequencer's own PRESC setting.
 */
#define EFR32_RAC_STIMER 0x030u
#define EFR32_RAC_STIMER_MASK 0x0000ffffu
#define EFR32_RAC_STIMER_TICKS_PER_10US 48u
#define EFR32_RAC_STIMER_NS_PER_10US 10000u

/*
 * RAC.STATUS fields (public RAC_S.STATUS).  RXENS follows the asserted RX
 * source: stock read 0x89400002 in TX with SWRXEN set (RXENS, RXMASK bit 1),
 * stage 3 0x09400000 in TX without it (SS7.21/SS7.63), and stock read
 * 0x63400000 in RXFRAME after withdrawing SWRXEN and issuing TXEN: TXENS set,
 * RXENS clear (SS7.65).
 */
#define EFR32_RAC_STATUS_STATE_SHIFT 24u
#define EFR32_RAC_STATUS_SEQSLEEPING (1u << 22)
#define EFR32_RAC_STATUS_SEQACTIVE (1u << 28)
#define EFR32_RAC_STATUS_DEMODENS (1u << 29)
#define EFR32_RAC_STATUS_TXENS (1u << 30)
#define EFR32_RAC_STATUS_RXENS (1u << 31)
#define EFR32_RAC_STATUS_RXMASK_SWRXEN EFR32_RAC_RXENSRCEN_SWRXEN

/*
 * FRC.TRAILRXDATA is configuration (public FRC_S, read-write): with CRCOK set
 * FRC appends one octet after each received frame, [I] whose bit 7 is the CRC
 * result; its low bits vary with the signal on silicon (owo-open-fw
 * docs/radio-tables.md SS7.66).  The model's medium has no bit errors, so the
 * octet always reports CRC success, and its signal-dependent low bits are a
 * synthetic zero.  The other trailer fields (RSSI, PROTIMER/RTC stamps) are
 * not modelled.
 */
#define EFR32_FRC_TRAILRXDATA 0x054u
#define EFR32_FRC_TRAILRXDATA_CRCOK (1u << 1)
#define EFR32_FRC_TRAILER_CRCOK (1u << 7)

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
#define EFR32_CMU_CLKEN0 0x064u
#define EFR32_CMU_CLKEN1 0x068u
#define EFR32_CMU_SYSCLKCTRL 0x070u
#define EFR32_CMU_RADIOCLKCTRL 0x280u
#define EFR32_CMU_RADIOCLKCTRL_EN (1u << 0)
#define EFR32_CMU_CLKEN1_AGC (1u << 0)
#define EFR32_CMU_CLKEN1_MODEM (1u << 1)
#define EFR32_CMU_CLKEN1_RFCRC (1u << 2)
#define EFR32_CMU_CLKEN1_FRC (1u << 3)
#define EFR32_CMU_CLKEN1_PROTIMER (1u << 4)
#define EFR32_CMU_CLKEN1_RAC (1u << 5)
#define EFR32_CMU_CLKEN1_SYNTH (1u << 6)
#define EFR32_CMU_CLKEN1_BUFC (1u << 11)
#define EFR32_CMU_CLKEN1_RDSCRATCHPAD (1u << 7)
#define EFR32_CMU_CLKEN1_RDMAILBOX0 (1u << 8)
#define EFR32_CMU_CLKEN1_RDMAILBOX1 (1u << 9)
#define EFR32_CMU_CLKEN1_PRORTC (1u << 10)
#define EFR32_CMU_CLKEN0_SYSCFG (1u << 16)
/*
 * Blocks whose whole 16 KiB window (normal, SET, CLR, TGL) answers a read
 * with a bus error while its CMU bus clock is off, and drops writes (a
 * debugger write is accepted and the value is unchanged once the clock is
 * back).  Measured on silicon by gating each CLKEN bit alone from the
 * debugger, owo-open-fw docs/radio-tables.md SS7.72; the same probe placed
 * RDSCRATCHPAD at 0xa8024000 and confirmed PRORTC at 0xa8000000.
 */
#define EFR32_SYSCFG_BASE 0x4007c000u
#define EFR32_PRORTC_BASE 0xa8000000u
#define EFR32_RDSCRATCHPAD_BASE 0xa8024000u
#define EFR32_RDMAILBOX0_BASE 0xa8028000u
#define EFR32_RDMAILBOX1_BASE 0xa802c000u
#define EFR32_CRYPTOACC_BASE 0x4c020000u
#define EFR32_CRYPTOACC_END 0x4c02c000u
#define EFR32_CMU_CLKEN1_CRYPTOACC (1u << 13)
#define EFR32_CMU_CLKEN0_GPCRC (1u << 3)
#define EFR32_CMU_CLKEN0_RADIOAES (1u << 2)
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_MASK 0x7u
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_FSRCO 1u
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_HFXO 3u
#define EFR32_CMU_DPLLREFCLKCTRL 0x100u
#define EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_HFXO 1u
#define EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_LFXO 2u
#define EFR32_CMU_EM01GRPACLKCTRL 0x120u
#define EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_HFRCODPLL 1u
#define EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_HFXO 2u
#define EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_FSRCO 3u
/* Oscillator rates from xg22.dtsi. */
#define EFR32_HFXO_HZ 38400000ULL
#define EFR32_LFXO_HZ 32768ULL
/* FSRCO against HFXO on silicon: 19.804-19.811 MHz (n = 8, 1 ms windows;
 * owo-open-fw docs/radio-tables.md SS7.73); xg22.dtsi says 20 MHz. */
#define EFR32_FSRCO_MEASURED_HZ 19807000ULL
#define EFR32_HFRCO_RESET_HZ 19000000ULL
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
/*
 * EMU.RSTCAUSE (RM SS12.3.x, SS12.5.15): the cause of the last reset, kept
 * across resets until software writes EMU_CMD.RSTCAUSECLR ("Once read,
 * EMU_RSTCAUSE should be cleared").  Power-up sets POR; the M33's
 * AIRCR.SYSRESETREQ sets SYSREQ.  The Gecko bootloader and AppLoader honour
 * the reset-reason word in RAM only when SYSREQ is set.
 * [I] Causes accumulate until cleared (the RM's instruction to clear after
 * reading); unmeasured.  Pin, WDOG0 and lockup causes are modelled; BOD,
 * EM4 and DCI causes are not.
 *
 * EMU_RSTCTRL (RM 12.5.14, Table 12.8): whether a soft reset source resets
 * the device.  SYSRMODE (reset 1) for SYSRESETREQ, WDOG0RMODE (1) for the
 * watchdog, LOCKUPRMODE (0) for an M33 Lockup; 0 = "Reset request is
 * blocked".  RSTCTRL is EMU state that the soft resets (SYSREQ, WDOG0,
 * LOCKUP) leave alone and the hard ones (POR, PIN) restore.  [I] A blocked
 * request leaves no RSTCAUSE bit; a blocked watchdog runs on as with
 * CFG.WDOGRSTDIS.
 */
#define EFR32_EMU_CMD 0x070u
#define EFR32_EMU_CMD_RSTCAUSECLR (1u << 17)
#define EFR32_EMU_RSTCTRL 0x090u
#define EFR32_EMU_RSTCTRL_WDOG0RMODE (1u << 0)
#define EFR32_EMU_RSTCTRL_SYSRMODE (1u << 2)
#define EFR32_EMU_RSTCTRL_LOCKUPRMODE (1u << 3)
#define EFR32_EMU_RSTCTRL_WRITABLE 0x000104cdu /* bits 16, 10, 7, 6, 3, 2, 0 */
#define EFR32_EMU_RSTCTRL_RESET 0x00010405u    /* DCI, DECBOD, SYS, WDOG0 */
#define EFR32_EMU_RSTCAUSE 0x094u
#define EFR32_EMU_RSTCAUSE_POR (1u << 0)
#define EFR32_EMU_RSTCAUSE_PIN (1u << 1)
#define EFR32_EMU_RSTCAUSE_LOCKUP (1u << 5)
#define EFR32_EMU_RSTCAUSE_SYSREQ (1u << 6)

/*
 * Part identity (EFR32xG22 reference manual rev. 1.2, SS6.3 Table 6.1 and
 * SS6.4).  The information block at 0x0FE00000 holds the 1 KB User Data
 * page; the 1 KB Device Information page sits at 0x0FE08000.  Software,
 * and the MSC, size the part from DEVINFO.MSIZE (SS6.4.2.4): FLASH in KB
 * (bits 15:0), SRAM in KB (bits 26:16).  The main flash is 16 KB to
 * 512 KB in 8 KB pages (SS6.3), RAM up to 32 KB (SS4.1).
 */
#define EFR32_INFO_BASE 0x0fe00000u
/* The model's information-block window; contents beyond the two pages
 * below were never read from silicon (zero / erased placeholders). */
#define EFR32_INFO_WINDOW_SIZE 0x10000u
#define EFR32_USERDATA_OFFSET 0x0000u
#define EFR32_USERDATA_SIZE 0x400u
#define EFR32_DEVINFO_OFFSET 0x8000u
#define EFR32_DEVINFO_SIZE 0x400u
/* Span after the DI page that the model has always read as zero. */
#define EFR32_DEVINFO_WINDOW_SIZE 0x2000u
#define EFR32_FLASH_ERASED_BYTE 0xffu
#define EFR32_FLASH_PAGE_SIZE 0x2000u
/* GPIO ports A-D, 16 pins each at most (RM SS24.3.2 input routing). */
#define EFR32_GPIO_PORTS 4
#define EFR32_GPIO_PINS 16
#define EFR32_DEVINFO_PART 0x004u
#define EFR32_DEVINFO_PART_FAMILYNUM_SHIFT 16
#define EFR32_DEVINFO_PART_FAMILYNUM_MASK 0x3fu
#define EFR32_DEVINFO_PART_FAMILY_SHIFT 24
#define EFR32_DEVINFO_PART_FAMILY_MASK 0x3fu
/* DEVINFO.PART.FAMILY codes (public efr32bg22_devinfo.h). */
#define EFR32_DEVINFO_PART_FAMILY_FG 0u
#define EFR32_DEVINFO_PART_FAMILY_MG 1u
#define EFR32_DEVINFO_PART_FAMILY_BG 2u
#define EFR32_DEVINFO_PART_FAMILY_PG 5u
#define EFR32_DEVINFO_PART_FAMILYNUM_XG22 22u

/*
 * SYSCFG.CHIPREV (RM SS6.6.6.4) resets to 0 in the register description,
 * yet software reads the part's family there (SYSTEM_ChipRevisionGet) and
 * NVM3 stamps it into every page header: the suit's NVM3 pages, written on
 * silicon, carry family 0x35, the RM's BG22 code.  With CHIPREV = 0 the
 * stock application judged every NVM3 page foreign and erased them.  The
 * family follows DEVINFO.PART (FAMILY, FAMILYNUM 22) through the RM's
 * enumeration.  [U] MAJOR and MINOR stay 0: never read from silicon.
 */
#define EFR32_SYSCFG_CHIPREV 0x014u
#define EFR32_SYSCFG_CHIPREV_FAMILY_SHIFT 6
#define EFR32_SYSCFG_CHIPREV_FAMILY_PG22 24u
#define EFR32_SYSCFG_CHIPREV_FAMILY_MG22 52u
#define EFR32_SYSCFG_CHIPREV_FAMILY_BG22 53u
#define EFR32_SYSCFG_CHIPREV_FAMILY_FG22 55u
/*
 * Core identity: CPUID of the BG22's Cortex-M33, r0p4 as OpenOCD reads it
 * on the EK and the suit (owo-open-fw docs/debug-port-model.md).
 */
#define EFR32_M33_CPUID 0x410fd214u

/*
 * MSC flash programming (RM SS6.3.12, register map SS6.7-6.8, reset values
 * from the public efr32bg22_msc.h).  WREN in WRITECTRL enables the
 * controller; ADDRB holds the target, INVADDR flags an address outside the
 * main flash and the User Data page; WRITECMD.ERASEPAGE erases the page of
 * ADDRB; each WDATA write programs one word (bits only go from 1 to 0) and
 * advances ADDRB by 4.  PAGELOCKn and MISCLOCKWORD.UDLOCKBIT refuse the
 * operation (STATUS.LOCKED).  MSC_LOCK with any value but the unlock key
 * locks READCTRL and WRITECTRL (STATUS.REGLOCK).
 * [U] Operations complete at once: write and erase times, the BUSY window
 * and the flash read stall are unmeasured (TODO).
 */
#define EFR32_MSC_BASE 0x40030000u
#define EFR32_MSC_READCTRL 0x008u
#define EFR32_MSC_WRITECTRL 0x00cu
#define EFR32_MSC_WRITECTRL_WREN (1u << 0)
#define EFR32_MSC_WRITECMD 0x010u
#define EFR32_MSC_WRITECMD_ERASEPAGE (1u << 1)
#define EFR32_MSC_WRITECMD_ERASEMAIN0 (1u << 8)
#define EFR32_MSC_ADDRB 0x014u
#define EFR32_MSC_WDATA 0x018u
#define EFR32_MSC_STATUS 0x01cu
#define EFR32_MSC_STATUS_LOCKED (1u << 1)
#define EFR32_MSC_STATUS_INVADDR (1u << 2)
#define EFR32_MSC_STATUS_REGLOCK (1u << 16)
#define EFR32_MSC_STATUS_RESET 0x08000008u
#define EFR32_MSC_IF 0x020u
#define EFR32_MSC_IF_ERASE (1u << 0)
#define EFR32_MSC_IF_WRITE (1u << 1)
#define EFR32_MSC_IEN 0x024u
#define EFR32_MSC_USERDATASIZE 0x034u
#define EFR32_MSC_USERDATASIZE_RESET 0x4u
#define EFR32_MSC_USERDATASIZE_UNIT 256u
#define EFR32_MSC_LOCK 0x03cu
#define EFR32_MSC_LOCK_UNLOCK_KEY 0x1b71u
#define EFR32_MSC_LOCK_LOCKKEY_MASK 0xffffu
#define EFR32_MSC_MISCLOCKWORD 0x040u
#define EFR32_MSC_MISCLOCKWORD_MELOCKBIT (1u << 0)
#define EFR32_MSC_MISCLOCKWORD_UDLOCKBIT (1u << 4)
#define EFR32_MSC_MISCLOCKWORD_RESET 0x00000011u
#define EFR32_MSC_PAGELOCK0 0x120u
#define EFR32_MSC_PAGELOCK1 0x124u
#define EFR32_MSC_PAGES_PER_LOCKWORD 32u
#define EFR32_MSC_IPVERSION_VALUE 0x1u
#define EFR32_MSC_IRQ 49
/* READCTRL resets to MODE = 2 (RM SS6.8.2) */
#define EFR32_MSC_READCTRL_RESET 0x00200000u
/* WREN, IRQERASEABORT and LPWRITE are the writable fields (RM SS6.8.3) */
#define EFR32_MSC_WRITECTRL_MASK 0x0000000bu
#define EFR32_MSC_MISCLOCKWORD_MASK (EFR32_MSC_MISCLOCKWORD_MELOCKBIT | \
                                     EFR32_MSC_MISCLOCKWORD_UDLOCKBIT)
/* PWRCTRL resets to PWROFFDLY 0x10, PWROFFONEM1PENTRY (RM SS6.8.14) */
#define EFR32_MSC_PWRCTRL 0x050u
#define EFR32_MSC_PWRCTRL_RESET 0x00100002u

/* EFR32BG22 TIMER2/4: public Series-2 register layout, IRQs 9/11. */
#define EFR32_TIMER2_BASE 0x40050000u
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
#define EFR32_TIMER_HZ 38400000ULL

/*
 * TIMER0 is the BG22's 32-bit TIMER (TIMER0_CNTWIDTH = 0x20, TIMER0_IRQn = 7
 * in efr32bg22c224f512im40.h).  Offsets, masks and the TOP reset value follow
 * efr32bg22_timer.h: CFG.PRESC is bits 27:18, CC channel n starts at
 * 0x060 + 0x20 * n with CFG at +0x0 and OC at +0x8, CCn IF is bit 4 + n and
 * OF is bit 0.  Only up-counting output compare and overflow are modelled;
 * input capture, DTI, PWM outputs and the sync-busy windows are not.
 */
#define EFR32_TIMER0_BASE 0x40048000u
#define EFR32_TIMER0_IRQ 7
#define EFR32_TIMER0_CHANNELS 3u
#define EFR32_TIMER_CFG 0x004u
#define EFR32_TIMER_CFG_MASK 0x0fff1ffbu
#define EFR32_TIMER_CFG_MODE_MASK 0x3u
#define EFR32_TIMER_CFG_PRESC_SHIFT 18
#define EFR32_TIMER_CFG_PRESC_MASK 0x0ffc0000u
#define EFR32_TIMER_CC_CFG(n) (0x060u + 0x20u * (n))
#define EFR32_TIMER_CC_OC(n) (0x068u + 0x20u * (n))
#define EFR32_TIMER_CC_CFG_MODE_MASK 0x3u
#define EFR32_TIMER_CC_CFG_MODE_OUTPUTCOMPARE 0x2u
#define EFR32_TIMER_IF_OF (1u << 0)
#define EFR32_TIMER_IF_CC(n) (1u << (4 + (n)))
#define EFR32_TIMER_IF_MASK 0x07770077u

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

/* EFR32BG22 SMU peripheral attribution used by Zephyr's secure-only setup. */
#define EFR32_SMU_BASE 0x44008000u
#define EFR32_SMU_PPUSATD1 0x064u
#define EFR32_SMU_PPUSATD1_AHBRADIO (1u << 15)

#define EFR32_BUFC_BASE 0xaa000000u
#define EFR32_BUFC_FIRST 0x00cu
#define EFR32_BUFC_STRIDE 0x040u
#define EFR32_BUFC_ADDR 0x004u
#define EFR32_BUFC_IF 0x114u
#define EFR32_BUFC_IF_BUF1OF (1u << 8)
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
#define EFR32_RF_RX_DESCRIPTOR_BUFC_CHANNEL 2
#define EFR32_BUFC_DESCRIPTOR_OVERFLOW (1u << 16)

/*
 * The M33-facing RF mailbox has two endpoints in the radio address space.
 * The board maps the 0xb802.... non-secure window as an alias of this secure
 * window, so register handling uses these secure canonical addresses.
 */
#define EFR32_RFMAILBOX_B_BASE 0xa8028000u
#define EFR32_RFMAILBOX_C_BASE 0xa802c000u
#define EFR32_RFMAILBOX_FLAGS  0x040u
/*
 * RDMAILBOX0/1: four message pointers (MSGPTR0-3 at 0x000-0x00c) with one
 * flag each in IF (0x040), enabled by IEN (0x044), SET/CLR aliases as usual.
 * Directions, from the stock firmware that runs on silicon: the M0+ posts
 * on mailbox 0 (MSGPTR0, IF_SET bit 0, then waits for the bit to clear) and
 * the M33 enables IEN on mailbox 0 and acknowledges it in its RDMAILBOX
 * (IRQ 39) handler; the M0+ enables IEN on mailbox 1 and its vector table
 * services mailbox 1's IF & IEN at M0+ IRQ 23.  [U] Delivery latency.
 */
#define EFR32_RFMAILBOX_IEN    0x044u
#define EFR32_RFMAILBOX_IF_MASK 0xfu
#define EFR32_RDMAILBOX_M33_IRQ 39
#define EFR32_RDMAILBOX_SEQ_IRQ 23

struct Efr32Periph
{
    SysBusDevice parent_obj;
    MemoryRegion peripheral_mmio;
    MemoryRegion radio_mmio;
    MemoryRegion *memory;
    MemoryRegion *m33_radio_secure;
    MemoryRegion *m33_radio_nonsecure;
    AddressSpace dma_as;
    bool dma_as_initialized;
    GHashTable *registers;
    Efr32Trng trng;
    Efr32Gpcrc gpcrc;
    Efr32Interconnect ic;
    Efr32RadioAes radioaes;
    uint32_t rac_state;
    bool hfxo_fsm_locked;
    bool rx_offer_pending;
    bool usart_tx_enabled[2];
    bool usart_tx_complete[2];
    unsigned endpoint_id;
    Clock *protimer_clk;
    ptimer_state *protimer_counter;
    Clock *burtc_clk;
    /* BURTC: CNT = base + edges since cnt_edge while running (see below). */
    bool burtc_run;
    bool burtc_irq_asserted;
    uint32_t burtc_cnt_base;
    int64_t burtc_cnt_edge;
    int64_t burtc_last_edge;
    uint32_t burtc_comp;
    int64_t burtc_en_busy_until;
    struct {
        bool valid;
        uint32_t value;
        int64_t edge;
    } burtc_pend[3];
    QEMUTimer *burtc_timer;
    /* RTCC: pre-counter and counter as one edge count, see efr32_rtcc_*. */
    bool rtcc_run;
    uint64_t rtcc_pos_base;
    int64_t rtcc_edge_base;
    int64_t rtcc_last_edge;
    QEMUTimer *rtcc_timer;
    /* TIMERn counts base_count + elapsed ticks since base_ns while counting. */
    struct {
        bool running;
        uint32_t base_count;
        int64_t base_ns;
        uint32_t pending_flags;
        int64_t sync_until_ns;
        QEMUTimer *event;
    } timers[5];
    /* Oscillator request / enable instants, -1 while off. */
    int64_t hfxo_request_ns;
    int64_t lfxo_request_ns;
    int64_t lfrco_request_ns;
    int64_t lfrco_off_ns;
    int64_t dpll_enable_ns;
    int64_t dpll_off_ns;
    QEMUTimer *dpll_lock_timer;
    QEMUTimer *rac_tx_timer;
    QEMUTimer *rac_rx_timer;
    GByteArray *bufc_data[4];
    Efr32RfMedium *rf_medium;
    /* Main flash size of this part (DEVINFO.MSIZE), for the MSC. */
    uint32_t flash_size;
    /* EMU.RSTCAUSE: survives system reset until RSTCAUSECLR. */
    uint32_t rstcause;
    /* EMU.RSTCTRL: survives soft resets, restored by POR and PIN. */
    uint32_t rstctrl;
    /* SYSCFG.CHIPREV as the part leaves boot (from DEVINFO.PART). */
    uint32_t chiprev;
    /* Board: millivolts on each GPIO pin, -1 when undriven (IADC input). */
    int32_t analog_mv[EFR32_GPIO_PORTS][EFR32_GPIO_PINS];
    /* IADC single (0) and scan (1) FIFOs. */
    struct {
        uint32_t data[4];
        unsigned count;
    } iadc_fifo[2];
    /* Board analog inputs not named by "analog" (an analog-source). */
    Object *analog_source;
    /* Fault injection: IADC conversions never complete. */
    uint32_t iadc_stall;
    /* IADC conversions in progress, in completion order (efr32_iadc_*). */
    QEMUTimer *iadc_timer;
    struct {
        int64_t done_ns;
        uint8_t queue, id;
        bool last;
        uint32_t input;
    } iadc_conv[2 * 16 + 1];
    unsigned iadc_nconv;
    int64_t iadc_busy_until_ns;
    /* Warm (ADCWARM) for this PRESCALE/ADCMODE/REFSEL key; or in standby. */
    bool iadc_warm, iadc_standby;
    uint32_t iadc_warm_key;
    uint32_t iadc_last_input;
    bool iadc_have_input;
    /* TIMERn CCm compare/PWM outputs, to the GPIO routes. */
    qemu_irq timer_cc_out[5 * 3];
    /* SoC services for the blocks outside this model (efr32-soc-link.h). */
    Efr32SocLink link;
    /* Producer lines into the PRS, numbered as its inputs (efr32-prs.h). */
    qemu_irq prs_async[EFR32_PRS_ASYNC_INPUTS];
    qemu_irq prs_sync[EFR32_PRS_SYNC_INPUTS];
    /* TIMERn CCm input logic and capture FIFO (efr32_timer_ic_*). */
    struct {
        uint32_t fifo[2];
        bool fifo_falling[2];
        unsigned count;
        uint32_t icof;
        bool async_level;
        bool sync_level;
        /* The level the 2-clock sampler last passed (PRSASYNCLEVEL). */
        bool sampled;
        QEMUTimer *sampler;
        struct Efr32Periph *owner;
        uint8_t timer, channel;
    } timer_ic[5][3];
    /* The access in progress comes from a debug access port. */
    bool debugger_access;
    /* The PRS, and the radio's pending PRS pulses (efr32_rac_prs_*). */
    DeviceState *prs;
    QEMUTimer *prs_radio_timer;
    int64_t prs_syncsent_ns;
    int64_t prs_framesent_ns;
    /* IADC queues enabled for triggers (CMD.xSTART..xSTOP); local timer. */
    bool iadc_qen[2];
    bool iadc_timer_on;
    QEMUTimer *iadc_local_timer;
    /* When the request being queued was triggered (efr32_iadc_queue). */
    int64_t iadc_request_ns;
    /* TIMERn LDMA requests: CC0-2, UFOF (efr32-ldma.h signals). */
    qemu_irq timer_dma[5][4];
    /* TIMERn CCm PRS output in PRSCONF = LEVEL. */
    bool timer_prs_level[5][3];
    /* RTCC CCn PRS outputs (CMOA). */
    bool rtcc_ccv[3];
    /* To the debug port: RAC clock retiming wedges it (SS7.80) */
    qemu_irq debug_wedge;
    bool timer_cc_level[5][3];
    QEMUTimer *timer_pwm[5];
    /* TOPB (EFR32_TIMER_BUF_TOPB) and OCBn (bit n) awaiting the overflow. */
    uint32_t timer_buf_pending[5];
    /* WDOG0: counter epoch (virtual ns) while counting, see efr32_wdog_*. */
    bool wdog_running;
    int64_t wdog_epoch_ns;
    QEMUTimer *wdog_timer;
    /* The warning of the current period has fired. */
    bool wdog_warned;
    /* Frozen (debug halt with DEBUGRUN = 0) since wdog_frozen_ns. */
    bool wdog_frozen;
    int64_t wdog_frozen_ns;
    bool m33_debug_halted;
    /* A CMD write is synchronising to the watchdog clock (SYNCBUSY.CMD). */
    bool wdog_sync_pending;
    uint32_t wdog_cmd_pending;
    QEMUTimer *wdog_sync_timer;
    qemu_irq irq[64];
    qemu_irq seq_irq_state[EFR32_RAC_SEQ_STATE_EVENTS];
    qemu_irq seq_irq_mailbox;
    CPUState *seq_cpu;
    /* Per-CPU clocks: the M33, and both cores' SysTick processor clocks. */
    CPUState *m33_cpu;
    Clock *m33_cpuclk;
    Clock *seq_cpuclk;
    /* PCLK = HCLK / (PCLKPRESC + 1): the USARTs' clock. */
    Clock *pclk;
    IcountBusModel bus_model;
    bool seq_released;
    bool synth_running;
    /* A sequencer-driven TX reached completion with the synthesizer off. */
    bool tx_waiting_for_synth;
    /* RAC.PAENCTRL.PARAMP changed; TXRAMPDONE latches at pa_ramp_done_ns. */
    bool pa_ramp_pending;
    int64_t pa_ramp_done_ns;
    /* SYNTH.CMD.ENABLEIF in force; INLOCK reported from synth_lock_ns. */
    bool synth_if_enabled;
    int64_t synth_lock_ns;
    /* SYNTH.LOCNTSTATUS.BUSY until this time (LOCNTCTRL.RUN). */
    int64_t locnt_busy_until_ns;
    /* A LO count running or ended but not yet read (efr32_synth_locnt_run). */
    bool locnt_pending;
    /* The count counts: the synthesizer stopped, or LOCNTCTRL.FCALRUNCLKEN. */
    bool locnt_counts;
    /* LO cycles of the running count up to locnt_acc_ns. */
    double locnt_acc_cycles;
    int64_t locnt_acc_ns;
    /* The count's start and its exact length, 2^(NUMCYCLE + 1) HFXO periods. */
    int64_t locnt_run_ns;
    double locnt_window_s;
    /* The closed-loop LO trajectory (efr32_synth_lo_closed_hz). */
    double lo_from_hz;
    double lo_to_hz;
    int64_t lo_change_ns;
    bool lo_tx_loop;
    /* A latched TX request: RAC.STATUS.TXENS. */
    bool tx_requested;
    /* RXDIS seen in RXSEARCH, applied once STATERXSEARCH is clear. */
    bool rx_disable_requested;
    /* EOF has passed; rac_frame_timer next fires for the RXFRAME exit. */
    bool rx_frame_ended;
    /* A frame-driven exit waiting for the current state's own event. */
    bool rac_transition_held;
    uint32_t rac_held_state;
    /* End of the frame captured at TX entry: when TX is left. */
    int64_t tx_end_ns;
    /* Receive timing of rx_frame: sync (RXFRAME entry) and frame end. */
    int64_t rx_sync_ns;
    int64_t rx_eof_ns;
    /* Virtual time of the last RAC state change; RAC.STIMER counts from it. */
    int64_t rac_stimer_epoch_ns;
    /* Frame being received in RXFRAME; written to BUFC at frame end. */
    GBytes *rx_frame;
    QEMUTimer *rac_frame_timer;
    QEMUTimer *rac_ack_timer;
};

static uint32_t efr32_register_get(Efr32Periph *s, uint32_t address);
static void efr32_register_put(Efr32Periph *s, uint32_t address,
                               uint32_t value);

/*
 * PRS producers (RM SS13.3.3): a level, or a pulse (a raise and a lower at
 * the same instant; see efr32-prs.c for why that width suffices).
 */
static void efr32_prs_level(Efr32Periph *s, unsigned source, unsigned signal,
                            bool level)
{
    qemu_set_irq(s->prs_async[EFR32_PRS_LINE(source, signal)], level);
}

static void efr32_prs_pulse(Efr32Periph *s, unsigned source, unsigned signal)
{
    qemu_irq_pulse(s->prs_async[EFR32_PRS_LINE(source, signal)]);
}

static void efr32_prs_sync_pulse(Efr32Periph *s, unsigned source,
                                 unsigned signal)
{
    qemu_irq_pulse(s->prs_sync[EFR32_PRS_LINE(source, signal)]);
}

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
    {0x0c0, 0xffffffff}, {0x0c4, 0xff800fff}, {0x0cc, 0x00ffffff},
    {0x0e0, 0x000001ff}, {0x0e4, 0x7fffffff}, {0x0ec, 0xeffff8ff}, {0x0f4, 0xffffffff}, {0x0f8, 0x7fffffff},
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

/*
 * Every entry below 0x094 was already here, derived and verified before
 * this list existed. Everything from 0x094 on was added in one pass from
 * Silicon Labs' own public RAC_S_register.py (build/reference-models/,
 * fetched directly off github.com/SiliconLabs/simplicity_sdk) -- every
 * writable RAC register this project had not yet modeled at all, each
 * masked to its documented writable-bits field, none given any simulated
 * *behavior* yet (per the project's own standing rule against asserting
 * meaning this project hasn't verified). This only ever narrows what a
 * guest write can set, matching real silicon's own documented write
 * masks; it cannot change any existing test's outcome for a register
 * that was already behaving as an unmasked generic latch, except to make
 * out-of-spec bit patterns silently drop the same way real hardware does.
 */
static const Efr32RegisterMask efr32_rac_masks[] = {
    {0x004, 0x00000001}, {0x008, 0x00003fff}, {0x010, 0xc000fdff},
    {0x014, 0x1f0107ef}, {0x018, 0x0000000f}, {0x01c, 0x00ff000f},
    {0x020, 0x00ff000f}, {0x024, 0x00000003}, {0x028, 0x3fff000f},
    {0x02c, 0x3fff000f}, {0x034, 0x0000ffff}, {0x038, 0x1f00007f},
    {0x03c, 0x0000007f}, {0x040, 0xffffffff}, {0x044, 0xffffffff},
    {0x048, 0xffffffff}, {0x04c, 0xffffffff}, {0x050, 0x01ffffff},
    {0x054, 0x000000ff}, {0x05c, 0x00070033}, {0x094, 0x00100282},
    {0x098, 0x07001c00}, {0x09c, 0x000000ff}, {0x0a0, 0x00004000},
    {0x0a4, 0x00000400}, {0x0b0, 0x0ff80000}, {0x0b4, 0x10010100},
    {0x0b8, 0xff000004}, {0x0bc, 0x1fffffff}, {0x0c0, 0x000003ff},
    {0x0c4, 0x00003fff}, {0x0c8, 0xf31f0fff}, {0x0d0, 0xffdfffff},
    {0x0d4, 0x0001fdef}, {0x0d8, 0x00007fff}, {0x0e0, 0x0000003f},
    {0x0e4, 0x3fffffff}, {0x0e8, 0x000001ff}, {0x0ec, 0x00ff1f03},
    {0x0f4, 0x00000001}, {0x0f8, 0x1fffffff}, {0x0fc, 0x0007ffff},
    {0x104, 0x00000077}, {0x108, 0x00000001}, {0x10c, 0x0000003f},
    {0x110, 0x00003fff}, {0x114, 0x0007ffff}, {0x118, 0x00003fff},
    {0x11c, 0x007e7fff}, {0x120, 0x1fff033f}, {0x124, 0x00000007},
    {0x128, 0x000007ff}, {0x12c, 0x3f3f3f3f}, {0x130, 0x07fffeef},
    {0x134, 0x3f3f3f3f}, {0x138, 0x000f001f}, {0x13c, 0x00000007},
    {0x140, 0x0000001e}, {0x144, 0x00000fff}, {0x148, 0xe3cb00ff},
    {0x14c, 0x00000177}, {0x150, 0x003fefff}, {0x154, 0xe003ffff},
    {0x158, 0x03018700}, {0x15c, 0x00007fff}, {0x160, 0x80000663},
    {0x168, 0x00000e07}, {0x16c, 0x00000777}, {0x174, 0x00000777},
    {0x17c, 0x00000003}, {0x180, 0xfc07fbff}, {0x188, 0x03f0ffff},
    {0x3e0, 0xffffffff}, {0x3e4, 0xffffffff}, {0x3e8, 0xffffffff},
    {0x3ec, 0xffffffff}, {0x3f0, 0xffffffff}, {0x3f4, 0xffffffff},
    {0x3f8, 0xffffffff}, {0x3fc, 0xffffffff}, {0x7e8, 0x00000003},
    {0x7ec, 0x00000002}, {0x7f0, 0x00007f3f}, {0x7f4, 0x00000020},
    {0x7f8, 0xffc301ff}, {0x7fc, 0x0000ff3f},
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
 * backing store. The current RF-medium seam uses this path only for raw
 * channel-1 bytes and deliberately does not construct a stock RX descriptor.
 * Keeping it separate from MMIO dispatch gives a future evidenced producer the
 * same circular-buffer and guest-RAM behavior as sequencer programmed I/O.
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

/*
 * A normal channel-2 descriptor producer is part of the radio/sequencer
 * hardware and is not modeled here.  The in-process RF medium has a
 * simulator-only convention, however: when the guest has configured channel
 * 2 with both a valid FIFO size and backing address, it may publish one
 * received-packet record after the synthetic channel-1 handoff.  Keeping the
 * check here prevents an unconfigured guest from acquiring a descriptor as a
 * side effect of the raw-byte receive seam.
 */
static bool efr32_bufc_descriptor_channel_configured(Efr32Periph *s)
{
    uint32_t base = efr32_bufc_channel_base(
        EFR32_RF_RX_DESCRIPTOR_BUFC_CHANNEL);

    return efr32_bufc_capacity(s, EFR32_RF_RX_DESCRIPTOR_BUFC_CHANNEL) &&
           efr32_register_get(s, base + EFR32_BUFC_ADDR) != 0;
}

static void efr32_bufc_mark_descriptor_overflow(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF);

    efr32_register_put(s, EFR32_BUFC_BASE + EFR32_BUFC_IF,
                       pending | EFR32_BUFC_DESCRIPTOR_OVERFLOW);
    efr32_bufc_update_irq(s);
}

static bool efr32_bufc_descriptor_has_room(Efr32Periph *s)
{
    uint32_t capacity = efr32_bufc_capacity(
        s, EFR32_RF_RX_DESCRIPTOR_BUFC_CHANNEL);

    return capacity >= 8U &&
           efr32_bufc_count(s, EFR32_RF_RX_DESCRIPTOR_BUFC_CHANNEL) <=
               capacity - 8U;
}

/*
 * Publish the QEMU-only successful-RX record after channel 1 has received the
 * synthetic payload.  The endpoint is the channel-1 write offset, encoded as
 * the low 13 bits in little-endian order.  The class byte (0x06) is the
 * archive-observed received-packet class; all remaining bytes are reserved
 * and remain zero.  Each data-port write advances channel 2 and updates its
 * configured threshold before the caller raises any completion event.
 */
static bool efr32_bufc_enqueue_synthetic_rx_descriptor(Efr32Periph *s)
{
    uint32_t rx_base = efr32_bufc_channel_base(EFR32_RF_RX_BUFC_CHANNEL);
    uint32_t endpoint = efr32_register_get(
        s, rx_base + EFR32_BUFC_WRITEOFFSET) & 0x1fff;
    uint8_t record[8] = {
        endpoint & 0xff,
        (endpoint >> 8) & 0x1f,
        0x06,
        0,
        0,
        0,
        0,
        0,
    };

    if (!efr32_bufc_descriptor_channel_configured(s)) {
        return false;
    }

    /* Never overwrite an unconsumed synthetic record in the backing ring. */
    if (!efr32_bufc_descriptor_has_room(s)) {
        efr32_bufc_mark_descriptor_overflow(s);
        return false;
    }

    for (size_t index = 0; index < sizeof(record); index++) {
        efr32_bufc_write_data_port(s, EFR32_RF_RX_DESCRIPTOR_BUFC_CHANNEL,
                                   record[index], false);
    }
    return true;
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

/* Public revB2 IPVERSION reset values; these registers are read-only. */
static bool efr32_radio_ipversion(uint32_t address, uint32_t *value)
{
    switch (address) {
    case EFR32_FRC_BASE + EFR32_RADIO_IPVERSION:
    case EFR32_AGC_BASE + EFR32_RADIO_IPVERSION:
    case EFR32_MODEM_BASE + EFR32_RADIO_IPVERSION:
    case EFR32_SYNTH_BASE + EFR32_RADIO_IPVERSION:
    case EFR32_RAC_BASE + EFR32_RADIO_IPVERSION:
        *value = 1;
        return true;
    case EFR32_RFCRC_BASE + EFR32_RADIO_IPVERSION:
        *value = 0;
        return true;
    default:
        return false;
    }
}

/*
 * FRC.STATUS is hardware read-only (public revB2 FRC_S_register.py); no
 * modelled command sets or clears any of its bits. Reject writes so the
 * model cannot drift into a software-set value it would never reach on
 * real silicon, and mask reads to the public bit width.
 */
static bool efr32_frc_status_write_ignored(uint32_t address)
{
    return address == EFR32_FRC_BASE + EFR32_FRC_STATUS ||
           address == EFR32_SYNTH_BASE + EFR32_SYNTH_STATUS ||
           address == EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTSTATUS ||
           address == EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTTARGET;
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

/* The CMU CLKEN1 bit that gates a radio block (see efr32_bus_error). */
static uint32_t efr32_radio_bus_clock(uint32_t address)
{
    switch (address & ~0x3fffu) {
    case EFR32_FRC_BASE:
        return EFR32_CMU_CLKEN1_FRC;
    case EFR32_AGC_BASE:
        return EFR32_CMU_CLKEN1_AGC;
    case EFR32_RFCRC_BASE:
        return EFR32_CMU_CLKEN1_RFCRC;
    case EFR32_MODEM_BASE:
        return EFR32_CMU_CLKEN1_MODEM;
    case EFR32_SYNTH_BASE:
        return EFR32_CMU_CLKEN1_SYNTH;
    case EFR32_PROTIMER_BASE:
        return EFR32_CMU_CLKEN1_PROTIMER;
    case EFR32_RAC_BASE:
        return EFR32_CMU_CLKEN1_RAC;
    case EFR32_PRORTC_BASE:
        return EFR32_CMU_CLKEN1_PRORTC;
    case EFR32_RDSCRATCHPAD_BASE:
        return EFR32_CMU_CLKEN1_RDSCRATCHPAD;
    case EFR32_RDMAILBOX0_BASE:
        return EFR32_CMU_CLKEN1_RDMAILBOX0;
    case EFR32_RDMAILBOX1_BASE:
        return EFR32_CMU_CLKEN1_RDMAILBOX1;
    default:
        return 0;
    }
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

/*
 * The M33's two RAC lines both come from RAC.IF & RAC.IEN (public
 * RAC_S_field.py: IF bits 0-3 STATECHANGE, STIMCMPEV, SEQLOCKUP,
 * SEQRESETREQ; bits 16-23 SEQ, the sequencer's flags to the host).
 * RAC_RSM (IRQ 37) carries the first group: stock RAIL's RAC_RSM handler
 * masks IF with IEN and acts on STATECHANGE (docs/radio-tables.md, RAIL
 * archive).  RAC_SEQ (IRQ 38) carries IF.SEQ: the stock suit application
 * enables only IEN bit 18 (IF.SEQ[2]) and its IRQ 38 handler services
 * RAC.IF & RAC.IEN.  SEQIF/SEQIEN are the sequencer's own flags (to the
 * M0+, below): stock runs on silicon with SEQIEN = 0x3FFF0000 and IRQ 38
 * enabled, so they cannot drive the M33's line.
 */
#define EFR32_RAC_IF_RSM_MASK 0x0000000fu
#define EFR32_RAC_IF_SEQ_MASK 0x00ff0000u

static void efr32_rac_update_irqs(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IF) &
                       efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IEN);

    qemu_set_irq(s->irq[37], !!(pending & EFR32_RAC_IF_RSM_MASK));
    qemu_set_irq(s->irq[38], !!(pending & EFR32_RAC_IF_SEQ_MASK));
    for (unsigned state = 0; state < EFR32_RAC_SEQ_STATE_EVENTS; state++) {
        if (s->seq_irq_state[state]) {
            qemu_set_irq(s->seq_irq_state[state],
                         !!(efr32_register_get(s, EFR32_RAC_BASE +
                                                  EFR32_RAC_SEQIF) &
                            efr32_rac_seq_state_flag(state)));
        }
    }
}

static void efr32_rdmailbox_update_irqs(Efr32Periph *s)
{
    uint32_t to_m33 =
        efr32_register_get(s, EFR32_RFMAILBOX_B_BASE + EFR32_RFMAILBOX_FLAGS) &
        efr32_register_get(s, EFR32_RFMAILBOX_B_BASE + EFR32_RFMAILBOX_IEN);
    uint32_t to_seq =
        efr32_register_get(s, EFR32_RFMAILBOX_C_BASE + EFR32_RFMAILBOX_FLAGS) &
        efr32_register_get(s, EFR32_RFMAILBOX_C_BASE + EFR32_RFMAILBOX_IEN);

    qemu_set_irq(s->irq[EFR32_RDMAILBOX_M33_IRQ],
                 !!(to_m33 & EFR32_RFMAILBOX_IF_MASK));
    if (s->seq_irq_mailbox) {
        qemu_set_irq(s->seq_irq_mailbox, !!(to_seq & EFR32_RFMAILBOX_IF_MASK));
    }
}

static void efr32_rac_prs_state(Efr32Periph *s, uint32_t state);

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
    s->rac_stimer_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    efr32_register_put(s, EFR32_RAC_BASE + EFR32_RAC_IF,
                       efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IF) |
                           1);
    /*
     * Every state, SHUTDOWN included, latches its own event (SS7.69), except
     * RXOVERFLOW: on silicon it showed SEQACTIVE clear and the open
     * sequencer's IRQ6 never ran (SS7.80).
     */
    sequence_flag = state == EFR32_RAC_STATE_RXOVERFLOW
                        ? 0 : efr32_rac_seq_state_flag(state);
    efr32_register_put(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF,
                       efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF) |
                           sequence_flag | 1u);
    efr32_rac_update_irqs(s);
    efr32_rac_prs_state(s, state);
    if (state == EFR32_RAC_STATE_RXSEARCH) {
        /*
         * RXSEARCH can begin after a peer's TXEN but before that model
         * transaction completes. Offer that still-active snapshot now; the
         * medium rejects completed transactions and duplicate offers.
         */
        efr32_rf_medium_offer_active_tx_to_receiver(
            s->rf_medium, EFR32_RF_RADIO(s), &error_abort);
    }
    trace_efr32bg22_rac_state(s->endpoint_id,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                              previous_state, state);
}

/*
 * Radio PRS producers (RM Table 13.5) at the instants the model already
 * has, the same anchors that silicon timed through these very signals
 * (SS7.71, SS7.72; see the frame timing anchors above):
 *  - RACL.ACTIVE: the RAC is out of OFF ([I] SS7.71: its rise marks the
 *    TXEN write, the RAC leaving OFF; RACL.ACTIVE -> RACL.RX rise is the
 *    166.6 us RX ready delay, SWRXEN to RXSEARCH, SS7.79/7.82);
 *  - RACL.RX: RXSEARCH or RXFRAME (rises at RXSEARCH entry, falls
 *    EFR32_RAC_EOF_TO_RX_EXIT_NS after EOF as RXFRAME is left);
 *  - RACL.TX: the TX state (SYNCSENT 40.65 us after its rise, its fall
 *    0.9 us after FRAMESENT);
 *  - [I] RACL.LNAEN: RXWARM to RXFRAME and RX2RX; RACL.PAEN: TX;
 *  - MODEMH.SYNCSENT (pulse) as the access address ends,
 *    EFR32_RAC_TX_ENTRY_TO_SYNCSENT_NS after TX entry; MODEM.FRAMESENT
 *    (pulse) with the frame's last bit; MODEML.FRAMEDET (pulse) at RXFRAME
 *    entry; MODEMH.EOF (pulse) at the received frame's end.
 * The TX pulses are timed only while a PRS channel selects them, so a
 * firmware that never routes them sees no extra timer events.
 * [U] The other radio producers (AGC, BUFC, FRC, PROTIMER, SYNTH, MODEM
 * signals not listed) are not driven.
 */
static bool efr32_rac_rx_level(uint32_t state)
{
    return state == EFR32_RAC_STATE_RXSEARCH ||
           state == EFR32_RAC_STATE_RXFRAME;
}

static bool efr32_rac_lna_level(uint32_t state)
{
    return state == EFR32_RAC_STATE_RXWARM ||
           state == EFR32_RAC_STATE_RXSEARCH ||
           state == EFR32_RAC_STATE_RXFRAME ||
           state == EFR32_RAC_STATE_RX2RX;
}

static void efr32_rac_prs_state(Efr32Periph *s, uint32_t state)
{
    efr32_prs_level(s, EFR32_PRS_SRC_RACL, EFR32_PRS_SIG_RACL_ACTIVE,
                    state != EFR32_RAC_STATE_OFF);
    efr32_prs_level(s, EFR32_PRS_SRC_RACL, EFR32_PRS_SIG_RACL_RX,
                    efr32_rac_rx_level(state));
    efr32_prs_level(s, EFR32_PRS_SRC_RACL, EFR32_PRS_SIG_RACL_TX,
                    state == EFR32_RAC_STATE_TX);
    efr32_prs_level(s, EFR32_PRS_SRC_RACL, EFR32_PRS_SIG_RACL_LNAEN,
                    efr32_rac_lna_level(state));
    efr32_prs_level(s, EFR32_PRS_SRC_RACL, EFR32_PRS_SIG_RACL_PAEN,
                    state == EFR32_RAC_STATE_TX);
}

static void efr32_rac_prs_arm(Efr32Periph *s)
{
    int64_t next = MIN(s->prs_syncsent_ns, s->prs_framesent_ns);

    if (next == INT64_MAX) {
        timer_del(s->prs_radio_timer);
    } else {
        timer_mod(s->prs_radio_timer, next);
    }
}

static void efr32_rac_prs_expired(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->prs_syncsent_ns <= now) {
        s->prs_syncsent_ns = INT64_MAX;
        efr32_prs_pulse(s, EFR32_PRS_SRC_MODEMH, EFR32_PRS_SIG_MODEMH_SYNCSENT);
    }
    if (s->prs_framesent_ns <= now) {
        s->prs_framesent_ns = INT64_MAX;
        efr32_prs_pulse(s, EFR32_PRS_SRC_MODEM, EFR32_PRS_SIG_MODEM_FRAMESENT);
    }
    efr32_rac_prs_arm(s);
}

/* At TX entry, with s->tx_end_ns known. */
static void efr32_rac_prs_tx_events(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->prs_syncsent_ns = s->prs &&
        efr32_prs_selected(s->prs, EFR32_PRS_SRC_MODEMH,
                           EFR32_PRS_SIG_MODEMH_SYNCSENT)
        ? now + EFR32_RAC_TX_ENTRY_TO_SYNCSENT_NS : INT64_MAX;
    s->prs_framesent_ns = s->prs &&
        efr32_prs_selected(s->prs, EFR32_PRS_SRC_MODEM,
                           EFR32_PRS_SIG_MODEM_FRAMESENT)
        ? s->tx_end_ns - EFR32_RAC_FRAMESENT_TO_TX_EXIT_NS : INT64_MAX;
    efr32_rac_prs_arm(s);
}

static void efr32_rac_capture_tx_snapshot(Efr32Periph *s);
static void efr32_rac_schedule_tx_completion(Efr32Periph *s);
static void efr32_rac_cancel_tx_completion(Efr32Periph *s);
static void efr32_rac_schedule_rx_handoff(Efr32Periph *s);

static bool efr32_rac_swrxen(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_RXENSRCEN) &
           EFR32_RAC_RXENSRCEN_SWRXEN;
}

static bool efr32_rac_force_disabled(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_CTRL) &
           EFR32_RAC_CTRL_FORCEDISABLE;
}

/* The current state's own SEQIF event is still uncleared. */
static bool efr32_rac_own_event_pending(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF) &
           efr32_rac_seq_state_flag(s->rac_state);
}

/* Enter @state, with the work silicon performs on that entry. */
static void efr32_rac_enter_state(Efr32Periph *s, uint32_t state)
{
    if (s->rac_state == EFR32_RAC_STATE_RXSEARCH) {
        s->rx_disable_requested = false;
    }
    efr32_rac_set_state(s, state);
    if (state == EFR32_RAC_STATE_TX) {
        /*
         * TXENS drops on TX entry (STATUS 0x19400000 in TX, SS7.72).  The
         * completion timer is armed last: from a timer callback with every
         * vCPU idle, -icount sleep=off advances virtual time to the new
         * deadline as soon as it is armed.
         */
        s->tx_requested = false;
        efr32_rac_capture_tx_snapshot(s);
        efr32_rac_schedule_tx_completion(s);
    }
    if (state == EFR32_RAC_STATE_RXSEARCH && s->rx_offer_pending) {
        efr32_rac_schedule_rx_handoff(s);
    }
}

/*
 * The released-sequencer RAC state machine, from debugger-driven runs on
 * silicon with the M0+ parked in WFI and its NVIC masked, so that the
 * debugger alone cleared state events (owo-open-fw docs/radio-tables.md
 * SS7.72).  Each state is left only once its own SEQIF event is cleared,
 * whether or not SEQIEN enables it -- except OFF, which TXEN or SWRXEN leave
 * at once with STATEOFF still pending, and TX, which TXDIS leaves at once
 * (handled by the command).  Measured routes:
 *   OFF       -> TXWARM on a TX request, else RXWARM on SWRXEN, unless
 *                FORCEDISABLE is set;
 *   RXWARM    -> RXSEARCH, even if SWRXEN was withdrawn meanwhile;
 *   RXSEARCH  -> RXPD when SWRXEN is withdrawn or after RXDIS;
 *   TXWARM    -> TX, or TXPD when TXDIS withdrew the request;
 *   TX2RX, RX2RX -> RXSEARCH;  RXPD, TXPD, SHUTDOWN -> OFF;
 *   RX2TX     -> TX, or TXPD after the request was withdrawn (counted on
 *                running stage 3: TXPD entries = responses + cancels).
 * Exits driven by the frame (TX end, RX frame end, RXFRAME entry) are kept in
 * rac_held_state and obey the same own-event rule ([I] for those states:
 * their holds were not observable from the debugger).
 */
static void efr32_rac_seq_evaluate(Efr32Periph *s)
{
    for (unsigned step = 0; step < EFR32_RAC_STATE_COUNT; step++) {
        uint32_t state = s->rac_state;
        uint32_t next = state;
        bool own = efr32_rac_own_event_pending(s);

        if (s->rac_transition_held) {
            if (!own) {
                s->rac_transition_held = false;
                next = s->rac_held_state;
            }
        } else {
            switch (state) {
            case EFR32_RAC_STATE_OFF:
                if (!efr32_rac_force_disabled(s)) {
                    if (s->tx_requested) {
                        next = EFR32_RAC_STATE_TXWARM;
                    } else if (efr32_rac_swrxen(s)) {
                        next = EFR32_RAC_STATE_RXWARM;
                    }
                }
                break;
            case EFR32_RAC_STATE_RXWARM:
            case EFR32_RAC_STATE_TX2RX:
            case EFR32_RAC_STATE_RX2RX:
                next = own ? state : EFR32_RAC_STATE_RXSEARCH;
                break;
            case EFR32_RAC_STATE_RXSEARCH:
                if (!own && (!efr32_rac_swrxen(s) || s->rx_disable_requested)) {
                    next = EFR32_RAC_STATE_RXPD;
                }
                break;
            case EFR32_RAC_STATE_TXWARM:
            case EFR32_RAC_STATE_RX2TX:
                if (!own) {
                    next = s->tx_requested ? EFR32_RAC_STATE_TX
                                           : EFR32_RAC_STATE_TXPD;
                }
                break;
            case EFR32_RAC_STATE_RXPD:
            case EFR32_RAC_STATE_TXPD:
            case EFR32_RAC_STATE_SHUTDOWN:
                next = own ? state : EFR32_RAC_STATE_OFF;
                break;
            default:
                break;
            }
        }
        if (next == state) {
            return;
        }
        efr32_rac_enter_state(s, next);
    }
}

/*
 * A cleared state event releases the transition ~1.09 us later: measured on
 * running stage 3 as the M0+ SysTick time from its SEQIF clear to a changed
 * RAC.STATUS, identical for TX2RX, RXPD, TXPD and RX2TX (n > 100 each,
 * 1.09-1.22 us including the polling step; SS7.72).
 */
static void efr32_rac_seq_ack_timer(void *opaque)
{
    efr32_rac_seq_evaluate(EFR32_PERIPH(opaque));
}

static void efr32_rac_seq_acknowledged(Efr32Periph *s)
{
    if (!timer_pending(s->rac_ack_timer)) {
        timer_mod(s->rac_ack_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                        EFR32_RAC_ACK_TO_TRANSITION_NS);
    }
}

/* A frame-driven exit: applied once the current state's event is clear. */
static void efr32_rac_hardware_transition(Efr32Periph *s, uint32_t state)
{
    s->rac_transition_held = true;
    s->rac_held_state = state;
    efr32_rac_seq_evaluate(s);
}

/* Forget receive and turnaround work in progress (abort or reset). */
static void efr32_rac_cancel_turnaround(Efr32Periph *s)
{
    timer_del(s->rac_frame_timer);
    timer_del(s->rac_ack_timer);
    g_clear_pointer(&s->rx_frame, g_bytes_unref);
    s->rx_frame_ended = false;
    s->tx_requested = false;
    s->rx_disable_requested = false;
    s->rac_transition_held = false;
}

/* TX state exit, EFR32_RAC_FRAMESENT_TO_TX_EXIT_NS after the last bit. */
static void efr32_rac_tx_complete(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    /* Images that never release sequencer firmware keep the older QEMU-only
     * shortcut; a released sequencer must start the synthesizer itself. */
    if (s->seq_released && !s->synth_running) {
        s->tx_waiting_for_synth = true;
        return;
    }
    efr32_rf_medium_complete_tx(s->rf_medium, EFR32_RF_RADIO(s));
    if (s->seq_released) {
        /*
         * An RX request asserted during TX turns the RAC around (SS7.63).
         * [I] A TXEN latched during TX is dropped here; silicon's response
         * to it (TX2TX?) was not observed.
         */
        s->tx_requested = false;
        efr32_rac_hardware_transition(s, efr32_rac_swrxen(s)
                                             ? EFR32_RAC_STATE_TX2RX
                                             : EFR32_RAC_STATE_TXPD);
    } else {
        efr32_rac_set_state(s, EFR32_RAC_STATE_OFF);
    }
    efr32_frc_raise(s, EFR32_FRC_IF_TXDONE);
}

/*
 * The demodulator's sample rate follows the channel (owo-open-fw
 * docs/radio-tables.md SS7.83, silicon).  The IF ADC samples at f_LO / 2^9
 * with RAC.IFADCTRIM0.IFADCENHALFMODE (1M PHY) and f_LO / 2^8 without, f_LO =
 * f_RF + f_IF on the high side (SYNTH.IFFREQ.LOSIDE), and MODEM.SRCCHF
 * converts that to the demodulator's fixed rate: SRCRATIO1 / 2^7 (when
 * SRCENABLE1) x SRCRATIO2 / 2^14 = f_ADC / f_SRC_OUT, with f_SRC_OUT 4 MHz in
 * half mode and 8 MHz otherwise (what each PHY table's own 2402 MHz SRCCHF
 * gives).  Stock recomputes SRCCHF for every channel; with the 2402 MHz value
 * left on 2426 and 2480 MHz (1 % and 3.2 % rate error) silicon decoded the
 * header and failed the CRC of nearly every frame (ch38 0/48, ch39 0/6 frames
 * from a close transmitter; 2402 MHz 42/49).  [I] Modelled as a timing
 * drift: a frame whose accumulated drift over its 8 x (octets + 3 CRC octets)
 * symbols exceeds half a symbol is delivered with CRCOK clear (the bytes
 * themselves are kept).  The threshold is inferred; the model does not model
 * the demodulator's timing tracking, the frequency error of a wrong
 * MODEM.DIGMIXCTRL, or Coded symbols.  An SRCCHF without SRCENABLE2 (reset
 * value, register-level tests) is not checked.
 */
#define EFR32_SYNTH_IFFREQ 0x038u
#define EFR32_SYNTH_IFFREQ_IFFREQ_MASK 0x000fffffu
#define EFR32_SYNTH_IFFREQ_LOSIDE (1u << 20)
#define EFR32_RAC_IFADCTRIM0 0x0e4u
#define EFR32_RAC_IFADCTRIM0_IFADCENHALFMODE (1u << 1)
#define EFR32_MODEM_SRCCHF 0x0ecu
#define EFR32_MODEM_SRCCHF_SRCRATIO1_MASK 0xffu
#define EFR32_MODEM_SRCCHF_SRCENABLE1 (1u << 11)
#define EFR32_MODEM_SRCCHF_SRCRATIO2_SHIFT 12
#define EFR32_MODEM_SRCCHF_SRCRATIO2_MASK 0x7fffu
#define EFR32_MODEM_SRCCHF_SRCENABLE2 (1u << 27)
#define EFR32_MODEM_SRCRATIO1_ONE (1u << 7)
#define EFR32_MODEM_SRCRATIO2_ONE (1u << 14)
#define EFR32_IFADC_LO_DIVIDER_HALF_MODE 512.0
#define EFR32_IFADC_LO_DIVIDER_FULL 256.0
#define EFR32_MODEM_SRC_OUT_HZ_HALF_MODE 4e6
#define EFR32_MODEM_SRC_OUT_HZ_FULL 8e6
#define EFR32_RF_CRC_OCTETS 3
#define EFR32_RX_DRIFT_LIMIT_SYMBOLS 0.5

static bool efr32_rx_demod_rate_drift_fails(Efr32Periph *s, gsize length)
{
    uint32_t srcchf = efr32_register_get(s, EFR32_MODEM_BASE + EFR32_MODEM_SRCCHF);
    uint32_t iffreq_reg = efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_IFFREQ);
    uint64_t freq = efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_FREQ) &
                    EFR32_SYNTH_FREQ_MASK;
    uint64_t iffreq = iffreq_reg & EFR32_SYNTH_IFFREQ_IFFREQ_MASK;
    bool half = efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IFADCTRIM0) &
                EFR32_RAC_IFADCTRIM0_IFADCENHALFMODE;
    double lo_hz, adc_hz, expected, ratio, error;

    if (!(srcchf & EFR32_MODEM_SRCCHF_SRCENABLE2) || freq == 0) {
        return false;
    }
    lo_hz = (double)((iffreq_reg & EFR32_SYNTH_IFFREQ_LOSIDE) ? freq + iffreq
                                                               : freq - iffreq) *
            EFR32_HFXO_HZ / (double)(1u << EFR32_SYNTH_FREQ_FRACTION_BITS);
    adc_hz = lo_hz / (half ? EFR32_IFADC_LO_DIVIDER_HALF_MODE
                           : EFR32_IFADC_LO_DIVIDER_FULL);
    expected = adc_hz / (half ? EFR32_MODEM_SRC_OUT_HZ_HALF_MODE
                              : EFR32_MODEM_SRC_OUT_HZ_FULL);
    ratio = (double)((srcchf >> EFR32_MODEM_SRCCHF_SRCRATIO2_SHIFT) &
                     EFR32_MODEM_SRCCHF_SRCRATIO2_MASK) /
            EFR32_MODEM_SRCRATIO2_ONE;
    if (srcchf & EFR32_MODEM_SRCCHF_SRCENABLE1) {
        ratio *= (double)(srcchf & EFR32_MODEM_SRCCHF_SRCRATIO1_MASK) /
                 EFR32_MODEM_SRCRATIO1_ONE;
    }
    error = fabs(ratio / expected - 1.0);
    return error * 8.0 * (double)(length + EFR32_RF_CRC_OCTETS) >
           EFR32_RX_DRIFT_LIMIT_SYMBOLS;
}

/*
 * A received frame, in three timed steps (efr32_rac_rx_handoff sets the
 * times): RXFRAME entry at frame detection, EOF -- FRC has written header,
 * length, payload and the TRAILRXDATA-selected trailer to BUFC RX channel 1,
 * the layout measured on silicon (SS7.66), and raises RXDONE -- and the RAC's
 * exit from RXFRAME EFR32_RAC_EOF_TO_RX_EXIT_NS later.  The exit goes to
 * RX2RX while an RX source is asserted, to RX2TX when TXEN was latched with
 * SWRXEN withdrawn (SS7.65), and otherwise to RXPD.  [I] RXPD for a frame end
 * with neither request, and dropping a TXEN latched with SWRXEN asserted,
 * are inferred, not measured.  [I] RXDONE is raised at EOF.
 */
static void efr32_rac_rx_frame_end(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    const uint8_t *data;
    gsize length;
    uint32_t next_state;

    if (s->rx_frame_ended) {
        s->rx_frame_ended = false;
        if (s->rac_state != EFR32_RAC_STATE_RXFRAME) {
            return;
        }
        if (efr32_rac_swrxen(s)) {
            s->tx_requested = false;
            next_state = EFR32_RAC_STATE_RX2RX;
        } else if (s->tx_requested) {
            next_state = EFR32_RAC_STATE_RX2TX;
        } else {
            next_state = EFR32_RAC_STATE_RXPD;
        }
        efr32_rac_hardware_transition(s, next_state);
        return;
    }
    if (!s->rx_frame) {
        return;
    }
    if (now < s->rx_eof_ns) {
        /* The frame has been detected: RXFRAME until EOF. */
        if (s->rac_state == EFR32_RAC_STATE_RXSEARCH) {
            efr32_rac_hardware_transition(s, EFR32_RAC_STATE_RXFRAME);
            efr32_prs_pulse(s, EFR32_PRS_SRC_MODEML,
                            EFR32_PRS_SIG_MODEML_FRAMEDET);
        }
        timer_mod(s->rac_frame_timer, s->rx_eof_ns);
        return;
    }
    if (s->rac_state != EFR32_RAC_STATE_RXFRAME) {
        g_clear_pointer(&s->rx_frame, g_bytes_unref);
        return;
    }

    efr32_prs_pulse(s, EFR32_PRS_SRC_MODEMH, EFR32_PRS_SIG_MODEMH_EOF);
    data = g_bytes_get_data(s->rx_frame, &length);
    if (efr32_bufc_capacity(s, EFR32_RF_RX_BUFC_CHANNEL) &&
        efr32_bufc_count(s, EFR32_RF_RX_BUFC_CHANNEL) + length +
                ((efr32_register_get(s, EFR32_FRC_BASE + EFR32_FRC_TRAILRXDATA) &
                  EFR32_FRC_TRAILRXDATA_CRCOK) ? 1 : 0) >
            efr32_bufc_capacity(s, EFR32_RF_RX_BUFC_CHANNEL)) {
        /*
         * RX FIFO overflow, measured on silicon (owo-open-fw
         * docs/radio-tables.md SS7.80): received frames filled the 512-octet
         * channel-1 FIFO (WRITEOFFSET 512, READOFFSET 0), BUFC.IF read 0x100
         * (channel 1 overflow) and the RAC sat in RXOVERFLOW with no state
         * event pending; RXDIS, SWRXEN withdrawn, BUFC CLEAR, FRC.CMD.RXABORT
         * and clearing BUFC.IF left it there, FORCEDISABLE took it to
         * SHUTDOWN and OFF.  [I] The frame's octets that fit are written,
         * no RXDONE is raised.
         */
        uint32_t room = efr32_bufc_capacity(s, EFR32_RF_RX_BUFC_CHANNEL) -
                        efr32_bufc_count(s, EFR32_RF_RX_BUFC_CHANNEL);

        for (gsize index = 0; index < MIN(length, room); index++) {
            efr32_bufc_write_data_port(s, EFR32_RF_RX_BUFC_CHANNEL, data[index],
                                       false);
        }
        efr32_register_put(s, EFR32_BUFC_BASE + EFR32_BUFC_IF,
                           efr32_register_get(s, EFR32_BUFC_BASE + EFR32_BUFC_IF) |
                               EFR32_BUFC_IF_BUF1OF);
        g_clear_pointer(&s->rx_frame, g_bytes_unref);
        efr32_rac_set_state(s, EFR32_RAC_STATE_RXOVERFLOW);
        return;
    }
    for (gsize index = 0; index < length; index++) {
        efr32_register_put(s, EFR32_FRC_BASE + EFR32_FRC_RXRAWDATA,
                           data[index]);
        efr32_bufc_write_data_port(s, EFR32_RF_RX_BUFC_CHANNEL, data[index],
                                   false);
    }
    if (efr32_register_get(s, EFR32_FRC_BASE + EFR32_FRC_TRAILRXDATA) &
        EFR32_FRC_TRAILRXDATA_CRCOK) {
        efr32_bufc_write_data_port(s, EFR32_RF_RX_BUFC_CHANNEL,
                                   efr32_rx_demod_rate_drift_fails(s, length)
                                       ? 0 : EFR32_FRC_TRAILER_CRCOK,
                                   false);
    }
    g_clear_pointer(&s->rx_frame, g_bytes_unref);
    trace_efr32bg22_rf_rx_handoff(s->endpoint_id, now, length);
    efr32_frc_raise(s, EFR32_FRC_IF_RXDONE);
    s->rx_frame_ended = true;
    timer_mod(s->rac_frame_timer, now + EFR32_RAC_EOF_TO_RX_EXIT_NS);
}

/*
 * RAC.CMD on a released sequencer (SS7.72, debugger-driven on silicon):
 *  - TXEN latches a TX request (RAC.STATUS.TXENS) in any state; OFF leaves
 *    for TXWARM at once, and RXFRAME uses it at frame end (SS7.65);
 *  - TXDIS withdraws the request; in TX it ends the frame at once, into
 *    TXPD, or TX2RX with SWRXEN asserted, even with STATETX uncleared;
 *  - RXDIS takes RXSEARCH to RXPD (once STATERXSEARCH is clear); it had no
 *    effect in RXWARM, RXPD or TX.  RXDIS does not end RXFRAME: the RAC
 *    finishes the frame it is receiving (silicon, SS7.80: an instrumented
 *    scanner build waited 297 us entered in RXFRAME, RAC.STATUS 0x23400000,
 *    for the RAC to become idle after RXDIS with SWRXEN withdrawn; by
 *    debugger, RXDIS left RXFRAME alone while FORCEDISABLE went to OFF at
 *    once, 5/5).  [I] RXDIS in RX2RX, RX2TX and TX2RX is likewise ignored
 *    here; not measured.
 */
static void efr32_rac_seq_command(Efr32Periph *s, uint32_t command)
{
    if (command & EFR32_RAC_CMD_TXEN) {
        s->tx_requested = true;
    }
    if (command & EFR32_RAC_CMD_TXDIS) {
        s->tx_requested = false;
        if (s->rac_state == EFR32_RAC_STATE_TX) {
            /* [I] An aborted frame raises no TXDONE and reaches no peer. */
            efr32_rac_cancel_tx_completion(s);
            efr32_rf_medium_withdraw_active_tx(s->rf_medium,
                                                EFR32_RF_RADIO(s));
            s->rac_transition_held = false;
            efr32_rac_enter_state(s, efr32_rac_swrxen(s)
                                         ? EFR32_RAC_STATE_TX2RX
                                         : EFR32_RAC_STATE_TXPD);
        }
    }
    if ((command & EFR32_RAC_CMD_RXDIS) &&
        s->rac_state == EFR32_RAC_STATE_RXSEARCH) {
        s->rx_disable_requested = true;
    }
    efr32_rac_seq_evaluate(s);
}

/*
 * FORCEDISABLE: SHUTDOWN at once from any state, dropping a TX request.  In
 * RXFRAME it abandons the frame being received (silicon, SS7.80: RXFRAME to
 * OFF at once, 5/5, where RXDIS lets the frame finish): nothing more of it
 * reaches BUFC and no RXDONE follows.
 */
static void efr32_rac_seq_force_disable(Efr32Periph *s)
{
    efr32_rac_cancel_tx_completion(s);
    timer_del(s->rac_rx_timer);
    timer_del(s->rac_frame_timer);
    g_clear_pointer(&s->rx_frame, g_bytes_unref);
    s->rx_frame_ended = false;
    efr32_rf_medium_withdraw_active_tx(s->rf_medium, EFR32_RF_RADIO(s));
    efr32_rac_cancel_turnaround(s);
    efr32_rac_enter_state(s, EFR32_RAC_STATE_SHUTDOWN);
}

static void efr32_rac_cancel_tx_completion(Efr32Periph *s)
{
    timer_del(s->rac_tx_timer);
    s->tx_waiting_for_synth = false;
    s->prs_syncsent_ns = s->prs_framesent_ns = INT64_MAX;
    efr32_rac_prs_arm(s);
}

/* The frame captured at TX entry ends at s->tx_end_ns (or now, if later). */
static void efr32_rac_schedule_tx_completion(Efr32Periph *s)
{
    timer_mod(s->rac_tx_timer,
              MAX(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), s->tx_end_ns));
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

/*
 * Whether the receive chain is up.  On silicon a receiver opened from OFF by
 * a sequencer without an RXWARM handler reached RXSEARCH with SYEN, SYLOEN,
 * LNAMIXEN, RFPATHEN, IFADCDBG and RAC.RX all 0 and SYNTH.STATUS 0x4 (not
 * started), and detected no frame in any window (0 RXFRAME in 90 windows with
 * advertisers in range); brought up as the TX->RX turnaround does (SYNTHSTART
 * with ENABLEIF, RFPATHEN.LNAMIXEN, IFADCDBG.IFADCENCLK among the rest) it
 * receives (owo-open-fw docs/radio-tables.md SS7.80).  [I] Which of these is
 * individually required is not measured; the model requires the started IF
 * synthesizer, the LNA/mixer on the RF path and the clocked IF ADC.
 */
#define EFR32_RAC_IFADCDBG 0x0e0u
#define EFR32_RAC_IFADCDBG_IFADCENCLK (1u << 1)
#define EFR32_RAC_RFPATHEN 0x140u
#define EFR32_RAC_RFPATHEN_LNAMIXEN (1u << 1)

static bool efr32_rx_chain_powered(Efr32Periph *s)
{
    return s->synth_running && s->synth_if_enabled &&
           (efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_RFPATHEN) &
            EFR32_RAC_RFPATHEN_LNAMIXEN) &&
           (efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_IFADCDBG) &
            EFR32_RAC_IFADCDBG_IFADCENCLK);
}

static void efr32_rac_rx_handoff(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    Efr32RfRxDeliveryRequest *request;
    const Efr32RfRxMetadata *metadata;
    const uint8_t *data;
    gsize length;
    bool descriptor_configured;
    bool descriptor_published;

    if (s->rac_state != EFR32_RAC_STATE_RXSEARCH) {
        return;
    }

    /*
     * With sequencer firmware released and no QEMU-only descriptor ring, a
     * frame is received the way silicon does it: the RAC enters RXFRAME as
     * it starts and FRC delivers it to BUFC at frame end, raising RXDONE
     * then (efr32_rac_rx_frame_end).
     */
    if (s->seq_released && !efr32_bufc_descriptor_channel_configured(s)) {
        request = efr32_rf_medium_take_rx_offer_for_radio(
            s->rf_medium, EFR32_RF_RADIO(s));
        if (!request) {
            s->rx_offer_pending = false;
            return;
        }
        metadata = efr32_rf_rx_delivery_request_get_metadata(request);
        data = g_bytes_get_data(efr32_rf_rx_delivery_request_get_data(request),
                                &length);
        s->rx_offer_pending = efr32_rf_medium_has_rx_offer_for_radio(
            s->rf_medium, EFR32_RF_RADIO(s));
        if (!efr32_rx_chain_powered(s)) {
            /* Searching with the receive chain off detects nothing. */
            efr32_rf_rx_delivery_request_free(request);
            return;
        }
        /*
         * TODO(per-channel receive): on silicon the open image's receive is
         * channel-dependent -- CRC-ok ch37 77 %, ch38 1 %, ch39 0 %, frames
         * failing part-way with a correct header (owo-open-fw
         * docs/radio-tables.md SS7.80) -- while stock receives on all three.
         * The cause is being root-caused on silicon (CPU on the 76.8 MHz
         * DPLL vs stock's 38.4 MHz HFXO is the remaining difference).  The
         * model receives every channel; once the cause is known, its
         * register condition belongs here, next to the receive-chain check.
         */
        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >
            metadata->air_start_ns + EFR32_RF_FRAMESENT_TO_PEER_EOF_NS) {
            /*
             * [I] A receiver that was not searching when the preamble
             * reached it cannot synchronise to this frame.  The arrival
             * delay is taken equal to the measured FRAMESENT -> EOF one.
             */
            efr32_rf_rx_delivery_request_free(request);
            return;
        }
        g_clear_pointer(&s->rx_frame, g_bytes_unref);
        s->rx_frame = g_bytes_ref(
            efr32_rf_rx_delivery_request_get_data(request));
        s->rx_sync_ns = metadata->air_start_ns + EFR32_RF_LE1M_SYNC_NS +
                        EFR32_RF_FRAMESENT_TO_PEER_EOF_NS +
                        EFR32_RAC_SYNC_TO_FRAMEDET_NS;
        s->rx_eof_ns = metadata->air_start_ns +
                       efr32_rf_le1m_airtime_ns(length) +
                       EFR32_RF_FRAMESENT_TO_PEER_EOF_NS;
        s->rx_frame_ended = false;
        efr32_rf_rx_delivery_request_free(request);
        timer_mod(s->rac_frame_timer, s->rx_sync_ns);
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

    descriptor_configured = efr32_bufc_descriptor_channel_configured(s);
    if (descriptor_configured && !efr32_bufc_descriptor_has_room(s)) {
        /*
         * Check the metadata ring before touching channel 1.  A dropped
         * synthetic record must not leave bytes that a later endpoint could
         * accidentally claim as a successful packet.
         */
        efr32_bufc_mark_descriptor_overflow(s);
        efr32_rf_rx_delivery_request_free(request);
        s->rx_offer_pending = efr32_rf_medium_has_rx_offer_for_radio(
            s->rf_medium, EFR32_RF_RADIO(s));
        return;
    }

    /*
     * The retained sequencer reads FRC.RXRAWDATA then writes its four byte
     * lanes to secure BUFC channel 1 WRITEDATA.  Replay only that observed
     * data path: these bytes are not framed or CRC checked.  If the guest has
     * configured channel 2, publish the QEMU-only successful-RX descriptor
     * convention after those bytes; this makes the descriptor threshold
     * available before the synthetic RXDONE completion below.  It does not
     * claim that QEMU has identified or modeled the silicon descriptor
     * producer.  With channel 2 unconfigured, the handoff remains a raw-byte
     * FIFO seam and emits no descriptor.
     */
    for (gsize index = 0; index < length; index++) {
        efr32_register_put(s, EFR32_FRC_BASE + EFR32_FRC_RXRAWDATA,
                           data[index]);
        efr32_bufc_write_data_port(s, EFR32_RF_RX_BUFC_CHANNEL,
                                   data[index], false);
    }
    descriptor_published = efr32_bufc_enqueue_synthetic_rx_descriptor(s);
    efr32_rf_rx_delivery_request_free(request);

    if (descriptor_configured && !descriptor_published) {
        /*
         * A full descriptor ring is a dropped QEMU-only receive, not a
         * successful packet.  In particular, do not raise RXDONE while the
         * consumer still has no new descriptor and might revisit old data.
         */
        s->rx_offer_pending = efr32_rf_medium_has_rx_offer_for_radio(
            s->rf_medium, EFR32_RF_RADIO(s));
        return;
    }

    /* This completion remains a simulator-only boundary. */
    trace_efr32bg22_rf_rx_handoff(s->endpoint_id,
                                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                  length);
    efr32_frc_raise(s, EFR32_FRC_IF_RXDONE);

    /* One mailbox record is consumed per deferred handoff. */
    s->rx_offer_pending = efr32_rf_medium_has_rx_offer_for_radio(
        s->rf_medium, EFR32_RF_RADIO(s));
    if (s->rx_offer_pending) {
        efr32_rac_schedule_rx_handoff(s);
    }
}

/*
 * Returns the real FRC dynamic-length value for WCNTCMP0, or UINT32_MAX if
 * the configured DFLCTRL mode isn't the one this project's firmware (and
 * this function's derivation) is confirmed against -- see the DFLCTRL
 * comment above EFR32_FRC_DFLCTRL for the hardware evidence.
 */
static uint32_t efr32_frc_dynamic_length_wcntcmp0(const uint8_t *bytes,
                                                   gsize length,
                                                   uint32_t dflctrl)
{
    uint32_t dflmode = dflctrl & EFR32_FRC_DFLCTRL_DFLMODE_MASK;
    uint32_t dfloffset = (dflctrl >> EFR32_FRC_DFLCTRL_DFLOFFSET_SHIFT) &
                          EFR32_FRC_DFLCTRL_DFLOFFSET_MASK;
    uint32_t dflbits = (dflctrl >> EFR32_FRC_DFLCTRL_DFLBITS_SHIFT) &
                        EFR32_FRC_DFLCTRL_DFLBITS_MASK;
    uint32_t dflshift = (dflctrl >> EFR32_FRC_DFLCTRL_DFLSHIFT_SHIFT) &
                         EFR32_FRC_DFLCTRL_DFLSHIFT_MASK;
    uint32_t minlength = (dflctrl >> EFR32_FRC_DFLCTRL_MINLENGTH_SHIFT) &
                          EFR32_FRC_DFLCTRL_MINLENGTH_MASK;
    bool dflbitorder = dflctrl & EFR32_FRC_DFLCTRL_DFLBITORDER;
    uint32_t field;

    if (dflmode != EFR32_FRC_DFLCTRL_DFLMODE_LENGTH_FIELD ||
        dfloffset >= length || dflbits == 0) {
        return UINT32_MAX;
    }

    field = (bytes[dfloffset] >> dflshift) & ((1u << dflbits) - 1u);
    if (dflbitorder) {
        uint32_t reversed = 0;

        for (uint32_t bit = 0; bit < dflbits; bit++) {
            reversed |= ((field >> bit) & 1u) << (dflbits - 1u - bit);
        }
        field = reversed;
    }

    return field + minlength;
}

/* The programmed carrier in SYNTH.FREQ units (see EFR32_SYNTH_FREQ). */
static uint64_t efr32_rf_carrier_word(Efr32Periph *s)
{
    uint32_t caloffset = efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_CALOFFSET) &
                         EFR32_SYNTH_CALOFFSET_MASK;
    int64_t offset = (caloffset & EFR32_SYNTH_CALOFFSET_SIGN)
                         ? (int64_t)caloffset - (int64_t)(EFR32_SYNTH_CALOFFSET_SIGN << 1)
                         : (int64_t)caloffset;

    return (uint64_t)((int64_t)(efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_FREQ) &
                                EFR32_SYNTH_FREQ_MASK) +
                      (int64_t)(efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_CHCTRL) &
                                EFR32_SYNTH_CHCTRL_CHNO_MASK) *
                          (efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_CHSP) &
                           EFR32_SYNTH_CHSP_CHSP_MASK) +
                      offset);
}

/* The programmed carrier in Hz. */
static uint64_t efr32_rf_carrier_hz(Efr32Periph *s)
{
    uint64_t word = efr32_rf_carrier_word(s);

    return (word * EFR32_HFXO_HZ + (1ull << (EFR32_SYNTH_FREQ_FRACTION_BITS - 1)))
           >> EFR32_SYNTH_FREQ_FRACTION_BITS;
}

/*
 * Whether the synthesizer reaches @p hz (EFR32_SYNTH_LOCK_MIN_HZ..MAX_HZ), compared to the kHz: a
 * programmed carrier is off by up to half a SYNTH.FREQ unit (37 Hz).
 */
#define EFR32_HZ_PER_KHZ 1000u

static bool efr32_synth_in_range(uint64_t hz)
{
    uint64_t khz = (hz + EFR32_HZ_PER_KHZ / 2) / EFR32_HZ_PER_KHZ;

    return khz >= EFR32_SYNTH_LOCK_MIN_HZ / EFR32_HZ_PER_KHZ &&
           khz <= EFR32_SYNTH_LOCK_MAX_HZ / EFR32_HZ_PER_KHZ;
}

/*
 * The LO in time and the LO counter (owo-open-fw docs/radio-tables.md SS7.90, measured on a
 * BGM220P with the counter itself, receive only).
 *
 * LO counter: after LOCNTCTRL.RUN, LOCNTSTATUS.LOCOUNT (19 bits) holds the LO cycles in
 * 2^(NUMCYCLE + 1) HFXO periods; NUMCYCLE saturates at 12 (13-15 read and last as 12). With the
 * synthesizer running it counts only with LOCNTCTRL.FCALRUNCLKEN (0 without); stopped, in the
 * calibration state, it counts the open-loop VCO. LOCNTTARGET there is the programmed LO's
 * count for the NUMCYCLE of the last RUN; CAPCALSTART and CLEAR set the NUMCYCLE-0 value.
 * [I] Outside calibration TARGET holds (silicon read the NUMCYCLE-5 value after a calibrated
 * start, not modelled). [I] The count is rounded to the nearest cycle.
 *
 * Open-loop VCO, VCAPSEL 16: the codes the silicon calibration chose for 83 LOs from 2166.7 to
 * 2630.7 MHz, and LO counts at codes 0 and 2047; other VCAPSELs: open-loop counts at codes 0,
 * 1024 and 2047. [I] Linear in between (the top code bit's own step, 14 MHz between codes 960
 * and 1024, falls between two calibrated LOs).
 *
 * Closed loop, a changed carrier (FREQ, CHCTRL, CHSP, CALOFFSET, IFFREQ, IF mode): a second-
 * order step fitted to staggered NUMCYCLE-3 counts of 1 and 4 MHz steps up and down (n = 240
 * windows each loop, rms 0.09 / 0.08 of the step): receive loop zeta 0.45, f_n 196 kHz
 * (overshoot 20 %, settled to 2 % in about 7 us); with the loop configured for TX
 * (RAC.SYNTHENCTRL.LPFBWSEL) zeta 0.46, f_n 683 kHz. Steps beyond the fast range (+9.9 /
 * -11.6 MHz from where the LO was, at 2399.7 MHz) reach it at once and cover the rest at
 * 10.6 MHz/ms ([I] mean of +24 MHz, -24 MHz and back; silicon moves in uneven stages). After
 * SYNTHSTART the LO starts from the open-loop VCO at the calibrated code.
 */
#define EFR32_SYNTH_VCOTUNING 0x04cu
#define EFR32_SYNTH_VCOTUNING_CODE_MASK 0x7ffu
#define EFR32_SYNTH_VCOTUNING_VCAPSEL_SHIFT 11
#define EFR32_SYNTH_VCOTUNING_VCAPSEL_MASK 0x1fu
#define EFR32_SYNTH_CMD_CAPCALSTART (1u << 4)
#define EFR32_SYNTH_LOCNTCTRL_CLEAR (1u << 1)
#define EFR32_SYNTH_LOCNTCTRL_FCALRUNCLKEN (1u << 11)
#define EFR32_SYNTH_LOCNT_NUMCYCLE_MAX 12u
#define EFR32_SYNTH_LOCOUNT_MASK 0x7ffffu
#define EFR32_RAC_SYNTHENCTRL 0x094u
#define EFR32_RAC_SYNTHENCTRL_LPFBWSEL (1u << 20)

/*
 * VCAPSEL 16, open loop: VCOTUNING code against frequency in kHz. Codes 1-2044: the code the
 * silicon calibration chose for an LO 2 or 10 MHz apart (the open-loop VCO at that code is at or
 * just above that LO); codes 0 and 2047: LO counts of the open-loop VCO.
 */
#define EFR32_VCO_COUNT8_PERIODS 512u
static const struct {
    uint16_t code;
    uint32_t khz;
} efr32_vco_vcapsel16[] = {
    { 0, 2631300 }, { 1, 2630700 }, { 4, 2628700 }, { 10, 2626700 }, { 13, 2624700 },
    { 19, 2622700 }, { 25, 2620700 }, { 28, 2618700 }, { 36, 2616700 }, { 42, 2614700 },
    { 45, 2612700 }, { 52, 2610700 }, { 57, 2608700 }, { 68, 2606700 }, { 74, 2604700 },
    { 80, 2602700 }, { 83, 2600700 }, { 86, 2598700 }, { 92, 2596700 }, { 100, 2594700 },
    { 105, 2592700 }, { 113, 2590700 }, { 116, 2588700 }, { 131, 2586700 }, { 137, 2584700 },
    { 141, 2582700 }, { 146, 2580700 }, { 173, 2570700 }, { 209, 2560700 }, { 235, 2550700 },
    { 285, 2540700 }, { 322, 2530700 }, { 353, 2520700 }, { 379, 2510700 }, { 420, 2500700 },
    { 453, 2490700 }, { 485, 2480700 }, { 555, 2470700 }, { 589, 2460700 }, { 621, 2450700 },
    { 665, 2440700 }, { 697, 2430700 }, { 733, 2420700 }, { 790, 2410700 }, { 824, 2400700 },
    { 861, 2390700 }, { 908, 2380700 }, { 1061, 2370700 }, { 1102, 2360700 }, { 1138, 2350700 },
    { 1185, 2340700 }, { 1226, 2330700 }, { 1286, 2320700 }, { 1322, 2310700 }, { 1366, 2300700 },
    { 1416, 2290700 }, { 1453, 2280700 }, { 1498, 2270700 }, { 1580, 2260700 }, { 1624, 2250700 },
    { 1674, 2240700 }, { 1716, 2230700 }, { 1768, 2220700 }, { 1830, 2210700 }, { 1881, 2200700 },
    { 1885, 2198700 }, { 1896, 2196700 }, { 1907, 2194700 }, { 1924, 2192700 }, { 1932, 2190700 },
    { 1941, 2188700 }, { 1948, 2186700 }, { 1958, 2184700 }, { 1970, 2182700 }, { 1977, 2180700 },
    { 1990, 2178700 }, { 2002, 2176700 }, { 2007, 2174700 }, { 2020, 2172700 }, { 2032, 2170700 },
    { 2037, 2168700 }, { 2044, 2166700 }, { 2047, 2165700 },
};
/* Counts of NUMCYCLE 8 at VCOTUNING 0, 1024 and 2047 for each VCAPSEL measured. */
static const uint8_t efr32_vco_vcapsels[] = { 0, 8, 16, 24, 31 };
static const uint16_t efr32_vco_codes[] = { 0, 1024, 2047 };
static const uint16_t efr32_vco_count8_by_vcapsel[][3] = {
    { 35905, 32360, 29329 },
    { 35470, 32047, 29089 },
    { 35084, 31765, 28876 },
    { 34753, 31514, 28687 },
    { 34489, 31313, 28538 },
};
#define EFR32_VCO_VCAPSEL_REFERENCE_ROW 2u
/*
 * Where the closed-loop LO stops when the carrier is beyond the VCO (the calibration code at
 * 2047 or 0): LO counts with VCAPSEL 0, 16 and 31 pinned at 2189.34 / 2703.76, 2156.47 /
 * 2647.57 and 2134.12 / 2600.92 MHz for every carrier past them (2 MHz steps, 400 us after the
 * calibration). [I] Linear in VCAPSEL in between.
 */
static const uint8_t efr32_vco_limit_vcapsels[] = { 0, 16, 31 };
static const double efr32_vco_limit_low_hz[] = { 2189.34e6, 2156.47e6, 2134.12e6 };
static const double efr32_vco_limit_high_hz[] = { 2703.76e6, 2647.57e6, 2600.92e6 };

#define EFR32_LO_RX_ZETA 0.45
#define EFR32_LO_RX_FN_HZ 196000.0
#define EFR32_LO_TX_ZETA 0.46
#define EFR32_LO_TX_FN_HZ 683000.0
#define EFR32_LO_FAST_UP_HZ 9.9e6
#define EFR32_LO_FAST_DOWN_HZ 11.6e6
#define EFR32_LO_SLEW_HZ_PER_S 10.6e9
/* Past this the second-order part is below 1e-9 of the step. */
#define EFR32_LO_SETTLE_NS 60000
#define EFR32_LO_INTEGRATION_STEP_NS 25

static double efr32_lerp(double x0, double y0, double x1, double y1, double x)
{
    return x1 == x0 ? y0 : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

/* Open-loop VCO frequency at @p vcotuning (code and VCAPSEL). */
static double efr32_vco_open_hz(uint32_t vcotuning)
{
    uint32_t code = vcotuning & EFR32_SYNTH_VCOTUNING_CODE_MASK;
    uint32_t vcapsel = (vcotuning >> EFR32_SYNTH_VCOTUNING_VCAPSEL_SHIFT) &
                       EFR32_SYNTH_VCOTUNING_VCAPSEL_MASK;
    size_t i = 0;
    double count;
    double delta[G_N_ELEMENTS(efr32_vco_codes)];
    size_t v = 0;

    while (i + 2 < G_N_ELEMENTS(efr32_vco_vcapsel16) && code > efr32_vco_vcapsel16[i + 1].code) {
        i++;
    }
    count = efr32_lerp(efr32_vco_vcapsel16[i].code, efr32_vco_vcapsel16[i].khz,
                       efr32_vco_vcapsel16[i + 1].code, efr32_vco_vcapsel16[i + 1].khz, code) *
            EFR32_HZ_PER_KHZ * EFR32_VCO_COUNT8_PERIODS / (double)EFR32_HFXO_HZ;
    while (v + 2 < G_N_ELEMENTS(efr32_vco_vcapsels) && vcapsel > efr32_vco_vcapsels[v + 1]) {
        v++;
    }
    for (size_t j = 0; j < G_N_ELEMENTS(efr32_vco_codes); j++) {
        delta[j] = efr32_lerp(efr32_vco_vcapsels[v], efr32_vco_count8_by_vcapsel[v][j],
                              efr32_vco_vcapsels[v + 1], efr32_vco_count8_by_vcapsel[v + 1][j],
                              vcapsel) -
                   efr32_vco_count8_by_vcapsel[EFR32_VCO_VCAPSEL_REFERENCE_ROW][j];
    }
    count += code < efr32_vco_codes[1]
                 ? efr32_lerp(efr32_vco_codes[0], delta[0], efr32_vco_codes[1], delta[1], code)
                 : efr32_lerp(efr32_vco_codes[1], delta[1], efr32_vco_codes[2], delta[2], code);
    return count * (double)EFR32_HFXO_HZ / EFR32_VCO_COUNT8_PERIODS;
}

/* The closed-loop LO range at the current VCAPSEL. */
static void efr32_vco_limits_hz(Efr32Periph *s, double *low, double *high)
{
    uint32_t vcapsel = (efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_VCOTUNING) >>
                        EFR32_SYNTH_VCOTUNING_VCAPSEL_SHIFT) &
                       EFR32_SYNTH_VCOTUNING_VCAPSEL_MASK;
    size_t i = vcapsel > efr32_vco_limit_vcapsels[1] ? 1 : 0;

    *low = efr32_lerp(efr32_vco_limit_vcapsels[i], efr32_vco_limit_low_hz[i],
                      efr32_vco_limit_vcapsels[i + 1], efr32_vco_limit_low_hz[i + 1], vcapsel);
    *high = efr32_lerp(efr32_vco_limit_vcapsels[i], efr32_vco_limit_high_hz[i],
                       efr32_vco_limit_vcapsels[i + 1], efr32_vco_limit_high_hz[i + 1], vcapsel);
}

/* The LO the loop is steered to: the carrier, plus the IF on the high side while receiving. */
static double efr32_synth_target_lo_hz(Efr32Periph *s)
{
    uint32_t iffreq_reg = efr32_register_get(s, EFR32_SYNTH_BASE + EFR32_SYNTH_IFFREQ);
    double lo = (double)efr32_rf_carrier_word(s);

    if (s->synth_if_enabled) {
        double iffreq = iffreq_reg & EFR32_SYNTH_IFFREQ_IFFREQ_MASK;

        lo += (iffreq_reg & EFR32_SYNTH_IFFREQ_LOSIDE) ? iffreq : -iffreq;
    }
    return lo * (double)EFR32_HFXO_HZ / (double)(1u << EFR32_SYNTH_FREQ_FRACTION_BITS);
}

/* The closed-loop LO at virtual time @p ns on the current trajectory. */
static double efr32_synth_lo_closed_hz(Efr32Periph *s, int64_t ns)
{
    double tau = (double)MAX(ns - s->lo_change_ns, 0) / NANOSECONDS_PER_SECOND;
    double zeta = s->lo_tx_loop ? EFR32_LO_TX_ZETA : EFR32_LO_RX_ZETA;
    double wn = 2.0 * M_PI * (s->lo_tx_loop ? EFR32_LO_TX_FN_HZ : EFR32_LO_RX_FN_HZ);
    double wd = wn * sqrt(1.0 - zeta * zeta);
    double fast = MIN(MAX(s->lo_to_hz - s->lo_from_hz, -EFR32_LO_FAST_DOWN_HZ),
                      EFR32_LO_FAST_UP_HZ);
    double goal = s->lo_from_hz + fast;
    double rest = s->lo_to_hz - goal;
    double h = exp(-zeta * wn * tau) *
               (cos(wd * tau) + zeta / sqrt(1.0 - zeta * zeta) * sin(wd * tau));
    double lo = goal + (s->lo_from_hz - goal) * h;

    if (rest != 0.0) {
        lo += copysign(MIN(fabs(rest), EFR32_LO_SLEW_HZ_PER_S * tau), rest);
    }
    return lo;
}

/* The LO at virtual time @p ns: the open-loop VCO while the synthesizer is stopped. */
static double efr32_synth_lo_hz(Efr32Periph *s, int64_t ns)
{
    if (!s->synth_running) {
        return efr32_vco_open_hz(efr32_register_get(s, EFR32_SYNTH_BASE +
                                                       EFR32_SYNTH_VCOTUNING));
    }
    return efr32_synth_lo_closed_hz(s, ns);
}

/* LO cycles from @p from_ns to @p to_ns on the current trajectory. */
static double efr32_synth_lo_cycles(Efr32Periph *s, int64_t from_ns, int64_t to_ns)
{
    double rest = fabs(s->lo_to_hz - s->lo_from_hz) -
                  (s->lo_to_hz > s->lo_from_hz ? EFR32_LO_FAST_UP_HZ : EFR32_LO_FAST_DOWN_HZ);
    int64_t settled_ns = s->lo_change_ns + EFR32_LO_SETTLE_NS +
                         (rest > 0 ? (int64_t)(rest / EFR32_LO_SLEW_HZ_PER_S *
                                               NANOSECONDS_PER_SECOND) : 0);
    double cycles = 0.0;
    int64_t t = from_ns;

    if (!s->synth_running) {
        return efr32_synth_lo_hz(s, from_ns) * (double)(to_ns - from_ns) /
               NANOSECONDS_PER_SECOND;
    }
    while (t < MIN(to_ns, settled_ns)) {
        int64_t dt = MIN((int64_t)EFR32_LO_INTEGRATION_STEP_NS, MIN(to_ns, settled_ns) - t);

        cycles += efr32_synth_lo_hz(s, t + dt / 2) * (double)dt / NANOSECONDS_PER_SECOND;
        t += dt;
    }
    if (t < to_ns) {
        cycles += s->lo_to_hz * (double)(to_ns - t) / NANOSECONDS_PER_SECOND;
    }
    return cycles;
}

/* A LO count in progress takes the trajectory so far before the trajectory changes. */
static void efr32_synth_locnt_accumulate(Efr32Periph *s, int64_t now)
{
    if (s->locnt_pending && now > s->locnt_acc_ns) {
        int64_t until = MIN(now, s->locnt_busy_until_ns);

        if (until > s->locnt_acc_ns) {
            s->locnt_acc_cycles += efr32_synth_lo_cycles(s, s->locnt_acc_ns, until);
            s->locnt_acc_ns = until;
        }
    }
}

/* The carrier, IF mode, loop configuration or run state changed: steer the LO from where it is. */
static void efr32_synth_retarget(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double low, high;

    efr32_synth_locnt_accumulate(s, now);
    s->lo_from_hz = efr32_synth_lo_hz(s, now);
    efr32_vco_limits_hz(s, &low, &high);
    s->lo_to_hz = MIN(MAX(efr32_synth_target_lo_hz(s), low), high);
    s->lo_change_ns = now;
    s->lo_tx_loop = (efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SYNTHENCTRL) &
                     EFR32_RAC_SYNTHENCTRL_LPFBWSEL) != 0;
}

/* The programmed LO's count for NUMCYCLE @p numcycle (LOCNTTARGET). */
static uint32_t efr32_synth_locnt_target(Efr32Periph *s, unsigned numcycle)
{
    return (uint32_t)lround(efr32_synth_target_lo_hz(s) *
                            (double)(2u << MIN(numcycle, EFR32_SYNTH_LOCNT_NUMCYCLE_MAX)) /
                            (double)EFR32_HFXO_HZ) &
           EFR32_SYNTH_LOCOUNT_MASK;
}

/* LOCNTCTRL.RUN: a count of 2^(NUMCYCLE + 1) HFXO periods from now. */
static void efr32_synth_locnt_run(Efr32Periph *s, uint32_t locntctrl)
{
    unsigned numcycle = MIN((locntctrl & EFR32_SYNTH_LOCNTCTRL_NUMCYCLE_MASK) >>
                                EFR32_SYNTH_LOCNTCTRL_NUMCYCLE_SHIFT,
                            EFR32_SYNTH_LOCNT_NUMCYCLE_MAX);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->locnt_busy_until_ns = now + (int64_t)((2ull << numcycle) * NANOSECONDS_PER_SECOND /
                                             EFR32_HFXO_HZ);
    s->locnt_pending = true;
    s->locnt_counts = !s->synth_running || (locntctrl & EFR32_SYNTH_LOCNTCTRL_FCALRUNCLKEN);
    s->locnt_acc_cycles = 0.0;
    s->locnt_acc_ns = now;
    s->locnt_run_ns = now;
    s->locnt_window_s = (double)(2ull << numcycle) / (double)EFR32_HFXO_HZ;
    if (!s->synth_running) {
        efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTTARGET,
                           efr32_synth_locnt_target(s, numcycle));
    }
}

/* LOCNTSTATUS.LOCOUNT once the count has ended. */
static void efr32_synth_locnt_complete(Efr32Periph *s, int64_t now)
{
    double cycles;

    if (!s->locnt_pending || now < s->locnt_busy_until_ns) {
        return;
    }
    efr32_synth_locnt_accumulate(s, now);
    s->locnt_pending = false;
    /* The busy time is whole nanoseconds: scale to the exact window. */
    cycles = s->locnt_acc_cycles * s->locnt_window_s * NANOSECONDS_PER_SECOND /
             (double)(s->locnt_busy_until_ns - s->locnt_run_ns);
    efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTSTATUS,
                       s->locnt_counts ? (uint32_t)llround(cycles) & EFR32_SYNTH_LOCOUNT_MASK : 0);
}

/* Whether the LO has arrived at the carrier the medium is asked about. */
static bool efr32_synth_lo_arrived(Efr32Periph *s)
{
    return !s->synth_running ||
           fabs(efr32_synth_lo_hz(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) -
                efr32_synth_target_lo_hz(s)) <=
               EFR32_RF_RASTER_HZ / 2;
}

/*
 * The medium's channel: the carrier on a 1 MHz raster from EFR32_RF_RASTER0_HZ, so Bluetooth
 * carriers (2 MHz apart) and amateur-plan carriers on any whole MHz each match only themselves.
 * A carrier the synthesizer cannot reach matches no receiver, nor does one the running LO has
 * not yet reached (a large step without recalibration slews, SS7.90).
 */
static uint16_t efr32_rf_channel(Efr32Periph *s)
{
    uint64_t hz = efr32_rf_carrier_hz(s);

    if (!efr32_synth_in_range(hz) ||
        !efr32_synth_lo_arrived(s) ||
        hz + EFR32_RF_RASTER_HZ / 2 < EFR32_RF_RASTER0_HZ) {
        return EFR32_RF_NO_CHANNEL;
    }
    return MIN((hz + EFR32_RF_RASTER_HZ / 2 - EFR32_RF_RASTER0_HZ) / EFR32_RF_RASTER_HZ,
               EFR32_RF_NO_CHANNEL - 1);
}

/*
 * On-off keying (EFR32xG22 reference manual 5.1.2, "ASK / OOK"): MODEM.CTRL0.MODFORMAT = 6
 * (public register model bits 8:6; the Silicon Labs configurator's OOK/ASK code). Symbol rate
 * f_HFXO / (8 x TXBRNUM / TXBRDEN) (MODEM.TXBR, the configurator's calc_txbr_value); the frame
 * is the preamble (PRE.TXBASES bases of PRE.BASEBITS + 1 bits), the sync word
 * (CTRL1.SYNCBITS + 1 bits) and the FIFO's octets, least significant bit first, 1 = carrier on.
 * [I] The keying waveform, its edges and its power are not modelled; a Bluetooth receiver gets
 * nothing from an OOK frame.
 */
#define EFR32_MODEM_CTRL0 0x020u
#define EFR32_MODEM_CTRL0_MODFORMAT_SHIFT 6
#define EFR32_MODEM_CTRL0_MODFORMAT_MASK 0x7u
#define EFR32_MODEM_MODFORMAT_OOKASK 6u
#define EFR32_MODEM_CTRL1 0x024u
#define EFR32_MODEM_CTRL1_SYNCBITS_MASK 0x1fu
#define EFR32_MODEM_TXBR 0x058u
#define EFR32_MODEM_TXBR_NUM_MASK 0xffffu
#define EFR32_MODEM_TXBR_DEN_SHIFT 16
#define EFR32_MODEM_TXBR_DEN_MASK 0xffu
#define EFR32_MODEM_TXBR_HFXO_DIVIDER 8u
#define EFR32_MODEM_PRE 0x064u
#define EFR32_MODEM_PRE_BASEBITS_SHIFT 4
#define EFR32_MODEM_PRE_BASEBITS_MASK 0x3u
#define EFR32_MODEM_PRE_TXBASES_SHIFT 16
#define EFR32_MODEM_PRE_TXBASES_MASK 0xffffu
#define EFR32_OOK_TRACE_OCTETS_MAX 160u

static bool efr32_modem_ook(Efr32Periph *s)
{
    return ((efr32_register_get(s, EFR32_MODEM_BASE + EFR32_MODEM_CTRL0) >>
             EFR32_MODEM_CTRL0_MODFORMAT_SHIFT) & EFR32_MODEM_CTRL0_MODFORMAT_MASK) ==
           EFR32_MODEM_MODFORMAT_OOKASK;
}

/* An OOK frame: its airtime, and a trace of its carrier and key pattern. */
static int64_t efr32_rac_ook_tx(Efr32Periph *s, const uint8_t *bytes, gsize length)
{
    uint32_t txbr = efr32_register_get(s, EFR32_MODEM_BASE + EFR32_MODEM_TXBR);
    uint32_t num = txbr & EFR32_MODEM_TXBR_NUM_MASK;
    uint32_t den = (txbr >> EFR32_MODEM_TXBR_DEN_SHIFT) & EFR32_MODEM_TXBR_DEN_MASK;
    uint32_t pre = efr32_register_get(s, EFR32_MODEM_BASE + EFR32_MODEM_PRE);
    uint64_t preamble = (uint64_t)((pre >> EFR32_MODEM_PRE_TXBASES_SHIFT) &
                                   EFR32_MODEM_PRE_TXBASES_MASK) *
                        (((pre >> EFR32_MODEM_PRE_BASEBITS_SHIFT) &
                          EFR32_MODEM_PRE_BASEBITS_MASK) + 1u);
    uint64_t sync = (efr32_register_get(s, EFR32_MODEM_BASE + EFR32_MODEM_CTRL1) &
                     EFR32_MODEM_CTRL1_SYNCBITS_MASK) + 1u;
    uint64_t symbols = preamble + sync + (uint64_t)length * BITS_PER_BYTE;
    uint64_t hz = efr32_rf_carrier_hz(s);
    g_autofree char *hex = g_malloc0(EFR32_OOK_TRACE_OCTETS_MAX * 2 + 1);
    int64_t airtime_ns;

    if (num == 0 || den == 0) {
        return 0;
    }
    /* symbols / (f_HFXO x DEN / (8 x NUM)) seconds */
    airtime_ns = (int64_t)(symbols * EFR32_MODEM_TXBR_HFXO_DIVIDER * num *
                           NANOSECONDS_PER_SECOND / ((uint64_t)EFR32_HFXO_HZ * den));
    for (gsize i = 0; i < MIN(length, EFR32_OOK_TRACE_OCTETS_MAX); i++) {
        snprintf(hex + 2 * i, 3, "%02x", bytes[i]);
    }
    trace_efr32bg22_ook_tx(s->endpoint_id, hz / 1000u,
                           (uint64_t)EFR32_HFXO_HZ * den /
                               ((uint64_t)EFR32_MODEM_TXBR_HFXO_DIVIDER * num),
                           (unsigned)(preamble + sync), (unsigned)length, airtime_ns,
                           efr32_synth_in_range(hz), hex);
    return airtime_ns;
}

static void efr32_rac_capture_tx_snapshot(Efr32Periph *s)
{
    Efr32RfTxMetadata metadata = {
        .trigger = EFR32_RF_TX_TRIGGER_RAC_CMD_TXEN,
        .buffer_channel = EFR32_RF_TX_BUFC_CHANNEL,
        .channel = efr32_rf_channel(s),
    };
    GByteArray *data = efr32_bufc_snapshot_data(
        s, EFR32_RF_TX_BUFC_CHANNEL);
    Efr32RfTxSnapshot *snapshot;
    const uint8_t *bytes;
    gsize length;
    uint32_t first_word = 0;

    /*
     * BUFC channel 0 is copied only once the modeled sequencer has accepted
     * TXWARM and advanced RAC to TX.  Callers without a released private core
     * take the legacy abstract path and reach this helper directly at TXEN.
     * A raw TXEN from a dual-core guest is not itself an RF transmission.
     */
    metadata.air_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            EFR32_RAC_TX_ENTRY_TO_AIR_NS;
    if (efr32_modem_ook(s)) {
        s->tx_end_ns = metadata.air_start_ns + efr32_rac_ook_tx(s, data->data, data->len) +
                       EFR32_RAC_FRAMESENT_TO_TX_EXIT_NS;
        g_byte_array_unref(data);
        efr32_rac_prs_tx_events(s);
        return;
    }
    s->tx_end_ns = metadata.air_start_ns +
                   efr32_rf_le1m_airtime_ns(data->len) +
                   EFR32_RAC_FRAMESENT_TO_TX_EXIT_NS;
    efr32_rac_prs_tx_events(s);
    snapshot = efr32_rf_medium_snapshot_tx(
        s->rf_medium, EFR32_RF_RADIO(s), &metadata, data->data, data->len,
        &error_abort);
    g_byte_array_unref(data);
    bytes = g_bytes_get_data(efr32_rf_tx_snapshot_get_data(snapshot), &length);
    for (unsigned index = 0; index < MIN(length, sizeof(first_word)); index++) {
        first_word |= (uint32_t)bytes[index] << (index * 8);
    }
    {
        uint32_t dflctrl = efr32_register_get(s, EFR32_FRC_BASE +
                                              EFR32_FRC_DFLCTRL);
        uint32_t wcntcmp0 = efr32_frc_dynamic_length_wcntcmp0(bytes, length,
                                                              dflctrl);

        if (wcntcmp0 != UINT32_MAX) {
            efr32_register_put(s, EFR32_FRC_BASE + EFR32_FRC_WCNTCMP0,
                               wcntcmp0);
        }
    }
    trace_efr32bg22_tx_snapshot(s->endpoint_id,
                                 efr32_rf_tx_snapshot_get_sequence(snapshot),
                                 length, first_word);
    efr32_rf_medium_set_latest_tx_snapshot(s->rf_medium, snapshot);
    trace_efr32bg22_tx_offer_count(s->endpoint_id,
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
    if ((address & ~0x3fffu) == EFR32_RFCRC_BASE)
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

    /*
     * Every bit below was sampled on silicon (owo-open-fw
     * docs/radio-tables.md SS7.21, SS7.63, SS7.65 and SS7.72: running stage 3
     * sampled by state, and debugger-driven runs with the M0+ parked):
     *  - SEQACTIVE follows any uncleared SEQIF state event, whether or not
     *    SEQIEN enables it (0x10400000 in OFF until STATEOFF is cleared,
     *    0x19400000 -> 0x09400000 in TX on clearing STATETX);
     *  - TXENS is the latched TX request: set by TXEN in OFF, RXWARM,
     *    RXFRAME and TX, cleared on TX entry, by TXDIS and by FORCEDISABLE;
     *  - RXENS and RXMASK bit 1 follow RXENSRCEN.SWRXEN in every state;
     *  - DEMODENS was set in RXSEARCH and RXFRAME and clear in every other
     *    state sampled (RXWARM, RXPD, RX2TX, TX2RX, TX, TXWARM, OFF);
     *    [I] RX2RX was not sampled;
     *  - SEQSLEEPING is set while the sequencer core sleeps in WFI and
     *    clear while it runs a handler or is halted by the debugger.
     */
    bool demod = state == EFR32_RAC_STATE_RXSEARCH ||
                 state == EFR32_RAC_STATE_RXFRAME;
    bool sleeping = s->seq_released && s->seq_cpu && s->seq_cpu->halted;
    bool events = efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SEQIF) &
                  EFR32_RAC_SEQIF_STATE_EVENTS;

    return state << EFR32_RAC_STATUS_STATE_SHIFT |
           (events ? EFR32_RAC_STATUS_SEQACTIVE : 0) |
           (demod ? EFR32_RAC_STATUS_DEMODENS : 0) |
           (sleeping ? EFR32_RAC_STATUS_SEQSLEEPING : 0) |
           (s->tx_requested ? EFR32_RAC_STATUS_TXENS : 0) |
           (efr32_rac_swrxen(s) ? EFR32_RAC_STATUS_RXENS |
                                      EFR32_RAC_STATUS_RXMASK_SWRXEN : 0);
}

/* RAC.STIMER ticks since the last RAC state change (SS7.66, SS7.68). */
static uint32_t efr32_rac_stimer(Efr32Periph *s)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                      s->rac_stimer_epoch_ns;

    return muldiv64(elapsed, EFR32_RAC_STIMER_TICKS_PER_10US,
                    EFR32_RAC_STIMER_NS_PER_10US) & EFR32_RAC_STIMER_MASK;
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

/*
 * The counter does not run from reset (silicon, owo-open-fw
 * docs/radio-tables.md): it read 0 and frozen with EN = 1 and CTRL = 0 on
 * live stage 3 (SS7.32, SS7.71), and still 0 after CTRL alone was poked to
 * stock's 0x00011102 (SS7.32).  Stock's sequence EN, CTRL 0x00011102,
 * +0x030 0x1233, +0x038 0x80015558, CMD = 1 started it (SS7.33:
 * 0xb980, then 0x1031a2 about 450 ms later).  [I] CMD bit 0 is the start:
 * the one write of that sequence that is a command (stock pairs CMD = 1
 * with enabling the PROTIMER IRQ, SS7.9); nothing here stops it again.
 */
#define EFR32_PROTIMER_CMD_START (1u << 0)

static void efr32_protimer_command(Efr32Periph *s, uint32_t command)
{
    if (command & EFR32_PROTIMER_CMD_START) {
        ptimer_transaction_begin(s->protimer_counter);
        ptimer_run(s->protimer_counter, 0);
        ptimer_transaction_commit(s->protimer_counter);
    }
}

/*
 * EM01GRPACLK, which clocks TIMER0 (xg22.dtsi: CLOCK_BRANCH_EM01GRPACLK).
 * CMU_EM01GRPACLKCTRL.CLKSEL resets to HFRCODPLL and selects HFXO (2) or
 * FSRCO (3) otherwise (efr32bg22_cmu.h).  A locked DPLL runs HFRCODPLL at
 * f_ref * (N + 1) / (M + 1) from DPLL_CFG1 (efr32bg22_dpll.h), with the
 * reference chosen by CMU_DPLLREFCLKCTRL; without it HFRCODPLL keeps its
 * 19 MHz reset band.  Oscillator rates are the xg22.dtsi values.
 */
static uint64_t efr32_hfrcodpll_hz(Efr32Periph *s);

static uint64_t efr32_em01grpa_hz(Efr32Periph *s)
{
    uint32_t clksel = efr32_register_get(s, EFR32_CMU_BASE +
                                            EFR32_CMU_EM01GRPACLKCTRL) & 0x3u;

    switch (clksel) {
    case EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_HFXO:
        return EFR32_HFXO_HZ;
    case EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_FSRCO:
        return EFR32_FSRCO_MEASURED_HZ; /* SS7.73, not the nominal 20 MHz */
    default:
        return efr32_hfrcodpll_hz(s);
    }
}

static uint64_t efr32_hfrcodpll_hz(Efr32Periph *s)
{
    uint32_t cfg1;
    uint64_t ref;

    if (!efr32_dpll_enabled(s)) {
        return EFR32_HFRCO_RESET_HZ;
    }
    switch (efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_DPLLREFCLKCTRL) & 0x3u) {
    case EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_HFXO:
        ref = EFR32_HFXO_HZ;
        break;
    case EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_LFXO:
        ref = EFR32_LFXO_HZ;
        break;
    default:
        return EFR32_HFRCO_RESET_HZ; /* no reference: the DPLL cannot lock */
    }
    cfg1 = efr32_register_get(s, EFR32_DPLL0_BASE + EFR32_DPLL_CFG1);
    return ref * (((cfg1 >> 16) & 0xfffu) + 1) / ((cfg1 & 0xfffu) + 1);
}


static bool efr32_hfxo_hw_requested(Efr32Periph *s);

/* HFXO on: FORCEEN or an on-demand consumer (SYSCLK, EM01GRPA, DPLL ref). */
static bool efr32_hfxo_requested(Efr32Periph *s)
{
    return (efr32_register_get(s, EFR32_HFXO_BASE + EFR32_HFXO_CTRL) &
            EFR32_HFXO_CTRL_FORCEEN) || efr32_hfxo_hw_requested(s);
}

/*
 * ---------------------------------------------------------------------------
 * Timers and oscillators, timed from silicon.
 *
 * Every latency below was measured on a BG22 (BGM220 explorer kit) running
 * from SRAM after stage 3 had configured its clocks (HCLK = EM01GRPACLK =
 * HFRCODPLL 76.8 MHz locked to HFXO, PCLK = HCLK / 2, BURTC on LFXO), with
 * DWT.CYCCNT and PRS -> TIMER1 input capture as the timebase; see owo-open-fw
 * docs/radio-tables.md (timer findings) for each method, sample count and
 * range.  "[I]" marks a rule inferred from the measured ones rather than
 * measured itself; "UNMEASURED" marks a placeholder.
 * ---------------------------------------------------------------------------
 */

/* HCLK of the measurement setup: HFRCODPLL locked at 76.8 MHz. */
#define EFR32_MEASURE_HCLK_HZ 76800000ULL

/*
 * HFXO cold start with stage 3's own configuration, CMU_DPLLLock() entry,
 * FORCEEN -> STATUS bits, timed in FSRCO cycles (n = 3): ENS 25, RDY
 * 5455-5480, COREBIASOPTRDY 5663-5665.  The restart from a warm crystal could
 * not be measured (the radio holds HWREQ), so it reuses the cold numbers.
 */
#define EFR32_HFXO_ENS_FSRCO_CYCLES 25ULL
#define EFR32_HFXO_RDY_FSRCO_CYCLES 5455ULL
#define EFR32_HFXO_COREBIASOPTRDY_FSRCO_CYCLES 5663ULL

/*
 * DPLL0 relock with the HFXO reference running: EN -> STATUS.RDY and IF.LOCK
 * after 11588 HFXO cycles every time (n = 16).  EN = 0 -> STATUS.ENS clear
 * after 38-41 HFXO cycles (n = 16).  STATUS bit 31 is the configuration LOCK
 * bit: 0 on running stage 3 (STATUS = 0x3).  [I] With the reference still
 * starting, the lock is counted from the reference's RDY.  UNMEASURED: the
 * time to ENS after EN, modelled as immediate; an LFXO-referenced lock, given
 * the HFXO figure.
 */
#define EFR32_DPLL_LOCK_HFXO_CYCLES 11588ULL
#define EFR32_DPLL_ENS_CLEAR_HFXO_CYCLES 41ULL

/*
 * LFXO: STATUS.RDY follows STATUS.ENS by CFG.TIMEOUT LFXO cycles; with stage
 * 3's CYCLES4K that measured 124.6-125.4 ms (n = 5, debugger polling, about
 * 0.45 ms resolution) against 125.0 ms for 4096 cycles.  ENS was already set
 * at the first poll after the enable, 0.41-0.54 ms later: UNMEASURED below
 * that bound, so it is immediate here.  The TIMEOUT encodings are
 * efr32bg22_lfxo.h's CYCLES2 ... CYCLES32K.
 */
#define EFR32_LFXO_BASE 0x40020000u
#define EFR32_LFXO_CTRL 0x004u
#define EFR32_LFXO_CFG 0x008u
#define EFR32_LFXO_STATUS 0x010u
#define EFR32_LFXO_CTRL_FORCEEN (1u << 0)
#define EFR32_LFXO_CFG_TIMEOUT_SHIFT 8
#define EFR32_LFXO_CFG_TIMEOUT_MASK 0x700u
#define EFR32_LFXO_CFG_RESET 0x00000701u /* efr32bg22_lfxo.h: TIMEOUT 32K */
#define EFR32_OSC_STATUS_RDY (1u << 0)
#define EFR32_OSC_STATUS_ENS (1u << 16)

static const uint32_t efr32_lfxo_timeout_cycles[] = {
    2, 256, 1024, 2048, 4096, 8192, 16384, 32768,
};

/*
 * LFRCO: FORCEEN -> ENS 46-47, RDY 16048-16049 HCLK cycles; FORCEEN clear ->
 * ENS clear 51-65 HCLK cycles (n = 4 each).
 */
#define EFR32_LFRCO_BASE 0x40024000u
#define EFR32_LFRCO_CTRL 0x004u
#define EFR32_LFRCO_STATUS 0x008u
#define EFR32_LFRCO_CTRL_FORCEEN (1u << 0)
#define EFR32_LFRCO_ENS_HCLK_CYCLES 47ULL
#define EFR32_LFRCO_RDY_HCLK_CYCLES 16049ULL
#define EFR32_LFRCO_ENS_CLEAR_HCLK_CYCLES 65ULL

#define EFR32_CMU_EM23GRPACLKCTRL 0x140u
#define EFR32_CMU_EM4GRPACLKCTRL 0x160u
#define EFR32_CMU_LFCLKSEL_LFRCO 1u
#define EFR32_CMU_LFCLKSEL_LFXO 2u
#define EFR32_CMU_LFCLKSEL_ULFRCO 3u
#define EFR32_CMU_LFCLKSEL_MASK 0x3u
#define EFR32_ULFRCO_HZ 1000ULL /* xg22.dtsi ulfrco clock-frequency */

static int64_t efr32_cycles_ns(uint64_t cycles, uint64_t hz)
{
    return muldiv64_round_up(cycles, NANOSECONDS_PER_SECOND, hz);
}

static uint32_t efr32_cmu_get(Efr32Periph *s, uint32_t offset)
{
    return efr32_register_get(s, EFR32_CMU_BASE + offset);
}

/* Requests that hold an oscillator on; FORCEEN or an on-demand consumer. */
static bool efr32_hfxo_hw_requested(Efr32Periph *s)
{
    return (efr32_cmu_get(s, EFR32_CMU_SYSCLKCTRL) &
            EFR32_CMU_SYSCLKCTRL_CLKSEL_MASK) ==
               EFR32_CMU_SYSCLKCTRL_CLKSEL_HFXO ||
           (efr32_cmu_get(s, EFR32_CMU_EM01GRPACLKCTRL) & 0x3u) ==
               EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_HFXO ||
           (efr32_dpll_enabled(s) &&
            (efr32_cmu_get(s, EFR32_CMU_DPLLREFCLKCTRL) & 0x3u) ==
                EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_HFXO);
}

static bool efr32_lfxo_requested(Efr32Periph *s)
{
    return (efr32_register_get(s, EFR32_LFXO_BASE + EFR32_LFXO_CTRL) &
            EFR32_LFXO_CTRL_FORCEEN) ||
           (efr32_cmu_get(s, EFR32_CMU_EM4GRPACLKCTRL) &
            EFR32_CMU_LFCLKSEL_MASK) == EFR32_CMU_LFCLKSEL_LFXO ||
           (efr32_cmu_get(s, EFR32_CMU_EM23GRPACLKCTRL) &
            EFR32_CMU_LFCLKSEL_MASK) == EFR32_CMU_LFCLKSEL_LFXO ||
           (efr32_dpll_enabled(s) &&
            (efr32_cmu_get(s, EFR32_CMU_DPLLREFCLKCTRL) & 0x3u) ==
                EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_LFXO);
}

static bool efr32_burtc_enabled(Efr32Periph *s);

static bool efr32_lfrco_requested(Efr32Periph *s)
{
    return (efr32_register_get(s, EFR32_LFRCO_BASE + EFR32_LFRCO_CTRL) &
            EFR32_LFRCO_CTRL_FORCEEN) ||
           ((efr32_cmu_get(s, EFR32_CMU_EM4GRPACLKCTRL) &
             EFR32_CMU_LFCLKSEL_MASK) == EFR32_CMU_LFCLKSEL_LFRCO &&
            efr32_burtc_enabled(s));
}

static int64_t efr32_hfxo_ready_ns(Efr32Periph *s)
{
    return s->hfxo_request_ns +
           efr32_cycles_ns(EFR32_HFXO_RDY_FSRCO_CYCLES, EFR32_FSRCO_MEASURED_HZ);
}

static int64_t efr32_lfxo_ready_ns(Efr32Periph *s)
{
    uint32_t cfg = efr32_register_get(s, EFR32_LFXO_BASE + EFR32_LFXO_CFG);
    uint32_t timeout = (cfg & EFR32_LFXO_CFG_TIMEOUT_MASK) >>
                       EFR32_LFXO_CFG_TIMEOUT_SHIFT;

    return s->lfxo_request_ns +
           efr32_cycles_ns(efr32_lfxo_timeout_cycles[timeout], EFR32_LFXO_HZ);
}

static int64_t efr32_dpll_lock_ns(Efr32Periph *s)
{
    int64_t from = s->dpll_enable_ns;
    uint32_t ref = efr32_cmu_get(s, EFR32_CMU_DPLLREFCLKCTRL) & 0x3u;

    if (ref == EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_HFXO && s->hfxo_request_ns >= 0) {
        from = MAX(from, efr32_hfxo_ready_ns(s));
    } else if (ref == EFR32_CMU_DPLLREFCLKCTRL_CLKSEL_LFXO &&
               s->lfxo_request_ns >= 0) {
        from = MAX(from, efr32_lfxo_ready_ns(s));
    }
    return from + efr32_cycles_ns(EFR32_DPLL_LOCK_HFXO_CYCLES, EFR32_HFXO_HZ);
}

static void efr32_dpll_lock_expired(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    if (s->dpll_enable_ns < 0) {
        return;
    }
    efr32_register_put(s, EFR32_DPLL0_BASE + EFR32_DPLL_IF,
                       efr32_register_get(s, EFR32_DPLL0_BASE + EFR32_DPLL_IF) |
                           EFR32_DPLL_IF_LOCK);
}

/* Track request edges; called after every register write and at reset. */
static void efr32_osc_update(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool hfxo = efr32_hfxo_requested(s);
    bool lfxo = efr32_lfxo_requested(s);
    bool lfrco = efr32_lfrco_requested(s);
    bool dpll = efr32_dpll_enabled(s);

    if (hfxo && s->hfxo_request_ns < 0) {
        s->hfxo_request_ns = now;
    } else if (!hfxo) {
        s->hfxo_request_ns = -1;
    }
    if (lfxo && s->lfxo_request_ns < 0) {
        s->lfxo_request_ns = now;
    } else if (!lfxo) {
        s->lfxo_request_ns = -1;
    }
    if (lfrco && s->lfrco_request_ns < 0) {
        s->lfrco_request_ns = now;
    } else if (!lfrco && s->lfrco_request_ns >= 0) {
        s->lfrco_request_ns = -1;
        s->lfrco_off_ns = now;
    }
    if (dpll && s->dpll_enable_ns < 0) {
        s->dpll_enable_ns = now;
        timer_mod(s->dpll_lock_timer, efr32_dpll_lock_ns(s));
    } else if (!dpll && s->dpll_enable_ns >= 0) {
        s->dpll_enable_ns = -1;
        s->dpll_off_ns = now;
        timer_del(s->dpll_lock_timer);
    }
}

static uint32_t efr32_hfxo_status(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t el;
    uint32_t status = 0;

    if (s->hfxo_request_ns < 0) {
        return 0;
    }
    el = now - s->hfxo_request_ns;
    if (el >= efr32_cycles_ns(EFR32_HFXO_ENS_FSRCO_CYCLES,
                              EFR32_FSRCO_MEASURED_HZ)) {
        status |= EFR32_HFXO_STATUS_ENS;
    }
    if (el >= efr32_cycles_ns(EFR32_HFXO_RDY_FSRCO_CYCLES,
                              EFR32_FSRCO_MEASURED_HZ)) {
        status |= EFR32_HFXO_STATUS_RDY;
    }
    if (el >= efr32_cycles_ns(EFR32_HFXO_COREBIASOPTRDY_FSRCO_CYCLES,
                              EFR32_FSRCO_MEASURED_HZ)) {
        status |= EFR32_HFXO_STATUS_COREBIASOPTRDY;
    }
    /*
     * Running stage 3 (no FORCEEN) reads 0x40030003: HWREQ, and no ISWARM;
     * FORCEEN alone reads 0x40010003, without HWREQ.
     */
    if (efr32_hfxo_hw_requested(s)) {
        status |= EFR32_HFXO_STATUS_HWREQ;
    }
    if (s->hfxo_fsm_locked) {
        status |= EFR32_HFXO_STATUS_FSMLOCK;
    }
    return status;
}

static uint32_t efr32_dpll_status(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->dpll_enable_ns < 0) {
        return s->dpll_off_ns >= 0 &&
               now - s->dpll_off_ns <
                   efr32_cycles_ns(EFR32_DPLL_ENS_CLEAR_HFXO_CYCLES,
                                   EFR32_HFXO_HZ) ? EFR32_DPLL_STATUS_ENS : 0;
    }
    return EFR32_DPLL_STATUS_ENS |
           (now >= efr32_dpll_lock_ns(s) ? EFR32_DPLL_STATUS_RDY : 0);
}

static uint32_t efr32_lfxo_status(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->lfxo_request_ns < 0) {
        return 0;
    }
    return EFR32_OSC_STATUS_ENS |
           (now >= efr32_lfxo_ready_ns(s) ? EFR32_OSC_STATUS_RDY : 0);
}

static uint32_t efr32_lfrco_status(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t el;
    uint32_t status = 0;

    if (s->lfrco_request_ns < 0) {
        return s->lfrco_off_ns >= 0 &&
               now - s->lfrco_off_ns <
                   efr32_cycles_ns(EFR32_LFRCO_ENS_CLEAR_HCLK_CYCLES,
                                   EFR32_MEASURE_HCLK_HZ) ? EFR32_OSC_STATUS_ENS
                                                          : 0;
    }
    el = now - s->lfrco_request_ns;
    if (el >= efr32_cycles_ns(EFR32_LFRCO_ENS_HCLK_CYCLES,
                              EFR32_MEASURE_HCLK_HZ)) {
        status |= EFR32_OSC_STATUS_ENS;
    }
    if (el >= efr32_cycles_ns(EFR32_LFRCO_RDY_HCLK_CYCLES,
                              EFR32_MEASURE_HCLK_HZ)) {
        status |= EFR32_OSC_STATUS_RDY;
    }
    return status;
}

/*
 * --------------------------------------------------------- debug halt
 *
 * TIMER0-4, BURTC, RTCC and WDOG0 each have a DEBUGRUN bit, reset 0, in
 * the public efr32bg22_{timer,burtc,rtcc,wdog}.h ("Debug Mode Run
 * Enable"; TIMER_CFG bit 6, BURTC_CFG bit 0, RTCC_CFG bit 0, WDOG_CFG
 * bit 4).  With it clear the peripheral's count stops while the M33 is
 * halted in Debug state: the SDK documents the field as "Stop counter
 * during debug halt" (sl_hal_timer.h, and silabs_burtc_clock_event.c
 * configures BURTC "stop during debug halt"); for WDOG0 see RM SS27.3.2
 * and the WDOG0 section.  The halt is the debug unit's "halted" output
 * (DHCSR.S_HALT, GPIO in "m33-debug-halted"), not the sequencer M0+'s.
 * [I] unmeasured on silicon: the clock edge at which a TIMER/BURTC/RTCC
 * count stops and restarts relative to the halt (here the edge at the
 * halt and resume instants), and whether STATUS.RUNNING reads 1 meanwhile
 * (kept 1 here: the timer is still started).
 */
#define EFR32_TIMER_CFG_DEBUGRUN (1u << 6)
#define EFR32_BURTC_CFG_DEBUGRUN (1u << 0)
#define EFR32_RTCC_CFG_DEBUGRUN (1u << 0)

/* A peripheral with DEBUGRUN = @debugrun has its count stopped now. */
static bool efr32_debug_frozen(Efr32Periph *s, bool debugrun)
{
    return s->m33_debug_halted && !debugrun;
}

/*
 * ------------------------------------------------------------------ BURTC
 *
 * The counter advances on edges of the clock CMU.EM4GRPACLKCTRL selects
 * (LFRCO/LFXO at the board's 32.768 kHz, ULFRCO at 1 kHz), on a grid fixed at
 * time zero: the LF oscillators run across a system reset.
 *
 * Measured (BURTC on LFXO, DIV1, writes phase-swept across the LF period):
 *  - COMP, CNT, CMD.START and CMD.STOP writes take effect at the third LF
 *    edge after the write (n = 64 COMP, 32 CNT, 32 START, 32 STOP);
 *    SYNCBUSY.<reg> is set until then; COMP and CNT read their old values
 *    meanwhile.
 *  - The effective edge itself still runs on the old state: STOP lets that
 *    edge count (CNT advanced by 3 after the write), START does not (the
 *    first increment is the fourth edge), a written CNT is what reads at
 *    that edge.
 *  - IF.COMP sets on the edge where CNT advances from COMP to COMP + 1, and
 *    NVIC pending with it (n = 72).  COMP = CNT + d written at any phase
 *    fires for d >= 3 and waits for the 2^32 wrap for d <= 2 (n = 8 phases
 *    per d); a COMP rewritten while CNT reads COMP - k still fires the old
 *    match for k <= 2 and not for k >= 3 (n = 8 per k).  Both follow from
 *    the two rules above: the comparison on an edge uses the COMP that was
 *    effective before it.
 *  - EN: SYNCBUSY.EN clears 5.12-5.87 LF periods after EN is set (n = 8,
 *    phase swept) and 3.50 after it is cleared (n = 8, one phase), at the
 *    sixth / fourth falling edge after the write.  [I] that the grid of
 *    these completions is the falling one: no LF phase reference was
 *    available while the counter was stopped.  CNT survives EN = 0 -> 1 and
 *    the counter does not run again until START.
 *  - The counter and its LF clock keep running with CLKEN0.BURTC cleared
 *    (100 of 100 ticks, n = 3).
 * [I] CFG.CNTPRESC divides the count by 2^n while the synchronizers keep the
 * undivided LF edges.  UNMEASURED: PRECNT, a second write to a register
 * whose sync is still pending (the later write replaces it here), writes
 * while EN = 0 (synchronized the same way here), counting before the
 * selected oscillator reports RDY (not gated here).
 */
#define EFR32_BURTC_SYNCBUSY 0x028u
#define EFR32_BURTC_SYNCBUSY_START (1u << 0)
#define EFR32_BURTC_SYNCBUSY_STOP (1u << 1)
#define EFR32_BURTC_SYNCBUSY_CNT (1u << 3)
#define EFR32_BURTC_SYNCBUSY_COMP (1u << 4)
#define EFR32_BURTC_SYNCBUSY_EN (1u << 5)
#define EFR32_BURTC_IF_COMP (1u << 1)
#define EFR32_BURTC_CFG_MASK 0xf3u
#define EFR32_BURTC_CFG_CNTPRESC_SHIFT 4
#define EFR32_BURTC_CFG_CNTPRESC_MASK 0xf0u
#define EFR32_BURTC_IRQ 18
#define EFR32_BURTC_SYNC_EDGES 3
#define EFR32_BURTC_EN_SET_FALLING_EDGES 6
#define EFR32_BURTC_EN_CLR_FALLING_EDGES 4

enum { EFR32_BURTC_PEND_CMD, EFR32_BURTC_PEND_CNT, EFR32_BURTC_PEND_COMP,
       EFR32_BURTC_PEND_COUNT };

static const uint32_t efr32_burtc_pend_busy[EFR32_BURTC_PEND_COUNT] = {
    [EFR32_BURTC_PEND_CNT] = EFR32_BURTC_SYNCBUSY_CNT,
    [EFR32_BURTC_PEND_COMP] = EFR32_BURTC_SYNCBUSY_COMP,
};

static bool efr32_burtc_enabled(Efr32Periph *s)
{
    return efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_EN) &
           EFR32_BURTC_EN_EN;
}

static uint64_t efr32_burtc_hz(Efr32Periph *s)
{
    switch (efr32_cmu_get(s, EFR32_CMU_EM4GRPACLKCTRL) &
            EFR32_CMU_LFCLKSEL_MASK) {
    case EFR32_CMU_LFCLKSEL_LFRCO:
    case EFR32_CMU_LFCLKSEL_LFXO:
        return clock_get_hz(s->burtc_clk);
    case EFR32_CMU_LFCLKSEL_ULFRCO:
        return EFR32_ULFRCO_HZ;
    default:
        return 0;
    }
}

/* Index of the last LF rising edge at or before @t. */
static int64_t efr32_burtc_edge_at(Efr32Periph *s, int64_t t)
{
    return muldiv64(t, efr32_burtc_hz(s), NANOSECONDS_PER_SECOND);
}

static int64_t efr32_burtc_edge_ns(Efr32Periph *s, int64_t edge)
{
    return muldiv64_round_up(edge, NANOSECONDS_PER_SECOND, efr32_burtc_hz(s));
}

/* Time of the @n-th LF falling edge strictly after @t. */
static int64_t efr32_burtc_falling_ns(Efr32Periph *s, int64_t t, unsigned n)
{
    uint64_t hz2 = efr32_burtc_hz(s) * 2;
    int64_t half = muldiv64(t, hz2, NANOSECONDS_PER_SECOND);

    half += (half & 1) ? 2 : 1; /* first odd half-period index after t */
    half += 2 * (int64_t)(n - 1);
    return muldiv64_round_up(half, NANOSECONDS_PER_SECOND, hz2);
}

static uint32_t efr32_burtc_div(Efr32Periph *s)
{
    return 1u << ((efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_CFG) &
                   EFR32_BURTC_CFG_CNTPRESC_MASK) >>
                  EFR32_BURTC_CFG_CNTPRESC_SHIFT);
}

static bool efr32_burtc_counting(Efr32Periph *s)
{
    return s->burtc_run && efr32_burtc_enabled(s) &&
           !efr32_debug_frozen(s, efr32_register_get(s, EFR32_BURTC_BASE +
                                                        EFR32_BURTC_CFG) &
                                      EFR32_BURTC_CFG_DEBUGRUN);
}

static uint32_t efr32_burtc_cnt_at(Efr32Periph *s, int64_t edge)
{
    if (!efr32_burtc_counting(s) || edge <= s->burtc_cnt_edge) {
        return s->burtc_cnt_base;
    }
    return s->burtc_cnt_base +
           (uint32_t)((edge - s->burtc_cnt_edge) / efr32_burtc_div(s));
}

/* Restart the count's linear segment at @edge. */
static void efr32_burtc_rebase(Efr32Periph *s, int64_t edge)
{
    s->burtc_cnt_base = efr32_burtc_cnt_at(s, edge);
    s->burtc_cnt_edge = edge;
}

/* The next edge after burtc_last_edge where CNT advances out of COMP. */
static int64_t efr32_burtc_match_edge(Efr32Periph *s)
{
    uint64_t div = efr32_burtc_div(s);
    uint32_t d;
    int64_t m;

    if (!efr32_burtc_counting(s)) {
        return INT64_MAX;
    }
    d = s->burtc_comp - s->burtc_cnt_base;
    m = s->burtc_cnt_edge + ((int64_t)d + 1) * (int64_t)div;
    while (m <= s->burtc_last_edge) {
        m += (INT64_C(1) << 32) * (int64_t)div;
    }
    return m;
}

static void efr32_burtc_update_irq(Efr32Periph *s)
{
    uint32_t pending = efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_IF);
    uint32_t enabled = efr32_register_get(s, EFR32_BURTC_BASE + EFR32_BURTC_IEN);
    bool asserted = !!(pending & enabled);

    if (s->burtc_irq_asserted != asserted) {
        trace_efr32bg22_burtc_irq(s->endpoint_id,
                                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                  asserted, pending, enabled);
        s->burtc_irq_asserted = asserted;
    }
    qemu_set_irq(s->irq[EFR32_BURTC_IRQ], asserted);
}

static int64_t efr32_burtc_next_op_edge(Efr32Periph *s)
{
    int64_t e = INT64_MAX;

    for (int i = 0; i < EFR32_BURTC_PEND_COUNT; i++) {
        if (s->burtc_pend[i].valid) {
            e = MIN(e, s->burtc_pend[i].edge);
        }
    }
    return e;
}

/* Apply every edge up to now in order, then arm for the next one. */
static void efr32_burtc_process(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t k, next;

    timer_del(s->burtc_timer);
    if (!efr32_burtc_hz(s)) {
        return;
    }
    k = efr32_burtc_edge_at(s, now);
    for (;;) {
        int64_t e_op = efr32_burtc_next_op_edge(s);
        int64_t e_m = efr32_burtc_match_edge(s);
        int64_t e = MIN(e_op, e_m);

        if (e > k) {
            break;
        }
        if (e_m == e) {
            uint32_t flags = efr32_register_get(s, EFR32_BURTC_BASE +
                                                   EFR32_BURTC_IF);

            efr32_register_put(s, EFR32_BURTC_BASE + EFR32_BURTC_IF,
                               flags | EFR32_BURTC_IF_COMP);
            trace_efr32bg22_burtc_compare_expire(s->endpoint_id, now,
                                                 efr32_burtc_cnt_at(s, e),
                                                 s->burtc_comp);
            /* PRS BURTC COMP (RM Table 13.5) with IF.COMP. */
            efr32_prs_pulse(s, EFR32_PRS_SRC_BURTC, EFR32_PRS_SIG_BURTC_COMP);
        }
        if (e_op == e) {
            efr32_burtc_rebase(s, e);
            for (int i = 0; i < EFR32_BURTC_PEND_COUNT; i++) {
                if (!s->burtc_pend[i].valid || s->burtc_pend[i].edge != e) {
                    continue;
                }
                s->burtc_pend[i].valid = false;
                switch (i) {
                case EFR32_BURTC_PEND_CMD:
                    s->burtc_run = s->burtc_pend[i].value ==
                                   EFR32_BURTC_CMD_START;
                    break;
                case EFR32_BURTC_PEND_CNT:
                    s->burtc_cnt_base = s->burtc_pend[i].value;
                    break;
                case EFR32_BURTC_PEND_COMP:
                    s->burtc_comp = s->burtc_pend[i].value;
                    break;
                }
            }
        }
        s->burtc_last_edge = e;
    }
    efr32_burtc_update_irq(s);
    /* Arm last: see the -icount sleep=off note on timers armed in callbacks. */
    next = MIN(efr32_burtc_next_op_edge(s), efr32_burtc_match_edge(s));
    if (next != INT64_MAX) {
        timer_mod(s->burtc_timer, efr32_burtc_edge_ns(s, next));
    }
}

static void efr32_burtc_timer_expired(void *opaque)
{
    efr32_burtc_process(EFR32_PERIPH(opaque));
}

static void efr32_burtc_pend(Efr32Periph *s, int which, uint32_t value)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    efr32_burtc_process(s);
    s->burtc_pend[which].valid = true;
    s->burtc_pend[which].value = value;
    s->burtc_pend[which].edge = efr32_burtc_edge_at(s, now) +
                                EFR32_BURTC_SYNC_EDGES;
    trace_efr32bg22_burtc_compare_program(s->endpoint_id, now,
                                          efr32_burtc_cnt_at(s,
                                              efr32_burtc_edge_at(s, now)),
                                          value, EFR32_BURTC_SYNC_EDGES);
    efr32_burtc_process(s);
}

static uint32_t efr32_burtc_counter(Efr32Periph *s)
{
    efr32_burtc_process(s);
    return efr32_burtc_cnt_at(s, efr32_burtc_edge_at(s,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
}

static uint32_t efr32_burtc_comp(Efr32Periph *s)
{
    efr32_burtc_process(s);
    return s->burtc_comp;
}

static uint32_t efr32_burtc_syncbusy(Efr32Periph *s)
{
    uint32_t busy = 0;

    efr32_burtc_process(s);
    for (int i = 0; i < EFR32_BURTC_PEND_COUNT; i++) {
        if (!s->burtc_pend[i].valid) {
            continue;
        }
        busy |= i == EFR32_BURTC_PEND_CMD
                    ? (s->burtc_pend[i].value == EFR32_BURTC_CMD_START
                           ? EFR32_BURTC_SYNCBUSY_START
                           : EFR32_BURTC_SYNCBUSY_STOP)
                    : efr32_burtc_pend_busy[i];
    }
    if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->burtc_en_busy_until) {
        busy |= EFR32_BURTC_SYNCBUSY_EN;
    }
    return busy;
}

static uint32_t efr32_burtc_status(Efr32Periph *s)
{
    efr32_burtc_process(s);
    return s->burtc_run && efr32_burtc_enabled(s) ? EFR32_BURTC_STATUS_RUNNING
                                                  : 0;
}

/* Returns true when the write was fully handled. */
static bool efr32_burtc_write(Efr32Periph *s, uint32_t offset, uint32_t old,
                              uint32_t next)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    switch (offset) {
    case EFR32_BURTC_CMD:
        if (next & EFR32_BURTC_CMD_STOP) {
            efr32_burtc_pend(s, EFR32_BURTC_PEND_CMD, EFR32_BURTC_CMD_STOP);
        } else if (next & EFR32_BURTC_CMD_START) {
            efr32_burtc_pend(s, EFR32_BURTC_PEND_CMD, EFR32_BURTC_CMD_START);
        }
        return true;
    case EFR32_BURTC_CNT:
        efr32_burtc_pend(s, EFR32_BURTC_PEND_CNT, next);
        return true;
    case EFR32_BURTC_COMP:
        efr32_burtc_pend(s, EFR32_BURTC_PEND_COMP, next);
        return true;
    case EFR32_BURTC_STATUS:
    case EFR32_BURTC_SYNCBUSY:
        return true; /* generated */
    case EFR32_BURTC_EN:
        next &= EFR32_BURTC_EN_EN;
        efr32_burtc_process(s);
        efr32_burtc_rebase(s, efr32_burtc_edge_at(s, now));
        if (next != (old & EFR32_BURTC_EN_EN) && efr32_burtc_hz(s)) {
            s->burtc_en_busy_until =
                efr32_burtc_falling_ns(s, now,
                                       next ? EFR32_BURTC_EN_SET_FALLING_EDGES
                                            : EFR32_BURTC_EN_CLR_FALLING_EDGES);
        }
        efr32_register_put(s, EFR32_BURTC_BASE + offset, next);
        efr32_burtc_process(s);
        return true;
    case EFR32_BURTC_CFG:
        efr32_burtc_process(s);
        efr32_burtc_rebase(s, efr32_burtc_edge_at(s, now));
        efr32_register_put(s, EFR32_BURTC_BASE + offset,
                           next & EFR32_BURTC_CFG_MASK);
        efr32_burtc_process(s);
        return true;
    case EFR32_BURTC_IF:
    case EFR32_BURTC_IEN:
        efr32_register_put(s, EFR32_BURTC_BASE + offset, next & 0x3u);
        efr32_burtc_update_irq(s);
        return true;
    case EFR32_BURTC_PRECNT:
        efr32_register_put(s, EFR32_BURTC_BASE + offset, next & 0x7fffu);
        return true;
    default:
        return false;
    }
}

static void efr32_burtc_reset(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timer_del(s->burtc_timer);
    s->burtc_run = false;
    s->burtc_irq_asserted = false;
    s->burtc_cnt_base = 0;
    s->burtc_comp = UINT32_MAX;
    s->burtc_en_busy_until = 0;
    memset(s->burtc_pend, 0, sizeof(s->burtc_pend));
    s->burtc_cnt_edge = efr32_burtc_edge_at(s, now);
    s->burtc_last_edge = s->burtc_cnt_edge;
}

static void efr32_burtc_clock_update(void *opaque, ClockEvent event)
{
    /* The rate is read where it is used. */
}

/*
 * ------------------------------------------------------------- TIMER0-4
 *
 * One IP, five instances: TIMER0 is 32 bits wide, TIMER1-4 16 bits
 * (TIMERn_CNTWIDTH); each counts EM01GRPACLK / (CFG.PRESC + 1).  Only
 * CFG.MODE = UP and CC output compare are modelled.
 *
 * Measured (TIMER0 and TIMER1 at 76.8 MHz, CPU at the same clock):
 *  - PRESC = 0, 1, 3, 9 count exactly 1/(PRESC + 1) of HCLK (n = 4).
 *  - CMD.START and CMD.STOP act on the timer clock at the bus write, within
 *    +-1 cycle: a TOP = K overflow comes K + 10 TIMER1 ticks after a PRS
 *    software pulse issued one store earlier, the store spacing itself being
 *    10 (n = 32, K = 4..32); a START...STOP pair counts exactly the TIMER1
 *    ticks between their pulses (n = 16).  STATUS.SYNCBUSY stays set for 38
 *    HCLK cycles after a CMD write (n = 8).
 *  - An output compare fires on the tick where CNT advances out of OC: OC
 *    written equal to CNT fires at the end of that tick, OC = CNT + 1 one
 *    tick later, START with CNT == OC after one tick, OC behind CNT not
 *    until the wrap (PRESC = 1023, n = 6 / 6 / 6 / 4).  An OC write at full
 *    rate is effective within the 26-cycle read/write/read window that
 *    brackets it (threshold at OC = CNT + 19, n = 4 per value).
 *  - NVIC pending is seen in the same poll as IF (n = 16).
 *  - EN = 0: writes to CTRL, CMD, TOP, TOPB, CNT, CCn_CTRL, CCn_OC, CCn_OCB
 *    and DTCTRL are bus errors and their reads return 0, values retained;
 *    EN = 1: writes to CFG, CCn_CFG and DTCFG are bus errors (TIMER1 from
 *    the debugger, one write each; TIMER0 CC0_CTRL and CMD from the CPU,
 *    precise BusFault).  EN_SET followed by CMD.START in the next store
 *    starts the timer.
 *  - The counter keeps counting with its CLKEN bit cleared (n = 3).
 * UNMEASURED: the SYNCBUSY window after writes other than CMD; the effect of
 * EN = 0 on a running timer, kept running here.
 */
#define EFR32_TIMER_COUNT 5
#define EFR32_TIMER1_BASE 0x4004c000u
#define EFR32_TIMER3_BASE 0x40054000u
#define EFR32_TIMER_CTRL 0x008u
#define EFR32_TIMER_TOPB 0x020u
#define EFR32_TIMER_CC_CTRL(n) (0x064u + 0x20u * (n))
#define EFR32_TIMER_CC_OCB(n) (0x070u + 0x20u * (n))
#define EFR32_TIMER_DTCFG 0x0e0u
#define EFR32_TIMER_DTCTRL 0x0ecu
#define EFR32_TIMER_STATUS_SYNCBUSY (1u << 6)
#define EFR32_TIMER_CMD_SYNC_HCLK_CYCLES 38ULL
#define EFR32_TIMER_TOP_RESET 0xffffu

static const struct {
    uint32_t base;
    int irq;
    uint32_t width_mask;
    uint32_t clken_offset;
    uint32_t clken_bit;
} efr32_timers[EFR32_TIMER_COUNT] = {
    { EFR32_TIMER0_BASE, 7, UINT32_MAX, EFR32_CMU_CLKEN0, 1u << 4 },
    { EFR32_TIMER1_BASE, 8, 0xffffu, EFR32_CMU_CLKEN0, 1u << 5 },
    { EFR32_TIMER2_BASE, 9, 0xffffu, EFR32_CMU_CLKEN0, 1u << 6 },
    { EFR32_TIMER3_BASE, 10, 0xffffu, EFR32_CMU_CLKEN0, 1u << 7 },
    { EFR32_TIMER4_BASE, 11, 0xffffu, EFR32_CMU_CLKEN1, 1u << 18 },
};

#define EFR32_CMU_CLKEN0_BURTC (1u << 29)
#define EFR32_CMU_CLKEN0_RTCC (1u << 30)
#define EFR32_RTCC_BASE 0x48000000u

static int efr32_timer_index(uint32_t address)
{
    uint32_t block = address & ~0x3fffu;

    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        if (block == efr32_timers[i].base) {
            return i;
        }
    }
    return -1;
}

static uint32_t efr32_timer_reg(Efr32Periph *s, int i, uint32_t offset)
{
    return efr32_register_get(s, efr32_timers[i].base + offset);
}

static bool efr32_timer_enabled(Efr32Periph *s, int i)
{
    return efr32_timer_reg(s, i, EFR32_TIMER_EN) & EFR32_TIMER_EN_EN;
}

/* Started (STATUS.RUNNING), whether or not a debug halt stops the count. */
static bool efr32_timer_started(Efr32Periph *s, int i)
{
    return s->timers[i].running && efr32_timer_enabled(s, i) &&
           !(efr32_timer_reg(s, i, EFR32_TIMER_CFG) & EFR32_TIMER_CFG_MODE_MASK);
}

static bool efr32_timer_counting(Efr32Periph *s, int i)
{
    return efr32_timer_started(s, i) &&
           !efr32_debug_frozen(s, efr32_timer_reg(s, i, EFR32_TIMER_CFG) &
                                      EFR32_TIMER_CFG_DEBUGRUN);
}

static bool efr32_timer_sync_reg(uint32_t offset)
{
    switch (offset) {
    case EFR32_TIMER_CTRL:
    case EFR32_TIMER_CMD:
    case EFR32_TIMER_TOP:
    case EFR32_TIMER_TOPB:
    case EFR32_TIMER_CNT:
    case EFR32_TIMER_DTCTRL:
        return true;
    default:
        for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
            if (offset == EFR32_TIMER_CC_CTRL(n) ||
                offset == EFR32_TIMER_CC_OC(n) ||
                offset == EFR32_TIMER_CC_OCB(n)) {
                return true;
            }
        }
        return false;
    }
}

static bool efr32_timer_config_reg(uint32_t offset)
{
    if (offset == EFR32_TIMER_CFG || offset == EFR32_TIMER_DTCFG) {
        return true;
    }
    for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
        if (offset == EFR32_TIMER_CC_CFG(n)) {
            return true;
        }
    }
    return false;
}

static bool efr32_timer_clock_gated(Efr32Periph *s, uint32_t block)
{
    int i = efr32_timer_index(block);

    if (i >= 0) {
        return !(efr32_cmu_get(s, efr32_timers[i].clken_offset) &
                 efr32_timers[i].clken_bit);
    }
    if (block == EFR32_BURTC_BASE) {
        return !(efr32_cmu_get(s, EFR32_CMU_CLKEN0) & EFR32_CMU_CLKEN0_BURTC);
    }
    /*
     * RTCC with CLKEN0.RTCC (RM 8.5.10 bit 30) clear: a read is a bus error
     * (silicon, owo-open-fw docs/radio-tables.md SS7.73, debugger AP), and a
     * CPU load right after CLKEN0_SET does not fault.  [I] A gated write
     * faults as TIMER1/TIMER2 CFG and PRS SWLEVEL writes measured there do.
     */
    if (block == EFR32_RTCC_BASE) {
        return !(efr32_cmu_get(s, EFR32_CMU_CLKEN0) & EFR32_CMU_CLKEN0_RTCC);
    }
    return false;
}

/* A write the TIMER IP answers with a bus error in its current EN state. */
static bool efr32_timer_write_rejected(Efr32Periph *s, uint32_t address)
{
    int i = efr32_timer_index(address);
    uint32_t offset = address & 0xfffu;

    if (i < 0) {
        return false;
    }
    return efr32_timer_enabled(s, i) ? efr32_timer_config_reg(offset)
                                     : efr32_timer_sync_reg(offset);
}

static uint64_t efr32_timer_hz(Efr32Periph *s, int i)
{
    uint32_t presc = (efr32_timer_reg(s, i, EFR32_TIMER_CFG) &
                      EFR32_TIMER_CFG_PRESC_MASK) >> EFR32_TIMER_CFG_PRESC_SHIFT;

    return efr32_em01grpa_hz(s) / (presc + 1);
}

static uint64_t efr32_timer_period(Efr32Periph *s, int i)
{
    return (uint64_t)(efr32_timer_reg(s, i, EFR32_TIMER_TOP) &
                      efr32_timers[i].width_mask) + 1;
}

static uint64_t efr32_timer_elapsed(Efr32Periph *s, int i, int64_t now)
{
    if (!efr32_timer_counting(s, i) || now <= s->timers[i].base_ns) {
        return 0;
    }
    return muldiv64(now - s->timers[i].base_ns, efr32_timer_hz(s, i),
                    NANOSECONDS_PER_SECOND);
}

static uint32_t efr32_timer_counter(Efr32Periph *s, int i)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return ((uint64_t)s->timers[i].base_count +
            efr32_timer_elapsed(s, i, now)) % efr32_timer_period(s, i);
}

static void efr32_timer_update_irq(Efr32Periph *s, int i)
{
    uint32_t pending = efr32_timer_reg(s, i, EFR32_TIMER_IF);
    uint32_t enabled = efr32_timer_reg(s, i, EFR32_TIMER_IEN);

    qemu_set_irq(s->irq[efr32_timers[i].irq], !!(pending & enabled));
}

/* Freeze the current count as the new base before a rate or state change. */
static void efr32_timer_rebase(Efr32Periph *s, int i)
{
    s->timers[i].base_count = efr32_timer_counter(s, i);
    s->timers[i].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void efr32_timer0_rebase(Efr32Periph *s)
{
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        efr32_timer_rebase(s, i);
    }
}

/*
 * Schedule the next tick that raises a flag.  CC channel n and overflow both
 * fire on the tick where CNT advances out of their value (OC, TOP), so that
 * value is ((value - CNT) mod period) + 1 ticks away.
 */
static void efr32_timer_schedule(Efr32Periph *s, int i)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t period = efr32_timer_period(s, i);
    uint64_t elapsed, count, best = UINT64_MAX;
    uint32_t flags = 0;

    timer_del(s->timers[i].event);
    s->timers[i].pending_flags = 0;
    if (!efr32_timer_counting(s, i)) {
        return;
    }
    elapsed = efr32_timer_elapsed(s, i, now);
    count = ((uint64_t)s->timers[i].base_count + elapsed) % period;
    for (int n = -1; n < (int)EFR32_TIMER0_CHANNELS; n++) {
        uint64_t value, ticks;
        uint32_t flag;

        if (n < 0) {
            value = period - 1;
            flag = EFR32_TIMER_IF_OF;
        } else {
            if ((efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
                 EFR32_TIMER_CC_CFG_MODE_MASK) !=
                EFR32_TIMER_CC_CFG_MODE_OUTPUTCOMPARE) {
                continue;
            }
            value = efr32_timer_reg(s, i, EFR32_TIMER_CC_OC(n)) &
                    efr32_timers[i].width_mask;
            if (value >= period) {
                continue; /* never reached below TOP */
            }
            flag = EFR32_TIMER_IF_CC(n);
        }
        ticks = (value + period - count) % period + 1;
        if (ticks < best) {
            best = ticks;
            flags = flag;
        } else if (ticks == best) {
            flags |= flag;
        }
    }
    s->timers[i].pending_flags = flags;
    timer_mod(s->timers[i].event,
              s->timers[i].base_ns +
                  muldiv64_round_up(elapsed + best, NANOSECONDS_PER_SECOND,
                                    efr32_timer_hz(s, i)));
}

/*
 * Compare/PWM outputs (RM SS19.3.3.9, Figure 19.20): in up-count PWM mode
 * (CCn_CFG.MODE = PWM) the output is high while CNT < CCn_OC, so the period
 * is TOP + 1 and the output is high for OC cycles (constant high for
 * OC >= TOP + 1, constant low for OC = 0); CCn_CTRL.OUTINV inverts it.  A
 * channel not in PWM mode drives CCn_CFG.COIST (its initial state) and a
 * stopped timer holds its outputs.  Writes to TOPB and CCn_OCB set
 * STATUS.TOPBV/OCBVn and take effect at the next overflow ("Buffer update"
 * in Figure 19.20).  The outputs reach pins through the GPIO routes.
 * [U] Not modelled: output-compare match actions (CMOA/COFOA) on the pin,
 * up/down and 2x count, dead-time insertion, and the one-cycle output
 * delay the RM mentions.
 */
#define EFR32_TIMER_CC_CFG_MODE_PWM 0x3u
#define EFR32_TIMER_CC_CFG_COIST (1u << 4)
#define EFR32_TIMER_CC_CTRL_OUTINV (1u << 2)
#define EFR32_TIMER_STATUS_TOPBV (1u << 2)
#define EFR32_TIMER_STATUS_OCBV_SHIFT 8
#define EFR32_TIMER_BUF_TOPB (1u << 31)

static void efr32_timer_prs_cc_level(Efr32Periph *s, int i, unsigned n);

static bool efr32_timer_cc_is_pwm(Efr32Periph *s, int i, unsigned n)
{
    return (efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
            EFR32_TIMER_CC_CFG_MODE_MASK) == EFR32_TIMER_CC_CFG_MODE_PWM;
}

static bool efr32_timer_cc_level(Efr32Periph *s, int i, unsigned n,
                                 uint64_t count)
{
    bool level;

    if (!efr32_timer_cc_is_pwm(s, i, n)) {
        level = efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
                EFR32_TIMER_CC_CFG_COIST;
    } else if (!efr32_timer_counting(s, i)) {
        return s->timer_cc_level[i][n]; /* held */
    } else {
        level = count < (efr32_timer_reg(s, i, EFR32_TIMER_CC_OC(n)) &
                         efr32_timers[i].width_mask);
    }
    return level ^ !!(efr32_timer_reg(s, i, EFR32_TIMER_CC_CTRL(n)) &
                      EFR32_TIMER_CC_CTRL_OUTINV);
}

static void efr32_timer_pwm_update(Efr32Periph *s, int i)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t period = efr32_timer_period(s, i);
    uint64_t elapsed = efr32_timer_elapsed(s, i, now);
    uint64_t count = ((uint64_t)s->timers[i].base_count + elapsed) % period;
    uint64_t best = UINT64_MAX;

    timer_del(s->timer_pwm[i]);
    for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
        bool level = efr32_timer_cc_level(s, i, n, count);

        if (level != s->timer_cc_level[i][n]) {
            s->timer_cc_level[i][n] = level;
            qemu_set_irq(s->timer_cc_out[i * EFR32_TIMER0_CHANNELS + n],
                         level);
        }
        efr32_timer_prs_cc_level(s, i, n);
        if (efr32_timer_counting(s, i) && efr32_timer_cc_is_pwm(s, i, n)) {
            uint64_t oc = efr32_timer_reg(s, i, EFR32_TIMER_CC_OC(n)) &
                          efr32_timers[i].width_mask;

            if (count < oc && oc < period) {
                best = MIN(best, oc - count);      /* falls at CNT = OC */
            } else if (count >= oc && oc > 0) {
                best = MIN(best, period - count);  /* rises at the wrap */
            }
        }
    }
    if (efr32_timer_counting(s, i) && s->timer_buf_pending[i]) {
        best = MIN(best, period - count);          /* buffer update */
    }
    if (best != UINT64_MAX) {
        timer_mod(s->timer_pwm[i],
                  s->timers[i].base_ns +
                      muldiv64_round_up(elapsed + best,
                                        NANOSECONDS_PER_SECOND,
                                        efr32_timer_hz(s, i)));
    }
}

static void efr32_timer_pwm_expired(Efr32Periph *s, int i)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t period = efr32_timer_period(s, i);
    uint64_t count = ((uint64_t)s->timers[i].base_count +
                      efr32_timer_elapsed(s, i, now)) % period;

    if (count == 0 && s->timer_buf_pending[i]) {
        uint32_t pending = s->timer_buf_pending[i];

        efr32_timer_rebase(s, i);
        if (pending & EFR32_TIMER_BUF_TOPB) {
            efr32_register_put(s, efr32_timers[i].base + EFR32_TIMER_TOP,
                               efr32_timer_reg(s, i, EFR32_TIMER_TOPB) &
                                   efr32_timers[i].width_mask);
        }
        for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
            if (pending & (1u << n)) {
                efr32_register_put(s, efr32_timers[i].base +
                                          EFR32_TIMER_CC_OC(n),
                                   efr32_timer_reg(s, i,
                                                   EFR32_TIMER_CC_OCB(n)));
            }
        }
        s->timer_buf_pending[i] = 0;
        efr32_timer_schedule(s, i);
    }
    efr32_timer_pwm_update(s, i);
}

static void efr32_timer0_pwm(void *opaque) { efr32_timer_pwm_expired(EFR32_PERIPH(opaque), 0); }
static void efr32_timer1_pwm(void *opaque) { efr32_timer_pwm_expired(EFR32_PERIPH(opaque), 1); }
static void efr32_timer2_pwm(void *opaque) { efr32_timer_pwm_expired(EFR32_PERIPH(opaque), 2); }
static void efr32_timer3_pwm(void *opaque) { efr32_timer_pwm_expired(EFR32_PERIPH(opaque), 3); }
static void efr32_timer4_pwm(void *opaque) { efr32_timer_pwm_expired(EFR32_PERIPH(opaque), 4); }

static QEMUTimerCB *const efr32_timer_pwm_callbacks[EFR32_TIMER_COUNT] = {
    efr32_timer0_pwm, efr32_timer1_pwm, efr32_timer2_pwm, efr32_timer3_pwm,
    efr32_timer4_pwm,
};

static void efr32_timer0_schedule(Efr32Periph *s)
{
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        efr32_timer_schedule(s, i);
        efr32_timer_pwm_update(s, i);
    }
}

/*
 * PRS outputs (RM SS19.3.6, Tables 13.4/13.5): OF and UF are pulses; a CC
 * channel pulses on its compare match or capture with CCn_CFG.PRSCONF =
 * PULSE, and with PRSCONF = LEVEL follows its compare output
 * (efr32_timer_prs_cc_level).  Each goes to both the asynchronous and the
 * synchronous producer of the timer.
 */
static const uint8_t efr32_timer_prs_source[EFR32_TIMER_COUNT] = {
    EFR32_PRS_SRC_TIMER0, EFR32_PRS_SRC_TIMER1, EFR32_PRS_SRC_TIMER2,
    EFR32_PRS_SRC_TIMER3, EFR32_PRS_SRC_TIMER4,
};
static const uint8_t efr32_timer_prs_sync_source[EFR32_TIMER_COUNT] = {
    EFR32_PRS_SYNC_SRC_TIMER0, EFR32_PRS_SYNC_SRC_TIMER1,
    EFR32_PRS_SYNC_SRC_TIMER2, EFR32_PRS_SYNC_SRC_TIMER3,
    EFR32_PRS_SYNC_SRC_TIMER4,
};
#define EFR32_TIMER_CFG_OSMEN (1u << 4)
#define EFR32_TIMER_CC_CFG_PRSCONF_LEVEL (1u << 19)

static void efr32_timer_prs_event(Efr32Periph *s, int i, unsigned signal)
{
    efr32_prs_pulse(s, efr32_timer_prs_source[i], signal);
    efr32_prs_sync_pulse(s, efr32_timer_prs_sync_source[i], signal);
}

static void efr32_timer_prs_cc_event(Efr32Periph *s, int i, unsigned n)
{
    if (!(efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
          EFR32_TIMER_CC_CFG_PRSCONF_LEVEL)) {
        efr32_timer_prs_event(s, i, EFR32_PRS_SIG_TIMER_CC(n));
    }
}

static void efr32_timer_prs_cc_level(Efr32Periph *s, int i, unsigned n)
{
    bool level = (efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
                  EFR32_TIMER_CC_CFG_PRSCONF_LEVEL) && s->timer_cc_level[i][n];

    if (level != s->timer_prs_level[i][n]) {
        s->timer_prs_level[i][n] = level;
        efr32_prs_level(s, efr32_timer_prs_source[i],
                        EFR32_PRS_SIG_TIMER_CC(n), level);
        qemu_set_irq(s->prs_sync[EFR32_PRS_LINE(
                         efr32_timer_prs_sync_source[i],
                         EFR32_PRS_SIG_TIMER_CC(n))], level);
    }
}

static void efr32_timer_event(Efr32Periph *s, int i)
{
    uint32_t flags = s->timers[i].pending_flags;

    if (!efr32_timer_counting(s, i)) {
        return;
    }
    efr32_register_put(s, efr32_timers[i].base + EFR32_TIMER_IF,
                       efr32_timer_reg(s, i, EFR32_TIMER_IF) | flags);
    if (i == 0) {
        trace_efr32bg22_timer0_event(s->endpoint_id,
                                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                     efr32_timer_counter(s, i), flags);
    }
    /*
     * One-shot (CFG.OSMEN, RM SS19.3.2.7): the hardware disables the
     * counter on the first update event, the overflow in up-count mode.
     * [I] It holds TOP afterwards (owo-open-fw scripts/bg22-tifs-probe.tcl:
     * "a capture long after EOF reads TOP"); the RM does not say.
     */
    if ((flags & EFR32_TIMER_IF_OF) &&
        (efr32_timer_reg(s, i, EFR32_TIMER_CFG) & EFR32_TIMER_CFG_OSMEN)) {
        efr32_timer_rebase(s, i);
        s->timers[i].running = false;
        s->timers[i].base_count = efr32_timer_period(s, i) - 1;
    }
    efr32_timer_update_irq(s, i);
    efr32_timer_schedule(s, i);
    if (flags & EFR32_TIMER_IF_OF) {
        efr32_timer_prs_event(s, i, EFR32_PRS_SIG_TIMER_OF);
        /* RM SS19.3.6: each event also sets a DMA request; [I] a pulse. */
        qemu_irq_pulse(s->timer_dma[i][EFR32_LDMA_SIG_TIMER_UFOF]);
    }
    for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
        if (flags & EFR32_TIMER_IF_CC(n)) {
            efr32_timer_prs_cc_event(s, i, n);
            qemu_irq_pulse(s->timer_dma[i][EFR32_LDMA_SIG_TIMER_CC(n)]);
        }
    }
}

/*
 * ------------------------------------------------- TIMER input capture
 *
 * RM SS19.3.3.1-19.3.3.4 and SS19.3.2.2.  CCn_CFG.INSEL picks the channel
 * input: PRSSYNC (the synchronous channel PRS_CONSUMER_TIMERn_CCx.SPRSSEL
 * selects), PRSASYNCLEVEL or PRSASYNCPULSE (the asynchronous channel
 * PRSSEL selects).
 *  - PRSASYNCPULSE recognises edges directly (no level qualification).
 *  - PRSASYNCLEVEL passes through the 2-clock input sampler: the new level
 *    must hold for 2 HFPERCLKTIMERn (EM01GRPACLK) cycles, 7 with
 *    CCn_CFG.FILT, and is recognised when it has.  A producer's pulse is
 *    never seen in this mode, which is what silicon shows for the MODEM
 *    events ("PRSASYNCLEVEL captured nothing", owo-open-fw
 *    docs/radio-tables.md SS7.71).
 *  - PRSSYNC is synchronous already: no sampler; [I] FILT's 5 cycles are
 *    applied like the sampler's.
 * A recognised edge that matches CCn_CTRL.ICEDGE captures CNT in input
 * capture mode: a 2-entry FIFO read through CCn_ICF (popping; underflow
 * sets IF.ICFUFn), CCn_ICOF the newest capture (overwritten on overflow,
 * IF.ICFOFn), IF.ICFWLFULLn when the free entries are <= CCn_CFG.ICFWL,
 * STATUS.ICFEMPTYn and CCPOLn (the edge of the oldest entry).  IF.CCn only
 * while the timer runs ("the timer will only trigger interrupts when it is
 * running"); the capture itself happens either way, as the probe's
 * one-shot timer on silicon shows (captures after it stopped read TOP).
 * Channel 0's input edges also run CTRL.RISEA / FALLA (start, stop,
 * reload-and-start, the reload value being 0 in up-count), after the
 * capture so a period capture reads the period (Figure 19.13).
 * Debugger reads of CCn_ICF do not pop the FIFO (silicon, SS7.71).
 * Timing: silicon puts CMD.START and a PRS SWPULSE reload-start within +-1
 * timer cycle of each other (SS7.73); the edge acts at once here.
 * [U] Not modelled: INSEL = PIN (no input path from the GPIO routes),
 * the quadrature decoder, CC1 as the counter clock, DTI fault inputs, and
 * the 3-clock minimum spacing of PRSASYNCPULSE events.
 */
#define EFR32_TIMER_CTRL_RISEA_SHIFT 0
#define EFR32_TIMER_CTRL_FALLA_SHIFT 2
#define EFR32_TIMER_CTRL_ACTION_MASK 0x3u
#define EFR32_TIMER_ACTION_START 1u
#define EFR32_TIMER_ACTION_STOP 2u
#define EFR32_TIMER_ACTION_RELOADSTART 3u
#define EFR32_TIMER_CC_CFG_MODE_INPUTCAPTURE 0x1u
#define EFR32_TIMER_CC_CFG_INSEL_SHIFT 17
#define EFR32_TIMER_CC_CFG_INSEL_MASK 0x3u
#define EFR32_TIMER_INSEL_PIN 0u
#define EFR32_TIMER_INSEL_PRSSYNC 1u
#define EFR32_TIMER_INSEL_PRSASYNCLEVEL 2u
#define EFR32_TIMER_INSEL_PRSASYNCPULSE 3u
#define EFR32_TIMER_CC_CFG_FILT (1u << 20)
#define EFR32_TIMER_CC_CFG_ICFWL (1u << 21)
#define EFR32_TIMER_CC_CTRL_ICEDGE_SHIFT 24
#define EFR32_TIMER_CC_CTRL_ICEDGE_MASK 0x3u
#define EFR32_TIMER_ICEDGE_RISING 0u
#define EFR32_TIMER_ICEDGE_FALLING 1u
#define EFR32_TIMER_ICEDGE_BOTH 2u
#define EFR32_TIMER_CC_ICF(n) (0x074u + 0x20u * (n))
#define EFR32_TIMER_CC_ICOF(n) (0x078u + 0x20u * (n))
#define EFR32_TIMER_IF_ICFWLFULL(n) (1u << (16 + (n)))
#define EFR32_TIMER_IF_ICFOF(n) (1u << (20 + (n)))
#define EFR32_TIMER_IF_ICFUF(n) (1u << (24 + (n)))
#define EFR32_TIMER_STATUS_ICFEMPTY(n) (1u << (16 + (n)))
#define EFR32_TIMER_STATUS_CCPOL(n) (1u << (24 + (n)))
#define EFR32_TIMER_IC_FIFO_DEPTH 2u
#define EFR32_TIMER_SAMPLER_CYCLES 2u
#define EFR32_TIMER_FILTER_CYCLES 5u

typedef typeof(((Efr32Periph *)0)->timer_ic[0][0]) Efr32TimerIc;

static unsigned efr32_timer_insel(Efr32Periph *s, int i, unsigned n)
{
    return (efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) >>
            EFR32_TIMER_CC_CFG_INSEL_SHIFT) & EFR32_TIMER_CC_CFG_INSEL_MASK;
}

static void efr32_timer_ic_capture(Efr32Periph *s, int i, unsigned n,
                                   bool rising)
{
    Efr32TimerIc *ic = &s->timer_ic[i][n];
    uint32_t value = efr32_timer_counter(s, i);
    uint32_t flags = 0;
    unsigned free_entries;

    if (ic->count < EFR32_TIMER_IC_FIFO_DEPTH) {
        ic->fifo[ic->count] = value;
        ic->fifo_falling[ic->count] = !rising;
        ic->count++;
    } else {
        flags |= EFR32_TIMER_IF_ICFOF(n);
    }
    ic->icof = value;
    free_entries = EFR32_TIMER_IC_FIFO_DEPTH - ic->count;
    if (free_entries <= !!(efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
                           EFR32_TIMER_CC_CFG_ICFWL)) {
        flags |= EFR32_TIMER_IF_ICFWLFULL(n);
    }
    if (efr32_timer_started(s, i)) {
        flags |= EFR32_TIMER_IF_CC(n);
    }
    efr32_register_put(s, efr32_timers[i].base + EFR32_TIMER_IF,
                       efr32_timer_reg(s, i, EFR32_TIMER_IF) | flags);
    efr32_timer_update_irq(s, i);
    efr32_timer_prs_cc_event(s, i, n);
    qemu_irq_pulse(s->timer_dma[i][EFR32_LDMA_SIG_TIMER_CC(n)]);
}

static void efr32_timer_counter_action(Efr32Periph *s, int i, unsigned action)
{
    switch (action) {
    case EFR32_TIMER_ACTION_START:
        efr32_timer_rebase(s, i);
        s->timers[i].running = true;
        break;
    case EFR32_TIMER_ACTION_STOP:
        efr32_timer_rebase(s, i);
        s->timers[i].running = false;
        break;
    case EFR32_TIMER_ACTION_RELOADSTART:
        efr32_timer_rebase(s, i);
        s->timers[i].base_count = 0;
        s->timers[i].running = true;
        break;
    default:
        return;
    }
    efr32_timer_schedule(s, i);
    efr32_timer_pwm_update(s, i);
}

/* A recognised edge of channel @n's selected input. */
static void efr32_timer_ic_edge(Efr32Periph *s, int i, unsigned n,
                                bool rising)
{
    uint32_t cfg = efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n));
    unsigned edge = (efr32_timer_reg(s, i, EFR32_TIMER_CC_CTRL(n)) >>
                     EFR32_TIMER_CC_CTRL_ICEDGE_SHIFT) &
                    EFR32_TIMER_CC_CTRL_ICEDGE_MASK;

    if (!efr32_timer_enabled(s, i)) {
        return;
    }
    if ((cfg & EFR32_TIMER_CC_CFG_MODE_MASK) ==
            EFR32_TIMER_CC_CFG_MODE_INPUTCAPTURE &&
        (edge == EFR32_TIMER_ICEDGE_BOTH ||
         (edge == EFR32_TIMER_ICEDGE_RISING && rising) ||
         (edge == EFR32_TIMER_ICEDGE_FALLING && !rising))) {
        efr32_timer_ic_capture(s, i, n, rising);
    }
    if (n == 0) {
        uint32_t ctrl = efr32_timer_reg(s, i, EFR32_TIMER_CTRL);
        unsigned action = (ctrl >> (rising ? EFR32_TIMER_CTRL_RISEA_SHIFT
                                           : EFR32_TIMER_CTRL_FALLA_SHIFT)) &
                          EFR32_TIMER_CTRL_ACTION_MASK;

        efr32_timer_counter_action(s, i, action);
    }
}

static void efr32_timer_ic_sampled(void *opaque)
{
    Efr32TimerIc *ic = opaque;
    Efr32Periph *s = ic->owner;
    unsigned insel = efr32_timer_insel(s, ic->timer, ic->channel);
    bool level = insel == EFR32_TIMER_INSEL_PRSSYNC ? ic->sync_level
                                                    : ic->async_level;

    if (level != ic->sampled) {
        ic->sampled = level;
        efr32_timer_ic_edge(s, ic->timer, ic->channel, level);
    }
}

static void efr32_timer_ic_input(Efr32Periph *s, int i, unsigned n,
                                 bool sync, bool level)
{
    Efr32TimerIc *ic = &s->timer_ic[i][n];
    unsigned insel = efr32_timer_insel(s, i, n);
    unsigned cycles = 0;

    if (sync) {
        ic->sync_level = level;
    } else {
        ic->async_level = level;
    }
    if (sync != (insel == EFR32_TIMER_INSEL_PRSSYNC) ||
        insel == EFR32_TIMER_INSEL_PIN) {
        return;
    }
    if (insel == EFR32_TIMER_INSEL_PRSASYNCLEVEL) {
        cycles = EFR32_TIMER_SAMPLER_CYCLES;
    }
    if (efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
        EFR32_TIMER_CC_CFG_FILT && insel != EFR32_TIMER_INSEL_PRSASYNCPULSE) {
        cycles += EFR32_TIMER_FILTER_CYCLES;
    }
    if (!cycles) {
        ic->sampled = level;
        efr32_timer_ic_edge(s, i, n, level);
        return;
    }
    timer_mod(ic->sampler, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                               efr32_cycles_ns(cycles, efr32_em01grpa_hz(s)));
}

static uint32_t efr32_timer_ic_status(Efr32Periph *s, int i)
{
    uint32_t status = 0;

    /*
     * [I] ICFEMPTYn resets to 0 (efr32bg22_timer.h, RM SS19.5.5), so it is
     * taken to report an empty FIFO only for an enabled capture channel.
     */
    for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
        bool capture = efr32_timer_enabled(s, i) &&
                       (efr32_timer_reg(s, i, EFR32_TIMER_CC_CFG(n)) &
                        EFR32_TIMER_CC_CFG_MODE_MASK) ==
                           EFR32_TIMER_CC_CFG_MODE_INPUTCAPTURE;

        if (!s->timer_ic[i][n].count) {
            status |= capture ? EFR32_TIMER_STATUS_ICFEMPTY(n) : 0;
        } else if (s->timer_ic[i][n].fifo_falling[0]) {
            status |= EFR32_TIMER_STATUS_CCPOL(n);
        }
    }
    return status;
}

static uint32_t efr32_timer_ic_pop(Efr32Periph *s, int i, unsigned n)
{
    Efr32TimerIc *ic = &s->timer_ic[i][n];
    uint32_t value;

    if (!ic->count) {
        /* [I] An empty FIFO reads the newest capture. */
        if (!s->debugger_access) {
            efr32_register_put(s, efr32_timers[i].base + EFR32_TIMER_IF,
                               efr32_timer_reg(s, i, EFR32_TIMER_IF) |
                                   EFR32_TIMER_IF_ICFUF(n));
            efr32_timer_update_irq(s, i);
        }
        return ic->icof;
    }
    value = ic->fifo[0];
    if (!s->debugger_access) {
        ic->fifo[0] = ic->fifo[1];
        ic->fifo_falling[0] = ic->fifo_falling[1];
        ic->count--;
    }
    return value;
}

static void efr32_timer0_expired(void *opaque) { efr32_timer_event(EFR32_PERIPH(opaque), 0); }
static void efr32_timer1_expired(void *opaque) { efr32_timer_event(EFR32_PERIPH(opaque), 1); }
static void efr32_timer2_expired(void *opaque) { efr32_timer_event(EFR32_PERIPH(opaque), 2); }
static void efr32_timer3_expired(void *opaque) { efr32_timer_event(EFR32_PERIPH(opaque), 3); }
static void efr32_timer4_expired(void *opaque) { efr32_timer_event(EFR32_PERIPH(opaque), 4); }

static QEMUTimerCB *const efr32_timer_callbacks[EFR32_TIMER_COUNT] = {
    efr32_timer0_expired, efr32_timer1_expired, efr32_timer2_expired,
    efr32_timer3_expired, efr32_timer4_expired,
};

/* Generated and EN-gated reads; returns false for plain register reads. */
static bool efr32_timer_read(Efr32Periph *s, uint32_t address, uint32_t *value)
{
    int i = efr32_timer_index(address);
    uint32_t offset = address & 0xfffu;

    if (i < 0) {
        return false;
    }
    if (!efr32_timer_enabled(s, i) && efr32_timer_sync_reg(offset)) {
        *value = 0;
        return true;
    }
    switch (offset) {
    case EFR32_TIMER_CNT:
        *value = efr32_timer_counter(s, i);
        return true;
    case EFR32_TIMER_CC_ICF(0):
    case EFR32_TIMER_CC_ICF(1):
    case EFR32_TIMER_CC_ICF(2):
        *value = efr32_timer_ic_pop(s, i, (offset - EFR32_TIMER_CC_ICF(0)) /
                                              0x20u);
        return true;
    case EFR32_TIMER_CC_ICOF(0):
    case EFR32_TIMER_CC_ICOF(1):
    case EFR32_TIMER_CC_ICOF(2):
        *value = s->timer_ic[i][(offset - EFR32_TIMER_CC_ICOF(0)) / 0x20u].icof;
        return true;
    case EFR32_TIMER_STATUS:
        *value = efr32_timer_ic_status(s, i) |
                 (efr32_timer_started(s, i) ? EFR32_TIMER_STATUS_RUNNING : 0) |
                 (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) <
                          s->timers[i].sync_until_ns
                      ? EFR32_TIMER_STATUS_SYNCBUSY : 0) |
                 (s->timer_buf_pending[i] & EFR32_TIMER_BUF_TOPB
                      ? EFR32_TIMER_STATUS_TOPBV : 0) |
                 ((s->timer_buf_pending[i] &
                   MAKE_64BIT_MASK(0, EFR32_TIMER0_CHANNELS))
                  << EFR32_TIMER_STATUS_OCBV_SHIFT);
        return true;
    case EFR32_TIMER_CMD:
        *value = 0;
        return true;
    default:
        return false;
    }
}

static void efr32_timer_write(Efr32Periph *s, uint32_t register_address,
                              uint32_t next)
{
    int i = efr32_timer_index(register_address);
    uint32_t offset = register_address & 0xfffu;
    uint32_t width = efr32_timers[i].width_mask;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    efr32_timer_rebase(s, i);
    switch (offset) {
    case EFR32_TIMER_CMD:
        if (next & EFR32_TIMER_CMD_STOP) {
            s->timers[i].running = false;
        } else if (next & EFR32_TIMER_CMD_START) {
            s->timers[i].running = true;
        }
        if (next & (EFR32_TIMER_CMD_STOP | EFR32_TIMER_CMD_START)) {
            s->timers[i].sync_until_ns =
                now + efr32_cycles_ns(EFR32_TIMER_CMD_SYNC_HCLK_CYCLES,
                                      EFR32_MEASURE_HCLK_HZ);
        }
        break;
    case EFR32_TIMER_CNT:
        s->timers[i].base_count = (next & width) % efr32_timer_period(s, i);
        break;
    case EFR32_TIMER_STATUS:
    case EFR32_TIMER_CC_ICF(0):
    case EFR32_TIMER_CC_ICF(1):
    case EFR32_TIMER_CC_ICF(2):
    case EFR32_TIMER_CC_ICOF(0):
    case EFR32_TIMER_CC_ICOF(1):
    case EFR32_TIMER_CC_ICOF(2):
        break;
    case EFR32_TIMER_EN:
        efr32_register_put(s, register_address, next & EFR32_TIMER_EN_EN);
        if (!(next & EFR32_TIMER_EN_EN)) {
            /* [I] Disabling the TIMER empties its capture FIFOs. */
            for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
                s->timer_ic[i][n].count = 0;
            }
        }
        break;
    case EFR32_TIMER_CFG:
        efr32_register_put(s, register_address, next & EFR32_TIMER_CFG_MASK);
        break;
    case EFR32_TIMER_IF:
    case EFR32_TIMER_IEN:
        efr32_register_put(s, register_address, next & EFR32_TIMER_IF_MASK);
        break;
    case EFR32_TIMER_TOP:
        efr32_register_put(s, register_address, next & width);
        s->timers[i].base_count %= efr32_timer_period(s, i);
        break;
    default:
        /* Count-wide: TOPB and the CCn OC/OCB values; CCn_CTRL holds ICEDGE
         * at bits 25:24 (efr32bg22_timer.h). */
        efr32_register_put(s, register_address,
                           efr32_timer_sync_reg(offset) &&
                           offset != EFR32_TIMER_CTRL &&
                           offset != EFR32_TIMER_DTCTRL &&
                           offset != EFR32_TIMER_CC_CTRL(0) &&
                           offset != EFR32_TIMER_CC_CTRL(1) &&
                           offset != EFR32_TIMER_CC_CTRL(2)
                               ? next & width : next);
        if (offset == EFR32_TIMER_TOPB) {
            s->timer_buf_pending[i] |= EFR32_TIMER_BUF_TOPB;
        }
        for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
            if (offset == EFR32_TIMER_CC_OCB(n)) {
                s->timer_buf_pending[i] |= 1u << n;
            }
        }
        break;
    }
    efr32_timer_schedule(s, i);
    efr32_timer_pwm_update(s, i);
    if (i == 0) {
        trace_efr32bg22_timer0_write(s->endpoint_id, now, offset, next,
                                     efr32_timer_counter(s, i));
    }
    efr32_timer_update_irq(s, i);
}

static void efr32_timers_reset(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        timer_del(s->timers[i].event);
        s->timers[i].running = false;
        s->timers[i].base_count = 0;
        s->timers[i].base_ns = now;
        s->timers[i].pending_flags = 0;
        s->timers[i].sync_until_ns = 0;
        qemu_set_irq(s->irq[efr32_timers[i].irq], 0);
        timer_del(s->timer_pwm[i]);
        s->timer_buf_pending[i] = 0;
        for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
            Efr32TimerIc *ic = &s->timer_ic[i][n];

            s->timer_cc_level[i][n] = false;
            qemu_set_irq(s->timer_cc_out[i * EFR32_TIMER0_CHANNELS + n], 0);
            timer_del(ic->sampler);
            ic->count = 0;
            ic->icof = 0;
            ic->sampled = ic->async_level;
            s->timer_prs_level[i][n] = false;
            efr32_prs_level(s, efr32_timer_prs_source[i],
                            EFR32_PRS_SIG_TIMER_CC(n), false);
            qemu_set_irq(s->prs_sync[EFR32_PRS_LINE(
                             efr32_timer_prs_sync_source[i],
                             EFR32_PRS_SIG_TIMER_CC(n))], 0);
        }
    }
}

static void efr32_pa_ramp_complete(Efr32Periph *s)
{
    s->pa_ramp_pending = false;
    efr32_register_put(s, EFR32_MODEM_BASE + EFR32_MODEM_IF,
                       efr32_register_get(s, EFR32_MODEM_BASE +
                                             EFR32_MODEM_IF) |
                           EFR32_MODEM_IF_TXRAMPDONE);
    efr32_register_put(s, EFR32_MODEM_BASE + EFR32_MODEM_SEQIF,
                       efr32_register_get(s, EFR32_MODEM_BASE +
                                             EFR32_MODEM_SEQIF) |
                           EFR32_MODEM_IF_TXRAMPDONE);
}

/*
 * ------------------------------------------------------------------- RTCC
 *
 * RM SS15.3 (register map SS15.4, fields from the public efr32bg22_rtcc.h).
 * EN requests the clock, CMD.START/STOP run the counter.  The 15-bit
 * pre-counter counts every rising edge of the clock CMU.RTCCCLKCTRL
 * selects (LFRCO/LFXO at the board's 32.768 kHz, ULFRCO 1 kHz); with
 * CFG.CNTTICK = PRESC, CNT advances every 2^CFG.CNTPRESC edges.  In output
 * compare mode with COMPBASE = CNT a channel matches when CNT equals its
 * OCVALUE (Figure 15.3: the match is the CNT == OCVALUE cycle) and sets
 * IF.CCn; CNT wrapping to 0 sets IF.OF.  IRQ 12 is IF & IEN.  "All RTCC
 * registers use the new immediate synchronization scheme" (SS15.3.1): writes
 * take effect at once and SYNCBUSY reads 0.
 * [I] IF.CCn sets on the LF edge where CNT becomes OCVALUE (the figure, not
 * a measurement; BURTC on silicon fires when CNT leaves COMP).  Input
 * capture from the PRS and the CCn PRS outputs: efr32_rtcc_prs_input,
 * efr32_rtcc_compare_output.  [U] Not modelled: CNTTICK = CCV0MATCH,
 * COMPBASE = PRECNT, CCV1TOP, LOCK; the timing of every edge is unmeasured.
 */
#define EFR32_RTCC_IRQ 12
#define EFR32_RTCC_EN 0x004u
#define EFR32_RTCC_EN_EN (1u << 0)
#define EFR32_RTCC_CFG 0x008u
#define EFR32_RTCC_CFG_CNTPRESC_SHIFT 4
#define EFR32_RTCC_CFG_CNTPRESC_MASK 0xf0u
#define EFR32_RTCC_CMD 0x00cu
#define EFR32_RTCC_CMD_START (1u << 0)
#define EFR32_RTCC_CMD_STOP (1u << 1)
#define EFR32_RTCC_STATUS 0x010u
#define EFR32_RTCC_STATUS_RUNNING (1u << 0)
#define EFR32_RTCC_IF 0x014u
#define EFR32_RTCC_IF_OF (1u << 0)
#define EFR32_RTCC_IF_CC0 (1u << 4)
#define EFR32_RTCC_IF_CC_STRIDE 2
#define EFR32_RTCC_IEN 0x018u
#define EFR32_RTCC_PRECNT 0x01cu
#define EFR32_RTCC_PRECNT_BITS 15
#define EFR32_RTCC_CNT 0x020u
#define EFR32_RTCC_COMBCNT 0x024u
#define EFR32_RTCC_SYNCBUSY 0x028u
#define EFR32_RTCC_CC0_CTRL 0x030u
#define EFR32_RTCC_CC_STRIDE 0x00cu
#define EFR32_RTCC_CC_OCVALUE 0x004u
#define EFR32_RTCC_CC_COUNT 3
#define EFR32_RTCC_CC_CTRL_MODE_MASK 0x3u
#define EFR32_RTCC_CC_CTRL_MODE_OUTPUTCOMPARE 0x2u
#define EFR32_RTCC_CC_CTRL_COMPBASE_PRECNT (1u << 4)
#define EFR32_RTCC_CNT_BITS 32
#define EFR32_CMU_RTCCCLKCTRL 0x240u
#define EFR32_CMU_RTCCCLKCTRL_RESET EFR32_CMU_LFCLKSEL_LFRCO

static uint32_t efr32_rtcc_get(Efr32Periph *s, uint32_t offset)
{
    return efr32_register_get(s, EFR32_RTCC_BASE + offset);
}

static uint64_t efr32_rtcc_hz(Efr32Periph *s)
{
    switch (efr32_cmu_get(s, EFR32_CMU_RTCCCLKCTRL) &
            EFR32_CMU_LFCLKSEL_MASK) {
    case EFR32_CMU_LFCLKSEL_LFRCO:
    case EFR32_CMU_LFCLKSEL_LFXO:
        return clock_get_hz(s->burtc_clk);
    case EFR32_CMU_LFCLKSEL_ULFRCO:
        return EFR32_ULFRCO_HZ;
    default:
        return 0;
    }
}

static int64_t efr32_rtcc_edge_at(Efr32Periph *s, int64_t t)
{
    uint64_t hz = efr32_rtcc_hz(s);

    return hz ? muldiv64(t, hz, NANOSECONDS_PER_SECOND) : 0;
}

static unsigned efr32_rtcc_presc(Efr32Periph *s)
{
    return (efr32_rtcc_get(s, EFR32_RTCC_CFG) &
            EFR32_RTCC_CFG_CNTPRESC_MASK) >> EFR32_RTCC_CFG_CNTPRESC_SHIFT;
}

static bool efr32_rtcc_counting(Efr32Periph *s)
{
    return s->rtcc_run && efr32_rtcc_hz(s) &&
           (efr32_rtcc_get(s, EFR32_RTCC_EN) & EFR32_RTCC_EN_EN) &&
           !efr32_debug_frozen(s, efr32_rtcc_get(s, EFR32_RTCC_CFG) &
                                      EFR32_RTCC_CFG_DEBUGRUN);
}

/* Edges counted (pre-counter and counter together) at LF edge @edge. */
static uint64_t efr32_rtcc_pos_at(Efr32Periph *s, int64_t edge)
{
    if (!efr32_rtcc_counting(s) || edge <= s->rtcc_edge_base) {
        return s->rtcc_pos_base;
    }
    return s->rtcc_pos_base + (uint64_t)(edge - s->rtcc_edge_base);
}

static uint32_t efr32_rtcc_cnt_of(Efr32Periph *s, uint64_t pos)
{
    return pos >> efr32_rtcc_presc(s);
}

/* First LF edge after @after at which CNT becomes @value. */
static int64_t efr32_rtcc_next_cnt_edge(Efr32Periph *s, int64_t after,
                                        uint32_t value)
{
    unsigned presc = efr32_rtcc_presc(s);
    uint64_t period = UINT64_C(1) << (EFR32_RTCC_CNT_BITS + presc);
    uint64_t pos = efr32_rtcc_pos_at(s, after);
    uint64_t target = (uint64_t)value << presc;
    uint64_t delta = (target - pos) & (period - 1);

    return after + (int64_t)(delta ? delta : period);
}

static void efr32_rtcc_update_irq(Efr32Periph *s)
{
    qemu_set_irq(s->irq[EFR32_RTCC_IRQ],
                 !!(efr32_rtcc_get(s, EFR32_RTCC_IF) &
                    efr32_rtcc_get(s, EFR32_RTCC_IEN)));
}

static bool efr32_rtcc_cc_compares(Efr32Periph *s, unsigned channel)
{
    uint32_t ctrl = efr32_rtcc_get(s, EFR32_RTCC_CC0_CTRL +
                                          channel * EFR32_RTCC_CC_STRIDE);

    return (ctrl & EFR32_RTCC_CC_CTRL_MODE_MASK) ==
               EFR32_RTCC_CC_CTRL_MODE_OUTPUTCOMPARE &&
           !(ctrl & EFR32_RTCC_CC_CTRL_COMPBASE_PRECNT);
}

/*
 * CCn PRS outputs (RM SS15.3.2-15.3.3, Figure 15.3): on a compare match
 * CCn_CTRL.CMOA pulses (one LFCLKRTCC cycle on silicon; see efr32-prs.c),
 * toggles, clears or sets the channel's output, PRS producer RTCC CCVn.
 */
#define EFR32_RTCC_CC_CTRL_CMOA_SHIFT 2
#define EFR32_RTCC_CC_CTRL_CMOA_MASK 0x3u
#define EFR32_RTCC_CMOA_PULSE 0u
#define EFR32_RTCC_CMOA_TOGGLE 1u
#define EFR32_RTCC_CMOA_CLEAR 2u
#define EFR32_RTCC_CMOA_SET 3u

static void efr32_rtcc_compare_output(Efr32Periph *s, unsigned channel)
{
    uint32_t ctrl = efr32_rtcc_get(s, EFR32_RTCC_CC0_CTRL +
                                          channel * EFR32_RTCC_CC_STRIDE);
    bool *level = &s->rtcc_ccv[channel];

    switch ((ctrl >> EFR32_RTCC_CC_CTRL_CMOA_SHIFT) &
            EFR32_RTCC_CC_CTRL_CMOA_MASK) {
    case EFR32_RTCC_CMOA_PULSE:
        efr32_prs_pulse(s, EFR32_PRS_SRC_RTCC, EFR32_PRS_SIG_RTCC_CCV(channel));
        return;
    case EFR32_RTCC_CMOA_TOGGLE:
        *level = !*level;
        break;
    case EFR32_RTCC_CMOA_CLEAR:
        *level = false;
        break;
    default:
        *level = true;
        break;
    }
    efr32_prs_level(s, EFR32_PRS_SRC_RTCC, EFR32_PRS_SIG_RTCC_CCV(channel),
                    *level);
}

/* Latch every event up to now, then arm the timer for the next one. */
static void efr32_rtcc_process(Efr32Periph *s)
{
    int64_t now = efr32_rtcc_edge_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    int64_t next = INT64_MAX;
    uint32_t flags = 0;

    timer_del(s->rtcc_timer);
    if (!efr32_rtcc_counting(s)) {
        s->rtcc_last_edge = now;
        return;
    }
    for (int channel = -1; channel < EFR32_RTCC_CC_COUNT; channel++) {
        uint32_t flag = channel < 0 ? EFR32_RTCC_IF_OF
                                    : EFR32_RTCC_IF_CC0
                                          << (channel *
                                              EFR32_RTCC_IF_CC_STRIDE);
        uint32_t value = channel < 0
                             ? 0
                             : efr32_rtcc_get(s, EFR32_RTCC_CC0_CTRL +
                                                     channel *
                                                         EFR32_RTCC_CC_STRIDE +
                                                     EFR32_RTCC_CC_OCVALUE);
        int64_t edge;

        if (channel >= 0 && !efr32_rtcc_cc_compares(s, channel)) {
            continue;
        }
        edge = efr32_rtcc_next_cnt_edge(s, s->rtcc_last_edge, value);
        if (edge <= now) {
            flags |= flag;
            edge = efr32_rtcc_next_cnt_edge(s, now, value);
        }
        next = MIN(next, edge);
    }
    s->rtcc_last_edge = now;
    if (flags) {
        efr32_register_put(s, EFR32_RTCC_BASE + EFR32_RTCC_IF,
                           efr32_rtcc_get(s, EFR32_RTCC_IF) | flags);
        efr32_rtcc_update_irq(s);
    }
    for (unsigned channel = 0; channel < EFR32_RTCC_CC_COUNT; channel++) {
        if (flags & (EFR32_RTCC_IF_CC0 << (channel *
                                           EFR32_RTCC_IF_CC_STRIDE))) {
            efr32_rtcc_compare_output(s, channel);
        }
    }
    if (next != INT64_MAX) {
        timer_mod(s->rtcc_timer,
                  muldiv64_round_up(next, NANOSECONDS_PER_SECOND,
                                    efr32_rtcc_hz(s)));
    }
}

static void efr32_rtcc_timer_expired(void *opaque)
{
    efr32_rtcc_process(EFR32_PERIPH(opaque));
}

/* Restart the count from @pos at the current edge. */
static void efr32_rtcc_rebase(Efr32Periph *s, uint64_t pos)
{
    s->rtcc_edge_base = s->rtcc_last_edge;
    s->rtcc_pos_base = pos;
}

static bool efr32_rtcc_read(Efr32Periph *s, uint32_t address, uint32_t *value)
{
    uint64_t pos;

    if ((address & ~0x3fffu) != EFR32_RTCC_BASE) {
        return false;
    }
    efr32_rtcc_process(s);
    pos = efr32_rtcc_pos_at(s, s->rtcc_last_edge);
    switch (address & 0xfffu) {
    case EFR32_RTCC_CNT:
        *value = efr32_rtcc_cnt_of(s, pos);
        return true;
    case EFR32_RTCC_PRECNT:
        *value = extract64(pos, 0, EFR32_RTCC_PRECNT_BITS);
        return true;
    case EFR32_RTCC_COMBCNT:
        *value = (efr32_rtcc_cnt_of(s, pos) << EFR32_RTCC_PRECNT_BITS) |
                 extract64(pos, 0, EFR32_RTCC_PRECNT_BITS);
        return true;
    case EFR32_RTCC_STATUS:
        *value = s->rtcc_run ? EFR32_RTCC_STATUS_RUNNING : 0;
        return true;
    case EFR32_RTCC_SYNCBUSY:
    case EFR32_RTCC_CMD:
        *value = 0;
        return true;
    default:
        return false;
    }
}

/* Returns true when the RTCC write is fully handled. */
static bool efr32_rtcc_write(Efr32Periph *s, uint32_t offset, uint32_t next)
{
    uint64_t pos;
    unsigned presc;

    efr32_rtcc_process(s);
    pos = efr32_rtcc_pos_at(s, s->rtcc_last_edge);
    presc = efr32_rtcc_presc(s);
    switch (offset) {
    case EFR32_RTCC_CMD:
        if (next & EFR32_RTCC_CMD_STOP) {
            efr32_rtcc_rebase(s, pos);
            s->rtcc_run = false;
        } else if (next & EFR32_RTCC_CMD_START) {
            efr32_rtcc_rebase(s, pos);
            s->rtcc_run = true;
        }
        break;
    case EFR32_RTCC_CNT:
        efr32_rtcc_rebase(s, ((uint64_t)next << presc) |
                                 (pos & ((UINT64_C(1) << presc) - 1)));
        break;
    case EFR32_RTCC_PRECNT:
        efr32_rtcc_rebase(s, (pos & ~MAKE_64BIT_MASK(0,
                                         EFR32_RTCC_PRECNT_BITS)) |
                                 extract32(next, 0, EFR32_RTCC_PRECNT_BITS));
        break;
    case EFR32_RTCC_STATUS:
    case EFR32_RTCC_COMBCNT:
    case EFR32_RTCC_SYNCBUSY:
        return true; /* read-only */
    case EFR32_RTCC_EN:
    case EFR32_RTCC_CFG:
        /* The count so far keeps its value across EN and CFG changes. */
        efr32_rtcc_rebase(s, pos);
        efr32_register_put(s, EFR32_RTCC_BASE + offset, next);
        break;
    default:
        efr32_register_put(s, EFR32_RTCC_BASE + offset, next);
        break;
    }
    efr32_rtcc_update_irq(s);
    efr32_rtcc_process(s);
    return true;
}

/*
 * Input capture (RM SS15.3.2): with CCn_CTRL.MODE = INPUTCAPTURE an edge of
 * the PRS channel PRS_CONSUMER_RTCC_CCn selects that matches ICEDGE
 * captures CNT into CCn_ICVALUE and sets IF.CCn.  [I] The edge acts at once
 * (on silicon the PRS input is synchronised to LFCLKRTCC first).
 */
#define EFR32_RTCC_CC_ICVALUE 0x008u
#define EFR32_RTCC_CC_CTRL_MODE_INPUTCAPTURE 0x1u
#define EFR32_RTCC_CC_CTRL_ICEDGE_SHIFT 5
#define EFR32_RTCC_CC_CTRL_ICEDGE_MASK 0x3u

static void efr32_rtcc_prs_input(Efr32Periph *s, unsigned channel,
                                 bool rising)
{
    uint32_t base = EFR32_RTCC_CC0_CTRL + channel * EFR32_RTCC_CC_STRIDE;
    uint32_t ctrl = efr32_rtcc_get(s, base);
    unsigned edge = (ctrl >> EFR32_RTCC_CC_CTRL_ICEDGE_SHIFT) &
                    EFR32_RTCC_CC_CTRL_ICEDGE_MASK;

    if ((ctrl & EFR32_RTCC_CC_CTRL_MODE_MASK) !=
            EFR32_RTCC_CC_CTRL_MODE_INPUTCAPTURE ||
        !(efr32_rtcc_get(s, EFR32_RTCC_EN) & EFR32_RTCC_EN_EN) ||
        !(edge == EFR32_TIMER_ICEDGE_BOTH ||
          (edge == EFR32_TIMER_ICEDGE_RISING && rising) ||
          (edge == EFR32_TIMER_ICEDGE_FALLING && !rising))) {
        return;
    }
    efr32_rtcc_process(s);
    efr32_register_put(s, EFR32_RTCC_BASE + base + EFR32_RTCC_CC_ICVALUE,
                       efr32_rtcc_cnt_of(s, efr32_rtcc_pos_at(
                                                s, s->rtcc_last_edge)));
    efr32_register_put(s, EFR32_RTCC_BASE + EFR32_RTCC_IF,
                       efr32_rtcc_get(s, EFR32_RTCC_IF) |
                           (EFR32_RTCC_IF_CC0 << (channel *
                                                  EFR32_RTCC_IF_CC_STRIDE)));
    efr32_rtcc_update_irq(s);
}

static void efr32_rtcc_reset(Efr32Periph *s)
{
    timer_del(s->rtcc_timer);
    for (unsigned channel = 0; channel < EFR32_RTCC_CC_COUNT; channel++) {
        s->rtcc_ccv[channel] = false;
        efr32_prs_level(s, EFR32_PRS_SRC_RTCC, EFR32_PRS_SIG_RTCC_CCV(channel),
                        false);
    }
    s->rtcc_run = false;
    s->rtcc_pos_base = 0;
    s->rtcc_edge_base = 0;
    s->rtcc_last_edge = 0;
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_RTCCCLKCTRL,
                       EFR32_CMU_RTCCCLKCTRL_RESET);
}

/*
 * ------------------------------------------------------------------- WDOG0
 *
 * RM chapter 27 (register map SS27.4, fields from the public
 * efr32bg22_wdog.h).  EN.EN starts the counter from 0 (and clearing EN
 * resets it, SS27.3); it counts the clock CMU.WDOG0CLKCTRL.CLKSEL selects
 * (LFRCO, LFXO, ULFRCO or HCLK / 1024, SS27.3.1).  It times out after
 * 2^(PERSEL + 3) + 1 cycles (SS27.3.1 and the PERSEL table: SEL0 = 9 cycles
 * ... SEL15 = 256k): IF.TOUT sets and, unless CFG.WDOGRSTDIS, the chip resets
 * with EMU.RSTCAUSE.WDOG0 (bit 3, efr32bg22_emu.h).  IF.WARN sets at
 * WARNSEL x 25 % of the period (SS27.3.4).  WDOG0 is IRQ 43 (IF & IEN).
 *
 * Register types (SS27.4, rules in SS4.2.4.4):
 *  - CFG is a CONFIG register: a write while EN.EN is set is a bus error
 *    (BusFault), SS4.2.4.4.2.
 *  - CMD is an LFSYNC register: a write is pending for EFR32_WDOG_LFSYNC_CYCLES
 *    watchdog-clock cycles (SS4.2.4.4.4: "3 - 4 PERCLK cycles", the model
 *    takes the upper bound), SYNCBUSY.CMD reads 1 meanwhile, CMD.CLEAR
 *    returns the counter to 0 when the write completes, and a CMD write
 *    while one is pending is a bus error (SS4.2.4.4.4).
 *  - EN is an ENABLE register: clearing it while an LFSYNC write is pending
 *    is a bus error (SS4.2.4.4.5).
 *  - IF is an INTFLAG register: flags stay set until software clears them
 *    (SS27.3: "Any pending interrupt flags will remain active until
 *    cleared"); clearing the counter does not clear IF.WARN.
 * Writing LOCK with anything but 0xABE8 locks EN and CFG (STATUS.LOCK,
 * SS27.3, SS27.5.8); 0xABE8 unlocks.  A write to a locked EN or CFG is
 * ignored (the RM does not say whether it faults) [U].
 * CFG.DEBUGRUN (SS27.3.2): while the M33 is in Debug state (DHCSR.S_HALT,
 * the "m33-debug-halted" input from the debug unit) and DEBUGRUN = 0 the
 * counter is frozen and continues where it left off on resume.  A stopped
 * QEMU (gdbstub, monitor stop) stops virtual time altogether; that is not a
 * debug halt, and it freezes every timer.
 * [U] Not modelled: freezing in EM2/EM3 (EM2RUN/EM3RUN, SS27.3.3; the
 * machine does not distinguish sleep depths, so the counter runs in every
 * sleep mode, which is the EM2RUN = EM3RUN = 1 behaviour; enabling with
 * either bit clear is logged as unimplemented), the enable delay of EN
 * (2 - 3 cycles, SS4.2.4.4.5), the window interrupt, PRS clear and
 * monitoring, EM4BLOCK, and the requirement that CLEAR be written 4 cycles
 * before the timeout (SS27.5.4).
 */
#define EFR32_WDOG0_BASE 0x4a018000u
#define EFR32_WDOG0_IRQ 43
#define EFR32_WDOG_EN 0x004u
#define EFR32_WDOG_EN_EN (1u << 0)
#define EFR32_WDOG_CFG 0x008u
#define EFR32_WDOG_CFG_RESET 0x000f0000u
#define EFR32_WDOG_CFG_EM2RUN (1u << 1)
#define EFR32_WDOG_CFG_EM3RUN (1u << 2)
#define EFR32_WDOG_CFG_DEBUGRUN (1u << 4)
#define EFR32_WDOG_CFG_WDOGRSTDIS (1u << 8)
#define EFR32_WDOG_CFG_PERSEL_SHIFT 16
#define EFR32_WDOG_CFG_PERSEL_MASK 0xfu
#define EFR32_WDOG_CFG_WARNSEL_SHIFT 24
#define EFR32_WDOG_CFG_WARNSEL_MASK 0x3u
#define EFR32_WDOG_CMD 0x00cu
#define EFR32_WDOG_CMD_CLEAR (1u << 0)
#define EFR32_WDOG_STATUS 0x014u
#define EFR32_WDOG_STATUS_LOCK (1u << 31)
#define EFR32_WDOG_IF 0x018u
#define EFR32_WDOG_IF_TOUT (1u << 0)
#define EFR32_WDOG_IF_WARN (1u << 1)
#define EFR32_WDOG_IF_MASK 0x1fu
#define EFR32_WDOG_IEN 0x01cu
#define EFR32_WDOG_LOCK 0x020u
#define EFR32_WDOG_LOCK_UNLOCK 0xabe8u
#define EFR32_WDOG_SYNCBUSY 0x024u
#define EFR32_WDOG_SYNCBUSY_CMD (1u << 0)
/* SS27.3.1: TTIMEOUT = [2^(PERSEL + 3) + 1] / f. */
#define EFR32_WDOG_PERSEL_BIAS 3
/* SS27.3.4: TWARNING = TTIMEOUT x WARNSEL / 4. */
#define EFR32_WDOG_WARN_QUARTERS 4
/* SS4.2.4.4.4: an LFSYNC write completes after 3 - 4 PERCLK cycles. [U] */
#define EFR32_WDOG_LFSYNC_CYCLES 4
#define EFR32_CMU_WDOG0CLKCTRL 0x200u
/*
 * RM SS8.5.4: CMU_WDOGLOCK, any key but 0x93F7 locks the WDOG clock selection
 * (CMU_STATUS.WDOGLOCK, SS8.5.2); the unlock key unlocks.  A write to a locked
 * WDOG0CLKCTRL is ignored [U: the RM says only "lock registers from editing"].
 */
#define EFR32_CMU_STATUS 0x008u
#define EFR32_CMU_STATUS_WDOGLOCK (1u << 30)
#define EFR32_CMU_WDOGLOCK 0x014u
#define EFR32_CMU_WDOGLOCK_UNLOCK 0x93f7u
#define EFR32_CMU_WDOGLOCK_KEY_MASK 0xffffu
#define EFR32_CMU_WDOG0CLKCTRL_RESET 1u
#define EFR32_CMU_WDOG0CLKCTRL_CLKSEL_MASK 0x7u
#define EFR32_CMU_WDOG0CLKCTRL_HCLKDIV1024 4u
#define EFR32_WDOG_HCLK_DIVIDER 1024u
#define EFR32_EMU_RSTCAUSE_WDOG0 (1u << 3)

static uint64_t efr32_sysclk_hz(Efr32Periph *s);

static uint32_t efr32_wdog_get(Efr32Periph *s, uint32_t offset)
{
    return efr32_register_get(s, EFR32_WDOG0_BASE + offset);
}

static void efr32_wdog_put(Efr32Periph *s, uint32_t offset, uint32_t value)
{
    efr32_register_put(s, EFR32_WDOG0_BASE + offset, value);
}

static uint64_t efr32_wdog_hz(Efr32Periph *s)
{
    switch (efr32_cmu_get(s, EFR32_CMU_WDOG0CLKCTRL) &
            EFR32_CMU_WDOG0CLKCTRL_CLKSEL_MASK) {
    case EFR32_CMU_LFCLKSEL_LFRCO:
    case EFR32_CMU_LFCLKSEL_LFXO:
        return clock_get_hz(s->burtc_clk);
    case EFR32_CMU_LFCLKSEL_ULFRCO:
        return EFR32_ULFRCO_HZ;
    case EFR32_CMU_WDOG0CLKCTRL_HCLKDIV1024:
        return efr32_sysclk_hz(s) / EFR32_WDOG_HCLK_DIVIDER;
    default:
        return 0;
    }
}

static uint64_t efr32_wdog_period_cycles(Efr32Periph *s)
{
    uint32_t persel = (efr32_wdog_get(s, EFR32_WDOG_CFG) >>
                       EFR32_WDOG_CFG_PERSEL_SHIFT) &
                      EFR32_WDOG_CFG_PERSEL_MASK;

    return (UINT64_C(1) << (persel + EFR32_WDOG_PERSEL_BIAS)) + 1;
}

/* Watchdog-clock cycles counted since the last return to 0. */
static uint64_t efr32_wdog_counter(Efr32Periph *s, int64_t now)
{
    if (!s->wdog_running) {
        return 0;
    }
    return muldiv64((s->wdog_frozen ? s->wdog_frozen_ns : now) -
                        s->wdog_epoch_ns,
                    efr32_wdog_hz(s), NANOSECONDS_PER_SECOND);
}

static void efr32_wdog_update_irq(Efr32Periph *s)
{
    qemu_set_irq(s->irq[EFR32_WDOG0_IRQ],
                 !!(efr32_wdog_get(s, EFR32_WDOG_IF) &
                    efr32_wdog_get(s, EFR32_WDOG_IEN)));
}

/* Arm the next event: the warning if still ahead, else the timeout. */
static void efr32_wdog_schedule(Efr32Periph *s)
{
    uint64_t hz = efr32_wdog_hz(s);
    uint64_t period = efr32_wdog_period_cycles(s);
    uint32_t warnsel = (efr32_wdog_get(s, EFR32_WDOG_CFG) >>
                        EFR32_WDOG_CFG_WARNSEL_SHIFT) &
                       EFR32_WDOG_CFG_WARNSEL_MASK;
    int64_t warn_ns, tout_ns;

    timer_del(s->wdog_timer);
    if (!s->wdog_running || s->wdog_frozen || !hz) {
        return;
    }
    tout_ns = s->wdog_epoch_ns +
              muldiv64(period, NANOSECONDS_PER_SECOND, hz);
    warn_ns = s->wdog_epoch_ns +
              muldiv64(period * warnsel, NANOSECONDS_PER_SECOND,
                       hz * EFR32_WDOG_WARN_QUARTERS);
    timer_mod(s->wdog_timer, warnsel && !s->wdog_warned ? warn_ns : tout_ns);
}

/* A reset of the part that owns these peripherals (efr32_soc_reset). */
static void efr32_periph_reset_request(Efr32Periph *s);

static void efr32_wdog_expired(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    uint64_t hz = efr32_wdog_hz(s);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t tout_ns;

    if (!s->wdog_running || s->wdog_frozen || !hz) {
        return;
    }
    tout_ns = s->wdog_epoch_ns +
              muldiv64(efr32_wdog_period_cycles(s), NANOSECONDS_PER_SECOND,
                       hz);
    if (now < tout_ns) {
        s->wdog_warned = true;
        efr32_wdog_put(s, EFR32_WDOG_IF, efr32_wdog_get(s, EFR32_WDOG_IF) |
                                             EFR32_WDOG_IF_WARN);
        trace_efr32_wdog(s->endpoint_id, now, "warning",
                         efr32_wdog_counter(s, now));
        efr32_wdog_update_irq(s);
        efr32_wdog_schedule(s);
        return;
    }
    efr32_wdog_put(s, EFR32_WDOG_IF, efr32_wdog_get(s, EFR32_WDOG_IF) |
                                         EFR32_WDOG_IF_TOUT);
    efr32_wdog_update_irq(s);
    trace_efr32_wdog(s->endpoint_id, now, "timeout",
                     efr32_wdog_get(s, EFR32_WDOG_CFG));
    if (!(efr32_wdog_get(s, EFR32_WDOG_CFG) & EFR32_WDOG_CFG_WDOGRSTDIS) &&
        (s->rstctrl & EFR32_EMU_RSTCTRL_WDOG0RMODE)) {
        s->rstcause |= EFR32_EMU_RSTCAUSE_WDOG0;
        s->wdog_running = false;
        efr32_periph_reset_request(s);
        return;
    }
    /* Reset disabled: the counter wraps and runs on. */
    s->wdog_epoch_ns = tout_ns;
    s->wdog_warned = false;
    efr32_wdog_schedule(s);
}

/* An LFSYNC write to CMD completes: CLEAR returns the counter to 0. */
static void efr32_wdog_sync_done(void *opaque)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool clear = s->wdog_cmd_pending & EFR32_WDOG_CMD_CLEAR;

    s->wdog_sync_pending = false;
    s->wdog_cmd_pending = 0;
    if (!clear || !s->wdog_running) {
        return;
    }
    trace_efr32_wdog(s->endpoint_id, now, "clear",
                     efr32_wdog_counter(s, now));
    s->wdog_epoch_ns = now;
    s->wdog_frozen_ns = now;
    s->wdog_warned = false;
    efr32_wdog_schedule(s);
}

/* SS27.3.2: DEBUGRUN = 0 freezes the counter while the core is halted. */
static void efr32_wdog_debug_halt(Efr32Periph *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool freeze = s->m33_debug_halted && s->wdog_running &&
                  !(efr32_wdog_get(s, EFR32_WDOG_CFG) &
                    EFR32_WDOG_CFG_DEBUGRUN);

    if (freeze == s->wdog_frozen) {
        return;
    }
    if (freeze) {
        s->wdog_frozen_ns = now;
    } else {
        s->wdog_epoch_ns += now - s->wdog_frozen_ns;
    }
    s->wdog_frozen = freeze;
    trace_efr32_wdog(s->endpoint_id, now, freeze ? "freeze" : "thaw",
                     efr32_wdog_counter(s, now));
    efr32_wdog_schedule(s);
}

/*
 * The M33's DHCSR.S_HALT changed (the debug unit's "halted" line, see
 * "debug halt"): each count with DEBUGRUN = 0 is taken up to now under the
 * old state, then stops or restarts from there.
 */
static void efr32_m33_debug_halted(void *opaque, int n, int level)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->m33_debug_halted == !!level) {
        return;
    }
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        efr32_timer_rebase(s, i);
    }
    efr32_burtc_process(s);
    if (efr32_burtc_hz(s)) {
        efr32_burtc_rebase(s, efr32_burtc_edge_at(s, now));
    }
    efr32_rtcc_process(s);
    efr32_rtcc_rebase(s, efr32_rtcc_pos_at(s, s->rtcc_last_edge));

    s->m33_debug_halted = level;

    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        efr32_timer_schedule(s, i);
        efr32_timer_pwm_update(s, i);
    }
    efr32_burtc_process(s);
    efr32_rtcc_process(s);
    efr32_wdog_debug_halt(s);
}

static bool efr32_wdog_read(Efr32Periph *s, uint32_t address,
                            uint32_t *value)
{
    if ((address & ~0x3fffu) != EFR32_WDOG0_BASE) {
        return false;
    }
    switch (address & 0xfffu) {
    case EFR32_WDOG_CMD:
    case EFR32_WDOG_LOCK:
        *value = 0;
        return true;
    case EFR32_WDOG_SYNCBUSY:
        *value = s->wdog_sync_pending ? EFR32_WDOG_SYNCBUSY_CMD : 0;
        return true;
    default:
        return false;
    }
}

/*
 * A write WDOG0 answers with a bus error (SS4.2.4.4): CFG while enabled,
 * CMD while a CMD write is still synchronising, and disabling while one is.
 * `address` is canonical (SET/CLR/TGL window in bits 13:12).
 */
static bool efr32_wdog_write_rejected(Efr32Periph *s, uint32_t address,
                                      uint32_t value)
{
    uint32_t en = efr32_wdog_get(s, EFR32_WDOG_EN);
    bool locked = efr32_wdog_get(s, EFR32_WDOG_STATUS) &
                  EFR32_WDOG_STATUS_LOCK;
    bool reject = false;

    if ((address & ~0x3fffu) != EFR32_WDOG0_BASE) {
        return false;
    }
    switch (address & 0xfffu) {
    case EFR32_WDOG_CFG:
        reject = en & EFR32_WDOG_EN_EN;
        break;
    case EFR32_WDOG_CMD:
        reject = s->wdog_sync_pending;
        break;
    case EFR32_WDOG_EN:
        switch ((address >> 12) & 3) {
        case 1:
            en |= value;
            break;
        case 2:
            en &= ~value;
            break;
        case 3:
            en ^= value;
            break;
        default:
            en = value;
        }
        reject = !locked && s->wdog_sync_pending && !(en & EFR32_WDOG_EN_EN);
        break;
    default:
        break;
    }
    if (reject) {
        trace_efr32_wdog(s->endpoint_id, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                         "bus-error", address);
    }
    return reject;
}

/* Returns true when the WDOG write is fully handled. */
static bool efr32_wdog_write(Efr32Periph *s, uint32_t offset, uint32_t next)
{
    bool locked = efr32_wdog_get(s, EFR32_WDOG_STATUS) &
                  EFR32_WDOG_STATUS_LOCK;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t cfg = efr32_wdog_get(s, EFR32_WDOG_CFG);
    uint64_t hz;

    switch (offset) {
    case EFR32_WDOG_EN:
        if (locked) {
            return true;
        }
        efr32_wdog_put(s, offset, next & EFR32_WDOG_EN_EN);
        if (!(next & EFR32_WDOG_EN_EN)) {
            s->wdog_running = false;
            s->wdog_frozen = false;
        } else if (!s->wdog_running) {
            s->wdog_running = true;
            s->wdog_epoch_ns = now;
            s->wdog_warned = false;
            s->wdog_frozen = s->m33_debug_halted &&
                             !(cfg & EFR32_WDOG_CFG_DEBUGRUN);
            s->wdog_frozen_ns = now;
            if ((cfg & (EFR32_WDOG_CFG_EM2RUN | EFR32_WDOG_CFG_EM3RUN)) !=
                (EFR32_WDOG_CFG_EM2RUN | EFR32_WDOG_CFG_EM3RUN)) {
                qemu_log_mask(LOG_UNIMP, "efr32bg22: WDOG0 EM2RUN/EM3RUN = 0 "
                              "is not modelled; the counter runs in every "
                              "sleep mode\n");
            }
        }
        trace_efr32_wdog(s->endpoint_id, now, "enable", next);
        break;
    case EFR32_WDOG_CFG:
        if (!locked) {
            efr32_wdog_put(s, offset, next);
            trace_efr32_wdog(s->endpoint_id, now, "config", next);
        }
        break;
    case EFR32_WDOG_CMD:
        hz = efr32_wdog_hz(s);
        if (!(next & EFR32_WDOG_CMD_CLEAR) || !hz) {
            return true;
        }
        s->wdog_sync_pending = true;
        s->wdog_cmd_pending = next & EFR32_WDOG_CMD_CLEAR;
        timer_mod(s->wdog_sync_timer,
                  now + muldiv64(EFR32_WDOG_LFSYNC_CYCLES,
                                 NANOSECONDS_PER_SECOND, hz));
        return true;
    case EFR32_WDOG_LOCK:
        if ((next & 0xffffu) != EFR32_WDOG_LOCK_UNLOCK) {
            efr32_wdog_put(s, EFR32_WDOG_STATUS, EFR32_WDOG_STATUS_LOCK);
        } else {
            efr32_wdog_put(s, EFR32_WDOG_STATUS, 0);
        }
        trace_efr32_wdog(s->endpoint_id, now, "lock", next);
        return true;
    case EFR32_WDOG_STATUS:
    case EFR32_WDOG_SYNCBUSY:
        return true;
    case EFR32_WDOG_IF:
    case EFR32_WDOG_IEN:
        efr32_wdog_put(s, offset, next & EFR32_WDOG_IF_MASK);
        break;
    default:
        return false;
    }
    efr32_wdog_update_irq(s);
    efr32_wdog_schedule(s);
    return true;
}

static void efr32_wdog_reset(Efr32Periph *s)
{
    timer_del(s->wdog_timer);
    timer_del(s->wdog_sync_timer);
    s->wdog_running = false;
    s->wdog_frozen = false;
    s->wdog_warned = false;
    s->wdog_sync_pending = false;
    s->wdog_cmd_pending = 0;
    efr32_wdog_put(s, EFR32_WDOG_EN, 0);
    efr32_wdog_put(s, EFR32_WDOG_CFG, EFR32_WDOG_CFG_RESET);
    efr32_wdog_put(s, EFR32_WDOG_STATUS, 0);
    efr32_wdog_put(s, EFR32_WDOG_IF, 0);
    efr32_wdog_put(s, EFR32_WDOG_IEN, 0);
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_WDOG0CLKCTRL,
                       EFR32_CMU_WDOG0CLKCTRL_RESET);
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_STATUS,
                       efr32_cmu_get(s, EFR32_CMU_STATUS) &
                           ~EFR32_CMU_STATUS_WDOGLOCK);
}

/* CMU_WDOGLOCK and the WDOG0CLKCTRL it guards; true when the write is handled. */
static bool efr32_cmu_wdoglock_write(Efr32Periph *s, uint32_t key,
                                     uint32_t next)
{
    uint32_t status = efr32_cmu_get(s, EFR32_CMU_STATUS);

    if (key == EFR32_CMU_BASE + EFR32_CMU_WDOGLOCK) {
        if ((next & EFR32_CMU_WDOGLOCK_KEY_MASK) != EFR32_CMU_WDOGLOCK_UNLOCK) {
            status |= EFR32_CMU_STATUS_WDOGLOCK;
        } else {
            status &= ~EFR32_CMU_STATUS_WDOGLOCK;
        }
        efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_STATUS, status);
        return true;
    }
    return key == EFR32_CMU_BASE + EFR32_CMU_WDOG0CLKCTRL &&
           (status & EFR32_CMU_STATUS_WDOGLOCK);
}

/*
 * ------------------------------------------------------------------- IADC
 *
 * RM SS24 (register map SS24.4, fields from the public efr32bg22_iadc.h).
 *  - Single queue: CMD.SINGLESTART converts the input SINGLE selects
 *    (PORTPOS/PINPOS against PORTNEG/PINNEG) with the CFGn that SINGLE.CFG
 *    names, pushes the result into the 4-entry single FIFO (RM SS24.3.5) and
 *    sets IF.SINGLEDONE; IF.SINGLEFIFODVL sets once the FIFO holds DVL + 1
 *    entries.  SINGLEFIFODATA pops; SINGLEFIFOSTAT counts;
 *    STATUS.SINGLEFIFODV reports a non-empty FIFO.
 *  - Scan queue (RM SS24.3.4): CMD.SCANSTART converts, in table order, every
 *    SCANTABLE entry whose MASKREQ bit is set (same input fields as SINGLE),
 *    each into the 4-entry scan FIFO with IF.SCANENTRYDONE, then sets
 *    IF.SCANTABLEDONE; IF.SCANFIFODVL / SCANFIFOOF as for the single FIFO.
 *    With SCANFIFOCFG.SHOWID the entry's index is in bits 31:24 of a
 *    right-aligned result and bits 7:0 of a left-aligned one (the layout
 *    sl_hal_iadc's result decoder reads).
 *  IRQ 48 is IF & IEN.
 * The analog side is the board: a pin's voltage comes from the machine's
 * "analog" property (millivolts, e.g. "PD02=968") if it names the pin, else
 * from the board's analog source (an "analog-source" object, sampled at the
 * conversion), else 0 V; GND reads 0 V.  Reference VDDX is the board supply
 * (EFR32_BOARD_VDDX_MV), VBGR the 1.21 V band-gap (RM SS24.3.3.1).
 * Result = Vin * gain / Vref at 12 bits, right-aligned (ALIGNMENT RIGHT12)
 * or left-aligned (LEFT12), unipolar for a single-ended input.
 * Fault injection: "x-iadc-stall" (periph property) makes conversions never
 * complete: commands are accepted and nothing ever comes out.
 * Results arrive after the RM's warm-up and conversion times
 * (efr32_iadc_queue: OSR, ADC_CLK, WARMUPMODE, TIMEBASE; [I] unmeasured).
 * Triggers (local timer, PRS, continuous): efr32_iadc_request and around.
 * Output resolution, 16/20-bit alignment and the window comparator:
 * efr32_iadc_convert, efr32_iadc_window_hit.
 * [U] Not modelled: bipolar output (TWOSCOMPL, differential inputs below
 * 0 read 0), gain/offset calibration (SCALEx), SUPPLY inputs.
 */
#define EFR32_IADC_BASE 0x4a004000u
#define EFR32_IADC_IRQ 48
#define EFR32_IADC_CMD 0x00cu
#define EFR32_IADC_CMD_SINGLESTART (1u << 0)
#define EFR32_IADC_CMD_SCANSTART (1u << 3)
#define EFR32_IADC_CMD_SINGLEFIFOFLUSH (1u << 24)
#define EFR32_IADC_CMD_SCANFIFOFLUSH (1u << 25)
#define EFR32_IADC_STATUS 0x014u
#define EFR32_IADC_STATUS_SINGLEFIFODV (1u << 8)
#define EFR32_IADC_STATUS_SCANFIFODV (1u << 9)
#define EFR32_IADC_STATUS_SINGLEQEN (1u << 0)
#define EFR32_IADC_STATUS_SCANQEN (1u << 3)
#define EFR32_IADC_STATUS_CONVERTING (1u << 6)
#define EFR32_IADC_STATUS_ADCWARM (1u << 30)
#define EFR32_IADC_STATUS_TIMERACTIVE (1u << 16)
#define EFR32_IADC_MASKREQ 0x018u
#define EFR32_IADC_MASKREQ_MASK 0xffffu
#define EFR32_IADC_IF 0x024u
#define EFR32_IADC_IF_SINGLEFIFODVL (1u << 0)
#define EFR32_IADC_IF_SCANFIFODVL (1u << 1)
#define EFR32_IADC_IF_SCANENTRYDONE (1u << 7)
#define EFR32_IADC_IF_SCANTABLEDONE (1u << 8)
#define EFR32_IADC_IF_SINGLEDONE (1u << 9)
#define EFR32_IADC_IF_SINGLEFIFOOF (1u << 16)
#define EFR32_IADC_IF_SCANFIFOOF (1u << 17)
#define EFR32_IADC_IF_SINGLEFIFOUF (1u << 18)
#define EFR32_IADC_IF_SCANFIFOUF (1u << 19)
#define EFR32_IADC_IEN 0x028u
#define EFR32_IADC_CFG0 0x048u
#define EFR32_IADC_CFG_STRIDE 0x010u
#define EFR32_IADC_CFG_ANALOGGAIN_SHIFT 12
#define EFR32_IADC_CFG_ANALOGGAIN_MASK 0x7u
#define EFR32_IADC_CFG_REFSEL_SHIFT 16
#define EFR32_IADC_CFG_REFSEL_MASK 0x7u
#define EFR32_IADC_CFG_REFSEL_VBGR 0u
#define EFR32_IADC_CFG_REFSEL_VDDX 3u
#define EFR32_IADC_SINGLEFIFOCFG 0x070u
#define EFR32_IADC_FIFOCFG_ALIGNMENT_MASK 0x7u
#define EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT12 3u
#define EFR32_IADC_FIFOCFG_SHOWID (1u << 3)
#define EFR32_IADC_FIFOCFG_DVL_SHIFT 4
#define EFR32_IADC_FIFOCFG_DVL_MASK 0x3u
#define EFR32_IADC_SINGLEFIFODATA 0x074u
#define EFR32_IADC_SINGLEFIFOSTAT 0x078u
#define EFR32_IADC_SINGLEDATA 0x07cu
#define EFR32_IADC_SCANFIFOCFG 0x080u
#define EFR32_IADC_SCANFIFODATA 0x084u
#define EFR32_IADC_SCANFIFOSTAT 0x088u
#define EFR32_IADC_SCANDATA 0x08cu
#define EFR32_IADC_SINGLE 0x098u
#define EFR32_IADC_SCANTABLE0 0x0a0u
#define EFR32_IADC_SCANTABLE_ENTRIES 16
#define EFR32_IADC_INPUT_PINNEG_SHIFT 0
#define EFR32_IADC_INPUT_PORTNEG_SHIFT 4
#define EFR32_IADC_INPUT_PINPOS_SHIFT 8
#define EFR32_IADC_INPUT_PORTPOS_SHIFT 12
#define EFR32_IADC_INPUT_FIELD_MASK 0xfu
#define EFR32_IADC_INPUT_CFG (1u << 16)
#define EFR32_IADC_PORT_GPIO_FIRST 8u /* PORTA; PORTB 9, PORTC 10, PORTD 11 */
#define EFR32_IADC_FIFO_DEPTH 4
#define EFR32_IADC_RESULT_BITS 12
#define EFR32_IADC_WORD_BITS 20
#define EFR32_IADC_CFG_OSRHS_SHIFT 2
#define EFR32_IADC_CFG_OSRHS_MASK 0x7u
#define EFR32_IADC_CFG_DIGAVG_SHIFT 21
#define EFR32_IADC_CFG_DIGAVG_MASK 0x7u
#define EFR32_IADC_FIFOCFG_ALIGNMENT_RIGHT16 1u
#define EFR32_IADC_FIFOCFG_ALIGNMENT_RIGHT20 2u
#define EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT16 4u
#define EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT20 5u
/* SHOWID: entry index in bits 31:24 (right-aligned), 7:0 (left-aligned). */
#define EFR32_IADC_ID_RIGHT_SHIFT 24
#define EFR32_IADC_SINGLE_ID 0x20u
#define EFR32_IADC_VBGR_MV 1210u
#define EFR32_BOARD_VDDX_MV 3300u
#define EFR32_UV_PER_MV 1000
/* ANALOGGAIN codes 1..5 are x0.5, x1, x2, x3, x4 (efr32bg22_iadc.h). */
static const uint32_t efr32_iadc_gain_half[] = { 0, 1, 2, 4, 6, 8 };

typedef struct Efr32IadcQueue {
    uint32_t fifocfg, fifodata, fifostat, data;
    uint32_t if_dvl, if_of, if_uf, status_dv, flush;
} Efr32IadcQueue;

static const Efr32IadcQueue efr32_iadc_single = {
    EFR32_IADC_SINGLEFIFOCFG, EFR32_IADC_SINGLEFIFODATA,
    EFR32_IADC_SINGLEFIFOSTAT, EFR32_IADC_SINGLEDATA,
    EFR32_IADC_IF_SINGLEFIFODVL, EFR32_IADC_IF_SINGLEFIFOOF,
    EFR32_IADC_IF_SINGLEFIFOUF, EFR32_IADC_STATUS_SINGLEFIFODV,
    EFR32_IADC_CMD_SINGLEFIFOFLUSH,
};
static const Efr32IadcQueue efr32_iadc_scan = {
    EFR32_IADC_SCANFIFOCFG, EFR32_IADC_SCANFIFODATA,
    EFR32_IADC_SCANFIFOSTAT, EFR32_IADC_SCANDATA,
    EFR32_IADC_IF_SCANFIFODVL, EFR32_IADC_IF_SCANFIFOOF,
    EFR32_IADC_IF_SCANFIFOUF, EFR32_IADC_STATUS_SCANFIFODV,
    EFR32_IADC_CMD_SCANFIFOFLUSH,
};

static uint32_t efr32_iadc_get(Efr32Periph *s, uint32_t offset)
{
    return efr32_register_get(s, EFR32_IADC_BASE + offset);
}

static void efr32_iadc_put(Efr32Periph *s, uint32_t offset, uint32_t value)
{
    efr32_register_put(s, EFR32_IADC_BASE + offset, value);
}

static void efr32_iadc_update(Efr32Periph *s)
{
    uint32_t status = efr32_iadc_get(s, EFR32_IADC_STATUS) &
                      ~(EFR32_IADC_STATUS_SINGLEFIFODV |
                        EFR32_IADC_STATUS_SCANFIFODV |
                        EFR32_IADC_STATUS_SINGLEQEN |
                        EFR32_IADC_STATUS_SCANQEN |
                        EFR32_IADC_STATUS_CONVERTING |
                        EFR32_IADC_STATUS_ADCWARM |
                        EFR32_IADC_STATUS_TIMERACTIVE);

    if (s->iadc_qen[0]) {
        status |= EFR32_IADC_STATUS_SINGLEQEN;
    }
    if (s->iadc_qen[1]) {
        status |= EFR32_IADC_STATUS_SCANQEN;
    }
    if (s->iadc_timer_on) {
        status |= EFR32_IADC_STATUS_TIMERACTIVE;
    }
    for (unsigned i = 0; i < s->iadc_nconv; i++) {
        status |= EFR32_IADC_STATUS_CONVERTING |
                  (s->iadc_conv[i].queue ? EFR32_IADC_STATUS_SCANQEN
                                         : EFR32_IADC_STATUS_SINGLEQEN);
    }
    if (s->iadc_warm) {
        status |= EFR32_IADC_STATUS_ADCWARM;
    }
    if (s->iadc_fifo[0].count) {
        status |= EFR32_IADC_STATUS_SINGLEFIFODV;
    }
    if (s->iadc_fifo[1].count) {
        status |= EFR32_IADC_STATUS_SCANFIFODV;
    }
    efr32_iadc_put(s, EFR32_IADC_STATUS, status);
    efr32_iadc_put(s, EFR32_IADC_SINGLEFIFOSTAT, s->iadc_fifo[0].count);
    efr32_iadc_put(s, EFR32_IADC_SCANFIFOSTAT, s->iadc_fifo[1].count);
    qemu_set_irq(s->irq[EFR32_IADC_IRQ],
                 !!(efr32_iadc_get(s, EFR32_IADC_IF) &
                    efr32_iadc_get(s, EFR32_IADC_IEN)));
}

/* Millivolts on an IADC input; GND, SUPPLY and undriven pins read 0. */
static uint32_t efr32_iadc_input_mv(Efr32Periph *s, uint32_t port,
                                    uint32_t pin)
{
    int32_t mv;

    if (port < EFR32_IADC_PORT_GPIO_FIRST ||
        port - EFR32_IADC_PORT_GPIO_FIRST >= EFR32_GPIO_PORTS ||
        pin >= EFR32_GPIO_PINS) {
        return 0;
    }
    port -= EFR32_IADC_PORT_GPIO_FIRST;
    mv = s->analog_mv[port][pin];
    if (mv < 0 && s->analog_source) {
        char net[8];

        snprintf(net, sizeof(net), "P%c%02u", 'A' + port, pin);
        mv = analog_source_read_uv(ANALOG_SOURCE(s->analog_source), net) /
             EFR32_UV_PER_MV;
    }
    return mv > 0 ? mv : 0;
}

/* Convert the input an entry (SINGLE or a SCANTABLE word) selects. */
static uint32_t efr32_iadc_convert(Efr32Periph *s, uint32_t input,
                                   uint32_t fifocfg, uint32_t *code12)
{
    uint32_t cfg = efr32_iadc_get(s, EFR32_IADC_CFG0 +
                                         ((input & EFR32_IADC_INPUT_CFG)
                                              ? EFR32_IADC_CFG_STRIDE
                                              : 0));
    uint32_t gain = (cfg >> EFR32_IADC_CFG_ANALOGGAIN_SHIFT) &
                    EFR32_IADC_CFG_ANALOGGAIN_MASK;
    uint32_t refsel = (cfg >> EFR32_IADC_CFG_REFSEL_SHIFT) &
                      EFR32_IADC_CFG_REFSEL_MASK;
    uint32_t vref = refsel == EFR32_IADC_CFG_REFSEL_VDDX ? EFR32_BOARD_VDDX_MV
                    : refsel == EFR32_IADC_CFG_REFSEL_VBGR
                        ? EFR32_IADC_VBGR_MV
                        : 0;
    uint32_t full;
    uint64_t pos = efr32_iadc_input_mv(
        s, (input >> EFR32_IADC_INPUT_PORTPOS_SHIFT) &
               EFR32_IADC_INPUT_FIELD_MASK,
        (input >> EFR32_IADC_INPUT_PINPOS_SHIFT) &
            EFR32_IADC_INPUT_FIELD_MASK);
    uint64_t neg = efr32_iadc_input_mv(
        s, (input >> EFR32_IADC_INPUT_PORTNEG_SHIFT) &
               EFR32_IADC_INPUT_FIELD_MASK,
        (input >> EFR32_IADC_INPUT_PINNEG_SHIFT) &
            EFR32_IADC_INPUT_FIELD_MASK);
    /*
     * Output resolution (RM SS24.3.7.4): 11 + log2(OSR x DIGAVG) bits, at
     * most 20, MSB-aligned in the 20-bit word and truncated to the 12, 16
     * or 20 bits ALIGNMENT asks for (Figure 24.17).
     */
    unsigned osr_log2 = 1 + ((cfg >> EFR32_IADC_CFG_OSRHS_SHIFT) &
                             EFR32_IADC_CFG_OSRHS_MASK);
    unsigned avg_log2 = MIN((cfg >> EFR32_IADC_CFG_DIGAVG_SHIFT) &
                            EFR32_IADC_CFG_DIGAVG_MASK, 4u);
    unsigned bits = MIN(EFR32_IADC_RESULT_BITS - 1 + osr_log2 + avg_log2,
                        EFR32_IADC_WORD_BITS);
    uint64_t code;
    uint32_t code20;

    full = (1u << bits) - 1;
    if (!vref || gain >= ARRAY_SIZE(efr32_iadc_gain_half) || pos <= neg) {
        code = 0;
    } else {
        /* (pos - neg) * gain / vref of full scale, rounded, clipped. */
        code = MIN((uint64_t)full,
                   ((pos - neg) * efr32_iadc_gain_half[gain] * full +
                    vref) / (2 * (uint64_t)vref));
    }
    code20 = code << (EFR32_IADC_WORD_BITS - bits);
    if (code12) {
        *code12 = code20 >> (EFR32_IADC_WORD_BITS - EFR32_IADC_RESULT_BITS);
    }
    switch (fifocfg & EFR32_IADC_FIFOCFG_ALIGNMENT_MASK) {
    case EFR32_IADC_FIFOCFG_ALIGNMENT_RIGHT16:
        return code20 >> 4;
    case EFR32_IADC_FIFOCFG_ALIGNMENT_RIGHT20:
        return code20;
    case EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT12:
        return (code20 >> 8) << 20;
    case EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT16:
        return (code20 >> 4) << 16;
    case EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT20:
        return code20 << 12;
    default:
        return code20 >> 8;
    }
}

/*
 * Window compare (RM SS24.3.8): for an input with its CMP bit, the 12-bit
 * result against the upper 12 bits of CMPTHR's 16-bit left-justified ADGT
 * (>=) and ADLT (<=): inside the window when ADLT >= ADGT, outside it
 * otherwise.
 */
#define EFR32_IADC_CMPTHR 0x01cu
#define EFR32_IADC_CMPTHR_ADGT_SHIFT 16
#define EFR32_IADC_CMPTHR_FIELD_MASK 0xfff0u
#define EFR32_IADC_INPUT_CMP (1u << 17)
#define EFR32_IADC_IF_SINGLECMP (1u << 2)
#define EFR32_IADC_IF_SCANCMP (1u << 3)

static bool efr32_iadc_window_hit(Efr32Periph *s, uint32_t code12)
{
    uint32_t thr = efr32_iadc_get(s, EFR32_IADC_CMPTHR);
    uint32_t lt = thr & EFR32_IADC_CMPTHR_FIELD_MASK;
    uint32_t gt = (thr >> EFR32_IADC_CMPTHR_ADGT_SHIFT) &
                  EFR32_IADC_CMPTHR_FIELD_MASK;
    uint32_t data = code12 << 4;

    return lt >= gt ? data <= lt && data >= gt : data <= lt || data >= gt;
}

/* Push one result into a queue's FIFO; returns the IF bits it raises. */
static uint32_t efr32_iadc_push(Efr32Periph *s, unsigned q,
                                const Efr32IadcQueue *queue, uint32_t result,
                                uint32_t id)
{
    uint32_t fifocfg = efr32_iadc_get(s, queue->fifocfg);
    uint32_t dvl = (fifocfg >> EFR32_IADC_FIFOCFG_DVL_SHIFT) &
                   EFR32_IADC_FIFOCFG_DVL_MASK;
    uint32_t flags = 0;

    if (fifocfg & EFR32_IADC_FIFOCFG_SHOWID) {
        /* Figure 24.18: bits 7:0 when left-aligned, 31:24 when right. */
        result |= (fifocfg & EFR32_IADC_FIFOCFG_ALIGNMENT_MASK) >=
                          EFR32_IADC_FIFOCFG_ALIGNMENT_LEFT12
                      ? id
                      : id << EFR32_IADC_ID_RIGHT_SHIFT;
    }
    if (s->iadc_fifo[q].count < EFR32_IADC_FIFO_DEPTH) {
        s->iadc_fifo[q].data[s->iadc_fifo[q].count++] = result;
    } else {
        flags |= queue->if_of;
    }
    efr32_iadc_put(s, queue->data, result);
    if (s->iadc_fifo[q].count >= dvl + 1) {
        flags |= queue->if_dvl;
    }
    return flags;
}

/*
 * Conversion timing, RM SS24.3.2-24.3.3 ([I]: the RM's formulas; never
 * measured on silicon):
 *  - CLK_CMU_ADC: CMU_IADCCLKCTRL.CLKSEL, EM01GRPACLK (1, reset) or FSRCO
 *    (2); CLK_SRC_ADC = CLK_CMU_ADC / (CTRL.HSCLKRATE + 1); ADC_CLK =
 *    CLK_SRC_ADC / (SCHEDx.PRESCALE + 1), PRESCALE taken as 1 when it is 0
 *    and HSCLKRATE is not.
 *  - Warm-up: five TIMEBASE intervals ((CTRL.TIMEBASE + 1) CLK_SRC_ADC
 *    cycles) from power-down (WARMUPMODE NORMAL, after every request) or on
 *    a change of PRESCALE, ADCMODE or REFSEL; one from standby
 *    (KEEPINSTANDBY); none while kept warm (KEEPWARM).
 *  - Each conversion: ((4 * OSR) + 2) ADC_CLK cycles, OSR = 2 << OSRHS, plus
 *    2 ADC_CLK cycles when the input changes from the previous conversion.
 *  - Requests are served in the order they arrive (SINGLETAILGATE off); the
 *    input is sampled when its conversion completes.  Triggers: see
 *    efr32_iadc_command.
 */
#define EFR32_IADC_CTRL 0x008u
#define EFR32_IADC_CTRL_WARMUPMODE_SHIFT 4
#define EFR32_IADC_CTRL_WARMUPMODE_MASK 0x3u
#define EFR32_IADC_WARMUP_KEEPINSTANDBY 1u
#define EFR32_IADC_WARMUP_KEEPWARM 2u
#define EFR32_IADC_CTRL_TIMEBASE_SHIFT 16
#define EFR32_IADC_CTRL_TIMEBASE_MASK 0x7fu
#define EFR32_IADC_CTRL_HSCLKRATE_SHIFT 28
#define EFR32_IADC_CTRL_HSCLKRATE_MASK 0x7u
#define EFR32_IADC_SCHED0 0x054u
#define EFR32_IADC_SCHED_PRESCALE_MASK 0x3ffu
#define EFR32_IADC_CFG_ADCMODE_MASK 0x3u
#define EFR32_IADC_CFG_OSRHS_SHIFT 2
#define EFR32_IADC_CFG_OSRHS_MASK 0x7u
#define EFR32_CMU_IADCCLKCTRL 0x180u
#define EFR32_CMU_IADCCLKCTRL_CLKSEL_MASK 0x3u
#define EFR32_CMU_IADCCLKCTRL_EM01GRPACLK 1u
#define EFR32_CMU_IADCCLKCTRL_FSRCO 2u
#define EFR32_IADC_WARMUP_INTERVALS_OFF 5
#define EFR32_IADC_WARMUP_INTERVALS_STANDBY 1
#define EFR32_IADC_CONV_SWITCH_CYCLES 2

static uint64_t efr32_iadc_cmu_hz(Efr32Periph *s)
{
    switch (efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_IADCCLKCTRL) &
            EFR32_CMU_IADCCLKCTRL_CLKSEL_MASK) {
    case 0:
    case EFR32_CMU_IADCCLKCTRL_EM01GRPACLK:
        /* Reset value EM01GRPACLK (the register store reads 0 before). */
        return efr32_em01grpa_hz(s);
    case EFR32_CMU_IADCCLKCTRL_FSRCO:
        return EFR32_FSRCO_MEASURED_HZ;
    default:
        return 0;
    }
}

/* Schedule one conversion of @input (config group from it). */
static void efr32_iadc_queue(Efr32Periph *s, unsigned queue, unsigned id,
                             uint32_t input, bool last)
{
    uint32_t ctrl = efr32_iadc_get(s, EFR32_IADC_CTRL);
    unsigned group = (input & EFR32_IADC_INPUT_CFG) ? 1 : 0;
    uint32_t cfg = efr32_iadc_get(s, EFR32_IADC_CFG0 +
                                         group * EFR32_IADC_CFG_STRIDE);
    uint32_t prescale = efr32_iadc_get(s, EFR32_IADC_SCHED0 +
                                              group * EFR32_IADC_CFG_STRIDE) &
                        EFR32_IADC_SCHED_PRESCALE_MASK;
    uint32_t hsclkrate = (ctrl >> EFR32_IADC_CTRL_HSCLKRATE_SHIFT) &
                         EFR32_IADC_CTRL_HSCLKRATE_MASK;
    uint32_t mode = (ctrl >> EFR32_IADC_CTRL_WARMUPMODE_SHIFT) &
                    EFR32_IADC_CTRL_WARMUPMODE_MASK;
    uint32_t key, osr, cycles;
    uint64_t src_hz = efr32_iadc_cmu_hz(s) / (hsclkrate + 1), adc_hz;
    int64_t t;

    if (hsclkrate && !prescale) {
        prescale = 1;
    }
    adc_hz = src_hz / (prescale + 1);
    if (!adc_hz || s->iadc_nconv == ARRAY_SIZE(s->iadc_conv)) {
        return;
    }
    t = MAX(MAX(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), s->iadc_request_ns),
            s->iadc_busy_until_ns);
    key = prescale | (cfg & EFR32_IADC_CFG_ADCMODE_MASK) << 10 |
          ((cfg >> EFR32_IADC_CFG_REFSEL_SHIFT) &
           EFR32_IADC_CFG_REFSEL_MASK) << 12;
    if (!s->iadc_warm || key != s->iadc_warm_key) {
        unsigned intervals = s->iadc_standby && key == s->iadc_warm_key
                                 ? EFR32_IADC_WARMUP_INTERVALS_STANDBY
                                 : EFR32_IADC_WARMUP_INTERVALS_OFF;

        t += muldiv64((uint64_t)intervals *
                          (((ctrl >> EFR32_IADC_CTRL_TIMEBASE_SHIFT) &
                            EFR32_IADC_CTRL_TIMEBASE_MASK) + 1),
                      NANOSECONDS_PER_SECOND, src_hz);
        s->iadc_warm = true;
        s->iadc_warm_key = key;
    }
    osr = 2u << ((cfg >> EFR32_IADC_CFG_OSRHS_SHIFT) &
                 EFR32_IADC_CFG_OSRHS_MASK);
    /* DIGAVG: 1-16 conversions back to back per result (SS24.3.7.3). */
    cycles = (4 * osr + 2) << MIN((cfg >> EFR32_IADC_CFG_DIGAVG_SHIFT) &
                                  EFR32_IADC_CFG_DIGAVG_MASK, 4u);
    if (s->iadc_have_input && input != s->iadc_last_input) {
        cycles += EFR32_IADC_CONV_SWITCH_CYCLES;
    }
    s->iadc_last_input = input;
    s->iadc_have_input = true;
    t += muldiv64(cycles, NANOSECONDS_PER_SECOND, adc_hz);
    s->iadc_busy_until_ns = t;
    s->iadc_conv[s->iadc_nconv++] = (typeof(s->iadc_conv[0])){
        t, queue, id, last, input,
    };
    if (last && mode != EFR32_IADC_WARMUP_KEEPWARM) {
        /* Powered down or in standby once the request is served. */
        s->iadc_warm = false;
        s->iadc_standby = mode == EFR32_IADC_WARMUP_KEEPINSTANDBY;
    }
    timer_mod(s->iadc_timer, s->iadc_conv[0].done_ns);
}

static bool efr32_iadc_continuous(Efr32Periph *s, unsigned q);
static void efr32_iadc_request(Efr32Periph *s, unsigned q, int64_t at);

/* Conversions whose time has come deliver their results. */
static void efr32_iadc_complete(void *opaque)
{
    Efr32Periph *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t flags = 0;
    unsigned done = 0;
    bool request_done[2] = { false, false };

    while (done < s->iadc_nconv && s->iadc_conv[done].done_ns <= now) {
        typeof(s->iadc_conv[0]) *c = &s->iadc_conv[done++];
        const Efr32IadcQueue *queue = c->queue ? &efr32_iadc_scan
                                               : &efr32_iadc_single;
        uint32_t code12;
        uint32_t result = efr32_iadc_convert(s, c->input,
                                             efr32_iadc_get(s,
                                                            queue->fifocfg),
                                             &code12);

        if ((c->input & EFR32_IADC_INPUT_CMP) &&
            efr32_iadc_window_hit(s, code12)) {
            flags |= c->queue ? EFR32_IADC_IF_SCANCMP
                              : EFR32_IADC_IF_SINGLECMP;
        }

        request_done[c->queue] |= c->last;
        flags |= efr32_iadc_push(s, c->queue, queue, result, c->id);
        if (c->queue) {
            flags |= EFR32_IADC_IF_SCANENTRYDONE;
            flags |= c->last ? EFR32_IADC_IF_SCANTABLEDONE : 0;
        } else {
            flags |= EFR32_IADC_IF_SINGLEDONE;
        }
    }
    memmove(s->iadc_conv, s->iadc_conv + done,
            (s->iadc_nconv - done) * sizeof(s->iadc_conv[0]));
    s->iadc_nconv -= done;
    for (unsigned q = 0; q < 2; q++) {
        if (request_done[q] && s->iadc_qen[q] && efr32_iadc_continuous(s, q)) {
            efr32_iadc_request(s, q, now);
        }
    }
    if (s->iadc_nconv) {
        timer_mod(s->iadc_timer, s->iadc_conv[0].done_ns);
    }
    efr32_iadc_put(s, EFR32_IADC_IF, efr32_iadc_get(s, EFR32_IADC_IF) | flags);
    efr32_iadc_update(s);
    /* PRS IADC0 producers, both tables (RM 13.4, 13.5), with their flags. */
    {
        static const struct {
            uint32_t flag;
            unsigned signal;
        } prs[] = {
            { EFR32_IADC_IF_SCANENTRYDONE, EFR32_PRS_SIG_IADC_SCANENTRYDONE },
            { EFR32_IADC_IF_SCANTABLEDONE, EFR32_PRS_SIG_IADC_SCANTABLEDONE },
            { EFR32_IADC_IF_SINGLEDONE, EFR32_PRS_SIG_IADC_SINGLEDONE },
        };

        for (unsigned n = 0; n < ARRAY_SIZE(prs); n++) {
            if (flags & prs[n].flag) {
                efr32_prs_pulse(s, EFR32_PRS_SRC_IADC0, prs[n].signal);
                efr32_prs_sync_pulse(s, EFR32_PRS_SYNC_SRC_IADC0,
                                     prs[n].signal);
            }
        }
    }
}

/*
 * Triggers (RM SS24.3.3.3, SS24.5.4, SS24.5.12).  CMD.SINGLESTART /
 * SCANSTART enable a queue's trigger and xSTOP disables it (STATUS.xQEN),
 * dropping its conversions not yet started.  TRIGGER.xTRIGSEL:
 *  - IMMEDIATE: START itself is the trigger; with xTRIGACTION = ONCE the
 *    queue is disabled again once its request is converted.
 *  - TIMER: the local timer (CMD.TIMEREN / TIMERDIS, STATUS.TIMERACTIVE)
 *    triggers every IADC_TIMER.TIMER CLK_SRC_ADC cycles.  [I] Its first
 *    event comes one period after TIMEREN.
 *  - PRSCLKGRP: a rising edge of the synchronous PRS channel
 *    (PRS_CONSUMER_IADC0_xTRIGGER.SPRSSEL), at once.
 *  - PRSPOS / PRSNEG: a rising / falling edge of the asynchronous channel
 *    (PRSSEL), "an additional delay of 1 to 2 CLK_SRC_ADC cycles for
 *    synchronization" ([I] 2 here).
 * PRS0 (the scan trigger) is PRS_CONSUMER_IADC0_SCANTRIGGER, PRS1 the
 * single one.  xTRIGACTION = CONTINUOUS requests again as soon as a request
 * has been converted, while the queue stays enabled.
 * [U] Not modelled: SINGLETAILGATE (requests are served in arrival order),
 * the 3-ADC_CLK re-arm time of PRSPOS/PRSNEG.
 */
#define EFR32_IADC_TIMER 0x010u
#define EFR32_IADC_TIMER_MASK 0xffffu
#define EFR32_IADC_TRIGGER 0x02cu
#define EFR32_IADC_TRIGGER_SCANTRIGSEL_SHIFT 0
#define EFR32_IADC_TRIGGER_SINGLETRIGSEL_SHIFT 8
#define EFR32_IADC_TRIGGER_TRIGSEL_MASK 0x7u
#define EFR32_IADC_TRIGGER_SCANTRIGACTION (1u << 4)
#define EFR32_IADC_TRIGGER_SINGLETRIGACTION (1u << 12)
#define EFR32_IADC_TRIGSEL_IMMEDIATE 0u
#define EFR32_IADC_TRIGSEL_TIMER 1u
#define EFR32_IADC_TRIGSEL_PRSCLKGRP 2u
#define EFR32_IADC_TRIGSEL_PRSPOS 3u
#define EFR32_IADC_TRIGSEL_PRSNEG 4u
#define EFR32_IADC_CMD_SINGLESTOP (1u << 1)
#define EFR32_IADC_CMD_SCANSTOP (1u << 4)
#define EFR32_IADC_CMD_TIMEREN (1u << 16)
#define EFR32_IADC_CMD_TIMERDIS (1u << 17)
#define EFR32_IADC_PRS_SYNC_CYCLES 2u

static uint64_t efr32_iadc_src_hz(Efr32Periph *s)
{
    return efr32_iadc_cmu_hz(s) /
           (((efr32_iadc_get(s, EFR32_IADC_CTRL) >>
              EFR32_IADC_CTRL_HSCLKRATE_SHIFT) &
             EFR32_IADC_CTRL_HSCLKRATE_MASK) + 1);
}

static unsigned efr32_iadc_trigsel(Efr32Periph *s, unsigned q)
{
    return (efr32_iadc_get(s, EFR32_IADC_TRIGGER) >>
            (q ? EFR32_IADC_TRIGGER_SCANTRIGSEL_SHIFT
               : EFR32_IADC_TRIGGER_SINGLETRIGSEL_SHIFT)) &
           EFR32_IADC_TRIGGER_TRIGSEL_MASK;
}

static bool efr32_iadc_continuous(Efr32Periph *s, unsigned q)
{
    return efr32_iadc_get(s, EFR32_IADC_TRIGGER) &
           (q ? EFR32_IADC_TRIGGER_SCANTRIGACTION
              : EFR32_IADC_TRIGGER_SINGLETRIGACTION);
}

/* One request of queue @q (single: SINGLE; scan: the MASKREQ entries). */
static void efr32_iadc_request(Efr32Periph *s, unsigned q, int64_t at)
{
    if (s->iadc_stall) {
        return;
    }
    s->iadc_request_ns = at;
    if (!q) {
        efr32_iadc_queue(s, 0, EFR32_IADC_SINGLE_ID,
                         efr32_iadc_get(s, EFR32_IADC_SINGLE), true);
    } else {
        uint32_t mask = efr32_iadc_get(s, EFR32_IADC_MASKREQ) &
                        EFR32_IADC_MASKREQ_MASK;

        for (unsigned id = 0; id < EFR32_IADC_SCANTABLE_ENTRIES; id++) {
            if (mask & (1u << id)) {
                efr32_iadc_queue(s, 1, id,
                                 efr32_iadc_get(s, EFR32_IADC_SCANTABLE0 +
                                                       4 * id),
                                 !(mask >> (id + 1)));
            }
        }
    }
    s->iadc_request_ns = 0;
}

static void efr32_iadc_local_timer_arm(Efr32Periph *s)
{
    uint32_t period = efr32_iadc_get(s, EFR32_IADC_TIMER) &
                      EFR32_IADC_TIMER_MASK;
    uint64_t hz = efr32_iadc_src_hz(s);

    timer_del(s->iadc_local_timer);
    if (s->iadc_timer_on && period && hz) {
        timer_mod(s->iadc_local_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      efr32_cycles_ns(period, hz));
    }
}

static void efr32_iadc_local_timer_expired(void *opaque)
{
    Efr32Periph *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (unsigned q = 0; q < 2; q++) {
        if (s->iadc_qen[q] &&
            efr32_iadc_trigsel(s, q) == EFR32_IADC_TRIGSEL_TIMER) {
            efr32_iadc_request(s, q, now);
        }
    }
    efr32_iadc_local_timer_arm(s);
    efr32_iadc_update(s);
}

/* PRS consumer lines of the IADC (efr32-prs.h numbering). */
static void efr32_iadc_prs_trigger(Efr32Periph *s, int n, bool sync,
                                   bool level)
{
    unsigned q, trigsel;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (n == EFR32_PRS_CON_IADC0_SCANTRIGGER) {
        q = 1;
    } else if (n == EFR32_PRS_CON_IADC0_SINGLETRIGGER) {
        q = 0;
    } else {
        return;
    }
    if (!s->iadc_qen[q]) {
        return;
    }
    trigsel = efr32_iadc_trigsel(s, q);
    if (sync) {
        if (trigsel == EFR32_IADC_TRIGSEL_PRSCLKGRP && level) {
            efr32_iadc_request(s, q, now);
        }
    } else if ((trigsel == EFR32_IADC_TRIGSEL_PRSPOS && level) ||
               (trigsel == EFR32_IADC_TRIGSEL_PRSNEG && !level)) {
        uint64_t hz = efr32_iadc_src_hz(s);

        if (hz) {
            efr32_iadc_request(s, q, now + efr32_cycles_ns(
                                              EFR32_IADC_PRS_SYNC_CYCLES, hz));
        }
    }
    efr32_iadc_update(s);
}

/* Drop queue @q's conversions that have not started (xSTOP). */
static void efr32_iadc_drop_pending(Efr32Periph *s, unsigned q)
{
    unsigned kept = 0;

    for (unsigned i = 0; i < s->iadc_nconv; i++) {
        /* [I] The head of the list is the conversion in progress. */
        if (i == 0 || s->iadc_conv[i].queue != q) {
            s->iadc_conv[kept++] = s->iadc_conv[i];
        }
    }
    s->iadc_nconv = kept;
    if (kept) {
        s->iadc_busy_until_ns = s->iadc_conv[kept - 1].done_ns;
    }
}

static void efr32_iadc_command(Efr32Periph *s, uint32_t command)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (command & EFR32_IADC_CMD_SINGLEFIFOFLUSH) {
        s->iadc_fifo[0].count = 0;
    }
    if (command & EFR32_IADC_CMD_SCANFIFOFLUSH) {
        s->iadc_fifo[1].count = 0;
    }
    for (unsigned q = 0; q < 2; q++) {
        uint32_t stop = q ? EFR32_IADC_CMD_SCANSTOP : EFR32_IADC_CMD_SINGLESTOP;
        uint32_t start = q ? EFR32_IADC_CMD_SCANSTART
                           : EFR32_IADC_CMD_SINGLESTART;

        if (command & stop) {
            s->iadc_qen[q] = false;
            efr32_iadc_drop_pending(s, q);
        } else if (command & start) {
            bool immediate = efr32_iadc_trigsel(s, q) ==
                             EFR32_IADC_TRIGSEL_IMMEDIATE;

            s->iadc_qen[q] = !immediate || efr32_iadc_continuous(s, q);
            if (immediate) {
                efr32_iadc_request(s, q, now);
            }
        }
    }
    if (command & EFR32_IADC_CMD_TIMERDIS) {
        s->iadc_timer_on = false;
        efr32_iadc_local_timer_arm(s);
    } else if (command & EFR32_IADC_CMD_TIMEREN) {
        s->iadc_timer_on = true;
        efr32_iadc_local_timer_arm(s);
    }
    efr32_iadc_update(s);
}

static bool efr32_iadc_read(Efr32Periph *s, uint32_t address, uint32_t *value)
{
    unsigned q;
    const Efr32IadcQueue *queue;

    if ((address & ~0x3fffu) != EFR32_IADC_BASE) {
        return false;
    }
    switch (address & 0xfffu) {
    case EFR32_IADC_SINGLEFIFODATA:
        q = 0;
        queue = &efr32_iadc_single;
        break;
    case EFR32_IADC_SCANFIFODATA:
        q = 1;
        queue = &efr32_iadc_scan;
        break;
    case EFR32_IADC_CMD:
        *value = 0;
        return true;
    default:
        return false;
    }
    if (!s->iadc_fifo[q].count) {
        efr32_iadc_put(s, EFR32_IADC_IF,
                       efr32_iadc_get(s, EFR32_IADC_IF) | queue->if_uf);
        *value = 0;
    } else {
        *value = s->iadc_fifo[q].data[0];
        memmove(s->iadc_fifo[q].data, s->iadc_fifo[q].data + 1,
                --s->iadc_fifo[q].count * sizeof(s->iadc_fifo[q].data[0]));
    }
    efr32_iadc_update(s);
    return true;
}

/* Returns true when the IADC write is fully handled. */
static bool efr32_iadc_write(Efr32Periph *s, uint32_t offset, uint32_t next)
{
    switch (offset) {
    case EFR32_IADC_CMD:
        efr32_iadc_command(s, next);
        return true;
    case EFR32_IADC_STATUS:
    case EFR32_IADC_SINGLEFIFOSTAT:
    case EFR32_IADC_SINGLEFIFODATA:
    case EFR32_IADC_SINGLEDATA:
    case EFR32_IADC_SCANFIFOSTAT:
    case EFR32_IADC_SCANFIFODATA:
    case EFR32_IADC_SCANDATA:
        return true; /* read-only */
    case EFR32_IADC_IF:
    case EFR32_IADC_IEN:
        efr32_iadc_put(s, offset, next);
        efr32_iadc_update(s);
        return true;
    default:
        return false;
    }
}

/* ---- MSC: flash erase and write (see EFR32_MSC_BASE above) ---- */

static uint32_t efr32_msc_get(Efr32Periph *s, uint32_t offset)
{
    return efr32_register_get(s, EFR32_MSC_BASE + offset);
}

static void efr32_msc_put(Efr32Periph *s, uint32_t offset, uint32_t value)
{
    efr32_register_put(s, EFR32_MSC_BASE + offset, value);
}

static void efr32_msc_update_irq(Efr32Periph *s)
{
    qemu_set_irq(s->irq[EFR32_MSC_IRQ],
                 !!(efr32_msc_get(s, EFR32_MSC_IF) &
                    efr32_msc_get(s, EFR32_MSC_IEN)));
}

static void efr32_msc_status(Efr32Periph *s, uint32_t clear, uint32_t set)
{
    efr32_msc_put(s, EFR32_MSC_STATUS,
                  (efr32_msc_get(s, EFR32_MSC_STATUS) & ~clear) | set);
}

/* The erasable page that holds address: main flash or User Data. */
static bool efr32_msc_page(Efr32Periph *s, uint32_t address,
                           uint32_t *base, uint32_t *size)
{
    uint32_t userdata = EFR32_INFO_BASE + EFR32_USERDATA_OFFSET;
    uint32_t userdata_size =
        efr32_msc_get(s, EFR32_MSC_USERDATASIZE) *
        EFR32_MSC_USERDATASIZE_UNIT;

    if (address < s->flash_size) {
        *base = address & ~(EFR32_FLASH_PAGE_SIZE - 1);
        *size = EFR32_FLASH_PAGE_SIZE;
        return true;
    }
    if (address >= userdata && address - userdata < userdata_size) {
        *base = userdata;
        *size = userdata_size;
        return true;
    }
    return false;
}

static bool efr32_msc_page_locked(Efr32Periph *s, uint32_t address)
{
    uint32_t page = address / EFR32_FLASH_PAGE_SIZE;

    if (address >= s->flash_size) {
        return efr32_msc_get(s, EFR32_MSC_MISCLOCKWORD) &
               EFR32_MSC_MISCLOCKWORD_UDLOCKBIT;
    }
    return efr32_msc_get(s, page < EFR32_MSC_PAGES_PER_LOCKWORD
                                ? EFR32_MSC_PAGELOCK0
                                : EFR32_MSC_PAGELOCK1) &
           (1u << (page % EFR32_MSC_PAGES_PER_LOCKWORD));
}

static void efr32_msc_fill(Efr32Periph *s, uint32_t base, uint32_t size)
{
    g_autofree uint8_t *erased = g_malloc(size);

    memset(erased, EFR32_FLASH_ERASED_BYTE, size);
    address_space_write(&s->dma_as, base, MEMTXATTRS_UNSPECIFIED, erased,
                        size);
}

/* An operation on this address may start: enabled, valid, not locked. */
static bool efr32_msc_may_modify(Efr32Periph *s, uint32_t address)
{
    uint32_t base, size;

    if (!(efr32_msc_get(s, EFR32_MSC_WRITECTRL) &
          EFR32_MSC_WRITECTRL_WREN) ||
        !efr32_msc_page(s, address, &base, &size)) {
        return false;
    }
    if (efr32_msc_page_locked(s, address)) {
        efr32_msc_status(s, 0, EFR32_MSC_STATUS_LOCKED);
        return false;
    }
    efr32_msc_status(s, EFR32_MSC_STATUS_LOCKED, 0);
    return true;
}

static void efr32_msc_command(Efr32Periph *s, uint32_t command)
{
    uint32_t address = efr32_msc_get(s, EFR32_MSC_ADDRB);
    uint32_t base, size;

    if ((command & EFR32_MSC_WRITECMD_ERASEPAGE) &&
        efr32_msc_may_modify(s, address) &&
        efr32_msc_page(s, address, &base, &size)) {
        efr32_msc_fill(s, base, size);
        efr32_msc_put(s, EFR32_MSC_IF,
                      efr32_msc_get(s, EFR32_MSC_IF) | EFR32_MSC_IF_ERASE);
    }
    if (command & EFR32_MSC_WRITECMD_ERASEMAIN0) {
        if (efr32_msc_get(s, EFR32_MSC_MISCLOCKWORD) &
            EFR32_MSC_MISCLOCKWORD_MELOCKBIT) {
            efr32_msc_status(s, 0, EFR32_MSC_STATUS_LOCKED);
        } else {
            efr32_msc_status(s, EFR32_MSC_STATUS_LOCKED, 0);
            efr32_msc_fill(s, 0, s->flash_size);
            efr32_msc_put(s, EFR32_MSC_IF, efr32_msc_get(s, EFR32_MSC_IF) |
                                               EFR32_MSC_IF_ERASE);
        }
    }
    /* WRITEEND, ERASEABORT, CLEARWDATA: nothing is ever in flight. */
    efr32_msc_update_irq(s);
}

static void efr32_msc_program(Efr32Periph *s, uint32_t data)
{
    uint32_t address = efr32_msc_get(s, EFR32_MSC_ADDRB) & ~3u;
    uint8_t word[4];

    if (efr32_msc_may_modify(s, address)) {
        /* Programming only clears bits (RM SS6.3.12). */
        address_space_read(&s->dma_as, address, MEMTXATTRS_UNSPECIFIED, word,
                           sizeof(word));
        stl_le_p(word, ldl_le_p(word) & data);
        address_space_write(&s->dma_as, address, MEMTXATTRS_UNSPECIFIED,
                            word, sizeof(word));
        efr32_msc_put(s, EFR32_MSC_IF,
                      efr32_msc_get(s, EFR32_MSC_IF) | EFR32_MSC_IF_WRITE);
    }
    /* "After each word is written, ADDRB is incremented automatically by
     * 4" (RM SS6.3.12). */
    efr32_msc_put(s, EFR32_MSC_ADDRB, address + 4);
    efr32_msc_update_irq(s);
}

static void efr32_msc_check_address(Efr32Periph *s)
{
    uint32_t base, size;
    bool valid = efr32_msc_page(s, efr32_msc_get(s, EFR32_MSC_ADDRB), &base,
                                &size);

    efr32_msc_status(s, EFR32_MSC_STATUS_INVADDR,
                     valid ? 0 : EFR32_MSC_STATUS_INVADDR);
    /*
     * Software checks INVADDR and LOCKED after loading ADDRB, before it
     * starts an operation (RM SS6.3.12), so LOCKED follows ADDRB too.
     */
    efr32_msc_status(s, EFR32_MSC_STATUS_LOCKED,
                     valid && efr32_msc_page_locked(
                         s, efr32_msc_get(s, EFR32_MSC_ADDRB)) ?
                     EFR32_MSC_STATUS_LOCKED : 0);
}

/* Returns true when the write to this MSC register is fully handled. */
static bool efr32_msc_write(Efr32Periph *s, uint32_t offset, uint32_t old,
                            uint32_t next)
{
    bool reglocked = efr32_msc_get(s, EFR32_MSC_STATUS) &
                     EFR32_MSC_STATUS_REGLOCK;

    switch (offset) {
    case EFR32_MSC_READCTRL:
        return reglocked;
    case EFR32_MSC_WRITECTRL:
        if (!reglocked) {
            efr32_msc_put(s, offset, next & EFR32_MSC_WRITECTRL_MASK);
        }
        return true;
    case EFR32_MSC_MISCLOCKWORD:
        /*
         * "Host is only allowed to write one. Root and debug can clear this
         * bit" (RM SS6.8.13): a core can only set the locks; a debugger
         * access (no current CPU: the debug port) may clear them.
         */
        efr32_msc_put(s, offset, (current_cpu ? old | next : next) &
                                 EFR32_MSC_MISCLOCKWORD_MASK);
        return true;
    case EFR32_MSC_WRITECMD:
        efr32_msc_command(s, next);
        return true;
    case EFR32_MSC_ADDRB:
        efr32_msc_put(s, offset, next);
        efr32_msc_check_address(s);
        return true;
    case EFR32_MSC_WDATA:
        efr32_msc_put(s, offset, next);
        efr32_msc_program(s, next);
        return true;
    case EFR32_MSC_STATUS:
    case EFR32_MSC_USERDATASIZE:
        return true; /* read-only */
    case EFR32_MSC_LOCK:
        efr32_msc_status(s, EFR32_MSC_STATUS_REGLOCK,
                         (next & EFR32_MSC_LOCK_LOCKKEY_MASK) ==
                                 EFR32_MSC_LOCK_UNLOCK_KEY
                             ? 0
                             : EFR32_MSC_STATUS_REGLOCK);
        return true;
    case EFR32_MSC_PAGELOCK0:
    case EFR32_MSC_PAGELOCK1:
        /* "Host is only allowed to write one" (RM SS6.8.15-16). */
        efr32_msc_put(s, offset, old | next);
        return true;
    case EFR32_MSC_IF:
    case EFR32_MSC_IEN:
        efr32_msc_put(s, offset, next);
        efr32_msc_update_irq(s);
        return true;
    default:
        return false;
    }
}

static void efr32_msc_reset(Efr32Periph *s)
{
    efr32_msc_put(s, EFR32_MSC_STATUS, EFR32_MSC_STATUS_RESET);
    efr32_msc_put(s, EFR32_MSC_USERDATASIZE, EFR32_MSC_USERDATASIZE_RESET);
    efr32_msc_put(s, EFR32_MSC_MISCLOCKWORD, EFR32_MSC_MISCLOCKWORD_RESET);
    efr32_msc_put(s, EFR32_MSC_READCTRL, EFR32_MSC_READCTRL_RESET);
    efr32_msc_put(s, EFR32_MSC_PWRCTRL, EFR32_MSC_PWRCTRL_RESET);
    efr32_msc_update_irq(s);
}

static uint32_t efr32_peripheral_read_word(Efr32Periph *s, uint32_t address)
{
    uint32_t ipversion;

    address = efr32_peripheral_canonical_address(address);
    uint32_t offset = address & 0xfff;
    unsigned channel;
    uint32_t register_offset;
    int usart_index = efr32_usart_index(address);

    if (efr32_radio_ipversion(address, &ipversion)) {
        return ipversion;
    }
    if (s->pa_ramp_pending &&
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->pa_ramp_done_ns &&
        (address == EFR32_MODEM_BASE + EFR32_MODEM_IF ||
         address == EFR32_MODEM_BASE + EFR32_MODEM_SEQIF)) {
        efr32_pa_ramp_complete(s);
    }
    if (address == EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTSTATUS) {
        uint32_t status;

        efr32_synth_locnt_complete(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        status = efr32_register_get(s, address) & ~EFR32_SYNTH_LOCNTSTATUS_BUSY;

        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->locnt_busy_until_ns) {
            status |= EFR32_SYNTH_LOCNTSTATUS_BUSY;
        }
        return status;
    }
    if (address == EFR32_SYNTH_BASE + EFR32_SYNTH_STATUS) {
        uint32_t status = efr32_register_get(s, address) &
                          ~(EFR32_SYNTH_STATUS_INLOCK |
                            EFR32_SYNTH_STATUS_IFFREQEN);

        if (s->synth_if_enabled) {
            status |= EFR32_SYNTH_STATUS_IFFREQEN;
        } else if (s->synth_running &&
                   qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->synth_lock_ns &&
                   efr32_synth_in_range(efr32_rf_carrier_hz(s))) {
            /* A carrier outside the measured range never locks (SS7.85). */
            status |= EFR32_SYNTH_STATUS_INLOCK;
        }
        return status;
    }

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
    {
        uint32_t rtcc_value;

        if (efr32_rtcc_read(s, address, &rtcc_value) ||
            efr32_iadc_read(s, address, &rtcc_value) ||
            efr32_wdog_read(s, address, &rtcc_value)) {
            return rtcc_value;
        }
    }
    if (address == EFR32_MSC_BASE + EFR32_MSC_LOCK) {
        /* "bit 0 is set when the lock is enabled" (RM SS6.8.12). */
        return !!(efr32_msc_get(s, EFR32_MSC_STATUS) &
                  EFR32_MSC_STATUS_REGLOCK);
    }
    if (address == EFR32_MSC_BASE + EFR32_MSC_WRITECMD) {
        return 0; /* write-only command strobes */
    }
    if (address == EFR32_MSC_BASE) {
        return EFR32_MSC_IPVERSION_VALUE;
    }
    if (address >= EFR32_RADIOAES_BASE &&
        address < EFR32_RADIOAES_BASE + EFR32_RADIOAES_SIZE) {
        uint32_t value;

        if (efr32_radioaes_read(&s->radioaes, address - EFR32_RADIOAES_BASE,
                                &value)) {
            return value;
        }
    }
    if (address >= EFR32_GPCRC_BASE &&
        address < EFR32_GPCRC_BASE + EFR32_GPCRC_SIZE) {
        uint32_t value;

        if (efr32_gpcrc_read(&s->gpcrc, address - EFR32_GPCRC_BASE,
                             &value)) {
            return value;
        }
    }
    if (address >= EFR32_TRNG_FIFO_BASE &&
        address < EFR32_TRNG_FIFO_BASE + EFR32_TRNG_FIFO_SIZE) {
        return efr32_trng_pop(&s->trng, clock_get_hz(s->pclk));
    }
    if (address >= EFR32_TRNG_CTRL_BASE &&
        address < EFR32_TRNG_CTRL_BASE + EFR32_TRNG_CTRL_SIZE) {
        uint32_t value;

        if (efr32_trng_read(&s->trng, address - EFR32_TRNG_CTRL_BASE,
                            clock_get_hz(s->pclk), &value)) {
            return value;
        }
    }
    if (address == 0x40008008)
        /* CMU.STATUS: every ready flag set; WDOGLOCK as CMU_WDOGLOCK left it. */
        return (UINT32_MAX & ~EFR32_CMU_STATUS_WDOGLOCK) |
               (efr32_cmu_get(s, EFR32_CMU_STATUS) & EFR32_CMU_STATUS_WDOGLOCK);
    if (address == 0x4001000c || address == 0x40018004 || address == 0x40028008)
        return 0x00010001;
    if (address == EFR32_HFXO_BASE + EFR32_HFXO_STATUS)
        return efr32_hfxo_status(s);
    if (address == EFR32_DPLL0_BASE + EFR32_DPLL_STATUS)
        return efr32_dpll_status(s);
    if (address == EFR32_LFXO_BASE + EFR32_LFXO_STATUS)
        return efr32_lfxo_status(s);
    if (address == EFR32_LFRCO_BASE + EFR32_LFRCO_STATUS)
        return efr32_lfrco_status(s);
    if (address == EFR32_DCDC_BASE + EFR32_DCDC_IF)
        return efr32_register_get(s, EFR32_DCDC_BASE + EFR32_DCDC_CTRL) & 1
                   ? EFR32_DCDC_IF_REGULATION
                   : 0;
    if (address == EFR32_DCDC_BASE + EFR32_DCDC_STATUS)
        return efr32_register_get(s, EFR32_DCDC_BASE + EFR32_DCDC_CTRL) & 1
                   ? EFR32_DCDC_STATUS_WARM | EFR32_DCDC_STATUS_RUNNING
                   : EFR32_DCDC_STATUS_WARM;
    if (address == EFR32_BURTC_BASE + EFR32_BURTC_CNT)
        return efr32_burtc_counter(s);
    if (address == EFR32_BURTC_BASE + EFR32_BURTC_STATUS)
        return efr32_burtc_status(s);
    if (address == EFR32_BURTC_BASE + EFR32_BURTC_SYNCBUSY)
        return efr32_burtc_syncbusy(s);
    if (address == EFR32_BURTC_BASE + EFR32_BURTC_COMP)
        return efr32_burtc_comp(s);
    {
        uint32_t timer_value;

        if (efr32_timer_read(s, address, &timer_value)) {
            return timer_value;
        }
    }
    if (address == EFR32_RAC_BASE + EFR32_RAC_STATUS)
        return efr32_rac_status(s);
    if (address == EFR32_RAC_BASE + EFR32_RAC_STIMER)
        return efr32_rac_stimer(s);
    if (((address & ~0x3fffu) == EFR32_PROTIMER_BASE ||
         (address & ~0x3fffu) == EFR32_PROTIMER_NS_BASE) &&
        offset == EFR32_PROTIMER_COUNTER)
    {
        return efr32_protimer_counter(s);
    }
    if (address == EFR32_AGC_BASE + EFR32_AGC_CMD)
        return 0; /* AGC.CMD is write-only in the public register model. */
    return efr32_register_get(s, address);
}
static uint64_t efr32_peripheral_read(Efr32Periph *s, uint32_t address,
                                      unsigned size)
{
    return extract32(efr32_peripheral_read_word(s, address & ~3u),
                     (address & 3) * 8, size * 8);
}
/*
 * RAC clock retiming and the debug port (silicon, owo-open-fw
 * docs/radio-tables.md SS7.80, TODO.md): with XORETIMECTRL.XORETIMEENRETIME
 * and DIGCLKRETIMECTRL.DIGCLKRETIMEENRETIME (bit 0 of each, public RAC_S
 * field model) enabled by the open sequencer in its RX tune, with stock's
 * RXSEARCH limit fields, the EK's debug port wedged within milliseconds of
 * every boot (efr32-dap.c dap_debug_wedge has what a probe sees).  The
 * mechanism is unknown.  [I] The trigger is the observed configuration:
 * both ENRETIME bits set while SYSCLK is not the HFXO -- the open image runs
 * SYSCLK from HFRCODPLL (76.8 MHz); stock, which clears ENRETIME itself
 * (fn_0xa76, SS7.7) and so also enables it, runs SYSCLK from the HFXO and
 * stays debuggable (AP3 captures of stock in RX, SS7.64), and SS7.80 names
 * that clock as the remaining difference.  The limit fields are not part of
 * the trigger (only stock's values were tried).  [I] The wedge starts at the
 * write; "within milliseconds" was not resolved further.
 */
#define EFR32_RAC_DIGCLKRETIMECTRL 0x16cu
#define EFR32_RAC_XORETIMECTRL 0x174u
#define EFR32_RAC_RETIME_ENRETIME (1u << 0)

static void efr32_rac_retime_update(Efr32Periph *s)
{
    bool retime =
        (efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_DIGCLKRETIMECTRL) &
         EFR32_RAC_RETIME_ENRETIME) &&
        (efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_XORETIMECTRL) &
         EFR32_RAC_RETIME_ENRETIME);
    bool sysclk_hfxo =
        (efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL) &
         EFR32_CMU_SYSCLKCTRL_CLKSEL_MASK) == EFR32_CMU_SYSCLKCTRL_CLKSEL_HFXO;

    qemu_set_irq(s->debug_wedge, retime && !sysclk_hfxo);
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
    int usart_index = efr32_usart_index(register_address);
    uint32_t ipversion;

    if (efr32_radio_ipversion(register_address, &ipversion)) {
        return;
    }
    if (efr32_frc_status_write_ignored(register_address)) {
        return;
    }
    if (register_address >= EFR32_RADIOAES_BASE &&
        register_address < EFR32_RADIOAES_BASE + EFR32_RADIOAES_SIZE &&
        efr32_radioaes_write(&s->radioaes, &s->dma_as,
                             register_address - EFR32_RADIOAES_BASE, next)) {
        return;
    }
    /* GPCRC decodes its own SET/CLR/TGL pages (input and command too). */
    if (efr32_peripheral_canonical_address(word) >= EFR32_GPCRC_BASE &&
        efr32_peripheral_canonical_address(word) <
            EFR32_GPCRC_BASE + EFR32_GPCRC_SIZE &&
        efr32_gpcrc_write(&s->gpcrc,
                          efr32_peripheral_canonical_address(word) -
                              EFR32_GPCRC_BASE,
                          (uint32_t)value << shift)) {
        return;
    }
    /*
     * CRYPTOACC has no SET/CLR/TGL aliases (RM SS4.2.4.3: none in its
     * register maps); its RNG block sits at +0x1000, so decode it before
     * the alias-folding register key.
     */
    if (efr32_peripheral_canonical_address(word) >= EFR32_TRNG_CTRL_BASE &&
        efr32_peripheral_canonical_address(word) <
            EFR32_TRNG_CTRL_BASE + EFR32_TRNG_CTRL_SIZE) {
        uint32_t offset = efr32_peripheral_canonical_address(word) -
                          EFR32_TRNG_CTRL_BASE;
        uint32_t current = 0;

        efr32_trng_read(&s->trng, offset, clock_get_hz(s->pclk), &current);
        efr32_trng_write(&s->trng, offset, clock_get_hz(s->pclk),
                         (current & ~mask) | (((uint32_t)value << shift) &
                                              mask));
        return;
    }

    if (word >= RADIO_BASE && word < RADIO_BASE + RADIO_SIZE)
    {
        uint32_t pc = current_cpu ? ARM_CPU(current_cpu)->env.regs[15] : 0;

        trace_efr32bg22_radio_write(pc, word, (uint32_t)value, size);
        trace_efr32bg22_endpoint_radio_write(s->endpoint_id, word, (uint32_t)value);
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

    if (register_address == EFR32_SMU_BASE + EFR32_SMU_PPUSATD1 &&
        ((uint32_t)value & EFR32_SMU_PPUSATD1_AHBRADIO) &&
        s->m33_radio_secure && s->m33_radio_nonsecure)
    {
        bool secure = ((address >> 12) & 3) != 2;

        /* SET attributes AHBRADIO secure; CLR attributes it non-secure. */
        memory_region_set_enabled(s->m33_radio_secure, secure);
        memory_region_set_enabled(s->m33_radio_nonsecure, !secure);
    }


    if (register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTCTRL &&
        ((address >> 12) & 3) != 2) {
        uint32_t written = ((uint32_t)value << shift) & mask;

        if ((written & EFR32_SYNTH_LOCNTCTRL_CLEAR) && !s->synth_running) {
            efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTTARGET,
                               efr32_synth_locnt_target(s, 0));
        }
        if (written & EFR32_SYNTH_LOCNTCTRL_RUN) {
            efr32_synth_locnt_run(s, next);
        }
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

    if (register_address == EFR32_EMU_BASE + EFR32_EMU_RSTCAUSE) {
        return; /* read-only */
    }
    if (register_address == EFR32_EMU_BASE + EFR32_EMU_RSTCTRL) {
        s->rstctrl = next & EFR32_EMU_RSTCTRL_WRITABLE;
        efr32_register_put(s, register_address, s->rstctrl);
        return;
    }
    if (register_address == EFR32_EMU_BASE + EFR32_EMU_CMD &&
        (next & EFR32_EMU_CMD_RSTCAUSECLR)) {
        s->rstcause = 0;
        efr32_register_put(s, EFR32_EMU_BASE + EFR32_EMU_RSTCAUSE, 0);
    }

    if ((register_address & ~0x3fffu) == EFR32_IADC_BASE &&
        efr32_iadc_write(s, register_address & 0xfffu, next)) {
        return;
    }
    if ((register_address & ~0x3fffu) == EFR32_WDOG0_BASE &&
        efr32_wdog_write(s, register_address & 0xfffu, next)) {
        return;
    }
    if (efr32_cmu_wdoglock_write(s, register_address, next)) {
        return;
    }

    if ((register_address & ~0x3fffu) == EFR32_RTCC_BASE &&
        efr32_rtcc_write(s, register_address & 0xfffu, next)) {
        return;
    }

    if ((register_address & ~0x3fffu) == EFR32_MSC_BASE &&
        efr32_msc_write(s, register_address & 0xfffu, old, next)) {
        return;
    }

    if ((register_address & ~0x3fffu) == EFR32_BURTC_BASE &&
        efr32_burtc_write(s, register_address & 0xfffu, old, next)) {
        efr32_osc_update(s);
        return;
    }

    if (efr32_timer_index(register_address) >= 0) {
        efr32_timer_write(s, register_address, next);
        return;
    }

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_PAENCTRL &&
        ((old ^ next) & EFR32_RAC_PAENCTRL_PARAMP))
    {
        efr32_register_put(s, register_address, next);
        s->pa_ramp_pending = true;
        s->pa_ramp_done_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                             ((next & EFR32_RAC_PAENCTRL_PARAMP)
                                  ? EFR32_PA_RAMP_UP_NS
                                  : EFR32_PA_RAMP_DOWN_NS);
        return;
    }
    if (register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_CMD)
    {
        /* Write-only command register: act on it, store nothing. */
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        efr32_synth_locnt_accumulate(s, now);
        if (next & EFR32_SYNTH_CMD_SYNTHSTOP) {
            s->synth_running = false;
        }
        if (next & EFR32_SYNTH_CMD_ENABLEIF) {
            s->synth_if_enabled = true;
        }
        if (next & EFR32_SYNTH_CMD_DISABLEIF) {
            s->synth_if_enabled = false;
        }
        if (next & EFR32_SYNTH_CMD_CAPCALSTART) {
            efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_LOCNTTARGET,
                               efr32_synth_locnt_target(s, 0));
        }
        if (s->synth_running && (next & (EFR32_SYNTH_CMD_ENABLEIF | EFR32_SYNTH_CMD_DISABLEIF))) {
            efr32_synth_retarget(s);
        }
        if ((next & EFR32_SYNTH_CMD_SYNTHSTART) &&
            (efr32_register_get(s, EFR32_RAC_BASE + EFR32_RAC_SYEN) &
             EFR32_RAC_SYEN_VCO_POWER) == EFR32_RAC_SYEN_VCO_POWER) {
            /*
             * The loop closes from the open-loop VCO at the calibrated code. [I] A start
             * with a code far from the carrier (no calibration) is taken as locked: not
             * measured.
             */
            double open_hz = efr32_synth_lo_hz(s, now);

            s->synth_running = true;
            efr32_synth_retarget(s);
            if (open_hz >= s->lo_to_hz - EFR32_LO_FAST_DOWN_HZ &&
                open_hz <= s->lo_to_hz + EFR32_LO_FAST_UP_HZ) {
                s->lo_from_hz = open_hz;
            } else {
                s->lo_from_hz = s->lo_to_hz;
            }
            s->synth_lock_ns = now + EFR32_SYNTH_LOCK_NS;
        }
        /* INLOCK and IFFREQEN are computed when STATUS is read. */
        efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_STATUS,
                           EFR32_SYNTH_STATUS_SYNTHREADY);
        if (s->synth_running && s->tx_waiting_for_synth) {
            /* Silicon drains the parked frame as soon as the VCO runs. */
            s->tx_waiting_for_synth = false;
            efr32_rac_schedule_tx_completion(s);
        }
        return;
    }
    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CMD)
    {
        next &= 0xc000fdff;
        if (s->seq_released) {
            efr32_rac_seq_command(s, next);
        }
        else if (next & (EFR32_RAC_CMD_TXDIS | EFR32_RAC_CMD_RXDIS))
        {
            efr32_rac_cancel_tx_completion(s);
            timer_del(s->rac_rx_timer);
            /*
             * Either disable command ends this model's active RAC operation,
             * including a synthetic TXEN transaction.  Do not let a TXEN
             * mailbox offer survive an explicit source abort and turn into a
             * later peer RXDONE.  There is no active source transaction to
             * withdraw when RXDIS merely stops receive, so this is harmless.
             */
            efr32_rf_medium_withdraw_active_tx(s->rf_medium,
                                                EFR32_RF_RADIO(s));
            efr32_rac_cancel_turnaround(s);
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
            /* Compatibility for images which have not installed and
             * released sequencer firmware yet. */
            efr32_rac_capture_tx_snapshot(s);
            efr32_rac_set_state(s, EFR32_RAC_STATE_TX);
            efr32_rac_schedule_tx_completion(s);
        }
        trace_efr32bg22_rac_command(s->endpoint_id,
                                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                    next, s->rac_state);
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

    /*
     * CMD is a write-only command port. RXABORT cancels the model's deferred
     * frame-controller receive handoff without changing RAC's separately
     * controlled RXSEARCH state; an already offered frame remains available
     * to a later receive operation. CONVINIT resets convolution state in
     * silicon, but this model deliberately has no convolutional codec state.
     */
    if (register_address == EFR32_FRC_BASE + EFR32_FRC_CMD)
    {
        if (next & EFR32_FRC_CMD_RXABORT) {
            timer_del(s->rac_rx_timer);
        }
        return;
    }

    /* FRC drives RXRAWDATA; the captured sequencer only reads it. */
    if (register_address == EFR32_FRC_BASE + EFR32_FRC_RXRAWDATA)
    {
        return;
    }

    /*
     * RFCRC.CMD is write-only. INITIALIZE loads the live CRC accumulator
     * (DATA) from the configured seed (INIT) -- confirmed on real hardware
     * (docs/radio-tables.md SS7.43); the production open driver was missing
     * this write entirely (fixed in modules/efr32-radio/hal/radio.c). This
     * model does not simulate the running per-byte CRC update itself (no
     * byte-stream hookup to BUFC exists here), only this one documented,
     * directly-observed register side effect.
     */
    if (register_address == EFR32_RFCRC_BASE + EFR32_RFCRC_CMD)
    {
        if (next & EFR32_RFCRC_CMD_INITIALIZE) {
            efr32_register_put(s, EFR32_RFCRC_BASE + EFR32_RFCRC_DATA,
                               efr32_register_get(s,
                                   EFR32_RFCRC_BASE + EFR32_RFCRC_INIT));
        }
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

    if (register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_VCOTUNING) {
        /* A LO count of the open-loop VCO takes the old code up to now. */
        efr32_synth_locnt_accumulate(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    efr32_register_put(s, register_address,
                       efr32_radio_mask_write(register_address, next));
    if (s->synth_running &&
        (register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_FREQ ||
         register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_CHCTRL ||
         register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_CHSP ||
         register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_CALOFFSET ||
         register_address == EFR32_SYNTH_BASE + EFR32_SYNTH_IFFREQ)) {
        efr32_synth_retarget(s);
    }

    /*
     * Matched hardware captures found that working RAIL enables BUFC and
     * PROTIMER before RAC, whereas the original open HAL enabled RAC first
     * and remained in SHUTDOWN.  Enabling the dependencies afterwards did
     * not repair that boot.  Preserve that ordering-sensitive failure here;
     * the successful proprietary sequencer hand-off itself is still outside
     * this model, so the valid order retains the existing abstract OFF state.
     * PROTIMER is not part of it: on silicon a build that never sets
     * PROTIMER.EN runs, transmits and receives (owo-open-fw
     * docs/radio-tables.md SS7.71; only RAC.STATUS bit 28 differs, meaning
     * unknown).  [I] BUFC.EN before RAC.EN stays required; never measured
     * on its own.
     */
    if (register_address == EFR32_RAC_BASE + EFR32_RADIO_EN &&
        !(old & 1u) && (next & 1u) &&
        !(efr32_register_get(s, EFR32_BUFC_BASE + EFR32_RADIO_EN) & 1u)) {
        efr32_rac_set_state(s, EFR32_RAC_STATE_SHUTDOWN);
    }

    /*
     * CMU_DPLLLock() clears DPLL.IF, enables DPLL0 and polls for LOCK; the
     * lock, IF.LOCK and STATUS.RDY come after the measured lock time
     * (efr32_osc_update, efr32_dpll_lock_ns).
     */
    efr32_osc_update(s);

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

    /*
     * Without sequencer firmware, silicon enters and remains in SHUTDOWN
     * when CTRL.FORCEDISABLE is asserted.  Do not silently complete the
     * proprietary sequencer hand-off that this model does not execute.
     */
    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CTRL &&
        s->seq_released) {
        /* SS7.72: SHUTDOWN at once on assertion; OFF may start RX/TX again
         * once it is released. */
        if ((next & EFR32_RAC_CTRL_FORCEDISABLE) &&
            !(old & EFR32_RAC_CTRL_FORCEDISABLE)) {
            efr32_rac_seq_force_disable(s);
        } else if (!(next & EFR32_RAC_CTRL_FORCEDISABLE) &&
                   (old & EFR32_RAC_CTRL_FORCEDISABLE)) {
            efr32_rac_seq_evaluate(s);
        }
    } else if (register_address == EFR32_RAC_BASE + EFR32_RAC_CTRL &&
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
        efr32_rac_cancel_turnaround(s);
        efr32_rac_set_state(s, EFR32_RAC_STATE_SHUTDOWN);
    }

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_CTRL && s->seq_cpu) {
        if (next & EFR32_RAC_CTRL_SEQRESET) {
            /*
             * The M0+ reset includes its NVIC (Armv6-M B1.5.5: reset makes
             * every exception inactive); without it an exception the core
             * was handling, e.g. the HardFault of a Lockup, stayed active.
             */
            device_cold_reset(DEVICE(ARM_CPU(s->seq_cpu)->env.nvic));
            cpu_reset(s->seq_cpu);
            s->seq_cpu->halted = 1;
            s->seq_released = false;
        }
        if ((next & EFR32_RAC_CTRL_CPUWAITDIS) && !s->seq_released) {
            /* The held sequencer samples its freshly loaded vector table as
             * the wait gate is released, not when SEQRESET was asserted
             * before the M33 copied the image.  CPUWAITDIS remains asserted
             * in normal RAC.CTRL presets; rewriting it while the sequencer is
             * already running does not reset the auxiliary core. */
            cpu_reset(s->seq_cpu);
            s->seq_released = true;
            icount_cpu_clocks_wake(s->seq_cpu);
            s->seq_cpu->halted = 0;
            s->seq_cpu->stopped = false;
            s->seq_cpu->start_powered_off = false;
            qemu_cpu_kick(s->seq_cpu);
        }
        /* SEQRESET is a write-only strobe. */
        efr32_register_put(s, EFR32_RAC_BASE + EFR32_RAC_CTRL,
                           next & ~EFR32_RAC_CTRL_SEQRESET);
    }

    /*
     * RAIL_StartRx writes SWRXEN through the RXENSRCEN SET alias.  The
     * hardware sequencer has many intermediate states; RXSEARCH is the
     * first stable receive state that its public register model exposes.
     *
     * With sequencer firmware released, the RX source is a level the RAC
     * state machine samples (efr32_rac_seq_evaluate): OFF goes to RXWARM,
     * and RXSEARCH to RXPD when it is withdrawn (SS7.72).
     */
    if (register_address == EFR32_RAC_BASE + EFR32_RAC_RXENSRCEN &&
        s->seq_released) {
        efr32_rac_seq_evaluate(s);
    } else if (register_address == EFR32_RAC_BASE + EFR32_RAC_RXENSRCEN &&
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
        if (s->seq_released) {
            efr32_rac_seq_acknowledged(s);
        }
        efr32_rac_update_irqs(s);
    }

    if (((register_address & ~0x3fffu) == EFR32_RFMAILBOX_B_BASE ||
         (register_address & ~0x3fffu) == EFR32_RFMAILBOX_C_BASE) &&
        ((register_address & 0xfffu) == EFR32_RFMAILBOX_FLAGS ||
         (register_address & 0xfffu) == EFR32_RFMAILBOX_IEN)) {
        efr32_rdmailbox_update_irqs(s);
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
    if (register_address == EFR32_PROTIMER_BASE + EFR32_PROTIMER_CMD) {
        efr32_protimer_command(s, next);
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

    if (register_address == EFR32_RAC_BASE + EFR32_RAC_DIGCLKRETIMECTRL ||
        register_address == EFR32_RAC_BASE + EFR32_RAC_XORETIMECTRL ||
        register_address == EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL)
    {
        efr32_rac_retime_update(s);
    }
}
/*
 * Bus clock gating, measured on silicon by clearing one CMU enable at a time
 * from the debugger and reading every 16 KiB radio window (owo-open-fw
 * docs/radio-tables.md SS7.72):
 *  - each CLKEN1 bit 0-10 gates exactly one radio block's whole window;
 *  - CLKEN1.BUFC (bit 11) gates nothing visible: BUFC stays readable;
 *  - RADIOCLKCTRL.EN gates every radio window and BUFC;
 *  - CLKEN0.SYSCFG gates SYSCFG.
 * A gated read is a bus error; a gated write is dropped (the value written
 * before gating reads back once the clock returns).  The windows at
 * 0xa8008000 and from 0xa8030000 up to at least 0xa80fffff answer every read
 * with a bus error whatever the enables.
 */
#define EFR32_RADIO_HOLE_A_BASE 0xa8008000u
#define EFR32_RADIO_HOLE_B_BASE 0xa8030000u
#define EFR32_RADIO_HOLE_B_END 0xa8100000u

static bool efr32_bus_error(Efr32Periph *s, uint32_t address)
{
    uint32_t block = efr32_peripheral_canonical_address(address) & ~0x3fffu;
    uint32_t clken1 = efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_CLKEN1);
    bool radio_clock =
        efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_RADIOCLKCTRL) &
        EFR32_CMU_RADIOCLKCTRL_EN;
    uint32_t gate;

    if (efr32_timer_clock_gated(s, block)) {
        return true;
    }
    if (block == EFR32_SYSCFG_BASE) {
        return !(efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_CLKEN0) &
                 EFR32_CMU_CLKEN0_SYSCFG);
    }
    if (block == EFR32_RADIO_HOLE_A_BASE ||
        (block >= EFR32_RADIO_HOLE_B_BASE && block < EFR32_RADIO_HOLE_B_END)) {
        return true;
    }
    if (block == EFR32_BUFC_BASE) {
        return !radio_clock;
    }
    /*
     * CRYPTOACC (PK control, DMA, RNG control, RNG FIFO, PK RAM;
     * efr32bg22c224f512im40.h) behind CMU_CLKEN1.CRYPTOACC (bit 13,
     * efr32bg22_cmu.h; Zephyr's TRNG driver enables it first).  RM
     * SS4.2.1.2: an access to a peripheral whose clock is disabled is a bus
     * fault.  [I] Not measured on silicon for this block.
     */
    if (block >= EFR32_CRYPTOACC_BASE && block < EFR32_CRYPTOACC_END) {
        return !(clken1 & EFR32_CMU_CLKEN1_CRYPTOACC);
    }
    /*
     * RADIOAES behind CMU_CLKEN0.RADIOAES (bit 2) and the radio bridge clock
     * CMU_RADIOCLKCTRL.EN (RM 8.5, 8.5.25; owo-open-fw docs/crypto.md:
     * Silicon Labs' sli_radioaes_acquire sets both, and so does the open
     * firmware).  [I] Unmeasured: a gated access faults as RM SS4.2.1.2
     * says (reads; writes are dropped like the other gated blocks here).
     */
    if (block == EFR32_RADIOAES_BASE) {
        return !(efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_CLKEN0) &
                 EFR32_CMU_CLKEN0_RADIOAES) || !radio_clock;
    }
    /* GPCRC behind CMU_CLKEN0.GPCRC (bit 3, efr32bg22_cmu.h), as above. */
    if (block == EFR32_GPCRC_BASE) {
        return !(efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_CLKEN0) &
                 EFR32_CMU_CLKEN0_GPCRC);
    }
    gate = efr32_radio_bus_clock(block);
    return gate && (!radio_clock || !(clken1 & gate));
}

/*
 * ---------------------------------------------------------------------------
 * Core clocks and bus access costs (owo-open-fw docs/radio-tables.md SS7.73)
 * ---------------------------------------------------------------------------
 * Under -icount with per-CPU clocks (icount_cpu_clocks_enable), each core
 * runs at its own clock and is charged for slow bus accesses.  Silicon, stage
 * 3 (SYSCLKCTRL = 0x00010402: HFRCODPLL 76.8 MHz, HCLK /1, PCLK /2,
 * RHCLKPRESC DIV2), each core running a loop while the other was halted,
 * timed by its own cycle counter and by TIMER1 (HFRCODPLL, calibrated by
 * packet airtime, SS7.71):
 *  - M33 DWT.CYCCNT and TIMER1 advance 1:1 (n = 10): HCLK = 76.8 MHz.
 *  - M0+ SysTick (CLKSOURCE = core) counts exactly half of TIMER1
 *    (21890 cycles in 43780 ticks, n = 10): 38.4 MHz.  With SYSCLK on HFXO
 *    it ran at 19.2 MHz and on FSRCO at 9.89 MHz, whatever HCLKPRESC (SS7.73,
 *    cross-core probes): the M0+ runs at SYSCLK / (RHCLKPRESC + 1).
 *  - RADIOCLKCTRL.EN = 0 stops the M0+ with its state kept.
 * At reset SYSCLK is FSRCO, 19.807 MHz (EFR32_FSRCO_MEASURED_HZ).
 */
#define EFR32_CMU_SYSCLKCTRL_CLKSEL_HFRCODPLL 2u
#define EFR32_CMU_SYSCLKCTRL_HCLKPRESC_SHIFT 12
#define EFR32_CMU_SYSCLKCTRL_HCLKPRESC_MASK 0xfu
#define EFR32_CMU_SYSCLKCTRL_RHCLKPRESC_SHIFT 16
#define EFR32_CMU_SYSCLKCTRL_RHCLKPRESC_MASK 0x1u
/* efr32bg22_cmu.h: _CMU_SYSCLKCTRL_PCLKPRESC_SHIFT/_MASK. */
#define EFR32_CMU_SYSCLKCTRL_PCLKPRESC_SHIFT 10
#define EFR32_CMU_SYSCLKCTRL_PCLKPRESC_MASK 0x1u
/*
 * Core cycles per instruction, from real code: a firmware function called
 * from the debugger at full speed (cycles) and single-stepped
 * (instructions), n = 6 calls each, no spread.
 *  - M33, stage 3 from flash, warm cache: snprintk 7973 cycles / 4342
 *    insns = 1.84 (compiled C); memcpy 1.60, memset 1.50, strlen 1.45
 *    (library loops).  The compiled-C average is used.
 *  - M0+, the open sequencer image: the Armv6-M timings (one cycle, loads
 *    and stores 2, taken branches 2, BL 3, ...; the translator counts them,
 *    icount_set_cpu_cycle_counting) plus 3 extra cycles per radio access
 *    reproduce each function exactly: rx_frontend_stop 61 cycles / 22
 *    insns, synth_config_tx 112 / 39, efr32_seq_delay 7 cycles per pass.
 *    A flat average does not: pga_dc_interp (ALU) runs at 1.19 cycles per
 *    instruction, synth_power_down at 2.5.  The average below only sizes
 *    the M0+'s slices.
 */
#define EFR32_M33_CYCLES_PER_INSN_MILLI 1840u
#define EFR32_SEQ_CYCLES_PER_INSN_MILLI 2000u
/*
 * Exception entry, SysTick reload to the first handler load (SS7.73,
 * cross-core probes): M0+ 16 cycles; M33 17 (18 from WFI) with its vector
 * table, handler and stack in SRAM.  [I] The M33 value is used for its
 * flash-resident vectors too.  The M0+ probe's handler reads CVR in its
 * second instruction, after a literal load (2 cycles), so the entry itself
 * is 14 (the same probe on the model reads 16); efr32-interconnect.c
 * scales it under M33 SEQRAM traffic.
 */
#define EFR32_SEQ_EXCEPTION_ENTRY_MILLICYCLES 14000u
#define EFR32_M33_EXCEPTION_ENTRY_MILLICYCLES 17000u
/*
 * Bus access costs and contention: hw/arm/efr32-interconnect.c (the RM's
 * bus matrix and bridges, the radio subsystem paths, calibrated to the
 * single-core and cross-core measurements of owo-open-fw SS7.74, SS7.75).
 */
#define EFR32_TIMER_BLOCKS_END (EFR32_TIMER4_BASE + 0x4000u)
#define EFR32_BUFC_WINDOW_END (RADIO_BASE + RADIO_SIZE)
#define EFR32_LFAPB_BASE 0x48000000u
#define EFR32_LFAPB_END 0x4c000000u
/* Main SRAM (RM SS4.2.3: up to 32 KB at 0x20000000). */
#define EFR32_IC_SRAM_BASE 0x20000000u
#define EFR32_IC_SRAM_END 0x20008000u
#define EFR32_SEQRAM_NS_BASE 0xb0000000u
#define EFR32_SEQRAM_S_BASE 0xa0000000u
#define EFR32_SEQRAM_WINDOW_SIZE 0x8000u
/*
 * [I] Longest round of the per-CPU clocks.  Device timers are exact (a
 * round never passes a deadline); an event one core raises directly for
 * the other is seen up to this late.
 */
#define EFR32_CORE_CLOCK_QUANTUM_NS 500

static uint64_t efr32_sysclk_hz(Efr32Periph *s)
{
    switch (efr32_register_get(s, EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL) &
            EFR32_CMU_SYSCLKCTRL_CLKSEL_MASK) {
    case EFR32_CMU_SYSCLKCTRL_CLKSEL_HFXO:
        return EFR32_HFXO_HZ;
    case EFR32_CMU_SYSCLKCTRL_CLKSEL_HFRCODPLL:
        return efr32_hfrcodpll_hz(s);
    default:
        /* FSRCO; [P] CLKIN0 (no external clock in this model) likewise. */
        return EFR32_FSRCO_MEASURED_HZ;
    }
}

static void efr32_cpu_clock_set(CPUState *cpu, Clock *clk, uint64_t hz,
                                uint32_t cycles_per_insn_milli)
{
    if (cpu) {
        icount_set_cpu_clock(cpu, hz, cycles_per_insn_milli);
    }
    if (clk && hz && clock_get_hz(clk) != hz) {
        clock_set_hz(clk, hz);
        clock_propagate(clk);
    }
}

/*
 * Follow SYSCLKCTRL and the oscillators: HCLK for the M33, SYSCLK /
 * (RHCLKPRESC + 1) for the M0+, stopped while RADIOCLKCTRL.EN is clear.
 */
static void efr32_cpu_clocks_update(Efr32Periph *s)
{
    uint32_t ctrl = efr32_register_get(s, EFR32_CMU_BASE +
                                          EFR32_CMU_SYSCLKCTRL);
    uint64_t sysclk = efr32_sysclk_hz(s);
    uint32_t hclk_div = ((ctrl >> EFR32_CMU_SYSCLKCTRL_HCLKPRESC_SHIFT) &
                         EFR32_CMU_SYSCLKCTRL_HCLKPRESC_MASK) + 1;
    uint32_t radio_div = ((ctrl >> EFR32_CMU_SYSCLKCTRL_RHCLKPRESC_SHIFT) &
                          EFR32_CMU_SYSCLKCTRL_RHCLKPRESC_MASK) + 1;
    bool radio_clock = efr32_register_get(s, EFR32_CMU_BASE +
                                             EFR32_CMU_RADIOCLKCTRL) &
                       EFR32_CMU_RADIOCLKCTRL_EN;

    efr32_cpu_clock_set(s->m33_cpu, s->m33_cpuclk, sysclk / hclk_div,
                        EFR32_M33_CYCLES_PER_INSN_MILLI);
    efr32_cpu_clock_set(s->seq_cpu, s->seq_cpuclk,
                        radio_clock ? sysclk / radio_div : 0,
                        EFR32_SEQ_CYCLES_PER_INSN_MILLI);
    efr32_cpu_clock_set(NULL, s->pclk,
                        sysclk / hclk_div /
                        (((ctrl >> EFR32_CMU_SYSCLKCTRL_PCLKPRESC_SHIFT) &
                          EFR32_CMU_SYSCLKCTRL_PCLKPRESC_MASK) + 1), 0);
}

/* The M0+ is executing: released, clocked and not sleeping. */
static bool efr32_seq_executing(Efr32Periph *s)
{
    return s->seq_cpu && s->seq_released && !s->seq_cpu->halted &&
           !s->seq_cpu->icount_clock_stopped;
}

/* Which interconnect target an address is (efr32-interconnect.h). */
static Efr32IcTarget efr32_ic_target(uint32_t addr)
{
    uint32_t block;

    if ((addr >= EFR32_SEQRAM_NS_BASE &&
         addr < EFR32_SEQRAM_NS_BASE + EFR32_SEQRAM_WINDOW_SIZE) ||
        (addr >= EFR32_SEQRAM_S_BASE &&
         addr < EFR32_SEQRAM_S_BASE + EFR32_SEQRAM_WINDOW_SIZE)) {
        return EFR32_IC_SEQRAM;
    }
    if (addr >= EFR32_IC_SRAM_BASE && addr < EFR32_IC_SRAM_END) {
        return EFR32_IC_SRAM;
    }
    if (addr >= EFR32_BUFC_BASE && addr < EFR32_BUFC_WINDOW_END) {
        return EFR32_IC_BUFC;
    }
    if (addr >= RADIO_BASE && addr < RADIO_BASE + RADIO_SIZE) {
        return EFR32_IC_RADIO;
    }
    block = efr32_peripheral_canonical_address(addr) & ~0x3fffu;
    if (block >= EFR32_TIMER0_BASE && block < EFR32_TIMER_BLOCKS_END) {
        return EFR32_IC_TIMER;
    }
    /* [I] RTCC (0x48...) and the 0x4A... group: the LF APB bridge. */
    if (block >= EFR32_LFAPB_BASE && block < EFR32_LFAPB_END) {
        return EFR32_IC_LFPERIPH;
    }
    return EFR32_IC_HFPERIPH;
}

static uint64_t efr32_bus_cost(void *opaque, CPUState *master, uint64_t addr,
                               unsigned size, bool is_write, unsigned *target)
{
    Efr32Periph *s = opaque;
    unsigned manager;
    bool sram_code = false;

    if (master == s->seq_cpu) {
        manager = EFR32_IC_SEQ;
    } else if (master == s->m33_cpu) {
        uint32_t pc = ARM_CPU(master)->env.regs[15];

        manager = EFR32_IC_M33;
        sram_code = pc >= EFR32_IC_SRAM_BASE && pc < EFR32_IC_SRAM_END;
    } else {
        *target = ICOUNT_BUS_TARGETS;
        return 0;
    }
    s->ic.hclk_hz = s->m33_cpuclk ? clock_get_hz(s->m33_cpuclk) : 0;
    s->ic.rhclk_hz = s->seq_cpuclk ? clock_get_hz(s->seq_cpuclk) : 0;
    s->ic.seq_executing = efr32_seq_executing(s);
    return efr32_ic_access(&s->ic, manager, efr32_ic_target(addr), sram_code,
                           target);
}

static void efr32_bus_round_end(void *opaque, int64_t start_ns,
                                int64_t end_ns)
{
    Efr32Periph *s = opaque;

    s->ic.seq_executing = efr32_seq_executing(s);
    s->ic.rhclk_hz = s->seq_cpuclk ? clock_get_hz(s->seq_cpuclk) : 0;
    efr32_ic_round_end(&s->ic, start_ns, end_ns);
    if (s->seq_cpu) {
        icount_set_cpu_exception_cycles(
            s->seq_cpu, efr32_ic_seq_exception_millicycles(&s->ic));
    }
}

/* Every CPU access to this device goes through the bus model. */
static void efr32_bus_access(Efr32Periph *s, uint32_t address, unsigned size,
                             bool is_write)
{
    icount_bus_access(&s->bus_model, current_cpu, address, size, is_write);
}

/* Every guest peripheral access, for coverage runs (trace only). */
static void efr32_trace_mmio(Efr32Periph *s, bool is_write, uint32_t address,
                             unsigned size, uint64_t value, MemTxResult r)
{
    if (trace_event_get_state_backends(TRACE_EFR32_MMIO_READ) ||
        trace_event_get_state_backends(TRACE_EFR32_MMIO_WRITE)) {
        int cpu = current_cpu ? current_cpu->cpu_index : -1;
        uint32_t pc = current_cpu ? ARM_CPU(current_cpu)->env.regs[15] : 0;

        if (is_write) {
            trace_efr32_mmio_write(s->endpoint_id, cpu, pc, address,
                                       size, value, r);
        } else {
            trace_efr32_mmio_read(s->endpoint_id, cpu, pc, address, size,
                                      value, r);
        }
    }
}

static MemTxResult efr32_peripheral_mmio_read(void *opaque, hwaddr offset,
                                              uint64_t *data, unsigned size,
                                              MemTxAttrs attrs)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    efr32_bus_access(s, PERIPH_BASE + offset, size, false);

    if (efr32_bus_error(s, PERIPH_BASE + offset)) {
        *data = 0;
        efr32_trace_mmio(s, false, PERIPH_BASE + offset, size, 0,
                         MEMTX_ERROR);
        return MEMTX_ERROR;
    }
    s->debugger_access = attrs.requester_id == EFR32_REQUESTER_DEBUG_AP;
    *data = efr32_peripheral_read(s, PERIPH_BASE + offset, size);
    s->debugger_access = false;
    efr32_trace_mmio(s, false, PERIPH_BASE + offset, size, *data, MEMTX_OK);
    return MEMTX_OK;
}
static MemTxResult efr32_peripheral_mmio_write(void *opaque, hwaddr offset,
                                               uint64_t value, unsigned size,
                                               MemTxAttrs attrs)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);
    efr32_bus_access(s, PERIPH_BASE + offset, size, true);
    efr32_trace_mmio(s, true, PERIPH_BASE + offset, size, value, MEMTX_OK);
    /*
     * TIMER and BURTC answer a gated write with a bus error too (each of
     * 20 debugger writes left the sticky error), and a TIMER rejects writes
     * that its EN state forbids (efr32_timer_write_rejected).
     */
    if (efr32_timer_clock_gated(s, efr32_peripheral_canonical_address(
                                       PERIPH_BASE + offset) & ~0x3fffu) ||
        efr32_timer_write_rejected(s, efr32_peripheral_canonical_address(
                                          PERIPH_BASE + offset)) ||
        efr32_wdog_write_rejected(s, efr32_peripheral_canonical_address(
                                         PERIPH_BASE + offset),
                                  size == 4 ? value : 0)) {
        return MEMTX_ERROR;
    }
    if (efr32_bus_error(s, PERIPH_BASE + offset)) {
        return MEMTX_OK;
    }
    uint32_t key = efr32_register_key(PERIPH_BASE + offset);
    /* A change to TIMER0's clock source must not rescale elapsed counts. */
    bool timer0_clock = key == EFR32_CMU_BASE + EFR32_CMU_EM01GRPACLKCTRL ||
                        key == EFR32_CMU_BASE + EFR32_CMU_DPLLREFCLKCTRL ||
                        (key & ~0x3fffu) == EFR32_DPLL0_BASE;

    if (timer0_clock) {
        efr32_timer0_rebase(s);
    }
    efr32_peripheral_write(s, PERIPH_BASE + offset, value, size);
    if (timer0_clock) {
        efr32_timer0_schedule(s);
    }
    if ((key & ~0x3fffu) == EFR32_CMU_BASE ||
        (key & ~0x3fffu) == EFR32_DPLL0_BASE) {
        efr32_cpu_clocks_update(s);
    }
    return MEMTX_OK;
}
static MemTxResult efr32_radio_mmio_read(void *opaque, hwaddr offset,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    efr32_bus_access(s, RADIO_BASE + offset, size, false);

    if (efr32_bus_error(s, RADIO_BASE + offset)) {
        *data = 0;
        efr32_trace_mmio(s, false, RADIO_BASE + offset, size, 0, MEMTX_ERROR);
        return MEMTX_ERROR;
    }
    *data = efr32_peripheral_read(s, RADIO_BASE + offset, size);
    efr32_trace_mmio(s, false, RADIO_BASE + offset, size, *data, MEMTX_OK);
    return MEMTX_OK;
}
static MemTxResult efr32_radio_mmio_write(void *opaque, hwaddr offset,
                                          uint64_t value, unsigned size,
                                          MemTxAttrs attrs)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    efr32_bus_access(s, RADIO_BASE + offset, size, true);
    efr32_trace_mmio(s, true, RADIO_BASE + offset, size, value, MEMTX_OK);

    if (!efr32_bus_error(s, RADIO_BASE + offset)) {
        efr32_peripheral_write(s, RADIO_BASE + offset, value, size);
    }
    return MEMTX_OK;
}
static const MemoryRegionOps po = {
    .read_with_attrs = efr32_peripheral_mmio_read,
    .write_with_attrs = efr32_peripheral_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4}};
static const MemoryRegionOps ro = {
    .read_with_attrs = efr32_radio_mmio_read,
    .write_with_attrs = efr32_radio_mmio_write,
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

/* ------------------------------------------- SoC link (efr32-soc-link.h) */
#define EFR32_CMU_EUART0CLKCTRL 0x220u
#define EFR32_CMU_EUART0CLKCTRL_RESET 1u
#define EFR32_CMU_EUART0CLKCTRL_EM01GRPACLK 1u
#define EFR32_CMU_EUART0CLKCTRL_EM23GRPACLK 2u

static bool efr32_link_bus_clock_enabled(void *opaque, uint32_t offset,
                                         uint32_t bit)
{
    return efr32_cmu_get(EFR32_PERIPH(opaque), offset) & bit;
}

/* An LF clock group (EM23GRPACLK, ...) from its CMU CLKSEL register. */
static uint64_t efr32_lf_group_hz(Efr32Periph *s, uint32_t clkctrl)
{
    switch (efr32_cmu_get(s, clkctrl) & EFR32_CMU_LFCLKSEL_MASK) {
    case EFR32_CMU_LFCLKSEL_LFRCO:
    case EFR32_CMU_LFCLKSEL_LFXO:
        return clock_get_hz(s->burtc_clk);
    case EFR32_CMU_LFCLKSEL_ULFRCO:
        return EFR32_ULFRCO_HZ;
    default:
        return 0;
    }
}

static uint64_t efr32_soc_tree_hz(void *opaque, Efr32ClockTree tree)
{
    Efr32Periph *s = EFR32_PERIPH(opaque);

    switch (tree) {
    case EFR32_TREE_HCLK:
        return clock_get_hz(s->m33_cpuclk);
    case EFR32_TREE_PCLK:
        return clock_get_hz(s->pclk);
    case EFR32_TREE_EM01GRPACLK:
        return efr32_em01grpa_hz(s);
    case EFR32_TREE_EM23GRPACLK:
        return efr32_lf_group_hz(s, EFR32_CMU_EM23GRPACLKCTRL);
    case EFR32_TREE_EUSART0CLK:
        switch (efr32_cmu_get(s, EFR32_CMU_EUART0CLKCTRL) & 0x3u) {
        case EFR32_CMU_EUART0CLKCTRL_EM01GRPACLK:
            return efr32_em01grpa_hz(s);
        case EFR32_CMU_EUART0CLKCTRL_EM23GRPACLK:
            return efr32_lf_group_hz(s, EFR32_CMU_EM23GRPACLKCTRL);
        default:
            return 0;
        }
    }
    return 0;
}

/* A PRS consumer line this model serves (efr32-prs.h numbering). */
static void efr32_prs_consume(Efr32Periph *s, int n, bool sync, bool level)
{
    if (n >= EFR32_PRS_CON_TIMER_CC(0, 0) &&
        n < EFR32_PRS_CON_TIMER_CC(EFR32_TIMER_COUNT, 0)) {
        unsigned k = n - EFR32_PRS_CON_TIMER0_CC0;
        unsigned channel = k % EFR32_PRS_TIMER_CONSUMERS;

        if (channel < EFR32_TIMER0_CHANNELS) {
            efr32_timer_ic_input(s, k / EFR32_PRS_TIMER_CONSUMERS, channel,
                                 sync, level);
        }
        return;
    }
    if (!sync && n >= EFR32_PRS_CON_RTCC_CC(0) &&
        n < EFR32_PRS_CON_RTCC_CC(EFR32_RTCC_CC_COUNT)) {
        efr32_rtcc_prs_input(s, n - EFR32_PRS_CON_RTCC_CC(0), level);
        return;
    }
    efr32_iadc_prs_trigger(s, n, sync, level);
}

static void efr32_prs_consumer_in(void *opaque, int n, int level)
{
    efr32_prs_consume(EFR32_PERIPH(opaque), n, false, level);
}

static void efr32_prs_consumer_sync_in(void *opaque, int n, int level)
{
    efr32_prs_consume(EFR32_PERIPH(opaque), n, true, level);
}

static void efr32_periph_init(Object *o)
{
    Efr32Periph *s = EFR32_PERIPH(o);

    s->prs_radio_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_prs_expired, s);
    s->prs_syncsent_ns = s->prs_framesent_ns = INT64_MAX;
    s->link.opaque = s;
    s->link.bus_clock = efr32_link_bus_clock_enabled;
    s->link.tree_hz = efr32_soc_tree_hz;
    s->link.bus_model = &s->bus_model;
    s->link.dma_as = &s->dma_as;
    qdev_init_gpio_out_named(DEVICE(o), s->prs_async, "prs-async",
                             EFR32_PRS_ASYNC_INPUTS);
    qdev_init_gpio_out_named(DEVICE(o), s->prs_sync, "prs-sync",
                             EFR32_PRS_SYNC_INPUTS);
    qdev_init_gpio_in_named(DEVICE(o), efr32_prs_consumer_in, "prs-consumer",
                            EFR32_PRS_CONSUMERS);
    qdev_init_gpio_out_named(DEVICE(o), &s->timer_dma[0][0], "timer-dma",
                             EFR32_TIMER_COUNT * 4);
    qdev_init_gpio_in_named(DEVICE(o), efr32_prs_consumer_sync_in,
                            "prs-consumer-sync", EFR32_PRS_CONSUMERS);
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        for (unsigned n = 0; n < EFR32_TIMER0_CHANNELS; n++) {
            Efr32TimerIc *ic = &s->timer_ic[i][n];

            ic->owner = s;
            ic->timer = i;
            ic->channel = n;
            ic->sampler = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       efr32_timer_ic_sampled, ic);
        }
    }
    s->registers = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->rstcause = EFR32_EMU_RSTCAUSE_POR;
    s->rstctrl = EFR32_EMU_RSTCTRL_RESET;
    efr32_trng_reset(&s->trng);
    efr32_gpcrc_reset(&s->gpcrc);
    efr32_radioaes_reset(&s->radioaes);
    s->rac_state = EFR32_RAC_STATE_OFF;
    s->seq_released = false;
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
    s->burtc_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_burtc_timer_expired, s);
    s->rtcc_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rtcc_timer_expired, s);
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        s->timers[i].event = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          efr32_timer_callbacks[i], s);
        s->timer_pwm[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       efr32_timer_pwm_callbacks[i], s);
    }
    s->wdog_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_wdog_expired, s);
    s->iadc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_iadc_complete, s);
    s->iadc_local_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_iadc_local_timer_expired, s);
    s->wdog_sync_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_wdog_sync_done, s);
    /* DHCSR.S_HALT of the M33, from its debug unit (WDOG0 DEBUGRUN). */
    qdev_init_gpio_in_named(DEVICE(o), efr32_m33_debug_halted,
                            "m33-debug-halted", 1);
    /* Fault injection (qom-set at run time): conversions never complete. */
    object_property_add_uint32_ptr(o, "x-iadc-stall", &s->iadc_stall,
                                 OBJ_PROP_FLAG_READWRITE);
    qdev_init_gpio_out_named(DEVICE(o), s->timer_cc_out, "timer-cc",
                             ARRAY_SIZE(s->timer_cc_out));
    qdev_init_gpio_out_named(DEVICE(o), &s->debug_wedge, "debug-wedge", 1);
    s->dpll_lock_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_dpll_lock_expired, s);
    s->rac_tx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_tx_complete, s);
    s->rac_rx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_rx_handoff, s);
    s->rac_frame_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_rx_frame_end, s);
    s->rac_ack_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, efr32_rac_seq_ack_timer, s);
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
    efr32_trng_reset(&s->trng);
    efr32_gpcrc_reset(&s->gpcrc);
    efr32_radioaes_reset(&s->radioaes);
    s->rac_state = EFR32_RAC_STATE_OFF;
    efr32_rac_prs_state(s, EFR32_RAC_STATE_OFF);
    s->seq_released = false;
    s->hfxo_fsm_locked = true;
    if (s->m33_radio_secure && s->m33_radio_nonsecure) {
        memory_region_set_enabled(s->m33_radio_secure, true);
        memory_region_set_enabled(s->m33_radio_nonsecure, false);
    }
    s->usart_tx_enabled[0] = false;
    s->usart_tx_enabled[1] = false;
    s->usart_tx_complete[0] = false;
    s->usart_tx_complete[1] = false;
    qemu_set_irq(s->irq[EFR32_BURTC_IRQ], 0);
    for (unsigned state = 0; state < EFR32_RAC_SEQ_STATE_EVENTS; state++) {
        if (s->seq_irq_state[state]) {
            qemu_set_irq(s->seq_irq_state[state], 0);
        }
    }
    if (s->seq_irq_mailbox) {
        qemu_set_irq(s->seq_irq_mailbox, 0);
    }
    efr32_timers_reset(s);
    timer_del(s->dpll_lock_timer);
    s->hfxo_request_ns = -1;
    s->lfxo_request_ns = -1;
    s->lfrco_request_ns = -1;
    s->lfrco_off_ns = -1;
    s->dpll_enable_ns = -1;
    s->dpll_off_ns = -1;
    ptimer_transaction_begin(s->protimer_counter);
    ptimer_stop(s->protimer_counter);
    ptimer_set_period_from_clock(s->protimer_counter, s->protimer_clk, 1);
    ptimer_set_limit(s->protimer_counter, UINT32_MAX, 1);
    /* Stopped at 0 until started (efr32_protimer_command). */
    ptimer_transaction_commit(s->protimer_counter);
    efr32_rac_cancel_tx_completion(s);
    timer_del(s->rac_rx_timer);
    efr32_rac_cancel_turnaround(s);
    s->rac_stimer_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->rx_offer_pending = false;
    s->synth_running = false;
    s->synth_if_enabled = false;
    s->locnt_busy_until_ns = 0;
    s->locnt_pending = false;
    s->lo_from_hz = 0.0;
    s->lo_to_hz = 0.0;
    s->lo_change_ns = 0;
    efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_STATUS, 0);
    s->pa_ramp_pending = false;
    efr32_rf_medium_reset_radio(s->rf_medium, EFR32_RF_RADIO(s));
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL,
                       EFR32_CMU_SYSCLKCTRL_CLKSEL_FSRCO);
    efr32_cpu_clocks_update(s);
    efr32_register_put(s, EFR32_HFXO_BASE + EFR32_HFXO_CTRL,
                       EFR32_HFXO_CTRL_DISONDEMAND);
    efr32_register_put(s, EFR32_EMU_BASE + EFR32_EMU_STATUS,
                       EFR32_EMU_STATUS_RESET);
    efr32_register_put(s, EFR32_EMU_BASE + EFR32_EMU_RSTCAUSE, s->rstcause);
    efr32_register_put(s, EFR32_EMU_BASE + EFR32_EMU_RSTCTRL, s->rstctrl);
    /*
     * SYNTH.STATUS.SYNTHREADY reads 1 with SYNTH.CMD never written: 0x4
     * throughout stage 3's stalled TX (SS7.42, SS7.60) and in an RXSEARCH
     * reached from OFF with nothing powered (SS7.80), on silicon.
     */
    efr32_register_put(s, EFR32_SYNTH_BASE + EFR32_SYNTH_STATUS,
                       EFR32_SYNTH_STATUS_SYNTHREADY);
    efr32_register_put(s, EFR32_MODEM_BASE + EFR32_MODEM_RAMPCTRL,
                       EFR32_MODEM_RAMPCTRL_RESET);
    efr32_register_put(s, EFR32_MODEM_BASE + EFR32_MODEM_RAMPLEV,
                       EFR32_MODEM_RAMPLEV_RESET);
    efr32_msc_reset(s);
    efr32_rtcc_reset(s);
    memset(s->iadc_fifo, 0, sizeof(s->iadc_fifo));
    timer_del(s->iadc_timer);
    s->iadc_nconv = 0;
    s->iadc_busy_until_ns = 0;
    s->iadc_warm = s->iadc_standby = s->iadc_have_input = false;
    s->iadc_qen[0] = s->iadc_qen[1] = false;
    s->iadc_timer_on = false;
    timer_del(s->iadc_local_timer);
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_EUART0CLKCTRL,
                       EFR32_CMU_EUART0CLKCTRL_RESET);
    efr32_wdog_reset(s);
    efr32_register_put(s, EFR32_SYSCFG_BASE + EFR32_SYSCFG_CHIPREV,
                       s->chiprev);
    /*
     * Zephyr's alarm-mode counter driver reports COMP as its top value.
     * The LL ticker requires a free-running 32-bit top, so this machine
     * integration default is UINT32_MAX until guest software programs COMP.
     */
    efr32_register_put(s, EFR32_BURTC_BASE + EFR32_BURTC_COMP, UINT32_MAX);
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        efr32_register_put(s, efr32_timers[i].base + EFR32_TIMER_TOP,
                           EFR32_TIMER_TOP_RESET);
    }
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_EM23GRPACLKCTRL,
                       EFR32_CMU_LFCLKSEL_LFRCO);
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_EM4GRPACLKCTRL,
                       EFR32_CMU_LFCLKSEL_LFRCO);
    efr32_register_put(s, EFR32_LFXO_BASE + EFR32_LFXO_CFG,
                       EFR32_LFXO_CFG_RESET);
    efr32_burtc_reset(s);
    efr32_register_put(s, EFR32_CMU_BASE + EFR32_CMU_EM01GRPACLKCTRL,
                       EFR32_CMU_EM01GRPACLKCTRL_CLKSEL_HFRCODPLL);
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
    timer_free(s->rac_frame_timer);
    timer_free(s->rac_ack_timer);
    g_clear_pointer(&s->rx_frame, g_bytes_unref);
    timer_free(s->rac_tx_timer);
    timer_free(s->burtc_timer);
    timer_free(s->rtcc_timer);
    for (int i = 0; i < EFR32_TIMER_COUNT; i++) {
        timer_free(s->timers[i].event);
        timer_free(s->timer_pwm[i]);
    }
    timer_free(s->wdog_timer);
    timer_free(s->dpll_lock_timer);
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

static uint16_t efr32_periph_get_channel(Efr32RfRadio *radio)
{
    return efr32_rf_channel(EFR32_PERIPH(radio));
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
    radio_class->get_channel = efr32_periph_get_channel;
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
    ARMv7MState seq_armv7m;
    Efr32Periph periph;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion radio_secure;
    MemoryRegion radio_ns;
    MemoryRegion radio_ram;
    MemoryRegion radio_ram_ns;
    MemoryRegion seq_memory;
    MemoryRegion seq_ram_alias;
    MemoryRegion seq_radio_alias;
    MemoryRegion seq_sram_alias;
    MemoryRegion seq_periph_ns_alias;
    MemoryRegion seq_periph_s_window;
    MemoryRegion devinfo;
    Clock *clk;
    Clock *seq_clk;
    Clock *protimer_clk;
    Clock *burtc_clk;
    Clock *pclk;
    unsigned endpoint_id;

    MemoryRegion *memory;
    Efr32RfMedium *rf_medium;
    Object *idau;
    uint32_t boot_offset;
    bool core_clocks;
    /* Part identity: optional DEVINFO and USERDATA page images. */
    char *devinfo_file;
    char *userdata_file;
    /* Board analog inputs: "PD02=968,PA05=0" in millivolts. */
    char *analog;
    /* Sized from DEVINFO.MSIZE at realize, as on silicon. */
    uint32_t flash_size;
    uint32_t sram_size;
    /* This part asked for a reset (efr32_soc_reset_request). */
    bool reset_pending;
    /* Told after each reset of this part (efr32bg22_soc_add_reset_notifier). */
    NotifierList reset_notifiers;
};

struct Efr32State
{
    MachineState parent_obj;
    Efr32Bg22SoCState soc;
    Efr32RfMedium *rf_medium;
    char *image;
    char *flash_overlay;
    char *devinfo;
    char *userdata;
    char *analog;
    /* The board around the SoC ("owo-suit"), or none. */
    char *board;
    uint32_t flash_offset;
    uint32_t flash_overlay_offset;
    uint32_t boot_offset;
    bool core_clocks;
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
    char *image[2];
    char *devinfo[2];
    char *userdata[2];
    char *analog[2];
    /* The board around each SoC ("owo-suit"), or none, as the single machine's "board". */
    char *board[2];
    bool core_clocks;
};

/* ARMv7M CPUs require an explicit board reset registration. */
static void efr32_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void efr32_seq_cpu_reset(void *opaque)
{
    CPUState *cpu = CPU(opaque);

    cpu_reset(cpu);
    cpu->halted = 1;
}

/*
 * Resets that the part raises itself (SYSRESETREQ, WDOG0, M33 Lockup, the
 * RESETn pin, the DCI device erase) reset that part alone: two chips on a
 * bench, the dual machine's two SoCs, do not reset each other.  The request
 * is taken where QEMU takes a system reset (the main loop, all vCPUs
 * paused), so it lands at the same point in virtual time as before; the
 * machine's reset hook then resets only the parts that asked
 * (efr32_machine_reset_socs).  A reset nobody asked for (power-on, QMP
 * system_reset) still resets the whole machine.
 */
static void efr32_soc_reset_request(Efr32Bg22SoCState *s)
{
    s->reset_pending = true;
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void efr32_periph_reset_request(Efr32Periph *p)
{
    efr32_soc_reset_request(container_of(p, Efr32Bg22SoCState, periph));
}

void efr32bg22_soc_add_reset_notifier(Object *soc, Notifier *n)
{
    notifier_list_add(&EFR32BG22_SOC(soc)->reset_notifiers, n);
}

/* Inside this SoC's QOM subtree. */
static bool efr32_soc_owns(Efr32Bg22SoCState *s, Object *obj)
{
    for (; obj; obj = obj->parent) {
        if (obj == OBJECT(s)) {
            return true;
        }
    }
    return false;
}

/*
 * Reset one SoC as a system reset resets it: its devices on the system bus
 * (and so the buses below them) in the system bus's order, through the same
 * three reset phases, then its two CPUs, as the root reset container orders
 * them (devices first, then the board's CPU reset handlers).
 */
static void efr32_soc_reset(Efr32Bg22SoCState *s, ResetType type)
{
    ResettableContainer *rc =
        RESETTABLE_CONTAINER(object_new(TYPE_RESETTABLE_CONTAINER));
    BusState *bus = sysbus_get_default();
    BusChild *kid;

    QTAILQ_FOREACH(kid, &bus->children, sibling) {
        if (efr32_soc_owns(s, OBJECT(kid->child))) {
            resettable_container_add(rc, OBJECT(kid->child));
        }
    }
    resettable_reset(OBJECT(rc), type);
    object_unref(OBJECT(rc));
    efr32_cpu_reset(s->armv7m.cpu);
    efr32_seq_cpu_reset(s->seq_armv7m.cpu);
    notifier_list_notify(&s->reset_notifiers, NULL);
}

static void efr32_machine_reset_socs(Efr32Bg22SoCState *socs, unsigned n,
                                     ResetType type)
{
    bool requested = false;

    for (unsigned i = 0; i < n; i++) {
        requested |= socs[i].reset_pending;
    }
    if (!requested) {
        qemu_devices_reset(type);
        for (unsigned i = 0; i < n; i++) {
            notifier_list_notify(&socs[i].reset_notifiers, NULL);
        }
        return;
    }
    for (unsigned i = 0; i < n; i++) {
        if (socs[i].reset_pending) {
            socs[i].reset_pending = false;
            efr32_soc_reset(&socs[i], type);
        }
    }
}

#define EFR32_DEVINFO_MSIZE 0x00cu
#define EFR32_DEVINFO_MSIZE_FLASH_MASK 0xffffu
#define EFR32_DEVINFO_MSIZE_SRAM_SHIFT 16
#define EFR32_DEVINFO_MSIZE_SRAM_MASK 0x7ffu
#define EFR32_DEVINFO_EUI64L 0x048u
#define EFR32_DEVINFO_EUI64H 0x04cu
#define EFR32_DEVINFO_MODULENAME0 0x130u
#define EFR32_DEVINFO_MODULENAME_WORDS 7
#define EFR32_DEVINFO_MODULEINFO 0x14cu
#define EFR32_FLASH_MAX_SIZE 0x80000u
#define EFR32_SRAM_MAX_SIZE 0x8000u
#define EFR32_SRAM_BASE 0x20000000u

/*
 * Default identity: the BGM220PC22HNA2 module of the BRD4314A explorer kit,
 * as observed on that hardware.  Only these fields are known; the remaining
 * DEVINFO words (calibration, ...) read zero.
 *
 * INFO, PART and MEMINFO (RM SS6.4.2.1-3): OpenOCD's efm32 driver decodes
 * them as it probes, and on the kit it reported "EFR32BG22 C224, rev 9" with
 * 8 KiB erase pages (owo-open-fw docs/hardware-bringup.md, evidence log).
 * Its "rev" is INFO.PRODREV: QEMU given the suit's DEVINFO dump (INFO
 * 0x07240110) prints "rev 36".  The suit's dump also fixes the PART
 * encoding: 0x021608AE = FAMILY 2 (BG), FAMILYNUM 22, DEVICENUM 2222 =
 * C222, its BG22C222F352GM40 die.  So the kit's C224 die reads
 *   PART    = BG << 24 | 22 << 16 | ('C' - 'A') * 1000 + 224 = 0x021608B0,
 *   INFO    = DEVINFOREV 7 (RM reset value; the suit's page reads 7) |
 *             PRODREV 9; CRC [U] never read (0),
 *   MEMINFO = FLASHPAGESIZE 3 (2^(3 + 10) = 8 KiB); DILEN and UDPAGESIZE
 *             [U] never read (0).
 * Without PART, OpenOCD's flash driver stopped at "Unknown MCU family 0".
 * OpenOCD reads that family from the byte at DEVINFO + 0x1FE (its Series
 * 0/1 location, kept for every series; 128 = "EFR32/EFM32 Series-2",
 * efm32.c).  The suit's page holds 0x80 there, the rest of that area
 * erased (word 0x1FC reads 0xFF80FFFF); OpenOCD identified the kit on
 * silicon, so its page has the same byte.
 */
#define EFR32_DEVINFO_SERIES2_ID 0x1feu
#define EFR32_DEVINFO_SERIES2_ID_VALUE 0x80u
#define EFR32_DEVINFO_INFO 0x000u
#define EFR32_DEVINFO_INFO_DEVINFOREV_SHIFT 24
#define EFR32_DEVINFO_INFO_PRODREV_SHIFT 16
#define EFR32_DEVINFO_MEMINFO 0x008u
#define EFR32_DEVINFO_MEMINFO_FLASHPAGESIZE_8K 3u
#define EFR32_DEVINFO_PART_DEVICENUM(letter, number) \
    (((letter) - 'A') * 1000u + (number))
#define EFR32_BGM220_HNA2_DEVINFOREV 7u
#define EFR32_BGM220_HNA2_PRODREV 9u
#define EFR32_BGM220_HNA2_PART \
    ((EFR32_DEVINFO_PART_FAMILY_BG << EFR32_DEVINFO_PART_FAMILY_SHIFT) | \
     (EFR32_DEVINFO_PART_FAMILYNUM_XG22 << \
      EFR32_DEVINFO_PART_FAMILYNUM_SHIFT) | \
     EFR32_DEVINFO_PART_DEVICENUM('C', 224u))
#define EFR32_BGM220_HNA2_FLASH_KB 512u
#define EFR32_BGM220_HNA2_SRAM_KB 32u
#define EFR32_BGM220_HNA2_EUI64L 0x0000beefu
#define EFR32_BGM220_HNA2_EUI64H 0x00000a00u
#define EFR32_BGM220_HNA2_MODULEINFO 0x00570100u
static const uint32_t efr32_bgm220_hna2_module_name[
    EFR32_DEVINFO_MODULENAME_WORDS] = {
    0x324d4742, /* "BGM2" */
    0x43503032, /* "20PC" */
    0x4e483232, /* "22HN" */
    0xffff3241, /* "A2" */
    UINT32_MAX, UINT32_MAX, UINT32_MAX,
};

/* SYSCFG.CHIPREV.FAMILY for a DEVINFO.PART; 0 for an unknown part. */
static uint32_t efr32_chiprev_from_part(uint32_t part)
{
    uint32_t family = (part >> EFR32_DEVINFO_PART_FAMILY_SHIFT) &
                      EFR32_DEVINFO_PART_FAMILY_MASK;
    uint32_t number = (part >> EFR32_DEVINFO_PART_FAMILYNUM_SHIFT) &
                      EFR32_DEVINFO_PART_FAMILYNUM_MASK;
    uint32_t code;

    if (number != EFR32_DEVINFO_PART_FAMILYNUM_XG22) {
        return 0;
    }
    switch (family) {
    case EFR32_DEVINFO_PART_FAMILY_FG:
        code = EFR32_SYSCFG_CHIPREV_FAMILY_FG22;
        break;
    case EFR32_DEVINFO_PART_FAMILY_MG:
        code = EFR32_SYSCFG_CHIPREV_FAMILY_MG22;
        break;
    case EFR32_DEVINFO_PART_FAMILY_BG:
        code = EFR32_SYSCFG_CHIPREV_FAMILY_BG22;
        break;
    case EFR32_DEVINFO_PART_FAMILY_PG:
        code = EFR32_SYSCFG_CHIPREV_FAMILY_PG22;
        break;
    default:
        return 0;
    }
    return code << EFR32_SYSCFG_CHIPREV_FAMILY_SHIFT;
}

static void efr32_info_put(uint8_t *info, uint32_t offset, uint32_t value)
{
    stl_le_p(info + offset, value);
}

static bool efr32_info_load(uint8_t *dest, const char *file, size_t size,
                            const char *what, Error **errp)
{
    g_autofree gchar *contents = NULL;
    gsize length;

    if (!file) {
        return true;
    }
    if (!g_file_get_contents(file, &contents, &length, NULL)) {
        error_setg(errp, "efr32bg22: cannot read %s image %s", what, file);
        return false;
    }
    if (length == 0 || length > size) {
        error_setg(errp, "efr32bg22: %s image %s is %zu bytes; the page "
                   "holds 1..%zu", what, file, (size_t)length, size);
        return false;
    }
    memcpy(dest, contents, length);
    return true;
}

/*
 * Fill the information block: the default module identity, or the DEVINFO
 * and USERDATA pages read from a part.  Then size flash and RAM from
 * DEVINFO.MSIZE.
 */
/* Parse "PD02=968,PA5=0": GPIO pin voltages in millivolts. */
static bool efr32_soc_parse_analog(Efr32Bg22SoCState *s, Error **errp)
{
    g_auto(GStrv) items = NULL;

    for (unsigned port = 0; port < EFR32_GPIO_PORTS; port++) {
        for (unsigned pin = 0; pin < EFR32_GPIO_PINS; pin++) {
            s->periph.analog_mv[port][pin] = -1;
        }
    }
    if (!s->analog || !*s->analog) {
        return true;
    }
    items = g_strsplit(s->analog, ",", -1);
    for (char **item = items; *item; item++) {
        unsigned pin, mv;
        char port, end;

        if (sscanf(*item, "P%c%u=%u%c", &port, &pin, &mv, &end) != 3 ||
            port < 'A' || port >= 'A' + EFR32_GPIO_PORTS ||
            pin >= EFR32_GPIO_PINS || mv > INT32_MAX) {
            error_setg(errp, "efr32bg22: analog input '%s' is not "
                       "P<port><pin>=<millivolts>", *item);
            return false;
        }
        s->periph.analog_mv[port - 'A'][pin] = mv;
    }
    return true;
}

static bool efr32_soc_load_identity(Efr32Bg22SoCState *s, Error **errp)
{
    uint8_t *info = memory_region_get_ram_ptr(&s->devinfo);
    uint8_t *devinfo = info + EFR32_DEVINFO_OFFSET;
    uint32_t msize, flash_kb, sram_kb;

    memset(info, EFR32_FLASH_ERASED_BYTE, EFR32_DEVINFO_OFFSET);
    memset(devinfo, 0, EFR32_DEVINFO_WINDOW_SIZE);
    if (s->devinfo_file) {
        /* A captured page is taken whole; erased words stay erased. */
        memset(devinfo, EFR32_FLASH_ERASED_BYTE, EFR32_DEVINFO_SIZE);
        if (!efr32_info_load(devinfo, s->devinfo_file, EFR32_DEVINFO_SIZE,
                             "DEVINFO", errp)) {
            return false;
        }
    } else {
        efr32_info_put(devinfo, EFR32_DEVINFO_INFO,
                       (EFR32_BGM220_HNA2_DEVINFOREV <<
                        EFR32_DEVINFO_INFO_DEVINFOREV_SHIFT) |
                       (EFR32_BGM220_HNA2_PRODREV <<
                        EFR32_DEVINFO_INFO_PRODREV_SHIFT));
        efr32_info_put(devinfo, EFR32_DEVINFO_PART, EFR32_BGM220_HNA2_PART);
        devinfo[EFR32_DEVINFO_SERIES2_ID] = EFR32_DEVINFO_SERIES2_ID_VALUE;
        efr32_info_put(devinfo, EFR32_DEVINFO_MEMINFO,
                       EFR32_DEVINFO_MEMINFO_FLASHPAGESIZE_8K);
        efr32_info_put(devinfo, EFR32_DEVINFO_MSIZE,
                       EFR32_BGM220_HNA2_FLASH_KB |
                       (EFR32_BGM220_HNA2_SRAM_KB <<
                        EFR32_DEVINFO_MSIZE_SRAM_SHIFT));
        efr32_info_put(devinfo, EFR32_DEVINFO_EUI64L,
                       EFR32_BGM220_HNA2_EUI64L);
        efr32_info_put(devinfo, EFR32_DEVINFO_EUI64H,
                       EFR32_BGM220_HNA2_EUI64H);
        for (unsigned i = 0; i < EFR32_DEVINFO_MODULENAME_WORDS; i++) {
            efr32_info_put(devinfo, EFR32_DEVINFO_MODULENAME0 + 4 * i,
                           efr32_bgm220_hna2_module_name[i]);
        }
        efr32_info_put(devinfo, EFR32_DEVINFO_MODULEINFO,
                       EFR32_BGM220_HNA2_MODULEINFO);
    }
    if (!efr32_info_load(info + EFR32_USERDATA_OFFSET, s->userdata_file,
                         EFR32_USERDATA_SIZE, "USERDATA", errp)) {
        return false;
    }

    msize = ldl_le_p(devinfo + EFR32_DEVINFO_MSIZE);
    flash_kb = msize & EFR32_DEVINFO_MSIZE_FLASH_MASK;
    sram_kb = (msize >> EFR32_DEVINFO_MSIZE_SRAM_SHIFT) &
              EFR32_DEVINFO_MSIZE_SRAM_MASK;
    s->flash_size = flash_kb * KiB;
    s->sram_size = sram_kb * KiB;
    if (s->flash_size == 0 || s->flash_size > EFR32_FLASH_MAX_SIZE ||
        s->flash_size % EFR32_FLASH_PAGE_SIZE) {
        error_setg(errp, "efr32bg22: DEVINFO.MSIZE 0x%08x: FLASH %u KB is "
                   "not a whole number of 8 KB pages up to 512 KB",
                   msize, flash_kb);
        return false;
    }
    if (s->sram_size == 0 || s->sram_size > EFR32_SRAM_MAX_SIZE) {
        error_setg(errp, "efr32bg22: DEVINFO.MSIZE 0x%08x: SRAM %u KB is "
                   "outside 1..32 KB", msize, sram_kb);
        return false;
    }
    return true;
}

/*
 * The M33's (and the debugger's) SEQRAM window: RADIOCLKCTRL.EN cleared made
 * SEQRAM at 0xb0000000 fault like every radio window and BUFC (silicon,
 * owo-open-fw docs/radio-tables.md SS7.72); a gated write is dropped, as the
 * radio windows' are.  The M0+'s own SEQRAM view is not affected: the M0+
 * stops while RADIOCLKCTRL.EN is clear (SS7.73).
 */
static bool efr32_seqram_gated(Efr32Bg22SoCState *s)
{
    return !(efr32_register_get(&s->periph,
                                EFR32_CMU_BASE + EFR32_CMU_RADIOCLKCTRL) &
             EFR32_CMU_RADIOCLKCTRL_EN);
}

static MemTxResult efr32_seqram_m33_read(void *opaque, hwaddr offset,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    Efr32Bg22SoCState *s = opaque;
    uint8_t buf[8];
    MemTxResult r;

    if (efr32_seqram_gated(s)) {
        *data = 0;
        return MEMTX_ERROR;
    }
    efr32_bus_access(&s->periph, EFR32_SEQRAM_NS_BASE + offset, size, false);
    r = address_space_read(&s->periph.dma_as, EFR32_SEQRAM_S_BASE + offset,
                           attrs, buf, size);
    *data = ldn_le_p(buf, size);
    return r;
}

static MemTxResult efr32_seqram_m33_write(void *opaque, hwaddr offset,
                                          uint64_t value, unsigned size,
                                          MemTxAttrs attrs)
{
    Efr32Bg22SoCState *s = opaque;
    uint8_t buf[8];

    if (efr32_seqram_gated(s)) {
        return MEMTX_ERROR;
    }
    efr32_bus_access(&s->periph, EFR32_SEQRAM_NS_BASE + offset, size, true);
    stn_le_p(buf, size, value);
    return address_space_write(&s->periph.dma_as, EFR32_SEQRAM_S_BASE + offset,
                               attrs, buf, size);
}

static const MemoryRegionOps efr32_seqram_m33_ops = {
    .read_with_attrs = efr32_seqram_m33_read,
    .write_with_attrs = efr32_seqram_m33_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4},
    .impl = {.min_access_size = 1, .max_access_size = 4},
};

/*
 * The M0+'s view of main SRAM (SS7.74: the same SRAM as the M33's): an
 * access through the matrix, so it goes through the bus model
 * (efr32-interconnect.c), which a RAM alias would bypass.
 */
static MemTxResult efr32_seq_sram_read(void *opaque, hwaddr offset,
                                       uint64_t *data, unsigned size,
                                       MemTxAttrs attrs)
{
    Efr32Bg22SoCState *s = opaque;
    uint8_t buf[8];
    MemTxResult r;

    efr32_bus_access(&s->periph, EFR32_SRAM_BASE + offset, size, false);
    r = address_space_read(&s->periph.dma_as, EFR32_SRAM_BASE + offset,
                           attrs, buf, size);
    *data = ldn_le_p(buf, size);
    return r;
}

static MemTxResult efr32_seq_sram_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size,
                                        MemTxAttrs attrs)
{
    Efr32Bg22SoCState *s = opaque;
    uint8_t buf[8];

    efr32_bus_access(&s->periph, EFR32_SRAM_BASE + offset, size, true);
    stn_le_p(buf, size, value);
    return address_space_write(&s->periph.dma_as, EFR32_SRAM_BASE + offset,
                               attrs, buf, size);
}

static const MemoryRegionOps efr32_seq_sram_ops = {
    .read_with_attrs = efr32_seq_sram_read,
    .write_with_attrs = efr32_seq_sram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4},
    .impl = {.min_access_size = 1, .max_access_size = 4},
};

/* AIRCR.SYSRESETREQ of the M33: a system reset with RSTCAUSE.SYSREQ. */
static void efr32_soc_sysresetreq(void *opaque, int n, int level)
{
    Efr32Bg22SoCState *s = opaque;

    if (level && (s->periph.rstctrl & EFR32_EMU_RSTCTRL_SYSRMODE)) {
        s->periph.rstcause |= EFR32_EMU_RSTCAUSE_SYSREQ;
        /* The debug port sees the system reset begin (efr32-dap) */
        efr32bg22_debug_system_reset(OBJECT(s));
        efr32_soc_reset_request(s);
    }
}

/*
 * The RESETn pin, driven by the debug probe's nRESET (efr32-dap "nreset"):
 * RM 12.3.x "RESET pin Reset", EMU_RSTCAUSE.PIN (RM 12.5.15 bit 1).  While
 * the pin is low the part is held in reset: the M33 does not run (the M0+ is
 * held after any reset until released by the RAC), and the SW-DP does not
 * answer (efr32-dap.c).  Release starts the part from its reset vector, as
 * the J-Link SRST pulse on the EK did (one fresh boot banner,
 * scratch/silicon-rx/recover2-srsttest.log).
 */
static void efr32_soc_nreset(void *opaque, int n, int level)
{
    Efr32Bg22SoCState *s = opaque;

    if (level) {
        s->periph.rstcause |= EFR32_EMU_RSTCAUSE_PIN;
        /* A hard reset: EMU_RSTCTRL returns to its reset value */
        s->periph.rstctrl = EFR32_EMU_RSTCTRL_RESET;
    }
    CPU(s->armv7m.cpu)->start_powered_off = level;
    efr32_soc_reset_request(s);
}

/* The reset that follows a DCI device erase (efr32-dap.c). */
static void efr32_soc_erase_reset(void *opaque, int n, int level)
{
    if (level) {
        efr32_soc_reset_request(opaque);
    }
}

/*
 * The M33's LOCKUP output (armv7m_nvic): an M33 Lockup resets the device
 * only with EMU_RSTCTRL.LOCKUPRMODE set (RM 12.5.14, reset 0, "Reset
 * Request is Block[ed]"), recorded as RSTCAUSE.LOCKUP; otherwise the core
 * stays in Lockup.  The radio M0+'s Lockup is not wired to any reset: on
 * silicon it stayed locked until the debugger or a reset cleared it
 * (owo-open-fw docs/radio-tables.md SS7.58, SS7.59).
 */
static void efr32_soc_lockup(void *opaque, int n, int level)
{
    Efr32Bg22SoCState *s = opaque;

    if (level && (s->periph.rstctrl & EFR32_EMU_RSTCTRL_LOCKUPRMODE)) {
        s->periph.rstcause |= EFR32_EMU_RSTCAUSE_LOCKUP;
        efr32_soc_reset_request(s);
    }
}

/*
 * The M0+'s view of the secure peripheral window 0x40000000-0x4fffffff
 * (docs/radio-tables.md SS7.74, measured): no fault, and TIMER1.CNT at
 * 0x4004C024 reads 0 while the same register at 0x5004C024 counts.
 * [U] Whether a write there lands is unmeasured; it is dropped here.
 */
static uint64_t efr32_seq_periph_s_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    return 0;
}

static void efr32_seq_periph_s_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
}

static const MemoryRegionOps efr32_seq_periph_s_ops = {
    .read = efr32_seq_periph_s_read,
    .write = efr32_seq_periph_s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 1, .max_access_size = 4},
};

static void efr32_soc_init(Object *obj)
{
    Efr32Bg22SoCState *s = EFR32BG22_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "radio-sequencer", &s->seq_armv7m,
                            TYPE_ARMV7M);
    object_initialize_child(obj, "periph", &s->periph, TYPE_EFR32_PERIPH);
    notifier_list_init(&s->reset_notifiers);
}

/* efr32bg22c224f512im40.h IRQn_Type. */
#define EFR32_USART0_RX_IRQ 13
#define EFR32_USART0_TX_IRQ 14
#define EFR32_USART1_RX_IRQ 15
#define EFR32_USART1_TX_IRQ 16
#define EFR32_GPIO_ODD_IRQ 25
#define EFR32_GPIO_EVEN_IRQ 26
#define EFR32_I2C0_IRQ 27
/* efr32bg22c224f512im40.h GPIO_S_BASE, I2C0_S_BASE. */
#define EFR32_GPIO_BASE 0x4003c000u
#define EFR32_I2C0_BASE 0x4a010000u

/*
 * USART0/1 (the dual machine's SoC n takes
 * -serial 2n and 2n+1 unless a chardev "socN.usartM" claims the port).
 */
static const Efr32UsartConfig efr32_usarts[] = {
    {.name = "usart0", .index = 0, .base = EFR32_USART0_BASE,
     .ns_offset = PERIPH_NONSECURE_OFFSET, .rx_irq = EFR32_USART0_RX_IRQ,
     .tx_irq = EFR32_USART0_TX_IRQ},
    {.name = "usart1", .index = 1, .base = EFR32_USART1_BASE,
     .ns_offset = PERIPH_NONSECURE_OFFSET, .rx_irq = EFR32_USART1_RX_IRQ,
     .tx_irq = EFR32_USART1_TX_IRQ},
};

/*
 * The PRS and its wiring: every producer of the core model and the GPIO
 * EXTI pins into it, its consumer lines to the blocks that serve them, its
 * channel outputs to the GPIO PRS routes.
 */
static void efr32_soc_create_reflex(Efr32Bg22SoCState *s, DeviceState *gpio)
{
    DeviceState *periph = DEVICE(&s->periph);
    DeviceState *prs = efr32_prs_create(OBJECT(s), s->memory,
                                        &s->periph.link, EFR32_PRS_BASE,
                                        PERIPH_NONSECURE_OFFSET);
    DeviceState *letimer = efr32_letimer_create(
        OBJECT(s), s->memory, &s->periph.link, EFR32_LETIMER0_BASE,
        PERIPH_NONSECURE_OFFSET,
        qdev_get_gpio_in(DEVICE(&s->armv7m), EFR32_LETIMER0_IRQ));
    DeviceState *euart = efr32_eusart_create(
        OBJECT(s), s->memory, &s->periph.link, EFR32_EUART0_BASE,
        PERIPH_NONSECURE_OFFSET,
        qdev_get_gpio_in(DEVICE(&s->armv7m), EFR32_EUART0_RX_IRQ),
        qdev_get_gpio_in(DEVICE(&s->armv7m), EFR32_EUART0_TX_IRQ));
    DeviceState *ldma = efr32_ldma_create(
        OBJECT(s), s->memory, &s->periph.link, PERIPH_NONSECURE_OFFSET,
        qdev_get_gpio_in(DEVICE(&s->armv7m), EFR32_LDMA_IRQ));
    static const uint8_t timer_dma_source[EFR32_TIMER_COUNT] = {
        EFR32_LDMA_SRC_TIMER0, EFR32_LDMA_SRC_TIMER1, EFR32_LDMA_SRC_TIMER2,
        EFR32_LDMA_SRC_TIMER3, EFR32_LDMA_SRC_TIMER4,
    };
    /* Consumer lines served outside the core model, and by whom. */
    struct {
        unsigned consumer;
        DeviceState *dev;
        const char *line;
        unsigned n;
    } served[] = {
        { EFR32_PRS_CON_LETIMER0_START, letimer, EFR32_LETIMER_PRS_IN, 0 },
        { EFR32_PRS_CON_LETIMER0_STOP, letimer, EFR32_LETIMER_PRS_IN, 1 },
        { EFR32_PRS_CON_LETIMER0_CLEAR, letimer, EFR32_LETIMER_PRS_IN, 2 },
        { EFR32_PRS_CON_EUART0_TRIGGER, euart, EFR32_EUSART_PRS_TRIGGER, 0 },
        { EFR32_PRS_CON_LDMAXBAR_DMAREQ0, ldma, EFR32_LDMA_REQ,
          EFR32_LDMA_REQ_LINE(EFR32_LDMA_SRC_LDMAXBAR, 0) },
        { EFR32_PRS_CON_LDMAXBAR_DMAREQ1, ldma, EFR32_LDMA_REQ,
          EFR32_LDMA_REQ_LINE(EFR32_LDMA_SRC_LDMAXBAR, 1) },
    };

    for (unsigned t = 0; t < EFR32_TIMER_COUNT; t++) {
        for (unsigned n = 0; n < 4; n++) {
            qdev_connect_gpio_out_named(periph, "timer-dma", t * 4 + n,
                                        qdev_get_gpio_in_named(
                                            ldma, EFR32_LDMA_REQ,
                                            EFR32_LDMA_REQ_LINE(
                                                timer_dma_source[t], n)));
        }
    }
    qdev_connect_gpio_out_named(euart, EFR32_EUSART_DMA_REQ, 0,
                                qdev_get_gpio_in_named(
                                    ldma, EFR32_LDMA_REQ,
                                    EFR32_LDMA_REQ_LINE(
                                        EFR32_LDMA_SRC_EUART0,
                                        EFR32_LDMA_SIG_EUART_TXFL)));
    qdev_connect_gpio_out_named(euart, EFR32_EUSART_DMA_REQ, 1,
                                qdev_get_gpio_in_named(
                                    ldma, EFR32_LDMA_REQ,
                                    EFR32_LDMA_REQ_LINE(
                                        EFR32_LDMA_SRC_EUART0,
                                        EFR32_LDMA_SIG_EUART_RXFL)));
    for (unsigned u = 0; u < 2; u++) {
        DeviceState *usart = DEVICE(object_resolve_path_component(
            OBJECT(s), u ? "usart1" : "usart0"));

        for (unsigned n = 0; n < EFR32_USART_DMA_REQS; n++) {
            qdev_connect_gpio_out_named(usart, EFR32_USART_DMA_REQ, n,
                                        qdev_get_gpio_in_named(
                                            ldma, EFR32_LDMA_REQ,
                                            EFR32_LDMA_REQ_LINE(
                                                EFR32_LDMA_SRC_USART0 + u,
                                                n)));
        }
    }
    for (unsigned ch = 0; ch < EFR32_PRS_LDMA_SYNC_CHANNELS; ch++) {
        qdev_connect_gpio_out_named(prs, EFR32_PRS_ASYNC_CH_LDMA, ch,
                                    qdev_get_gpio_in_named(
                                        ldma, EFR32_LDMA_PRS_SYNC, ch));
    }

    qdev_connect_gpio_out_named(euart, EFR32_EUSART_PRS_TXC, 0,
                                qdev_get_gpio_in_named(
                                    prs, EFR32_PRS_ASYNC_IN,
                                    EFR32_PRS_LINE(EFR32_PRS_SRC_EUART0,
                                                   EFR32_PRS_SIG_EUART_TXC)));
    qdev_connect_gpio_out_named(euart, EFR32_EUSART_PRS_RXFL, 0,
                                qdev_get_gpio_in_named(
                                    prs, EFR32_PRS_ASYNC_IN,
                                    EFR32_PRS_LINE(EFR32_PRS_SRC_EUART0,
                                                   EFR32_PRS_SIG_EUART_RXFL)));

    s->periph.prs = prs;
    for (unsigned n = 0; n < EFR32_PRS_ASYNC_INPUTS; n++) {
        qdev_connect_gpio_out_named(periph, "prs-async", n,
                                    qdev_get_gpio_in_named(
                                        prs, EFR32_PRS_ASYNC_IN, n));
    }
    for (unsigned n = 0; n < EFR32_PRS_SYNC_INPUTS; n++) {
        qdev_connect_gpio_out_named(periph, "prs-sync", n,
                                    qdev_get_gpio_in_named(
                                        prs, EFR32_PRS_SYNC_IN, n));
    }
    for (unsigned n = 0; n < EFR32_GPIO_PRS_PINS; n++) {
        qdev_connect_gpio_out_named(gpio, EFR32_GPIO_PRS_PIN, n,
                                    qdev_get_gpio_in_named(
                                        prs, EFR32_PRS_ASYNC_IN,
                                        EFR32_PRS_LINE(EFR32_PRS_SRC_GPIO,
                                                       n)));
    }
    for (unsigned n = 0; n < EFR32_GPIO_LETIMER_OUTS; n++) {
        qdev_connect_gpio_out_named(letimer, EFR32_LETIMER_OUT, n,
                                    qdev_get_gpio_in_named(
                                        gpio, EFR32_GPIO_LETIMER_OUT, n));
        qdev_connect_gpio_out_named(letimer, EFR32_LETIMER_PRS, n,
                                    qdev_get_gpio_in_named(
                                        prs, EFR32_PRS_ASYNC_IN,
                                        EFR32_PRS_LINE(EFR32_PRS_SRC_LETIMER0,
                                                       n)));
    }
    for (unsigned n = 0; n < EFR32_PRS_CONSUMERS; n++) {
        qemu_irq target = qdev_get_gpio_in_named(periph, "prs-consumer", n);

        for (unsigned i = 0; i < ARRAY_SIZE(served); i++) {
            if (served[i].consumer == n) {
                target = qdev_get_gpio_in_named(served[i].dev, served[i].line,
                                                served[i].n);
            }
        }
        qdev_connect_gpio_out_named(prs, EFR32_PRS_CONSUMER, n, target);
        qdev_connect_gpio_out_named(prs, EFR32_PRS_CONSUMER_SYNC, n,
                                    qdev_get_gpio_in_named(
                                        periph, "prs-consumer-sync", n));
    }
    for (unsigned ch = 0; ch < EFR32_PRS_ASYNC_CHANNELS; ch++) {
        qdev_connect_gpio_out_named(prs, EFR32_PRS_ASYNC_CH, ch,
                                    qdev_get_gpio_in_named(
                                        gpio, EFR32_GPIO_PRS_CH, ch));
    }
    for (unsigned ch = 0; ch < EFR32_PRS_SYNC_CHANNELS; ch++) {
        qdev_connect_gpio_out_named(prs, EFR32_PRS_SYNC_CH, ch,
                                    qdev_get_gpio_in_named(
                                        gpio, EFR32_GPIO_PRS_CH,
                                        EFR32_PRS_ASYNC_CHANNELS + ch));
    }
}

static void efr32_soc_realize(DeviceState *dev, Error **errp)
{
    Efr32Bg22SoCState *s = EFR32BG22_SOC(dev);
    DeviceState *nvic = DEVICE(&s->armv7m);
    SysBusDevice *periph = SYS_BUS_DEVICE(&s->periph);
    const gchar *name = object_get_canonical_path_component(OBJECT(s));
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
    s->seq_clk = clock_new(OBJECT(s), "HCLKRADIO");
    clock_set_hz(s->seq_clk, 38400000);
    s->protimer_clk = clock_new(OBJECT(s), "PROTIMERCLK");
    clock_set_hz(s->protimer_clk, 2000000);
    s->burtc_clk = clock_new(OBJECT(s), "BURTCCLK");
    clock_set_hz(s->burtc_clk, 32768);
    s->pclk = clock_new(OBJECT(s), "PCLK");

    /*
     * Identity first: flash and RAM are sized from DEVINFO.MSIZE.  Addresses
     * past the flash end are left unmapped, so an access there is a bus
     * fault, as RM SS4.2.1.2 and SS6.6.4 describe for an address that is not
     * assigned (SYSCFG_CTRL.ADDRFAULTEN, set by default).
     */
    memory_region_init_ram(&s->devinfo, OBJECT(s), devinfo_name,
                           EFR32_INFO_WINDOW_SIZE, &error_fatal);
    memory_region_add_subregion(s->memory, EFR32_INFO_BASE, &s->devinfo);
    if (!efr32_soc_load_identity(s, errp)) {
        return;
    }
    memory_region_init_ram(&s->flash, OBJECT(s), flash_name, s->flash_size,
                           &error_fatal);
    memory_region_add_subregion(s->memory, 0, &s->flash);
    memory_region_init_ram(&s->sram, OBJECT(s), sram_name, s->sram_size,
                           &error_fatal);
    memory_region_add_subregion(s->memory, EFR32_SRAM_BASE, &s->sram);
    s->periph.flash_size = s->flash_size;
    if (!efr32_soc_parse_analog(s, errp)) {
        return;
    }
    s->periph.chiprev = efr32_chiprev_from_part(
        ldl_le_p((uint8_t *)memory_region_get_ram_ptr(&s->devinfo) +
                 EFR32_DEVINFO_OFFSET + EFR32_DEVINFO_PART));

    qdev_prop_set_uint32(nvic, "num-irq", 64);
    qdev_connect_clock_in(nvic, "cpuclk", s->clk);
    qdev_prop_set_string(nvic, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    /*
     * EFR32 Series 2 implements the Armv8-M MPU with 16 regions in each
     * security state.  The generic Cortex-M33 default does not describe that
     * part accurately, and Zephyr programs its MPU during early boot.
     */
    qdev_prop_set_uint32(nvic, "mpu-ns-regions", 16);
    qdev_prop_set_uint32(nvic, "mpu-s-regions", 16);
    /*
     * Direct application-slot boot is a model convenience, not a bootloader
     * emulation.  Keep both reset VTORs at the selected slot so that a secure
     * or non-secure image begins from its own vector table.
     */
    qdev_prop_set_uint32(nvic, "init-svtor", s->boot_offset);
    qdev_prop_set_uint32(nvic, "init-nsvtor", s->boot_offset);
    object_property_set_link(OBJECT(&s->armv7m), "idau", s->idau,
                             &error_abort);
    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(s->memory),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }
    /*
     * CPUID: OpenOCD reports "Cortex-M33 r0p4 processor detected" on the
     * EK and the suit; QEMU's cortex-m33 is r0p3.
     */
    s->armv7m.cpu->midr = EFR32_M33_CPUID;
    qdev_connect_gpio_out_named(nvic, "SYSRESETREQ", 0,
                                qemu_allocate_irq(efr32_soc_sysresetreq, s,
                                                  0));
    qdev_connect_gpio_out_named(nvic, "LOCKUP", 0,
                                qemu_allocate_irq(efr32_soc_lockup, s, 0));
    efr32bg22_debug_realize(OBJECT(s), &s->armv7m, &s->flash, s->clk, errp);
    if (*errp) {
        return;
    }

    s->periph.rf_medium = s->rf_medium;
    s->periph.endpoint_id = s->endpoint_id;
    object_property_set_link(OBJECT(&s->periph), "memory", OBJECT(s->memory),
                             &error_abort);
    qdev_connect_clock_in(DEVICE(&s->periph), "protimerclk", s->protimer_clk);
    qdev_connect_clock_in(DEVICE(&s->periph), "burtcclk", s->burtc_clk);
    if (!sysbus_realize(periph, errp)) {
        return;
    }
    qdev_connect_gpio_out_named(
        DEVICE(object_resolve_path_component(OBJECT(s), "m33-debug")),
        "halted", 0,
        qdev_get_gpio_in_named(DEVICE(&s->periph), "m33-debug-halted", 0));
    {
        DeviceState *dap =
            DEVICE(object_resolve_path_component(OBJECT(s), "dap"));

        qdev_connect_gpio_out_named(DEVICE(&s->periph), "debug-wedge", 0,
                                    qdev_get_gpio_in_named(dap, "debug-wedge",
                                                           0));
        qdev_connect_gpio_out_named(dap, "nreset", 0,
                                    qemu_allocate_irq(efr32_soc_nreset, s, 0));
        qdev_connect_gpio_out_named(dap, "erase-reset", 0,
                                    qemu_allocate_irq(efr32_soc_erase_reset,
                                                      s, 0));
    }
    efr32_rf_medium_attach(s->rf_medium, EFR32_RF_RADIO(&s->periph), errp);
    if (*errp) {
        return;
    }
    memory_region_add_subregion(s->memory, PERIPH_BASE,
                                sysbus_mmio_get_region(periph, 0));
    memory_region_init_alias(&s->radio_secure, OBJECT(s), "radio-secure",
                             sysbus_mmio_get_region(periph, 1), 0, RADIO_SIZE);
    memory_region_add_subregion(s->memory, RADIO_BASE, &s->radio_secure);
    for (unsigned irq = 0; irq < 64; irq++) {
        sysbus_connect_irq(periph, irq, qdev_get_gpio_in(nvic, irq));
    }

    memory_region_init_alias(&s->radio_ns, OBJECT(s), "radio-ns",
                             sysbus_mmio_get_region(periph, 1), 0, RADIO_SIZE);
    memory_region_add_subregion(s->memory, 0xb8000000, &s->radio_ns);
    s->periph.m33_radio_secure = &s->radio_secure;
    s->periph.m33_radio_nonsecure = &s->radio_ns;
    memory_region_set_enabled(&s->radio_ns, false);
    memory_region_init_ram(&s->radio_ram, OBJECT(s), radio_ram_name, 0x8000,
                           &error_fatal);
    memory_region_add_subregion(s->memory, 0xa0000000, &s->radio_ram);
    /* The M33's non-secure SEQRAM window is an I/O view of the same RAM, so
     * its accesses reach the bus model (efr32_seqram_m33_ops). */
    memory_region_init_io(&s->radio_ram_ns, OBJECT(s), &efr32_seqram_m33_ops,
                          s, "efr32.radioram-ns", 0x8000);
    memory_region_add_subregion(s->memory, 0xb0000000, &s->radio_ram_ns);

    /* The BG22 radio sequencer is an asymmetric Cortex-M0-class core.  Its
     * local address zero is the first 16 KiB of shared SEQRAM, while radio
     * peripherals retain their secure 0xa8xx/0xaaxx addresses. */
    memory_region_init(&s->seq_memory, OBJECT(s), "efr32.seq-memory",
                       UINT64_C(1) << 32);
    memory_region_init_alias(&s->seq_ram_alias, OBJECT(s),
                             "efr32.seq-local-ram", &s->radio_ram, 0,
                             0x8000);
    memory_region_add_subregion(&s->seq_memory, 0, &s->seq_ram_alias);
    memory_region_init_alias(&s->seq_radio_alias, OBJECT(s),
                             "efr32.seq-radio",
                             sysbus_mmio_get_region(periph, 1), 0,
                             RADIO_SIZE);
    memory_region_add_subregion(&s->seq_memory, RADIO_BASE,
                                &s->seq_radio_alias);
    /*
     * SS7.74 (silicon, M0+ address map): main SRAM at 0x20000000 is the
     * M33's SRAM; the non-secure peripheral window 0x5xxxxxxx reaches the
     * peripherals (TIMER1, PRS, CMU); the secure window reads 0.  The stock
     * sequencer reads EMU.TEMP through the secure window (0x40004088).
     */
    memory_region_init_io(&s->seq_sram_alias, OBJECT(s),
                          &efr32_seq_sram_ops, s, "efr32.seq-sram",
                          s->sram_size);
    memory_region_add_subregion(&s->seq_memory, EFR32_SRAM_BASE,
                                &s->seq_sram_alias);
    memory_region_init_alias(&s->seq_periph_ns_alias, OBJECT(s),
                             "efr32.seq-periph-ns",
                             sysbus_mmio_get_region(periph, 0),
                             PERIPH_NONSECURE_OFFSET,
                             PERIPH_NONSECURE_END - PERIPH_NONSECURE_BASE);
    memory_region_add_subregion(&s->seq_memory, PERIPH_NONSECURE_BASE,
                                &s->seq_periph_ns_alias);
    memory_region_init_io(&s->seq_periph_s_window, OBJECT(s),
                          &efr32_seq_periph_s_ops, s,
                          "efr32.seq-periph-secure", PERIPH_NONSECURE_OFFSET);
    memory_region_add_subregion(&s->seq_memory, PERIPH_BASE,
                                &s->seq_periph_s_window);
    qdev_prop_set_uint32(DEVICE(&s->seq_armv7m), "num-irq", 64);
    qdev_connect_clock_in(DEVICE(&s->seq_armv7m), "cpuclk", s->seq_clk);
    qdev_prop_set_string(DEVICE(&s->seq_armv7m), "cpu-type",
                         ARM_CPU_TYPE_NAME("cortex-m0"));
    object_property_set_link(OBJECT(&s->seq_armv7m), "memory",
                             OBJECT(&s->seq_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->seq_armv7m), errp)) {
        return;
    }
    s->periph.seq_cpu = CPU(s->seq_armv7m.cpu);
    s->periph.seq_cpu->halted = 1;
    s->periph.m33_cpu = CPU(s->armv7m.cpu);
    s->periph.m33_cpuclk = s->clk;
    s->periph.seq_cpuclk = s->seq_clk;
    s->periph.pclk = s->pclk;
    if (s->core_clocks && icount_enabled() &&
        !icount_cpu_clocks_enable(EFR32_CORE_CLOCK_QUANTUM_NS)) {
        warn_report_once("efr32bg22: core-clocks needs a fixed -icount shift; "
                         "both cores share the instruction rate");
    }
    s->periph.bus_model.access = efr32_bus_cost;
    s->periph.bus_model.round_end = efr32_bus_round_end;
    s->periph.bus_model.opaque = &s->periph;
    s->periph.ic.cpu[EFR32_IC_M33] = s->periph.m33_cpu;
    s->periph.ic.cpu[EFR32_IC_SEQ] = s->periph.seq_cpu;
    icount_register_bus_model(&s->periph.bus_model);
    icount_set_cpu_cycle_counting(s->periph.seq_cpu, true);
    icount_set_cpu_exception_cycles(s->periph.seq_cpu,
                                    EFR32_SEQ_EXCEPTION_ENTRY_MILLICYCLES);
    icount_set_cpu_exception_cycles(s->periph.m33_cpu,
                                    EFR32_M33_EXCEPTION_ENTRY_MILLICYCLES);
    efr32_cpu_clocks_update(&s->periph);
    s->periph.seq_released = false;
    for (unsigned i = 0; i < G_N_ELEMENTS(efr32_usarts); i++) {
        Efr32UsartConfig usart = efr32_usarts[i];

        usart.serial_index = s->endpoint_id * G_N_ELEMENTS(efr32_usarts) + i;
        efr32_usart_create(OBJECT(s), s->memory, nvic, s->pclk,
                           &s->periph.bus_model, &usart);
    }
    {
        DeviceState *gpio = efr32_gpio_create(
            OBJECT(s), s->memory, nvic, &s->periph.bus_model,
            EFR32_GPIO_BASE, PERIPH_NONSECURE_OFFSET, EFR32_GPIO_ODD_IRQ,
            EFR32_GPIO_EVEN_IRQ);

        for (unsigned n = 0; n < EFR32_GPIO_TIMERS * EFR32_GPIO_TIMER_CCS;
             n++) {
            qdev_connect_gpio_out_named(
                DEVICE(&s->periph), "timer-cc", n,
                qdev_get_gpio_in_named(gpio, EFR32_GPIO_TIMER_CC, n));
        }
        efr32_i2c_create(OBJECT(s), s->memory, nvic, s->pclk,
                         &s->periph.bus_model, 0, EFR32_I2C0_BASE,
                         PERIPH_NONSECURE_OFFSET, EFR32_I2C0_IRQ);
        efr32_soc_create_reflex(s, gpio);
    }
    efr32_buram_create(OBJECT(s), s->memory, &s->periph.link,
                       EFR32_BURAM_BASE, PERIPH_NONSECURE_OFFSET);
    for (unsigned state = 0; state < EFR32_RAC_SEQ_STATE_EVENTS; state++) {
        s->periph.seq_irq_state[state] =
            qdev_get_gpio_in(DEVICE(&s->seq_armv7m), state);
    }
    s->periph.seq_irq_mailbox =
        qdev_get_gpio_in(DEVICE(&s->seq_armv7m), EFR32_RDMAILBOX_SEQ_IRQ);
}

static Property efr32_soc_properties[] = {
    DEFINE_PROP_LINK("memory", Efr32Bg22SoCState, memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("rf-medium", Efr32Bg22SoCState, rf_medium,
                     TYPE_EFR32_RF_MEDIUM, Efr32RfMedium *),
    DEFINE_PROP_LINK("idau", Efr32Bg22SoCState, idau, TYPE_IDAU_INTERFACE,
                     Object *),
    DEFINE_PROP_BOOL("core-clocks", Efr32Bg22SoCState, core_clocks, true),
    DEFINE_PROP_STRING("devinfo", Efr32Bg22SoCState, devinfo_file),
    DEFINE_PROP_STRING("userdata", Efr32Bg22SoCState, userdata_file),
    DEFINE_PROP_STRING("analog", Efr32Bg22SoCState, analog),
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

/* A raw image of n bytes at offset lies inside this part's main flash. */
static bool efr32_fits_flash(Efr32Bg22SoCState *soc, uint32_t offset,
                             gsize n)
{
    return offset <= soc->flash_size && n <= soc->flash_size - offset;
}

static void efr32_machine_reset(MachineState *m, ResetType type)
{
    efr32_machine_reset_socs(&EFR32BG22_MACHINE(m)->soc, 1, type);
}

static void efr32_dual_machine_reset(MachineState *m, ResetType type)
{
    Efr32DualState *s = EFR32BG22_DUAL_MACHINE(m);

    efr32_machine_reset_socs(s->soc, G_N_ELEMENTS(s->soc), type);
}

static void init(MachineState *m)
{
    Efr32State *s = EFR32BG22_MACHINE(m);
    s->rf_medium = EFR32_RF_MEDIUM(object_new(TYPE_EFR32_RF_MEDIUM));
    object_property_add_child(OBJECT(m), "rf-medium", OBJECT(s->rf_medium));
    object_initialize_child(OBJECT(m), "soc", &s->soc, TYPE_EFR32BG22_SOC);
    s->soc.boot_offset = s->boot_offset;
    object_property_set_bool(OBJECT(&s->soc), "core-clocks", s->core_clocks,
                             &error_fatal);
    object_property_set_link(OBJECT(&s->soc), "memory",
                             OBJECT(get_system_memory()), &error_fatal);
    object_property_set_link(OBJECT(&s->soc), "rf-medium", OBJECT(s->rf_medium),
                             &error_fatal);
    object_property_set_link(OBJECT(&s->soc), "idau", OBJECT(m), &error_fatal);
    if (s->devinfo) {
        object_property_set_str(OBJECT(&s->soc), "devinfo", s->devinfo,
                                &error_fatal);
    }
    if (s->userdata) {
        object_property_set_str(OBJECT(&s->soc), "userdata", s->userdata,
                                &error_fatal);
    }
    if (s->analog) {
        object_property_set_str(OBJECT(&s->soc), "analog", s->analog,
                                &error_fatal);
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);
    if (s->image)
    {
        gchar *d;
        gsize n;
        if (!g_file_get_contents(s->image, &d, &n, NULL) ||
            !efr32_fits_flash(&s->soc, s->flash_offset, n))
        {
            error_report("efr32bg22: cannot load flash image %s", s->image);
            exit(EXIT_FAILURE);
        }
        memcpy(memory_region_get_ram_ptr(&s->soc.flash) + s->flash_offset,
               d, n);
        g_free(d);
    }
    /* An optional raw overlay represents separately captured persistent data
     * at its physical flash address. It never supplies a default value. */
    if (s->flash_overlay) {
        gchar *d;
        gsize n;

        if (!g_file_get_contents(s->flash_overlay, &d, &n, NULL) ||
            !efr32_fits_flash(&s->soc, s->flash_overlay_offset, n)) {
            error_report("efr32bg22: cannot load flash overlay %s",
                         s->flash_overlay);
            exit(EXIT_FAILURE);
        }
        memcpy(memory_region_get_ram_ptr(&s->soc.flash) +
               s->flash_overlay_offset, d, n);
        g_free(d);
    }
    qemu_register_reset(efr32_cpu_reset, s->soc.armv7m.cpu);
    qemu_register_reset(efr32_seq_cpu_reset, s->soc.seq_armv7m.cpu);
    if (s->board && !strcmp(s->board, TYPE_OWO_SUIT)) {
        DeviceState *board = qdev_new(TYPE_OWO_SUIT);

        object_property_add_child(OBJECT(m), "suit", OBJECT(board));
        object_property_set_link(OBJECT(board), "soc", OBJECT(&s->soc),
                                 &error_fatal);
        qdev_realize_and_unref(board, NULL, &error_fatal);
        /* The board's analog nets feed the IADC (unless "analog" names
         * the pin). */
        s->soc.periph.analog_source = OBJECT(board);
    } else if (s->board) {
        error_report("efr32bg22: unknown board '%s' (known: %s)", s->board,
                     TYPE_OWO_SUIT);
        exit(EXIT_FAILURE);
    }
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
static char *flash_overlay_get(Object *o, Error **e)
{
    return g_strdup(EFR32BG22_MACHINE(o)->flash_overlay);
}
static void flash_overlay_set(Object *o, const char *v, Error **e)
{
    Efr32State *s = EFR32BG22_MACHINE(o);

    g_free(s->flash_overlay);
    s->flash_overlay = g_strdup(v);
}

static char *devinfo_get(Object *o, Error **e)
{
    return g_strdup(EFR32BG22_MACHINE(o)->devinfo);
}
static void devinfo_set(Object *o, const char *v, Error **e)
{
    Efr32State *s = EFR32BG22_MACHINE(o);

    g_free(s->devinfo);
    s->devinfo = g_strdup(v);
}
static char *analog_get(Object *o, Error **e)
{
    return g_strdup(EFR32BG22_MACHINE(o)->analog);
}
static void analog_set(Object *o, const char *v, Error **e)
{
    Efr32State *s = EFR32BG22_MACHINE(o);

    g_free(s->analog);
    s->analog = g_strdup(v);
}
static char *board_get(Object *o, Error **e)
{
    return g_strdup(EFR32BG22_MACHINE(o)->board);
}
static void board_set(Object *o, const char *v, Error **e)
{
    Efr32State *s = EFR32BG22_MACHINE(o);

    g_free(s->board);
    s->board = g_strdup(v);
}
static char *userdata_get(Object *o, Error **e)
{
    return g_strdup(EFR32BG22_MACHINE(o)->userdata);
}
static void userdata_set(Object *o, const char *v, Error **e)
{
    Efr32State *s = EFR32BG22_MACHINE(o);

    g_free(s->userdata);
    s->userdata = g_strdup(v);
}

static bool *efr32_core_clocks_field(Object *o)
{
    if (object_dynamic_cast(o, TYPE_EFR32BG22_DUAL_MACHINE)) {
        return &EFR32BG22_DUAL_MACHINE(o)->core_clocks;
    }
    return &EFR32BG22_MACHINE(o)->core_clocks;
}

static bool efr32_core_clocks_get(Object *o, Error **errp)
{
    return *efr32_core_clocks_field(o);
}

static void efr32_core_clocks_set(Object *o, bool value, Error **errp)
{
    *efr32_core_clocks_field(o) = value;
}

static void efr32_machine_instance_init(Object *obj)
{
    Efr32State *s = EFR32BG22_MACHINE(obj);

    /*
     * `flash` deliberately remains a raw-image property.  Slot images need
     * both offsets: one for their physical storage, one for the reset vector
     * table that a real bootloader would select before handing them control.
     */
    object_property_add_uint32_ptr(obj, "flash-offset", &s->flash_offset,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "flash-overlay-offset",
                                   &s->flash_overlay_offset,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "boot-offset", &s->boot_offset,
                                   OBJ_PROP_FLAG_READWRITE);
    /* Each core at its own clock under -icount (efr32_cpu_clocks_update). */
    s->core_clocks = true;
    object_property_add_bool(obj, "core-clocks", efr32_core_clocks_get,
                             efr32_core_clocks_set);
}

static void mc(ObjectClass *o, void *d)
{
    MachineClass *m = MACHINE_CLASS(o);
    IDAUInterfaceClass *i = IDAU_INTERFACE_CLASS(o);
    static const char *const cpu[] = {ARM_CPU_TYPE_NAME("cortex-m33"), NULL};
    i->check = idau;
    m->desc = "Silicon Labs EFR32BG22 (Cortex-M33), native peripherals";
    m->init = init;
    m->reset = efr32_machine_reset;
    m->valid_cpu_types = cpu;
    m->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    m->default_ram_size = EFR32_FLASH_MAX_SIZE;
    m->min_cpus = 2;
    m->max_cpus = 2;
    m->default_cpus = 2;
    m->ignore_memory_transaction_failures = false;
    object_class_property_add_str(o, "flash", image_get, image_set);
    object_class_property_add_str(o, "flash-overlay", flash_overlay_get,
                                  flash_overlay_set);
    /*
     * Part identity: raw DEVINFO (0x0FE08000) and USERDATA (0x0FE00000)
     * page images read from a chip.  Flash and RAM follow DEVINFO.MSIZE.
     * Without them the board is the BGM220PC22HNA2 explorer-kit module.
     */
    object_class_property_add_str(o, "devinfo", devinfo_get, devinfo_set);
    object_class_property_add_str(o, "userdata", userdata_get, userdata_set);
    /* Board analog inputs for the IADC: "PD02=968,..." in millivolts. */
    object_class_property_add_str(o, "analog", analog_get, analog_set);
    /*
     * The board around the SoC: "owo-suit" adds the suit's control-unit
     * parts and output stage on the SoC's buses (hw/misc/owo-suit.c);
     * tune it with -global owo-suit.<property>=<value>.
     */
    object_class_property_add_str(o, "board", board_get, board_set);
}
static const TypeInfo machine_info = {
    .name = TYPE_EFR32BG22_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Efr32State),
    .instance_init = efr32_machine_instance_init,
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
        s->soc[index].endpoint_id = index;
        object_property_set_bool(OBJECT(&s->soc[index]), "core-clocks",
                                 s->core_clocks, &error_fatal);
        object_property_set_link(OBJECT(&s->soc[index]), "memory",
                                 OBJECT(&s->memory[index]), &error_fatal);
        object_property_set_link(OBJECT(&s->soc[index]), "rf-medium",
                                 OBJECT(s->rf_medium), &error_fatal);
        object_property_set_link(OBJECT(&s->soc[index]), "idau", OBJECT(m),
                                 &error_fatal);
        if (s->devinfo[index]) {
            object_property_set_str(OBJECT(&s->soc[index]), "devinfo",
                                    s->devinfo[index], &error_fatal);
        }
        if (s->userdata[index]) {
            object_property_set_str(OBJECT(&s->soc[index]), "userdata",
                                    s->userdata[index], &error_fatal);
        }
        if (s->analog[index]) {
            object_property_set_str(OBJECT(&s->soc[index]), "analog",
                                    s->analog[index], &error_fatal);
        }
        sysbus_realize(SYS_BUS_DEVICE(&s->soc[index]), &error_fatal);
        if (s->image[index]) {
            gchar *contents;
            gsize length;

            if (!g_file_get_contents(s->image[index], &contents, &length,
                                     NULL) ||
                !efr32_fits_flash(&s->soc[index], 0, length)) {
                error_report("efr32bg22-dual: cannot load flash image %s",
                             s->image[index]);
                exit(EXIT_FAILURE);
            }
            memcpy(memory_region_get_ram_ptr(&s->soc[index].flash), contents,
                   length);
            g_free(contents);
        }
        qemu_register_reset(efr32_cpu_reset, s->soc[index].armv7m.cpu);
        qemu_register_reset(efr32_seq_cpu_reset,
                            s->soc[index].seq_armv7m.cpu);
        if (s->board[index] && !strcmp(s->board[index], TYPE_OWO_SUIT)) {
            gchar *child = g_strdup_printf("suit%u", index);
            DeviceState *board = qdev_new(TYPE_OWO_SUIT);

            object_property_add_child(OBJECT(m), child, OBJECT(board));
            object_property_set_link(OBJECT(board), "soc",
                                     OBJECT(&s->soc[index]), &error_fatal);
            qdev_realize_and_unref(board, NULL, &error_fatal);
            s->soc[index].periph.analog_source = OBJECT(board);
            g_free(child);
        } else if (s->board[index]) {
            error_report("efr32bg22-dual: unknown board '%s' (known: %s)",
                         s->board[index], TYPE_OWO_SUIT);
            exit(EXIT_FAILURE);
        }
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

static char *dual_image0_get(Object *o, Error **errp)
{
    return g_strdup(EFR32BG22_DUAL_MACHINE(o)->image[0]);
}

static void dual_image0_set(Object *o, const char *value, Error **errp)
{
    Efr32DualState *s = EFR32BG22_DUAL_MACHINE(o);

    g_free(s->image[0]);
    s->image[0] = g_strdup(value);
}

static char *dual_image1_get(Object *o, Error **errp)
{
    return g_strdup(EFR32BG22_DUAL_MACHINE(o)->image[1]);
}

static void dual_image1_set(Object *o, const char *value, Error **errp)
{
    Efr32DualState *s = EFR32BG22_DUAL_MACHINE(o);

    g_free(s->image[1]);
    s->image[1] = g_strdup(value);
}

/* Per-SoC board files of the dual machine: devinfo, userdata, analog 0/1. */
static char **efr32_dual_identity_field(Object *o, const char *name)
{
    Efr32DualState *s = EFR32BG22_DUAL_MACHINE(o);
    unsigned index = name[strlen(name) - 1] - '0';

    if (g_str_has_prefix(name, "devinfo")) {
        return &s->devinfo[index];
    }
    if (g_str_has_prefix(name, "analog")) {
        return &s->analog[index];
    }
    if (g_str_has_prefix(name, "board")) {
        return &s->board[index];
    }
    return &s->userdata[index];
}

static char *dual_identity_get(Object *o, Error **errp, const char *name)
{
    return g_strdup(*efr32_dual_identity_field(o, name));
}

static void dual_identity_set(Object *o, const char *value, Error **errp,
                              const char *name)
{
    char **field = efr32_dual_identity_field(o, name);

    g_free(*field);
    *field = g_strdup(value);
}

#define EFR32_DUAL_IDENTITY_ACCESSORS(prop)                                  \
    static char *dual_##prop##_get(Object *o, Error **errp)                  \
    {                                                                        \
        return dual_identity_get(o, errp, #prop);                            \
    }                                                                        \
    static void dual_##prop##_set(Object *o, const char *v, Error **errp)    \
    {                                                                        \
        dual_identity_set(o, v, errp, #prop);                                \
    }
EFR32_DUAL_IDENTITY_ACCESSORS(devinfo0)
EFR32_DUAL_IDENTITY_ACCESSORS(devinfo1)
EFR32_DUAL_IDENTITY_ACCESSORS(userdata0)
EFR32_DUAL_IDENTITY_ACCESSORS(userdata1)
EFR32_DUAL_IDENTITY_ACCESSORS(analog0)
EFR32_DUAL_IDENTITY_ACCESSORS(analog1)
EFR32_DUAL_IDENTITY_ACCESSORS(board0)
EFR32_DUAL_IDENTITY_ACCESSORS(board1)

static void efr32_dual_machine_instance_init(Object *obj)
{
    Efr32DualState *s = EFR32BG22_DUAL_MACHINE(obj);

    s->core_clocks = true;
    object_property_add_bool(obj, "core-clocks", efr32_core_clocks_get,
                             efr32_core_clocks_set);
}

static void dual_mc(ObjectClass *o, void *d)
{
    MachineClass *m = MACHINE_CLASS(o);
    IDAUInterfaceClass *i = IDAU_INTERFACE_CLASS(o);
    static const char *const cpu[] = {ARM_CPU_TYPE_NAME("cortex-m33"), NULL};

    i->check = idau;
    m->desc = "EFR32BG22 dual-MCU RF test topology";
    m->init = dual_init;
    m->reset = efr32_dual_machine_reset;
    m->valid_cpu_types = cpu;
    m->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    m->default_ram_size = 2 * EFR32_FLASH_MAX_SIZE;
    m->min_cpus = 4;
    m->max_cpus = 4;
    m->default_cpus = 4;
    m->ignore_memory_transaction_failures = false;
    object_class_property_add_str(o, "flash0", dual_image0_get,
                                  dual_image0_set);
    object_class_property_add_str(o, "flash1", dual_image1_get,
                                  dual_image1_set);
    object_class_property_add_str(o, "devinfo0", dual_devinfo0_get,
                                  dual_devinfo0_set);
    object_class_property_add_str(o, "devinfo1", dual_devinfo1_get,
                                  dual_devinfo1_set);
    object_class_property_add_str(o, "userdata0", dual_userdata0_get,
                                  dual_userdata0_set);
    object_class_property_add_str(o, "userdata1", dual_userdata1_get,
                                  dual_userdata1_set);
    object_class_property_add_str(o, "analog0", dual_analog0_get,
                                  dual_analog0_set);
    object_class_property_add_str(o, "analog1", dual_analog1_get,
                                  dual_analog1_set);
    object_class_property_add_str(o, "board0", dual_board0_get, dual_board0_set);
    object_class_property_add_str(o, "board1", dual_board1_get, dual_board1_set);
}

static const TypeInfo dual_machine_info = {
    .name = TYPE_EFR32BG22_DUAL_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Efr32DualState),
    .instance_init = efr32_dual_machine_instance_init,
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
