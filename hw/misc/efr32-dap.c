/*
 * EFR32 Series 2 debug access port, as a probe sees it on SWCLK/SWDIO:
 *  - SWJ-DP in SW-DP mode (ADIv5.2 / DPv2): line reset, JTAG-to-SWD select,
 *    packet request/ACK/data phases, DPIDR, CTRL/STAT, SELECT, RDBUFF,
 *    ABORT and the sticky-error FAULT rules;
 *  - AP0: the Cortex-M33 AHB-AP (a MEM-AP into the M33 bus);
 *  - AP1: the Secure Engine's Debug Challenge Interface (DCI), a MEM-AP
 *    whose bus holds the DCI mailbox registers.
 *
 * Protocol references: Arm Debug Interface Architecture Specification
 * ADIv5.0 to ADIv5.2 (IHI0031), chapters B2 (DP), B4 (SW-DP) and C2 (MEM-AP);
 * DCI register map and commands as used by OpenOCD's
 * tcl/target/silabs/series2.cfg (Silicon Labs' public Series 2 secure debug
 * documentation).  Identification values and the DCI status are silicon
 * captures, cited in owo-open-fw docs/debug-port-model.md.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/misc/efr32-dap.h"
#include "hw/misc/swd.h"
#include "hw/qdev-properties.h"
#include "sysemu/runstate.h"

/* SW-DP wire protocol (ADIv5.2 B4.2, B4.3) */
#define SWD_LINE_RESET_ONES 50
#define SWD_JTAG_TO_SWD_SELECT 0xe79e
#define SWD_REQUEST_BITS 8
#define SWD_ACK_BITS 3
#define SWD_DATA_BITS 33          /* 32 data bits and even parity */
#define SWD_REQ_START (1u << 0)
#define SWD_REQ_APNDP (1u << 1)
#define SWD_REQ_RNW (1u << 2)
#define SWD_REQ_A_SHIFT 3         /* A[3:2] */
#define SWD_REQ_A_MASK 0x3u
#define SWD_REQ_PARITY_SHIFT 5
#define SWD_REQ_PARITY_FIELD 0x0fu /* APnDP, RnW, A2, A3 (bits 1..4) */
#define SWD_REQ_STOP (1u << 6)
#define SWD_REQ_PARK (1u << 7)
#define SWD_ACK_OK 0x1u
#define SWD_ACK_WAIT 0x2u
#define SWD_ACK_FAULT 0x4u
#define SWD_DATA_PARITY_BIT 32

/* DP registers (ADIv5.2 B2.2) */
#define DP_DPIDR 0x0
#define DP_ABORT 0x0
#define DP_CTRL_STAT 0x4
#define DP_SELECT 0x8
#define DP_RESEND 0x8
#define DP_RDBUFF 0xc
#define DP_BANK_CTRL_STAT 0x0
#define DP_BANK_DLCR 0x1
#define DP_BANK_TARGETID 0x2
#define DP_BANK_DLPIDR 0x3
#define SELECT_DPBANKSEL_MASK 0xfu
#define SELECT_APBANKSEL_MASK 0xf0u
#define SELECT_APSEL_SHIFT 24
#define ABORT_STKCMPCLR (1u << 1)
#define ABORT_STKERRCLR (1u << 2)
#define ABORT_WDERRCLR (1u << 3)
#define ABORT_ORUNERRCLR (1u << 4)
#define CTRL_STICKYORUN (1u << 1)
#define CTRL_STICKYCMP (1u << 4)
#define CTRL_STICKYERR (1u << 5)
#define CTRL_READOK (1u << 6)
#define CTRL_WDATAERR (1u << 7)
#define CTRL_CDBGRSTREQ (1u << 26)
#define CTRL_CDBGRSTACK (1u << 27)
#define CTRL_CDBGPWRUPREQ (1u << 28)
#define CTRL_CDBGPWRUPACK (1u << 29)
#define CTRL_CSYSPWRUPREQ (1u << 30)
#define CTRL_CSYSPWRUPACK (1u << 31)
#define CTRL_STICKY (CTRL_STICKYORUN | CTRL_STICKYCMP | CTRL_STICKYERR | \
                     CTRL_WDATAERR)
/* ORUNDETECT, TRNMODE, MASKLANE, TRNCNT and the three request bits */
#define CTRL_WRITABLE ((0x00fffffdu & ~CTRL_STICKY & ~CTRL_READOK) | \
                       CTRL_CDBGRSTREQ | CTRL_CDBGPWRUPREQ | CTRL_CSYSPWRUPREQ)
/*
 * DPIDR read from the EFR32BG22 by OpenOCD ("SWD DPIDR 0x6ba02477", EK and
 * suit): DPv2, Arm designer, part 0xba, revision 6.
 */
#define EFR32_DPIDR 0x6ba02477u
/*
 * TARGETID and DLPIDR were not captured; only their read-as-one bit 0
 * (DESIGNER[0] is RAO, DLPIDR PROTVSN = 1 for SWD protocol version 2) is
 * certain (TODO.md).
 */
#define EFR32_TARGETID 0x00000001u
#define EFR32_DLPIDR 0x00000001u

/* MEM-AP registers (ADIv5.2 C2.6) */
#define AP_CSW 0x00
#define AP_TAR 0x04
#define AP_TAR_HI 0x08
#define AP_DRW 0x0c
#define AP_BD0 0x10
#define AP_BD3 0x1c
#define AP_BASE_HI 0xf0
#define AP_CFG 0xf4
#define AP_BASE 0xf8
#define AP_IDR 0xfc
#define AP_BD_ALIGN_MASK 0xfu
#define CSW_SIZE_MASK 0x7u
#define CSW_SIZE_8 0x0u
#define CSW_SIZE_16 0x1u
#define CSW_SIZE_32 0x2u
#define CSW_ADDRINC_SHIFT 4
#define CSW_ADDRINC_MASK (0x3u << CSW_ADDRINC_SHIFT)
#define CSW_ADDRINC_SINGLE (0x1u << CSW_ADDRINC_SHIFT)
#define CSW_ADDRINC_PACKED (0x2u << CSW_ADDRINC_SHIFT)
#define CSW_DEVICEEN (1u << 6)
#define CSW_SPIDEN (1u << 23)
#define CSW_HPROT_PRIV (1u << 25)      /* HPROT[1]: privileged */
#define CSW_HNONSEC (1u << 30)         /* AHB5: Non-secure transfer */
#define CSW_RW_MASK (CSW_SIZE_MASK | CSW_ADDRINC_MASK | 0x7f000f00u | \
                     (1u << 31))
/* TAR auto-increment is only guaranteed within a 1 KiB block (C2.2.2) */
#define TAR_AUTOINC_WRAP 0x400u
/*
 * AP0 identification: IDR 0x84770001 read from silicon; BASE points to the
 * Cortex-M33 PPB ROM table (0xe00ff000, format and present bits).  The
 * EFR32BG22's BASE was not captured (TODO.md).
 */
#define EFR32_AHBAP_IDR 0x84770001u
#define EFR32_AHBAP_BASE 0xe00ff003u
/* DCI AP identification was not captured; IDR reads as zero (TODO.md). */
#define EFR32_DCIAP_IDR 0x0u
#define EFR32_DAP_AP_M33 0
#define EFR32_DAP_AP_DCI 1

/* DCI mailbox on AP1's bus (OpenOCD silabs/series2.cfg _DCI_REG_*) */
#define DCI_BASE 0x1000
#define DCI_SIZE 0x100
#define DCI_BUS_SIZE 0x10000
#define DCI_WDATA 0x00
#define DCI_RDATA 0x04
#define DCI_STATUS 0x08
#define DCI_ID 0xfc
#define DCI_ID_VALUE 0xdc11du
#define DCI_STATUS_WPENDING (1u << 0)
#define DCI_STATUS_RDATAVALID (1u << 8)
/* Commands; the first word of a command is its length in bytes */
#define DCI_CMD_GET_STATUS 0xfe010000u
#define DCI_CMD_DEVICE_ERASE 0x430f0000u
/* GET_STATUS response: length word, then status words */
#define DCI_STATUS_WORDS 4
#define DCI_STATUS_DEBUGLOCK_INDEX 3
#define DCI_DEBUGLOCK_DEVICE_ERASE (1u << 1)
#define DCI_ERASED_BYTE 0xff
#define DCI_ERASE_CHUNK 4096

/* --- Reset window and system reset ------------------------------------ */

static bool dap_ap_blocked(Efr32DapState *s)
{
    return s->in_sysreset ||
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->blackout_until_ns;
}

/*
 * The SoC signals that a system reset (SYSRESETREQ, RM 12.3.2 SYSREQ) has
 * been requested; the SoC itself records RSTCAUSE and requests the reset.
 * AP accesses fail until the reset has been performed (reset hold below).
 */
static void dap_sysresetreq(void *opaque, int n, int level)
{
    Efr32DapState *s = opaque;

    if (level) {
        s->in_sysreset = true;
    }
}

/*
 * The debug-port wedge (silicon, owo-open-fw docs/radio-tables.md SS7.80
 * and TODO.md, EK, 2026-10-31): an image whose sequencer enabled RAC clock
 * retiming (XORETIMECTRL and DIGCLKRETIMECTRL ENRETIME, stock's RXSEARCH
 * limits) left the debug port wedged within milliseconds of every boot:
 * DPIDR still read 0x6ba02477, but debug power-up, the AHB-AP and the SE's
 * DCI AP no longer answered.  It came back after every pin reset (the image
 * enabled retiming again at its first scan window); in the window after
 * nRESET was released the AHB-AP answered (scratch/silicon-rx/
 * recover2-fpb3.log: DHCSR halt and an FPB comparator armed on the third
 * back-to-back attempt, core halted, recovery done).
 *
 * The mechanism is unknown.  Modelled is what a probe sees: DPIDR, CTRL/STAT
 * and the other DP registers still answer; CDBGPWRUPACK/CSYSPWRUPACK never
 * follow their requests; every AP access, AP0 and AP1 alike, is refused.
 * [I] The refusal is a FAULT with STICKYERR, as for a powered-down debug
 * domain; the real ack was not recorded.  [I] Once wedged, the port stays
 * wedged until a reset (the SoC's RAC is reset with it), whatever the
 * firmware writes afterwards; unmeasured.  The SoC decides when the wedge
 * starts (efr32bg22.c, efr32_rac_retime_update).
 */
static void dap_debug_wedge(void *opaque, int n, int level)
{
    Efr32DapState *s = opaque;

    if (level) {
        s->wedged = true;
    }
}

/*
 * nRESET held low by the probe: the SW-DP does not answer at all.  OpenOCD
 * reported "Error connecting DP: cannot read IDR" with nRESET held by the
 * reset button (scratch/recover-step1.log) and by the J-Link
 * (recover2-srsttest.log, recover2-auto.log, recover2-fpb2.log,
 * recover2-fpb3.log), on the EK; docs/hardware-bringup.md: SWD disappears
 * while reset is asserted.  [I] The DP comes out of the hold as after
 * power-up: SWJ-DP in JTAG mode, DP registers cleared, so a probe selects
 * SWD and line-resets again (OpenOCD's dap init does both).
 */
static void efr32_dap_set_nreset(SWDTarget *t, bool asserted)
{
    Efr32DapState *s = EFR32_DAP(t);

    if (asserted == s->nreset_asserted) {
        return;
    }
    s->nreset_asserted = asserted;
    if (asserted) {
        s->phase = EFR32_SWD_JTAG;
        s->select_shift = 0;
        s->ones = 0;
        s->ctrl_stat = 0;
        s->select = 0;
        s->rdbuff = 0;
        s->awaiting_dpidr = false;
    }
    qemu_set_irq(s->nreset_out, asserted);
}

/* --- DCI ------------------------------------------------------------- */

static void dci_respond(Efr32DapState *s, const uint32_t *words, unsigned n)
{
    memcpy(s->dci_resp, words, n * sizeof(*words));
    s->dci_resp_len = n;
    s->dci_resp_pos = 0;
}

static void dci_device_erase(Efr32DapState *s)
{
    uint8_t buf[DCI_ERASE_CHUNK];
    uint64_t size = memory_region_size(s->flash);
    MemTxAttrs attrs = { .secure = 1 };

    memset(buf, DCI_ERASED_BYTE, sizeof(buf));
    for (uint64_t off = 0; off < size; off += sizeof(buf)) {
        address_space_write(&s->ap0_as, off, attrs, buf,
                            MIN(sizeof(buf), size - off));
    }
    /* "debug should be available again after a reset" (series2.cfg) */
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void dci_execute(Efr32DapState *s)
{
    uint32_t command = s->dci_cmd[1];

    switch (command) {
    case DCI_CMD_GET_STATUS: {
        uint32_t resp[1 + DCI_STATUS_WORDS] = {
            [0] = sizeof(resp),
        };

        /*
         * Silicon (suit and EK): Debug lock (config) disabled, device erase
         * enabled, secure debug disabled, debug lock (hw) disabled.  The
         * other status words were not captured and read as zero (TODO.md).
         */
        if (s->dci_device_erase_enabled) {
            resp[1 + DCI_STATUS_DEBUGLOCK_INDEX] |= DCI_DEBUGLOCK_DEVICE_ERASE;
        }
        dci_respond(s, resp, ARRAY_SIZE(resp));
        break;
    }
    case DCI_CMD_DEVICE_ERASE:
        if (s->dci_device_erase_enabled) {
            dci_device_erase(s);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "efr32-dap: DCI command 0x%08x\n", command);
        break;
    }
}

static uint64_t dci_read(void *opaque, hwaddr offset, unsigned size)
{
    Efr32DapState *s = opaque;

    switch (offset) {
    case DCI_RDATA:
        if (s->dci_resp_pos < s->dci_resp_len) {
            return s->dci_resp[s->dci_resp_pos++];
        }
        return 0;
    case DCI_STATUS:
        return s->dci_resp_pos < s->dci_resp_len ? DCI_STATUS_RDATAVALID : 0;
    case DCI_ID:
        return DCI_ID_VALUE;
    default:
        return 0;
    }
}

static void dci_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    Efr32DapState *s = opaque;

    if (offset != DCI_WDATA || s->dci_resp_pos < s->dci_resp_len) {
        return;
    }
    if (s->dci_cmd_words < ARRAY_SIZE(s->dci_cmd)) {
        s->dci_cmd[s->dci_cmd_words] = value;
    }
    s->dci_cmd_words++;
    if (s->dci_cmd_words * sizeof(uint32_t) >= s->dci_cmd[0]) {
        dci_execute(s);
        s->dci_cmd_words = 0;
    }
}

static const MemoryRegionOps dci_ops = {
    .read = dci_read,
    .write = dci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* --- MEM-AP ---------------------------------------------------------- */

static unsigned mem_ap_size_bytes(uint32_t csw)
{
    switch (csw & CSW_SIZE_MASK) {
    case CSW_SIZE_8:
        return 1;
    case CSW_SIZE_16:
        return 2;
    case CSW_SIZE_32:
        return 4;
    default:
        return 0;
    }
}

static MemTxAttrs mem_ap_attrs(Efr32MemAp *ap)
{
    return (MemTxAttrs) {
        .secure = !(ap->csw & CSW_HNONSEC),
        .user = !(ap->csw & CSW_HPROT_PRIV),
    };
}

static bool mem_ap_transfer(Efr32MemAp *ap, uint32_t addr, unsigned bytes,
                            bool write, uint32_t *data)
{
    uint8_t buf[4];
    unsigned lane = (addr & 3) * 8;
    MemTxResult r;

    if (!bytes || (addr & (bytes - 1))) {
        return false;
    }
    if (write) {
        stn_le_p(buf, bytes, *data >> lane);
        r = address_space_write(ap->as, addr, mem_ap_attrs(ap), buf, bytes);
    } else {
        r = address_space_read(ap->as, addr, mem_ap_attrs(ap), buf, bytes);
        *data = (uint32_t)ldn_le_p(buf, bytes) << lane;
    }
    return r == MEMTX_OK;
}

static void mem_ap_autoinc(Efr32MemAp *ap, unsigned bytes)
{
    if ((ap->csw & CSW_ADDRINC_MASK) == CSW_ADDRINC_SINGLE) {
        ap->tar = (ap->tar & ~(TAR_AUTOINC_WRAP - 1)) |
                  ((ap->tar + bytes) & (TAR_AUTOINC_WRAP - 1));
    }
}

/* Returns false on a bus error. */
static bool mem_ap_access(Efr32DapState *s, Efr32MemAp *ap, unsigned reg,
                          bool write, uint32_t *data)
{
    unsigned bytes;
    bool ok = true;

    switch (reg) {
    case AP_CSW:
        if (write) {
            uint32_t csw = *data & CSW_RW_MASK;

            /* Packed transfers are not implemented: AddrInc 0b10 -> off */
            if ((csw & CSW_ADDRINC_MASK) == CSW_ADDRINC_PACKED) {
                csw &= ~CSW_ADDRINC_MASK;
            }
            ap->csw = csw;
        } else {
            *data = ap->csw | CSW_DEVICEEN | CSW_SPIDEN;
        }
        return true;
    case AP_TAR:
        if (write) {
            ap->tar = *data;
        } else {
            *data = ap->tar;
        }
        return true;
    case AP_DRW:
        bytes = mem_ap_size_bytes(ap->csw);
        ok = mem_ap_transfer(ap, ap->tar, bytes, write, data);
        mem_ap_autoinc(ap, bytes);
        return ok;
    case AP_BD0 ... AP_BD3:
        return mem_ap_transfer(ap, (ap->tar & ~AP_BD_ALIGN_MASK) +
                               (reg - AP_BD0), sizeof(uint32_t), write, data);
    case AP_CFG:
    case AP_BASE_HI:
    case AP_TAR_HI:
        if (!write) {
            *data = reg == AP_CFG ? ap->cfg : 0;
        }
        return true;
    case AP_BASE:
        if (!write) {
            *data = ap->base;
        }
        return true;
    case AP_IDR:
        if (!write) {
            *data = ap->idr;
        }
        return true;
    default:
        if (!write) {
            *data = 0;
        }
        return true;
    }
}

/* --- DP -------------------------------------------------------------- */

static uint32_t dap_ctrl_stat(Efr32DapState *s)
{
    uint32_t value = s->ctrl_stat;

    /* Wedged: debug power-up no longer answers (dap_debug_wedge) */
    if (s->wedged) {
        return value;
    }
    /* Power and reset acknowledges follow their requests at once */
    if (value & CTRL_CDBGPWRUPREQ) {
        value |= CTRL_CDBGPWRUPACK;
    }
    if (value & CTRL_CSYSPWRUPREQ) {
        value |= CTRL_CSYSPWRUPACK;
    }
    if (value & CTRL_CDBGRSTREQ) {
        value |= CTRL_CDBGRSTACK;
    }
    return value;
}

static Efr32MemAp *dap_selected_ap(Efr32DapState *s)
{
    unsigned apsel = s->select >> SELECT_APSEL_SHIFT;

    return apsel < EFR32_DAP_NUM_APS ? &s->ap[apsel] : NULL;
}

static unsigned dap_ap_reg(Efr32DapState *s, unsigned a)
{
    return (s->select & SELECT_APBANKSEL_MASK) | a;
}

/* An AP access cannot start: FAULT and STICKYERR (ADIv5.2 B4.2.3). */
static bool dap_ap_unavailable(Efr32DapState *s)
{
    if (!(s->ctrl_stat & CTRL_CDBGPWRUPREQ) || dap_ap_blocked(s) ||
        s->wedged) {
        s->ctrl_stat |= CTRL_STICKYERR;
        return true;
    }
    return false;
}

static uint8_t dap_read(Efr32DapState *s, bool apndp, unsigned a,
                        uint32_t *data)
{
    bool sticky = s->ctrl_stat & CTRL_STICKY;

    if (!apndp) {
        if (sticky && a != DP_DPIDR && a != DP_CTRL_STAT) {
            return SWD_ACK_FAULT;
        }
        switch (a) {
        case DP_DPIDR:
            *data = EFR32_DPIDR;
            s->awaiting_dpidr = false;
            break;
        case DP_CTRL_STAT:
            switch (s->select & SELECT_DPBANKSEL_MASK) {
            case DP_BANK_CTRL_STAT:
                *data = dap_ctrl_stat(s);
                break;
            case DP_BANK_TARGETID:
                *data = EFR32_TARGETID;
                break;
            case DP_BANK_DLPIDR:
                *data = EFR32_DLPIDR;
                break;
            default:
                *data = 0;
                break;
            }
            break;
        case DP_RESEND:
        case DP_RDBUFF:
            *data = s->rdbuff;
            break;
        }
        return SWD_ACK_OK;
    }

    if (sticky) {
        return SWD_ACK_FAULT;
    }
    if (dap_ap_unavailable(s)) {
        return SWD_ACK_FAULT;
    }
    /* AP reads are posted: return the previous result (B4.2.2) */
    *data = s->rdbuff;
    {
        Efr32MemAp *ap = dap_selected_ap(s);
        uint32_t value = 0;
        bool ok = !ap || mem_ap_access(s, ap, dap_ap_reg(s, a), false, &value);

        s->rdbuff = value;
        if (ok) {
            s->ctrl_stat |= CTRL_READOK;
        } else {
            s->ctrl_stat = (s->ctrl_stat & ~CTRL_READOK) | CTRL_STICKYERR;
        }
    }
    return SWD_ACK_OK;
}

static uint8_t dap_write_ack(Efr32DapState *s, bool apndp, unsigned a)
{
    if (s->ctrl_stat & CTRL_STICKY) {
        if (apndp || a != DP_ABORT) {
            return SWD_ACK_FAULT;
        }
    }
    if (apndp && dap_ap_unavailable(s)) {
        return SWD_ACK_FAULT;
    }
    return SWD_ACK_OK;
}

static void dap_write(Efr32DapState *s, bool apndp, unsigned a,
                      uint32_t data)
{
    if (apndp) {
        Efr32MemAp *ap = dap_selected_ap(s);

        if (ap && !mem_ap_access(s, ap, dap_ap_reg(s, a), true, &data)) {
            s->ctrl_stat |= CTRL_STICKYERR;
        }
        return;
    }
    switch (a) {
    case DP_ABORT:
        if (data & ABORT_STKCMPCLR) {
            s->ctrl_stat &= ~CTRL_STICKYCMP;
        }
        if (data & ABORT_STKERRCLR) {
            s->ctrl_stat &= ~CTRL_STICKYERR;
        }
        if (data & ABORT_WDERRCLR) {
            s->ctrl_stat &= ~CTRL_WDATAERR;
        }
        if (data & ABORT_ORUNERRCLR) {
            s->ctrl_stat &= ~CTRL_STICKYORUN;
        }
        break;
    case DP_CTRL_STAT:
        if ((s->select & SELECT_DPBANKSEL_MASK) == DP_BANK_CTRL_STAT) {
            s->ctrl_stat = (s->ctrl_stat & (CTRL_STICKY | CTRL_READOK)) |
                           (data & (CTRL_WRITABLE));
        }
        break;
    case DP_SELECT:
        s->select = data;
        break;
    default:
        /* TARGETSEL: single-drop DP, ignored */
        break;
    }
}

/* --- SWD wire -------------------------------------------------------- */

static bool swd_even_parity(uint64_t value)
{
    return __builtin_parityll(value);
}

static void swd_decode_request(Efr32DapState *s)
{
    uint8_t req = s->request;
    bool apndp = req & SWD_REQ_APNDP;
    bool rnw = req & SWD_REQ_RNW;
    unsigned a = ((req >> SWD_REQ_A_SHIFT) & SWD_REQ_A_MASK) << 2;
    bool parity = (req >> SWD_REQ_PARITY_SHIFT) & 1;
    uint32_t data = 0;

    if (!(req & SWD_REQ_START) || (req & SWD_REQ_STOP) ||
        !(req & SWD_REQ_PARK) ||
        parity != swd_even_parity((req >> 1) & SWD_REQ_PARITY_FIELD)) {
        s->phase = EFR32_SWD_LOCKOUT;
        return;
    }
    /* After a line reset only a DPIDR read is answered (B4.3.3) */
    if (s->awaiting_dpidr && (apndp || !rnw || a != DP_DPIDR)) {
        s->phase = EFR32_SWD_LOCKOUT;
        return;
    }
    if (rnw) {
        s->ack = dap_read(s, apndp, a, &data);
        s->shift = data |
                   ((uint64_t)swd_even_parity(data) << SWD_DATA_PARITY_BIT);
    } else {
        s->ack = dap_write_ack(s, apndp, a);
    }
    s->phase = EFR32_SWD_TURN_TO_ACK;
}

static bool swd_target_drives(Efr32DapState *s)
{
    return s->phase == EFR32_SWD_ACK || s->phase == EFR32_SWD_READ_DATA;
}

static bool efr32_dap_peek(SWDTarget *t, bool host_drives, bool host_bit)
{
    Efr32DapState *s = EFR32_DAP(t);

    if (s->nreset_asserted) {
        /* Held in reset: the target never drives SWDIO */
        return host_drives ? host_bit : true;
    }
    if (s->phase == EFR32_SWD_ACK) {
        return (s->ack >> s->bit_index) & 1;
    }
    if (s->phase == EFR32_SWD_READ_DATA) {
        return (s->shift >> s->bit_index) & 1;
    }
    /* SWDIO has an on-chip pull-up (RM 7.3.1) */
    return host_drives ? host_bit : true;
}

static void efr32_dap_clock(SWDTarget *t, bool host_drives, bool host_bit)
{
    Efr32DapState *s = EFR32_DAP(t);
    bool bit = host_drives ? host_bit : true;
    bool rnw;

    if (s->nreset_asserted) {
        return;
    }
    if (!swd_target_drives(s) && host_drives) {
        s->ones = host_bit ? s->ones + 1 : 0;
    }

    if (s->phase == EFR32_SWD_JTAG) {
        /* SWJ-DP powers up in JTAG mode (RM 7.3.1) */
        s->select_shift = (s->select_shift >> 1) | ((uint16_t)bit << 15);
        if (s->select_shift == SWD_JTAG_TO_SWD_SELECT) {
            s->phase = EFR32_SWD_LOCKOUT;
            s->ones = 0;
        }
        return;
    }
    if (!swd_target_drives(s) && s->ones >= SWD_LINE_RESET_ONES) {
        s->phase = EFR32_SWD_RESET;
        s->awaiting_dpidr = true;
        return;
    }

    switch (s->phase) {
    case EFR32_SWD_RESET:
    case EFR32_SWD_IDLE:
        if (bit) {
            s->request = SWD_REQ_START;
            s->bit_index = 1;
            s->phase = EFR32_SWD_REQUEST;
        } else {
            s->phase = EFR32_SWD_IDLE;
        }
        break;
    case EFR32_SWD_REQUEST:
        s->request |= bit << s->bit_index;
        if (++s->bit_index == SWD_REQUEST_BITS) {
            swd_decode_request(s);
        }
        break;
    case EFR32_SWD_TURN_TO_ACK:
        s->phase = EFR32_SWD_ACK;
        s->bit_index = 0;
        break;
    case EFR32_SWD_ACK:
        if (++s->bit_index == SWD_ACK_BITS) {
            rnw = s->request & SWD_REQ_RNW;
            s->bit_index = 0;
            s->phase = (s->ack == SWD_ACK_OK && rnw) ?
                EFR32_SWD_READ_DATA : EFR32_SWD_TURN_TO_HOST;
        }
        break;
    case EFR32_SWD_READ_DATA:
        if (++s->bit_index == SWD_DATA_BITS) {
            s->ack = 0;
            s->phase = EFR32_SWD_TURN_TO_HOST;
        }
        break;
    case EFR32_SWD_TURN_TO_HOST:
        rnw = s->request & SWD_REQ_RNW;
        if (s->ack == SWD_ACK_OK && !rnw) {
            s->phase = EFR32_SWD_WRITE_DATA;
            s->bit_index = 0;
            s->shift = 0;
        } else {
            s->phase = EFR32_SWD_IDLE;
        }
        break;
    case EFR32_SWD_WRITE_DATA:
        s->shift |= (uint64_t)bit << s->bit_index;
        if (++s->bit_index == SWD_DATA_BITS) {
            uint32_t data = s->shift;
            bool parity = (s->shift >> SWD_DATA_PARITY_BIT) & 1;

            if (parity != swd_even_parity(data)) {
                s->ctrl_stat |= CTRL_WDATAERR;
            } else {
                dap_write(s, s->request & SWD_REQ_APNDP,
                          ((s->request >> SWD_REQ_A_SHIFT) & SWD_REQ_A_MASK)
                          << 2, data);
            }
            s->phase = EFR32_SWD_IDLE;
        }
        break;
    case EFR32_SWD_LOCKOUT:
    case EFR32_SWD_JTAG:
        break;
    }
}

static uint32_t efr32_dap_vtref_mv(SWDTarget *t)
{
    return EFR32_DAP(t)->vtref_mv;
}

/* --- Device ---------------------------------------------------------- */

static void efr32_dap_reset_hold(Object *obj, ResetType type)
{
    Efr32DapState *s = EFR32_DAP(obj);

    /*
     * The DP and the AP registers are in the debug domain and survive a
     * system reset.  The reset itself ends here; AP accesses stay blocked
     * for sysreset-blackout-ns more.
     */
    if (s->in_sysreset) {
        s->in_sysreset = false;
        s->blackout_until_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                               s->sysreset_blackout_ns;
    }
    /* The wedge does not survive the reset (dap_debug_wedge) */
    s->wedged = false;
    /* The Secure Engine reboots: its mailbox is empty again */
    s->dci_cmd_words = 0;
    s->dci_resp_len = 0;
    s->dci_resp_pos = 0;
}

static void efr32_dap_init(Object *obj)
{
    Efr32DapState *s = EFR32_DAP(obj);

    qdev_init_gpio_in_named(DEVICE(obj), dap_sysresetreq, "sysresetreq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), dap_debug_wedge, "debug-wedge", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->nreset_out, "nreset", 1);
    memory_region_init(&s->dci_root, obj, "efr32-dci-bus", DCI_BUS_SIZE);
    memory_region_init_io(&s->dci_regs, obj, &dci_ops, s, "efr32-dci",
                          DCI_SIZE);
    memory_region_add_subregion(&s->dci_root, DCI_BASE, &s->dci_regs);
    s->phase = EFR32_SWD_JTAG;
}

static void efr32_dap_realize(DeviceState *dev, Error **errp)
{
    Efr32DapState *s = EFR32_DAP(dev);

    if (!s->ap0_memory || !s->flash) {
        error_setg(errp, "efr32-dap: 'ap0-memory' and 'flash' must be set");
        return;
    }
    address_space_init(&s->ap0_as, s->ap0_memory, "efr32-ahb-ap");
    address_space_init(&s->dci_as, &s->dci_root, "efr32-dci-ap");
    s->ap[EFR32_DAP_AP_M33] = (Efr32MemAp) {
        .idr = EFR32_AHBAP_IDR,
        .base = EFR32_AHBAP_BASE,
        .as = &s->ap0_as,
    };
    s->ap[EFR32_DAP_AP_DCI] = (Efr32MemAp) {
        .idr = EFR32_DCIAP_IDR,
        .as = &s->dci_as,
    };
}

static Property efr32_dap_properties[] = {
    DEFINE_PROP_LINK("ap0-memory", Efr32DapState, ap0_memory,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("flash", Efr32DapState, flash, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    /* VTref on the EK measured by the OB: 3.318-3.322 V (n = 66 sessions) */
    DEFINE_PROP_UINT32("vtref-mv", Efr32DapState, vtref_mv, 3320),
    /* Unmeasured: zero beyond the reset itself until measured (TODO.md) */
    DEFINE_PROP_UINT64("sysreset-blackout-ns", Efr32DapState,
                       sysreset_blackout_ns, 0),
    DEFINE_PROP_BOOL("dci-device-erase", Efr32DapState,
                     dci_device_erase_enabled, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void efr32_dap_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    SWDTargetClass *swd = SWD_TARGET_CLASS(oc);

    dc->realize = efr32_dap_realize;
    dc->user_creatable = false;
    device_class_set_props(dc, efr32_dap_properties);
    rc->phases.hold = efr32_dap_reset_hold;
    swd->peek = efr32_dap_peek;
    swd->clock = efr32_dap_clock;
    swd->vtref_mv = efr32_dap_vtref_mv;
    swd->set_nreset = efr32_dap_set_nreset;
}

static const TypeInfo efr32_dap_types[] = {
    {
        .name = TYPE_EFR32_DAP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Efr32DapState),
        .instance_init = efr32_dap_init,
        .class_init = efr32_dap_class_init,
        .interfaces = (InterfaceInfo[]) {
            { TYPE_SWD_TARGET },
            { }
        },
    },
};

DEFINE_TYPES(efr32_dap_types)
