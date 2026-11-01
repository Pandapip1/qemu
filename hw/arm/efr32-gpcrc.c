/*
 * EFR32xG22 GPCRC (see efr32-gpcrc.h), from RM rev. 1.2 chapter 14:
 *
 *  - The engine shifts data LSB first (SS14.3.5: input is made "little
 *    endian, LSB-first for CRC calculation"), so its polynomial is held in
 *    reversed form: the fixed IEEE 802.3 polynomial 0x04C11DB7 for CRC32
 *    (POLYSEL 0), reflected 0xEDB88320; for CRC16 (POLYSEL 1) POLY written
 *    reversed, x^0 in bit 15 and x^16 implicit (SS14.3.1, Figure 14.2).
 *    DATA holds the CRC "based on the polynomial in reversed order", DATAREV
 *    the same bit-reversed (32 bits for CRC32, bits 15:0 for CRC16), and
 *    DATABYTEREV byte-swapped ({B0,B1,B2,B3}; {0,0,B0,B1}) (SS14.5.10-12).
 *  - Input: INPUTDATA (32 bits, B0 first), INPUTDATAHWORD (bits 15:0),
 *    INPUTDATABYTE (bits 7:0); BYTEMODE uses the low byte of any write;
 *    BYTEREVERSE swaps the bytes of a word ({B0,B1,B2,B3}) or half-word
 *    ({0,0,B0,B1}) and is ignored for bytes; BITREVERSE reverses the bits of
 *    each byte (SS14.3.5, Table 14.1).
 *  - CMD.INIT loads INIT into DATA; AUTOINIT reloads it after a read of
 *    DATA, DATAREV or DATABYTEREV (SS14.3.3).  EN = 0: input writes do not
 *    update the CRC (SS14.5.2).
 *  - A CRC update takes one clock cycle (SS14.3.2); the bus model's access
 *    cost covers it.
 *  - CTRL mask 0x2710 (AUTOINIT, BYTEREVERSE, BITREVERSE, BYTEMODE,
 *    POLYSEL), IPVERSION reads 0 (efr32bg22_gpcrc.h).
 *  [I] SET/CLR/TGL writes to the input registers feed the written value; to
 *  CMD, any set bit acts.  The RM lists those pages without saying more.
 *
 * Checked against the standard CRC-32 (IEEE) and CRC-16/CCITT vectors in
 * qtest efr32bg22/gpcrc-crc-vectors, through the register sequence of
 * Silicon Labs' sl_hal_gpcrc.c and Zephyr's crc_silabs_gpcrc.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "hw/arm/efr32-gpcrc.h"

#define IPVERSION 0x000u
#define EN 0x004u
#define CTRL 0x008u
#define CMD 0x00cu
#define INIT 0x010u
#define POLY 0x014u
#define INPUTDATA 0x018u
#define INPUTDATAHWORD 0x01cu
#define INPUTDATABYTE 0x020u
#define DATA 0x024u
#define DATAREV 0x028u
#define DATABYTEREV 0x02cu

#define ALIAS_MASK 0x3000u
#define ALIAS_SET 0x1000u
#define ALIAS_CLR 0x2000u
#define ALIAS_TGL 0x3000u

#define EN_EN (1u << 0)
#define CTRL_MASK 0x2710u
#define CTRL_POLYSEL_CRC16 (1u << 4)
#define CTRL_BYTEMODE (1u << 8)
#define CTRL_BITREVERSE (1u << 9)
#define CTRL_BYTEREVERSE (1u << 10)
#define CTRL_AUTOINIT (1u << 13)
#define CMD_INIT (1u << 0)
#define POLY_MASK 0xffffu
#define CRC32_POLY_REVERSED 0xedb88320u

static bool crc16(Efr32Gpcrc *g)
{
    return g->ctrl & CTRL_POLYSEL_CRC16;
}

static uint8_t bitrev8(uint8_t b)
{
    return revbit32(b) >> 24;
}

/* Feed @bits of @value, LSB first. */
static void gpcrc_shift(Efr32Gpcrc *g, uint32_t value, unsigned bits)
{
    uint32_t poly = crc16(g) ? (g->poly & POLY_MASK) : CRC32_POLY_REVERSED;
    uint32_t crc = crc16(g) ? (g->data & POLY_MASK) : g->data;

    for (unsigned i = 0; i < bits; i++) {
        bool fb = (crc ^ (value >> i)) & 1u;

        crc >>= 1;
        if (fb) {
            crc ^= poly;
        }
    }
    g->data = crc16(g) ? (crc & POLY_MASK) : crc;
}

static void gpcrc_input(Efr32Gpcrc *g, uint32_t value, unsigned bytes)
{
    if (!(g->en & EN_EN)) {
        return;
    }
    if (g->ctrl & CTRL_BYTEMODE) {
        bytes = 1;
    }
    value &= bytes == 4 ? UINT32_MAX : (1u << (8 * bytes)) - 1;
    if ((g->ctrl & CTRL_BYTEREVERSE) && bytes == 4) {
        value = bswap32(value);
    } else if ((g->ctrl & CTRL_BYTEREVERSE) && bytes == 2) {
        value = bswap16(value);
    }
    if (g->ctrl & CTRL_BITREVERSE) {
        uint32_t r = 0;

        for (unsigned i = 0; i < bytes; i++) {
            r |= (uint32_t)bitrev8(value >> (8 * i)) << (8 * i);
        }
        value = r;
    }
    gpcrc_shift(g, value, 8 * bytes);
}

void efr32_gpcrc_reset(Efr32Gpcrc *g)
{
    memset(g, 0, sizeof(*g));
}

bool efr32_gpcrc_read(Efr32Gpcrc *g, uint32_t offset, uint32_t *value)
{
    bool output = true;

    switch (offset & ~ALIAS_MASK) {
    case IPVERSION:
    case CMD:
    case INPUTDATA:
    case INPUTDATAHWORD:
    case INPUTDATABYTE:
        *value = 0;
        return true;
    case EN:
        *value = g->en;
        return true;
    case CTRL:
        *value = g->ctrl;
        return true;
    case INIT:
        *value = g->init;
        return true;
    case POLY:
        *value = g->poly;
        return true;
    case DATA:
        *value = g->data;
        break;
    case DATAREV:
        *value = crc16(g) ? revbit32(g->data) >> 16 : revbit32(g->data);
        break;
    case DATABYTEREV:
        *value = crc16(g) ? bswap16(g->data) : bswap32(g->data);
        break;
    default:
        output = false;
        break;
    }
    if (!output) {
        return false;
    }
    if (g->ctrl & CTRL_AUTOINIT) {
        g->data = g->init;
    }
    return true;
}

static uint32_t alias(uint32_t offset, uint32_t old, uint32_t value)
{
    switch (offset & ALIAS_MASK) {
    case ALIAS_SET:
        return old | value;
    case ALIAS_CLR:
        return old & ~value;
    case ALIAS_TGL:
        return old ^ value;
    default:
        return value;
    }
}

bool efr32_gpcrc_write(Efr32Gpcrc *g, uint32_t offset, uint32_t value)
{
    switch (offset & ~ALIAS_MASK) {
    case EN:
        g->en = alias(offset, g->en, value) & EN_EN;
        return true;
    case CTRL:
        g->ctrl = alias(offset, g->ctrl, value) & CTRL_MASK;
        return true;
    case CMD:
        if (value & CMD_INIT) {
            g->data = g->init;
        }
        return true;
    case INIT:
        g->init = alias(offset, g->init, value);
        return true;
    case POLY:
        g->poly = alias(offset, g->poly, value) & POLY_MASK;
        return true;
    case INPUTDATA:
        gpcrc_input(g, value, 4);
        return true;
    case INPUTDATAHWORD:
        gpcrc_input(g, value, 2);
        return true;
    case INPUTDATABYTE:
        gpcrc_input(g, value, 1);
        return true;
    case IPVERSION:
    case DATA:
    case DATAREV:
    case DATABYTEREV:
        return true;
    default:
        return false;
    }
}
