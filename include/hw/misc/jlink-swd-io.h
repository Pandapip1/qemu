/*
 * J-Link SWD I/O command (libjaylink swd.c jaylink_swd_io(), CMD_SWD_IO
 * 0xcf) executed against an SWDTarget.  Kept apart from the USB device in
 * hw/misc/jlink.c so tests/unit/test-jlink-swd-io.c can pin the wire
 * format without a USB device controller.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_JLINK_SWD_IO_H
#define HW_MISC_JLINK_SWD_IO_H

#include "qemu/bswap.h"
#include "hw/misc/swd.h"

#define JLINK_CMD_SWD_IO 0xcf
/* { command, 0, bit count (u16 LE) } (jaylink_swd_io() buf[0..3]) */
#define JLINK_SWD_IO_HEADER 4
#define JLINK_SWD_IO_STATUS_OK 0

/* Bytes a command with @bits cycles occupies: header, direction, output */
static inline size_t jlink_swd_io_cmd_len(unsigned bits)
{
    return JLINK_SWD_IO_HEADER + 2 * DIV_ROUND_UP(bits, 8);
}

/* Bytes of its reply: the input bits and a status byte */
static inline size_t jlink_swd_io_reply_len(unsigned bits)
{
    return DIV_ROUND_UP(bits, 8) + 1;
}

static inline bool jlink_swd_io_bit(const uint8_t *buf, unsigned i)
{
    return (buf[i / 8] >> (i % 8)) & 1;
}

/*
 * Run the command @cmd (jlink_swd_io_cmd_len() bytes) and fill @reply
 * (jlink_swd_io_reply_len() bytes).  The command is the header, then the
 * direction bits (1: the probe drives SWDIO) and the output bits, LSB
 * first; the reply is the input bits followed by a status byte
 * (jaylink_swd_io(): transport_read() of num_bytes, then of 1).
 *
 * Input bit i is SWDIO as the probe samples it after cycle i's rising
 * SWCLK edge, i.e. the level of cycle i + 1.  OpenOCD's J-Link driver
 * depends on this (src/jtag/drivers/jlink.c, jlink_swd_queue_cmd() and
 * jlink_swd_run_queue()): after the 8 request bits it reads the 3 ACK
 * bits starting at the turnaround cycle's own position, the read data 3
 * bits later and the parity 35 bits later, although the line protocol
 * (ADIv5 B4.2) puts one turnaround cycle before the ACK.  Returning the
 * level before the edge, as remote_bitbang's sample-then-clock does, hands
 * OpenOCD the turnaround's pull-up as ACK bit 0 ("cannot read IDR").
 */
static inline void jlink_swd_io(SWDTarget *t, const uint8_t *cmd,
                                uint8_t *reply)
{
    SWDTargetClass *k = SWD_TARGET_GET_CLASS(t);
    unsigned bits = lduw_le_p(cmd + 2);
    unsigned bytes = DIV_ROUND_UP(bits, 8);
    const uint8_t *dir = cmd + JLINK_SWD_IO_HEADER;
    const uint8_t *out = dir + bytes;

    memset(reply, 0, bytes);
    for (unsigned i = 0; i < bits; i++) {
        /* After the last cycle the probe keeps SWDIO as it was */
        unsigned next = i + 1 < bits ? i + 1 : i;

        k->clock(t, jlink_swd_io_bit(dir, i), jlink_swd_io_bit(out, i));
        if (k->peek(t, jlink_swd_io_bit(dir, next),
                    jlink_swd_io_bit(out, next))) {
            reply[i / 8] |= 1u << (i % 8);
        }
    }
    reply[bytes] = JLINK_SWD_IO_STATUS_OK;
}

#endif
