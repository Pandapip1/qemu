/*
 * Wire format of the J-Link SWD I/O command (include/hw/misc/jlink-swd-io.h)
 * as its open host stack builds and decodes it:
 *
 *  - libjaylink 0.5.0 libjaylink/swd.c jaylink_swd_io() (line 54): the
 *    command is { CMD_SWD_IO, 0x00, length (u16 LE) } (lines 79-81), then
 *    num_bytes = (length + 7) / 8 direction bytes and as many output
 *    bytes; the reply is num_bytes input bytes and one status byte, 0 for
 *    success (line 70 transport_start_write_read(devh, 4 + 2 * num_bytes,
 *    num_bytes + 1, true); the transport_read()s at lines 107 and 115).
 *
 *  - OpenOCD (0.12.0+dev, 2026-10-24) src/jtag/drivers/jlink.c:
 *    jlink_swd_queue_cmd() (line 2287) queues the 8 request bits as output
 *    (direction 1, the tms_buffer), records first = tap_length (line
 *    2304), then queues 1 + 3 + 32 + 1 + 1 input bits for a read (line
 *    2311), or 1 + 3 + 1 input bits and 32 + 1 output bits for a write
 *    (line 2315); jlink_swd_run_queue() appends 8 idle output bits (line
 *    2246) and decodes ack = buf_get_u32(tdo_buffer, first, 3) (line
 *    2258), data at 3 + first and parity at 3 + 32 + first (lines
 *    2265-2266).  So input bit i must be the SWDIO level of cycle i + 1:
 *    the ACK is read at the turnaround cycle's own position.
 *
 * The SWD target here is a minimal ADIv5 SWD responder (B4.2: request,
 * turnaround, ACK, data + parity, turnaround) that does not depend on the
 * probe's sampling convention.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "hw/misc/jlink-swd-io.h"

#define TYPE_TEST_SWD_TARGET "test-swd-target"

/* ADIv5 B4.2 packet layout, in cycles from the start bit */
#define SWD_REQ_BITS 8
#define SWD_ACK_FIRST (SWD_REQ_BITS + 1)        /* after one turnaround */
#define SWD_ACK_BITS 3
#define SWD_DATA_FIRST (SWD_ACK_FIRST + SWD_ACK_BITS)
#define SWD_DATA_BITS 32
#define SWD_READ_PARITY (SWD_DATA_FIRST + SWD_DATA_BITS)
#define SWD_READ_END (SWD_READ_PARITY + 2)      /* parity, turnaround */
#define SWD_WRITE_DATA_FIRST (SWD_DATA_FIRST + 1) /* after a turnaround */
#define SWD_WRITE_END (SWD_WRITE_DATA_FIRST + SWD_DATA_BITS + 1)
#define SWD_REQ_RNW (1u << 2)
#define SWD_ACK_OK 1
#define SWD_ACK_WAIT 2

/* Requests (start, APnDP, RnW, A[3:2], parity, stop, park; LSB first) */
#define SWD_REQ_DP_READ_DPIDR 0xa5
#define SWD_REQ_DP_WRITE_ABORT 0x81

#define TEST_DPIDR 0x6ba02477u
#define TEST_WRITE_DATA 0x0000001eu

typedef struct TestSwdTarget {
    Object parent_obj;
    int pos;            /* cycle within the packet, -1 when idle */
    uint8_t request;
    uint8_t ack;
    uint32_t rdata;
    uint64_t wshift;    /* write data and parity as received */
} TestSwdTarget;

#define TEST_SWD_TARGET(obj) \
    OBJECT_CHECK(TestSwdTarget, (obj), TYPE_TEST_SWD_TARGET)

static bool test_swd_peek(SWDTarget *t, bool host_drives, bool host_bit)
{
    TestSwdTarget *s = TEST_SWD_TARGET(t);
    bool read = s->request & SWD_REQ_RNW;

    if (s->pos >= SWD_ACK_FIRST && s->pos < SWD_DATA_FIRST) {
        return (s->ack >> (s->pos - SWD_ACK_FIRST)) & 1;
    }
    if (read && s->ack == SWD_ACK_OK &&
        s->pos >= SWD_DATA_FIRST && s->pos < SWD_READ_PARITY) {
        return (s->rdata >> (s->pos - SWD_DATA_FIRST)) & 1;
    }
    if (read && s->ack == SWD_ACK_OK && s->pos == SWD_READ_PARITY) {
        return ctpop32(s->rdata) & 1;
    }
    return host_drives ? host_bit : true;   /* pull-up */
}

static void test_swd_clock(SWDTarget *t, bool host_drives, bool host_bit)
{
    TestSwdTarget *s = TEST_SWD_TARGET(t);
    bool bit = host_drives ? host_bit : true;
    int end;

    if (s->pos < 0) {
        if (host_drives && host_bit) {
            s->request = 1;     /* start bit */
            s->pos = 1;
        }
        return;
    }
    if (s->pos < SWD_REQ_BITS) {
        s->request |= bit << s->pos;
    }
    if (s->pos >= SWD_WRITE_DATA_FIRST && !(s->request & SWD_REQ_RNW)) {
        s->wshift |= (uint64_t)bit << (s->pos - SWD_WRITE_DATA_FIRST);
    }
    end = s->request & SWD_REQ_RNW ? SWD_READ_END : SWD_WRITE_END;
    if (s->ack != SWD_ACK_OK && s->pos >= SWD_DATA_FIRST) {
        end = SWD_DATA_FIRST + 1;   /* turnaround only */
    }
    s->pos = s->pos + 1 >= end ? -1 : s->pos + 1;
}

static uint32_t test_swd_vtref_mv(SWDTarget *t)
{
    return 3300;
}

static void test_swd_class_init(ObjectClass *oc, void *data)
{
    SWDTargetClass *k = SWD_TARGET_CLASS(oc);

    k->peek = test_swd_peek;
    k->clock = test_swd_clock;
    k->vtref_mv = test_swd_vtref_mv;
}

static void test_swd_init(Object *obj)
{
    TEST_SWD_TARGET(obj)->pos = -1;
}

static const TypeInfo test_swd_target_info = {
    .name = TYPE_TEST_SWD_TARGET,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(TestSwdTarget),
    .instance_init = test_swd_init,
    .class_init = test_swd_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_SWD_TARGET },
        { }
    },
};

/* OpenOCD's tap buffers (jlink.c tms_buffer, tdi_buffer, tdo_buffer) */
typedef struct Tap {
    uint8_t dir[16];
    uint8_t out[16];
    unsigned len;
} Tap;

static void tap_out(Tap *t, uint64_t v, unsigned n)
{
    for (unsigned i = 0; i < n; i++, t->len++) {
        t->dir[t->len / 8] |= 1u << (t->len % 8);
        t->out[t->len / 8] |= ((v >> i) & 1) << (t->len % 8);
    }
}

static void tap_in(Tap *t, unsigned n)
{
    t->len += n;
}

static uint64_t get_bits(const uint8_t *b, unsigned first, unsigned n)
{
    uint64_t v = 0;

    for (unsigned i = 0; i < n; i++) {
        v |= (uint64_t)jlink_swd_io_bit(b, first + i) << i;
    }
    return v;
}

/* jaylink_swd_io(): build the command, run it, check the reply framing */
static void run_swd_io(SWDTarget *t, const Tap *tap, uint8_t *in)
{
    unsigned bytes = DIV_ROUND_UP(tap->len, 8);
    g_autofree uint8_t *cmd = g_malloc0(jlink_swd_io_cmd_len(tap->len));

    g_assert_cmpuint(jlink_swd_io_cmd_len(tap->len), ==, 4 + 2 * bytes);
    g_assert_cmpuint(jlink_swd_io_reply_len(tap->len), ==, bytes + 1);
    cmd[0] = JLINK_CMD_SWD_IO;
    g_assert_cmpuint(cmd[0], ==, 0xcf);
    cmd[1] = 0x00;
    stw_le_p(cmd + 2, tap->len);
    memcpy(cmd + 4, tap->dir, bytes);
    memcpy(cmd + 4 + bytes, tap->out, bytes);
    memset(in, 0xaa, bytes + 1);
    jlink_swd_io(t, cmd, in);
    g_assert_cmpuint(in[bytes], ==, 0);     /* status: success */
}

static TestSwdTarget *new_target(uint8_t ack)
{
    TestSwdTarget *s = TEST_SWD_TARGET(object_new(TYPE_TEST_SWD_TARGET));

    s->ack = ack;
    s->rdata = TEST_DPIDR;
    return s;
}

/* jlink_swd_queue_cmd() + jlink_swd_run_queue() for a DPIDR read */
static void test_read(void)
{
    TestSwdTarget *s = new_target(SWD_ACK_OK);
    uint8_t in[16];
    Tap tap = { };
    unsigned first;

    tap_out(&tap, SWD_REQ_DP_READ_DPIDR, SWD_REQ_BITS);
    first = tap.len;
    tap_in(&tap, 1 + 3 + 32 + 1 + 1);
    tap_out(&tap, 0, 8);
    run_swd_io(SWD_TARGET(s), &tap, in);

    g_assert_cmpuint(s->request, ==, SWD_REQ_DP_READ_DPIDR);
    g_assert_cmpint(s->pos, ==, -1);
    g_assert_cmpuint(get_bits(in, first, 3), ==, SWD_ACK_OK);
    g_assert_cmphex(get_bits(in, 3 + first, 32), ==, TEST_DPIDR);
    g_assert_cmpuint(get_bits(in, 3 + 32 + first, 1), ==,
                     ctpop32(TEST_DPIDR) & 1);
    object_unref(OBJECT(s));
}

/* ... and for a DP ABORT write: ACK position and write data timing */
static void test_write(void)
{
    TestSwdTarget *s = new_target(SWD_ACK_OK);
    uint64_t data = TEST_WRITE_DATA |
                    ((uint64_t)(ctpop32(TEST_WRITE_DATA) & 1) << 32);
    uint8_t in[16];
    Tap tap = { };
    unsigned first;

    tap_out(&tap, SWD_REQ_DP_WRITE_ABORT, SWD_REQ_BITS);
    first = tap.len;
    tap_in(&tap, 1 + 3 + 1);
    tap_out(&tap, data, 32 + 1);
    tap_out(&tap, 0, 8);
    run_swd_io(SWD_TARGET(s), &tap, in);

    g_assert_cmpuint(s->request, ==, SWD_REQ_DP_WRITE_ABORT);
    g_assert_cmpint(s->pos, ==, -1);
    g_assert_cmpuint(get_bits(in, first, 3), ==, SWD_ACK_OK);
    g_assert_cmphex(s->wshift, ==, data);
    object_unref(OBJECT(s));
}

/* A WAIT reply is seen as WAIT, not shifted into JUNK */
static void test_wait(void)
{
    TestSwdTarget *s = new_target(SWD_ACK_WAIT);
    uint8_t in[16];
    Tap tap = { };
    unsigned first;

    tap_out(&tap, SWD_REQ_DP_READ_DPIDR, SWD_REQ_BITS);
    first = tap.len;
    tap_in(&tap, 1 + 3 + 32 + 1 + 1);
    tap_out(&tap, 0, 8);
    run_swd_io(SWD_TARGET(s), &tap, in);

    g_assert_cmpuint(get_bits(in, first, 3), ==, SWD_ACK_WAIT);
    object_unref(OBJECT(s));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    type_register_static(&test_swd_target_info);

    g_test_add_func("/jlink-swd-io/read", test_read);
    g_test_add_func("/jlink-swd-io/write", test_write);
    g_test_add_func("/jlink-swd-io/wait", test_wait);
    return g_test_run();
}
