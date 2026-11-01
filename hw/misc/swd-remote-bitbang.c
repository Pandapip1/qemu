/*
 * SWD probe speaking OpenOCD's remote_bitbang protocol on a chardev, so an
 * unmodified OpenOCD (`adapter driver remote_bitbang`, `transport select
 * swd`) can drive a modelled chip's debug port without USB.  Silicon
 * independent: it clocks any SWDTarget.
 *
 * Protocol: OpenOCD doc/manual/jtag/drivers/remote_bitbang.txt and
 * src/jtag/drivers/remote_bitbang.c.  Each byte is one command:
 *   'd'..'g'  SWCLK/SWDIO = (byte - 'd') >> 1, (byte - 'd') & 1
 *   'O' / 'o' the probe drives / releases SWDIO
 *   'c'       sample SWDIO, answered with '0' or '1'
 *   'R'       sample TDO (JTAG; answered '0', JTAG is not wired)
 *   'r'..'u'  resets: 'r' + (TRST << 1 | SRST), 1 = asserted; SRST drives
 *             the target's nRESET (the EK's J-Link OB drives it too), TRST
 *             is ignored (JTAG is not wired)
 *   'B' 'b' '0'..'7' 'Z' 'z' 'Q'  LEDs, JTAG pins, sleep, quit: accepted
 *             and ignored
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/misc/swd.h"

#define TYPE_SWD_REMOTE_BITBANG "swd-remote-bitbang"
OBJECT_DECLARE_SIMPLE_TYPE(SwdRemoteBitbangState, SWD_REMOTE_BITBANG)

#define RBB_SWD_WRITE_FIRST 'd'
#define RBB_SWD_WRITE_LAST 'g'
#define RBB_SWD_WRITE_CLK (1u << 1)
#define RBB_SWD_WRITE_DIO (1u << 0)
#define RBB_SWDIO_DRIVE 'O'
#define RBB_SWDIO_RELEASE 'o'
#define RBB_SWDIO_READ 'c'
#define RBB_TDO_READ 'R'
#define RBB_RESET_FIRST 'r'
#define RBB_RESET_LAST 'u'
#define RBB_RESET_SRST (1u << 0)
#define RBB_REPLY_LOW '0'
#define RBB_REPLY_HIGH '1'
#define RBB_RX_CHUNK 4096

struct SwdRemoteBitbangState {
    DeviceState parent_obj;

    CharBackend chr;
    Object *target_obj;
    SWDTarget *target;

    bool drive;
    bool swclk;
    bool swdio;
};

static int rbb_can_receive(void *opaque)
{
    return RBB_RX_CHUNK;
}

static void rbb_receive(void *opaque, const uint8_t *buf, int size)
{
    SwdRemoteBitbangState *s = opaque;
    SWDTargetClass *k = SWD_TARGET_GET_CLASS(s->target);
    g_autofree uint8_t *reply = g_malloc(size);
    int n = 0;

    for (int i = 0; i < size; i++) {
        uint8_t c = buf[i];

        if (c >= RBB_SWD_WRITE_FIRST && c <= RBB_SWD_WRITE_LAST) {
            unsigned v = c - RBB_SWD_WRITE_FIRST;
            bool clk = v & RBB_SWD_WRITE_CLK;

            s->swdio = v & RBB_SWD_WRITE_DIO;
            if (clk && !s->swclk) {
                k->clock(s->target, s->drive, s->swdio);
            }
            s->swclk = clk;
        } else if (c == RBB_SWDIO_DRIVE || c == RBB_SWDIO_RELEASE) {
            s->drive = c == RBB_SWDIO_DRIVE;
        } else if (c == RBB_SWDIO_READ) {
            reply[n++] = k->peek(s->target, s->drive, s->swdio) ?
                         RBB_REPLY_HIGH : RBB_REPLY_LOW;
        } else if (c == RBB_TDO_READ) {
            reply[n++] = RBB_REPLY_LOW;
        } else if (c >= RBB_RESET_FIRST && c <= RBB_RESET_LAST) {
            swd_target_set_nreset(s->target,
                                  (c - RBB_RESET_FIRST) & RBB_RESET_SRST);
        }
    }
    if (n) {
        qemu_chr_fe_write_all(&s->chr, reply, n);
    }
}

static void rbb_realize(DeviceState *dev, Error **errp)
{
    SwdRemoteBitbangState *s = SWD_REMOTE_BITBANG(dev);
    bool ambiguous = false;

    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "swd-remote-bitbang: 'chardev' is required");
        return;
    }
    if (!s->target_obj) {
        s->target_obj = object_resolve_path_type("", TYPE_SWD_TARGET,
                                                 &ambiguous);
    }
    if (!s->target_obj) {
        error_setg(errp, "swd-remote-bitbang: no unique SWD target; set "
                   "'target'");
        return;
    }
    s->target = SWD_TARGET(s->target_obj);
    qemu_chr_fe_set_handlers(&s->chr, rbb_can_receive, rbb_receive, NULL,
                             NULL, s, NULL, true);
}

static Property rbb_properties[] = {
    DEFINE_PROP_CHR("chardev", SwdRemoteBitbangState, chr),
    DEFINE_PROP_LINK("target", SwdRemoteBitbangState, target_obj,
                     TYPE_SWD_TARGET, Object *),
    DEFINE_PROP_END_OF_LIST(),
};

static void rbb_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rbb_realize;
    dc->user_creatable = true;
    dc->desc = "SWD probe on OpenOCD's remote_bitbang protocol";
    device_class_set_props(dc, rbb_properties);
}

static const TypeInfo rbb_info = {
    .name = TYPE_SWD_REMOTE_BITBANG,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SwdRemoteBitbangState),
    .class_init = rbb_class_init,
};

static void rbb_register_types(void)
{
    type_register_static(&rbb_info);
}

type_init(rbb_register_types)
