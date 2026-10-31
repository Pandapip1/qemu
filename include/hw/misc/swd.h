/*
 * Serial Wire Debug target interface: what a debug probe model sees on its
 * SWCLK/SWDIO/VTref pins.  Probes (J-Link, remote_bitbang) are
 * silicon-independent; the chip's debug port implements this interface.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SWD_H
#define HW_MISC_SWD_H

#include "qom/object.h"

#define TYPE_SWD_TARGET "swd-target"
typedef struct SWDTargetClass SWDTargetClass;
DECLARE_CLASS_CHECKERS(SWDTargetClass, SWD_TARGET, TYPE_SWD_TARGET)
#define SWD_TARGET(obj) INTERFACE_CHECK(SWDTarget, (obj), TYPE_SWD_TARGET)

typedef struct SWDTarget SWDTarget;

struct SWDTargetClass {
    InterfaceClass parent;

    /*
     * Level of SWDIO during the current SWCLK cycle as the probe samples it
     * (before the rising edge): the target's output when it drives the
     * line, otherwise @host_bit if the probe drives, otherwise the pull-up.
     */
    bool (*peek)(SWDTarget *t, bool host_drives, bool host_bit);
    /* Complete the current cycle with a rising SWCLK edge. */
    void (*clock)(SWDTarget *t, bool host_drives, bool host_bit);
    /* Target supply voltage at the debug connector's VTref pin, in mV. */
    uint32_t (*vtref_mv)(SWDTarget *t);
};

/* One SWCLK cycle: sample SWDIO, then clock the rising edge. */
static inline bool swd_target_cycle(SWDTarget *t, bool host_drives,
                                    bool host_bit)
{
    SWDTargetClass *k = SWD_TARGET_GET_CLASS(t);
    bool level = k->peek(t, host_drives, host_bit);

    k->clock(t, host_drives, host_bit);
    return level;
}

static inline uint32_t swd_target_vtref_mv(SWDTarget *t)
{
    return SWD_TARGET_GET_CLASS(t)->vtref_mv(t);
}

#endif
