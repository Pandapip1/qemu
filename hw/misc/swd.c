/*
 * Serial Wire Debug target interface (see include/hw/misc/swd.h).
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/swd.h"

static const TypeInfo swd_target_info = {
    .name = TYPE_SWD_TARGET,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(SWDTargetClass),
};

static void swd_register_types(void)
{
    type_register_static(&swd_target_info);
}

type_init(swd_register_types)
