/*
 * Internal RF medium for EFR32 radio peripherals.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/efr32-rf-medium.h"

struct Efr32RfMedium {
    Object parent_obj;

    GList *radios;
};

void efr32_rf_medium_attach(Efr32RfMedium *medium, Efr32RfRadio *radio,
                            Error **errp)
{
    if (!object_dynamic_cast(OBJECT(radio), TYPE_EFR32_RF_RADIO)) {
        error_setg(errp, "%s does not implement %s",
                   object_get_typename(OBJECT(radio)), TYPE_EFR32_RF_RADIO);
        return;
    }

    if (g_list_find(medium->radios, radio)) {
        return;
    }

    medium->radios = g_list_append(medium->radios, radio);
}

void efr32_rf_medium_detach(Efr32RfMedium *medium, Efr32RfRadio *radio)
{
    GList *member = g_list_find(medium->radios, radio);

    if (!member) {
        return;
    }

    medium->radios = g_list_delete_link(medium->radios, member);
}

bool efr32_rf_medium_has_radio(const Efr32RfMedium *medium,
                                const Efr32RfRadio *radio)
{
    return g_list_find(medium->radios, radio) != NULL;
}

static void efr32_rf_medium_finalize(Object *obj)
{
    Efr32RfMedium *medium = EFR32_RF_MEDIUM(obj);

    g_list_free(medium->radios);
}

static const TypeInfo efr32_rf_medium_info = {
    .name = TYPE_EFR32_RF_MEDIUM,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(Efr32RfMedium),
    .instance_finalize = efr32_rf_medium_finalize,
};

static const TypeInfo efr32_rf_radio_info = {
    .name = TYPE_EFR32_RF_RADIO,
    .parent = TYPE_INTERFACE,
};

void efr32_rf_medium_register_types(void)
{
    type_register_static(&efr32_rf_medium_info);
    type_register_static(&efr32_rf_radio_info);
}
