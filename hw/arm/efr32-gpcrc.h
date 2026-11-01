/*
 * EFR32xG22 GPCRC, general-purpose CRC (RM rev. 1.2 chapter 14), at
 * 0x40088000 (non-secure 0x50088000), bus clock CMU_CLKEN0.GPCRC (bit 3).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_GPCRC_H
#define HW_ARM_EFR32_GPCRC_H

#include <stdint.h>
#include <stdbool.h>

#define EFR32_GPCRC_BASE 0x40088000u
#define EFR32_GPCRC_SIZE 0x4000u

typedef struct Efr32Gpcrc {
    uint32_t en;
    uint32_t ctrl;
    uint32_t init;
    uint32_t poly;
    uint32_t data;
} Efr32Gpcrc;

void efr32_gpcrc_reset(Efr32Gpcrc *g);
/* @offset within the 16 KiB block, SET/CLR/TGL pages included. */
bool efr32_gpcrc_read(Efr32Gpcrc *g, uint32_t offset, uint32_t *value);
bool efr32_gpcrc_write(Efr32Gpcrc *g, uint32_t offset, uint32_t value);

#endif
