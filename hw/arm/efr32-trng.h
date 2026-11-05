/*
 * EFR32xG22 CRYPTOACC true random number generator (RM rev. 1.2 SS11.5).
 *
 * Registers: CRYPTOACC_RNGCTRL block at 0x4C021000 (non-secure 0x5C021000);
 * output FIFO window CRYPTOACC_RNGOUT_FIFO at 0x4C024000 (0x5C024000), 16 KiB,
 * every read pops one word (efr32bg22c224f512im40.h; Zephyr
 * entropy_gecko_trng.c reads it with memcpy from the window base).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_TRNG_H
#define HW_ARM_EFR32_TRNG_H

#include <stdint.h>
#include <stdbool.h>

#define EFR32_TRNG_CTRL_BASE 0x4c021000u
#define EFR32_TRNG_CTRL_SIZE 0x1000u
#define EFR32_TRNG_FIFO_BASE 0x4c024000u
#define EFR32_TRNG_FIFO_SIZE 0x4000u
/* RM SS11.5.2.4: FIFODEPTH reads 0x40 words. */
#define EFR32_TRNG_FIFO_WORDS 64u

typedef struct Efr32Trng {
    uint32_t ctrl;
    uint32_t key[4];
    uint32_t status;
    uint32_t initwaitval;
    uint32_t swofftmrval;
    uint32_t clkdiv;
    uint32_t ais31conf[3];
    uint32_t ais31status;
    uint32_t fifo[EFR32_TRNG_FIFO_WORDS];
    unsigned head, level;
    /* Virtual time the next 128-bit block is written (INT64_MAX: none). */
    int64_t next_block_ns;
    /* Before the first block (RNGSTATUS.STATE STARTUP). */
    bool starting;
    /* Words popped from an empty FIFO, for tests and traces. */
    uint64_t underruns;
} Efr32Trng;

void efr32_trng_reset(Efr32Trng *t);
/* @offset within the RNGCTRL block; @hclk_hz the sample clock's source. */
bool efr32_trng_read(Efr32Trng *t, uint32_t offset, uint64_t hclk_hz,
                     uint32_t *value);
bool efr32_trng_write(Efr32Trng *t, uint32_t offset, uint64_t hclk_hz,
                      uint32_t value);
/* One word from the output FIFO (any address in its window). */
uint32_t efr32_trng_pop(Efr32Trng *t, uint64_t hclk_hz);

#endif
