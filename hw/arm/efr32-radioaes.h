/*
 * EFR32xG22 RADIOAES: a BA411E AES engine behind the accelerator's scatter-gather DMA.
 *
 * Registers (offsets in the RADIOAES block, 0x44000000; the non-secure alias is 0x54000000):
 * EFR32xG22 Reference Manual rev. 1.2, 11.4 (the DMA interface, documented there for
 * CRYPTOACC with the same layout) and hal_silabs efr32bg22_aes.h (Zlib). Descriptor, tag and
 * BA411E configuration format: hal_silabs sli_protocol_crypto_radioaes.c (Zlib). The modes
 * modelled are those the open firmware drives (modules/efr32-radio/crypto/radioaes.c): ECB,
 * CMAC and CCM with AES-128 or AES-256.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_RADIOAES_H
#define HW_ARM_EFR32_RADIOAES_H

#include "exec/memory.h"

#define EFR32_RADIOAES_BASE 0x44000000u
#define EFR32_RADIOAES_SIZE 0x800u

typedef struct Efr32RadioAes {
    uint32_t fetchaddr;
    uint32_t fetchlen;
    uint32_t fetchtag;
    uint32_t pushaddr;
    uint32_t pushlen;
    uint32_t ien;
    uint32_t if_flags;
    uint32_t ctrl;
    /* The DMA reports busy until this virtual time (ns). */
    int64_t busy_until_ns;
    /* Operations started, for traces and tests. */
    uint64_t operations;
} Efr32RadioAes;

void efr32_radioaes_reset(Efr32RadioAes *a);
/* @offset within the block; returns false for offsets it does not model. */
bool efr32_radioaes_read(Efr32RadioAes *a, uint32_t offset, uint32_t *value);
bool efr32_radioaes_write(Efr32RadioAes *a, AddressSpace *as, uint32_t offset,
                          uint32_t value);

#endif
