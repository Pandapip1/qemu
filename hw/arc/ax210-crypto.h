/*
 * AX210 model: hash and public-key primitives behind the firmware's
 * crypto engines (fidelity finding W5).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARC_AX210_CRYPTO_H
#define HW_ARC_AX210_CRYPTO_H

#include <stdbool.h>
#include <stdint.h>

/* SHA-256 / SHA-384 compression state (FIPS 180-4). */
typedef struct {
    bool wide;                  /* SHA-384 (64-bit words) */
    uint32_t h32[8];
    uint64_t h64[8];
} AX210Sha;

void ax210_sha_init(AX210Sha *s, bool sha384);
void ax210_sha_block(AX210Sha *s, const uint8_t *block);

/* P-384 numbers: 12 32-bit words, least significant first. */
#define P384_WORDS 12

/*
 * ECDSA verify over P-384 (FIPS 186-4): true if (r, s) signs the hash e
 * for public key (qx, qy).
 */
bool ax210_p384_verify(const uint32_t *qx, const uint32_t *qy,
                       const uint32_t *r, const uint32_t *s,
                       const uint32_t *e);

#endif
