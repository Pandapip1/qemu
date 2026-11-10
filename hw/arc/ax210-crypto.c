/*
 * AX210 model: SHA-256/384 compression and P-384 ECDSA verification for
 * the firmware's hash and public-key engines (fidelity finding W5).
 * Straight implementations of FIPS 180-4 and FIPS 186-4; speed does not
 * matter (the firmware verifies one signature per boot).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "ax210-crypto.h"

/* ---- SHA-256 / SHA-384 ---- */

static const uint32_t k256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static const uint64_t k512[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL, 0xb5c0fbcfec4d3b2fULL,
    0xe9b5dba58189dbbcULL, 0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
    0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL, 0xd807aa98a3030242ULL,
    0x12835b0145706fbeULL, 0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL, 0x9bdc06a725c71235ULL,
    0xc19bf174cf692694ULL, 0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
    0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL, 0x2de92c6f592b0275ULL,
    0x4a7484aa6ea6e483ULL, 0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL, 0xb00327c898fb213fULL,
    0xbf597fc7beef0ee4ULL, 0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
    0x06ca6351e003826fULL, 0x142929670a0e6e70ULL, 0x27b70a8546d22ffcULL,
    0x2e1b21385c26c926ULL, 0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL, 0x81c2c92e47edaee6ULL,
    0x92722c851482353bULL, 0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
    0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL, 0xd192e819d6ef5218ULL,
    0xd69906245565a910ULL, 0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL, 0x2748774cdf8eeb99ULL,
    0x34b0bcb5e19b48a8ULL, 0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
    0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL, 0x748f82ee5defb2fcULL,
    0x78a5636f43172f60ULL, 0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL, 0xbef9a3f7b2c67915ULL,
    0xc67178f2e372532bULL, 0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
    0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL, 0x06f067aa72176fbaULL,
    0x0a637dc5a2c898a6ULL, 0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL, 0x3c9ebe0a15c9bebcULL,
    0x431d67c49c100d4cULL, 0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
    0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL,
};

static const uint32_t iv256[8] = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};

static const uint64_t iv384[8] = {
    0xcbbb9d5dc1059ed8ULL, 0x629a292a367cd507ULL, 0x9159015a3070dd17ULL,
    0x152fecd8f70e5939ULL, 0x67332667ffc00b31ULL, 0x8eb44a8768581511ULL,
    0xdb0c2e0d64f98fa7ULL, 0x47b5481dbefa4fa4ULL,
};

#define ROR32(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define ROR64(x, n) (((x) >> (n)) | ((x) << (64 - (n))))

void ax210_sha_init(AX210Sha *s, bool sha384)
{
    s->wide = sha384;
    memcpy(s->h32, iv256, sizeof(iv256));
    memcpy(s->h64, iv384, sizeof(iv384));
}

static void sha256_block(uint32_t *h, const uint8_t *p)
{
    uint32_t w[64], a, b, c, d, e, f, g, hh;

    for (int i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROR32(w[i - 15], 7) ^ ROR32(w[i - 15], 18) ^
                      (w[i - 15] >> 3);
        uint32_t s1 = ROR32(w[i - 2], 17) ^ ROR32(w[i - 2], 19) ^
                      (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = h[0]; b = h[1]; c = h[2]; d = h[3];
    e = h[4]; f = h[5]; g = h[6]; hh = h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = hh + (ROR32(e, 6) ^ ROR32(e, 11) ^ ROR32(e, 25)) +
                      ((e & f) ^ (~e & g)) + k256[i] + w[i];
        uint32_t t2 = (ROR32(a, 2) ^ ROR32(a, 13) ^ ROR32(a, 22)) +
                      ((a & b) ^ (a & c) ^ (b & c));
        hh = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

static void sha512_block(uint64_t *h, const uint8_t *p)
{
    uint64_t w[80], a, b, c, d, e, f, g, hh;

    for (int i = 0; i < 16; i++) {
        w[i] = ldq_be_p(p + 8 * i);
    }
    for (int i = 16; i < 80; i++) {
        uint64_t s0 = ROR64(w[i - 15], 1) ^ ROR64(w[i - 15], 8) ^
                      (w[i - 15] >> 7);
        uint64_t s1 = ROR64(w[i - 2], 19) ^ ROR64(w[i - 2], 61) ^
                      (w[i - 2] >> 6);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = h[0]; b = h[1]; c = h[2]; d = h[3];
    e = h[4]; f = h[5]; g = h[6]; hh = h[7];
    for (int i = 0; i < 80; i++) {
        uint64_t t1 = hh + (ROR64(e, 14) ^ ROR64(e, 18) ^ ROR64(e, 41)) +
                      ((e & f) ^ (~e & g)) + k512[i] + w[i];
        uint64_t t2 = (ROR64(a, 28) ^ ROR64(a, 34) ^ ROR64(a, 39)) +
                      ((a & b) ^ (a & c) ^ (b & c));
        hh = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

void ax210_sha_block(AX210Sha *s, const uint8_t *block)
{
    if (s->wide) {
        sha512_block(s->h64, block);
    } else {
        sha256_block(s->h32, block);
    }
}

/* ---- P-384 arithmetic (12 x 32-bit limbs, least significant first) ---- */

#define NW P384_WORDS
typedef uint32_t bn[NW];

static const bn p384_p = {
    0xffffffff, 0x00000000, 0x00000000, 0xffffffff, 0xfffffffe, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
};
static const bn p384_n = {
    0xccc52973, 0xecec196a, 0x48b0a77a, 0x581a0db2, 0xf4372ddf, 0xc7634d81,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
};
static const bn p384_gx = {
    0x72760ab7, 0x3a545e38, 0xbf55296c, 0x5502f25d, 0x82542a38, 0x59f741e0,
    0x8ba79b98, 0x6e1d3b62, 0xf320ad74, 0x8eb1c71e, 0xbe8b0537, 0xaa87ca22,
};
static const bn p384_gy = {
    0x90ea0e5f, 0x7a431d7c, 0x1d7e819d, 0x0a60b1ce, 0xb5f0b8c0, 0xe9da3113,
    0x289a147c, 0xf8f41dbd, 0x9292dc29, 0x5d9e98bf, 0x96262c6f, 0x3617de4a,
};

static int bn_cmp(const uint32_t *a, const uint32_t *b)
{
    for (int i = NW - 1; i >= 0; i--) {
        if (a[i] != b[i]) {
            return a[i] > b[i] ? 1 : -1;
        }
    }
    return 0;
}

static bool bn_zero(const uint32_t *a)
{
    for (int i = 0; i < NW; i++) {
        if (a[i]) {
            return false;
        }
    }
    return true;
}

/* r = a + b; returns the carry out. */
static uint32_t bn_add(uint32_t *r, const uint32_t *a, const uint32_t *b)
{
    uint64_t c = 0;

    for (int i = 0; i < NW; i++) {
        c += (uint64_t)a[i] + b[i];
        r[i] = (uint32_t)c;
        c >>= 32;
    }
    return (uint32_t)c;
}

/* r = a - b; returns the borrow out. */
static uint32_t bn_sub(uint32_t *r, const uint32_t *a, const uint32_t *b)
{
    int64_t c = 0;

    for (int i = 0; i < NW; i++) {
        c += (int64_t)a[i] - b[i];
        r[i] = (uint32_t)c;
        c >>= 32;
    }
    return c ? 1 : 0;
}

static void mod_add(uint32_t *r, const uint32_t *a, const uint32_t *b,
                    const uint32_t *m)
{
    uint32_t c = bn_add(r, a, b);

    if (c || bn_cmp(r, m) >= 0) {
        bn_sub(r, r, m);
    }
}

static void mod_sub(uint32_t *r, const uint32_t *a, const uint32_t *b,
                    const uint32_t *m)
{
    if (bn_sub(r, a, b)) {
        bn_add(r, r, m);
    }
}

/* r = a * b mod m, by shift-and-subtract reduction of the full product. */
static void mod_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                    const uint32_t *m)
{
    uint32_t t[2 * NW] = { 0 };
    bn acc = { 0 };

    for (int i = 0; i < NW; i++) {
        uint64_t c = 0;
        for (int j = 0; j < NW; j++) {
            c += (uint64_t)a[i] * b[j] + t[i + j];
            t[i + j] = (uint32_t)c;
            c >>= 32;
        }
        t[i + NW] = (uint32_t)c;
    }
    for (int bit = 2 * NW * 32 - 1; bit >= 0; bit--) {
        uint32_t top = acc[NW - 1] >> 31;
        for (int i = NW - 1; i > 0; i--) {
            acc[i] = (acc[i] << 1) | (acc[i - 1] >> 31);
        }
        acc[0] = (acc[0] << 1) | ((t[bit / 32] >> (bit % 32)) & 1);
        if (top || bn_cmp(acc, m) >= 0) {
            bn_sub(acc, acc, m);
        }
    }
    memcpy(r, acc, sizeof(acc));
}

/* r = a^(m-2) mod m: the inverse for prime m. */
static void mod_inv(uint32_t *r, const uint32_t *a, const uint32_t *m)
{
    bn e, x = { 1 };
    static const bn two = { 2 };

    bn_sub(e, m, two);
    for (int bit = NW * 32 - 1; bit >= 0; bit--) {
        mod_mul(x, x, x, m);
        if ((e[bit / 32] >> (bit % 32)) & 1) {
            mod_mul(x, x, a, m);
        }
    }
    memcpy(r, x, sizeof(x));
}

/* Jacobian point; z == 0 is the point at infinity. */
typedef struct {
    bn x, y, z;
} jpt;

static void pt_double(jpt *r, const jpt *p)
{
    const uint32_t *m = p384_p;
    bn delta, gamma, beta, alpha, t1, t2, x3, y3, z3;

    if (bn_zero(p->z) || bn_zero(p->y)) {
        memset(r, 0, sizeof(*r));
        return;
    }
    mod_mul(delta, p->z, p->z, m);
    mod_mul(gamma, p->y, p->y, m);
    mod_mul(beta, p->x, gamma, m);
    mod_sub(t1, p->x, delta, m);
    mod_add(t2, p->x, delta, m);
    mod_mul(alpha, t1, t2, m);
    mod_add(t1, alpha, alpha, m);
    mod_add(alpha, t1, alpha, m);               /* 3 (x - d)(x + d), a = -3 */
    mod_mul(x3, alpha, alpha, m);
    mod_add(t1, beta, beta, m);
    mod_add(t1, t1, t1, m);                     /* 4 beta */
    mod_add(t2, t1, t1, m);                     /* 8 beta */
    mod_sub(x3, x3, t2, m);
    mod_add(t2, p->y, p->z, m);
    mod_mul(z3, t2, t2, m);
    mod_sub(z3, z3, gamma, m);
    mod_sub(z3, z3, delta, m);
    mod_sub(t1, t1, x3, m);
    mod_mul(y3, alpha, t1, m);
    mod_mul(t2, gamma, gamma, m);
    mod_add(t2, t2, t2, m);
    mod_add(t2, t2, t2, m);
    mod_add(t2, t2, t2, m);                     /* 8 gamma^2 */
    mod_sub(y3, y3, t2, m);
    memcpy(r->x, x3, sizeof(bn));
    memcpy(r->y, y3, sizeof(bn));
    memcpy(r->z, z3, sizeof(bn));
}

static void pt_add(jpt *r, const jpt *p, const jpt *q)
{
    const uint32_t *m = p384_p;
    bn z1z1, z2z2, u1, u2, s1, s2, h, rr, h2, h3, u1h2, t, x3, y3, z3;

    if (bn_zero(p->z)) {
        *r = *q;
        return;
    }
    if (bn_zero(q->z)) {
        *r = *p;
        return;
    }
    mod_mul(z1z1, p->z, p->z, m);
    mod_mul(z2z2, q->z, q->z, m);
    mod_mul(u1, p->x, z2z2, m);
    mod_mul(u2, q->x, z1z1, m);
    mod_mul(t, q->z, z2z2, m);
    mod_mul(s1, p->y, t, m);
    mod_mul(t, p->z, z1z1, m);
    mod_mul(s2, q->y, t, m);
    if (bn_cmp(u1, u2) == 0) {
        if (bn_cmp(s1, s2) == 0) {
            pt_double(r, p);
        } else {
            memset(r, 0, sizeof(*r));
        }
        return;
    }
    mod_sub(h, u2, u1, m);
    mod_sub(rr, s2, s1, m);
    mod_mul(h2, h, h, m);
    mod_mul(h3, h, h2, m);
    mod_mul(u1h2, u1, h2, m);
    mod_mul(x3, rr, rr, m);
    mod_sub(x3, x3, h3, m);
    mod_sub(x3, x3, u1h2, m);
    mod_sub(x3, x3, u1h2, m);
    mod_sub(t, u1h2, x3, m);
    mod_mul(y3, rr, t, m);
    mod_mul(t, s1, h3, m);
    mod_sub(y3, y3, t, m);
    mod_mul(t, p->z, q->z, m);
    mod_mul(z3, t, h, m);
    memcpy(r->x, x3, sizeof(bn));
    memcpy(r->y, y3, sizeof(bn));
    memcpy(r->z, z3, sizeof(bn));
}

bool ax210_p384_verify(const uint32_t *qx, const uint32_t *qy,
                       const uint32_t *r, const uint32_t *s,
                       const uint32_t *e)
{
    bn w, u1, u2, ee, zi, zi2, x;
    jpt g = { .z = { 1 } }, q = { .z = { 1 } }, gq, acc = { 0 };

    if (bn_zero(r) || bn_zero(s) ||
        bn_cmp(r, p384_n) >= 0 || bn_cmp(s, p384_n) >= 0) {
        return false;
    }
    memcpy(ee, e, sizeof(bn));
    if (bn_cmp(ee, p384_n) >= 0) {
        bn_sub(ee, ee, p384_n);
    }
    mod_inv(w, s, p384_n);
    mod_mul(u1, ee, w, p384_n);
    mod_mul(u2, r, w, p384_n);

    memcpy(g.x, p384_gx, sizeof(bn));
    memcpy(g.y, p384_gy, sizeof(bn));
    memcpy(q.x, qx, sizeof(bn));
    memcpy(q.y, qy, sizeof(bn));
    pt_add(&gq, &g, &q);
    for (int bit = NW * 32 - 1; bit >= 0; bit--) {
        int b1 = (u1[bit / 32] >> (bit % 32)) & 1;
        int b2 = (u2[bit / 32] >> (bit % 32)) & 1;
        pt_double(&acc, &acc);
        if (b1 && b2) {
            pt_add(&acc, &acc, &gq);
        } else if (b1) {
            pt_add(&acc, &acc, &g);
        } else if (b2) {
            pt_add(&acc, &acc, &q);
        }
    }
    if (bn_zero(acc.z)) {
        return false;
    }
    mod_inv(zi, acc.z, p384_p);
    mod_mul(zi2, zi, zi, p384_p);
    mod_mul(x, acc.x, zi2, p384_p);
    if (bn_cmp(x, p384_n) >= 0) {
        bn_sub(x, x, p384_n);
    }
    return bn_cmp(x, r) == 0;
}
