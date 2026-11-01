/*
 * EFR32xG22 RADIOAES (see efr32-radioaes.h for sources).
 *
 * A start command runs the whole descriptor chain at once: the fetcher's
 * descriptors deliver configuration (key, mode word) and data to the BA411E
 * engine, the result stream is computed, and the pusher's descriptors write
 * or discard it. The DMA then reports busy until the operation's duration,
 * measured on silicon, has passed in virtual time; firmware reads its
 * outputs only after busy clears, as it must on silicon.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "crypto/aes.h"
#include "exec/address-spaces.h"
#include "hw/arm/efr32-radioaes.h"

/* Register offsets: RM 11.4.1. */
#define RA_FETCHADDR 0x000u
#define RA_FETCHLEN 0x008u
#define RA_FETCHTAG 0x00cu
#define RA_PUSHADDR 0x010u
#define RA_PUSHLEN 0x018u
#define RA_IEN 0x01cu
#define RA_IF 0x028u
#define RA_IF_CLR 0x030u
#define RA_CTRL 0x034u
#define RA_CMD 0x038u
#define RA_STATUS 0x03cu
#define RA_INCL_IPS_HW_CFG 0x400u
#define RA_BA411E_HW_CFG_1 0x404u
#define RA_BA411E_HW_CFG_2 0x408u

/* Bits: RM 11.4.2.7-11. */
#define IF_FETCHERSTOPPED (1u << 1)
#define IF_FETCHERERROR (1u << 2)
#define IF_PUSHERSTOPPED (1u << 4)
#define IF_PUSHERERROR (1u << 5)
#define CTRL_FETCHERSCATTERGATHER (1u << 0)
#define CTRL_PUSHERSCATTERGATHER (1u << 1)
#define CMD_STARTFETCHER (1u << 0)
#define CMD_STARTPUSHER (1u << 1)
#define STATUS_FETCHERBSY (1u << 0)
#define STATUS_PUSHERBSY (1u << 1)
/*
 * Idle STATUS: 0 on silicon (BGM220P, before any operation and after the
 * open firmware's tests: owo-open-fw tests/crypto-port, docs/crypto.md).
 * The model used to return WAITING (bit 5) here, from no measurement.
 */
#define STATUS_IDLE 0u

/*
 * Read-only hardware configuration as read on silicon (BGM220P): AES only.
 * efr32bg22_aes.h gives 0x00000011 as INCL_IPS_HW_CFG's reset value; the
 * part reads 0x00000001. The BA411E words match the header.
 */
#define INCL_IPS_HW_CFG_RESET 0x00000001u
#define BA411E_HW_CFG_1_RESET 0x05010127u
#define BA411E_HW_CFG_2_RESET 0x00000080u

/* Descriptor (sli_radioaes_dma_descr_t): address, next, length, tag. */
#define DESC_NEXT_STOP 0x1u
#define DESC_LEN_MASK 0x0fffffffu
#define DESC_LEN_CONSTADDR (1u << 28)
#define DESC_LEN_REALIGN (1u << 29)
#define DESC_LEN_DISCARD (1u << 30)
#define TAG_ENGINE_MASK 0xfu
#define TAG_ENGINE_BYPASS 0x0u
#define TAG_ENGINE_BA411E 0x1u
#define TAG_IS_CONFIG 0x10u
#define TAG_IS_LAST 0x20u
#define TAG_DATA_HEADER 0x40u
#define TAG_OFFSET(tag) (((tag) >> 8) & 0xffu)
#define TAG_INVALID(tag) (((tag) >> 8) & 0x1fu)

/* BA411E configuration (sli_protocol_crypto_radioaes.c AES_*). */
#define BA411E_CFG_SIZE 128u
#define BA411E_OFFSET_CFG 0u
#define BA411E_OFFSET_KEY 8u
#define MODE_DECRYPT 0x1u
#define MODE_KEYSIZE_MASK 0xcu
#define MODE_AES256 0x4u
#define MODE_AES192 0x8u
#define MODE_ECB 0x100u
#define MODE_CCM 0x2000u
#define MODE_CMAC 0x10000u

#define BLOCK 16u
#define MAX_DESCRIPTORS 64u
#define MAX_STREAM (64u * 1024u + 64u)

/*
 * Duration of one operation, start command to DMA idle, measured on silicon
 * (BGM220P, HCLK 76.8 MHz; owo-open-fw tests/crypto-port with
 * CONFIG_EFR32_RADIOAES_PROBE, docs/crypto.md), in CPU cycles: ECB 1 block
 * 133; CMAC 1 and 5 blocks 192 and 412; CCM-128 with 1, 27 and 251 payload
 * octets (13-octet nonce, 1 octet of AAD) 385, 495 and 2063. These fit a
 * start-up plus a time per AES block within 1 %: ECB and CMAC (CMAC counts
 * its subkey block) 80 + 55 per block; CCM (B0, each header block, each
 * payload block twice for CBC-MAC and CTR, S0) 105 + 56 per block.
 */
#define RADIOAES_START_NS 1042        /* 80 cycles at 76.8 MHz */
#define RADIOAES_BLOCK_NS 716         /* 55 cycles */
#define RADIOAES_CCM_START_NS 1367    /* 105 cycles */
#define RADIOAES_CCM_BLOCK_NS 729     /* 56 cycles */

typedef struct Stream {
    uint8_t *data;
    size_t len;
} Stream;

static void stream_append(Stream *st, const uint8_t *src, size_t len)
{
    if (st->len + len > MAX_STREAM) {
        len = st->len < MAX_STREAM ? MAX_STREAM - st->len : 0;
    }
    memcpy(st->data + st->len, src, len);
    st->len += len;
}

static void xor16(uint8_t *dst, const uint8_t *src)
{
    for (unsigned i = 0; i < BLOCK; i++) {
        dst[i] ^= src[i];
    }
}

static void cmac_double(uint8_t b[BLOCK])
{
    uint8_t carry = b[0] & 0x80u;

    for (unsigned i = 0; i < BLOCK - 1; i++) {
        b[i] = (uint8_t)((b[i] << 1) | (b[i + 1] >> 7));
    }
    b[BLOCK - 1] = (uint8_t)(b[BLOCK - 1] << 1);
    if (carry) {
        b[BLOCK - 1] ^= 0x87u; /* SP 800-38B R_128 */
    }
}

static void do_cmac(const AES_KEY *ek, const uint8_t *m, size_t len,
                    uint8_t out[BLOCK])
{
    uint8_t k1[BLOCK] = { 0 }, k2[BLOCK], x[BLOCK] = { 0 }, last[BLOCK] = { 0 };
    size_t n = len == 0 ? 1 : (len + BLOCK - 1) / BLOCK;
    bool complete = len != 0 && len % BLOCK == 0;

    AES_encrypt(k1, k1, ek);
    cmac_double(k1);
    memcpy(k2, k1, BLOCK);
    cmac_double(k2);

    for (size_t i = 0; i + 1 < n; i++) {
        xor16(x, m + i * BLOCK);
        AES_encrypt(x, x, ek);
    }
    if (complete) {
        memcpy(last, m + (n - 1) * BLOCK, BLOCK);
        xor16(last, k1);
    } else {
        size_t r = len - (n - 1) * BLOCK;

        memcpy(last, m + (n - 1) * BLOCK, r);
        last[r] = 0x80u;
        xor16(last, k2);
    }
    xor16(x, last);
    AES_encrypt(x, out, ek);
}

/* CCM (SP 800-38C) as the BA411E takes it: B0 and the encoded AAD in the header stream. */
static bool do_ccm(const AES_KEY *ek, bool decrypt, const Stream *hdr,
                   const Stream *pay, Stream *out, unsigned *blocks)
{
    uint8_t x[BLOCK], a[BLOCK], s[BLOCK], blk[BLOCK];
    size_t q, t, plen, alen = 0, pos;
    const uint8_t *b0 = hdr->data;
    uint8_t *res;

    if (hdr->len < BLOCK) {
        return false;
    }
    q = (b0[0] & 7u) + 1;
    t = 2 * ((b0[0] >> 3) & 7u) + 2;
    plen = 0;
    for (size_t i = 0; i < q; i++) {
        plen = (plen << 8) | b0[BLOCK - q + i];
    }
    if (b0[0] & 0x40u) {
        if (hdr->len < BLOCK + 2) {
            return false;
        }
        alen = ((size_t)hdr->data[BLOCK] << 8) | hdr->data[BLOCK + 1];
        if (hdr->len < BLOCK + 2 + alen) {
            return false;
        }
    }
    if (pay->len < plen + (decrypt ? t : 0)) {
        return false;
    }

    /* Reflected header blocks (the pusher discards them). */
    out->len = QEMU_ALIGN_UP(hdr->len, BLOCK);
    memset(out->data, 0, out->len);

    /* CBC-MAC over B0, the encoded AAD and the plaintext. */
    memcpy(x, b0, BLOCK);
    AES_encrypt(x, x, ek);
    *blocks = 1;
    for (pos = BLOCK; pos < hdr->len; pos += BLOCK) {
        memset(blk, 0, BLOCK);
        memcpy(blk, hdr->data + pos, MIN(BLOCK, hdr->len - pos));
        xor16(x, blk);
        AES_encrypt(x, x, ek);
        (*blocks)++;
    }

    res = out->data + out->len;
    memset(a, 0, BLOCK);
    a[0] = (uint8_t)(q - 1);
    memcpy(a + 1, b0 + 1, 15 - q);
    for (pos = 0; pos < plen; pos += BLOCK) {
        size_t n = MIN(BLOCK, plen - pos);
        uint64_t ctr = pos / BLOCK + 1;

        for (size_t i = 0; i < q; i++) {
            a[BLOCK - 1 - i] = (uint8_t)(ctr >> (8 * i));
        }
        AES_encrypt(a, s, ek);
        for (size_t i = 0; i < n; i++) {
            res[pos + i] = pay->data[pos + i] ^ s[i];
        }
        memset(blk, 0, BLOCK);
        memcpy(blk, decrypt ? res + pos : pay->data + pos, n);
        xor16(x, blk);
        AES_encrypt(x, x, ek);
        *blocks += 2;
    }
    pos = plen ? QEMU_ALIGN_UP(plen, BLOCK) : BLOCK;
    memset(res + plen, 0, pos - plen);
    out->len += pos;

    /* Tag: T = MSB_t(X ^ S0); a decryption pushes T ^ the received tag. */
    for (size_t i = 0; i < q; i++) {
        a[BLOCK - 1 - i] = 0;
    }
    AES_encrypt(a, s, ek);
    res = out->data + out->len;
    memset(res, 0, BLOCK);
    for (size_t i = 0; i < t; i++) {
        res[i] = x[i] ^ s[i];
        if (decrypt) {
            res[i] ^= pay->data[plen + i];
        }
    }
    out->len += BLOCK;
    (*blocks)++;
    return true;
}

static void radioaes_run(Efr32RadioAes *ra, AddressSpace *as)
{
    uint8_t cfg[BA411E_CFG_SIZE] = { 0 };
    Stream hdr = { g_malloc(MAX_STREAM), 0 };
    Stream pay = { g_malloc(MAX_STREAM), 0 };
    Stream out = { g_malloc0(2 * MAX_STREAM + 2 * BLOCK), 0 };
    uint32_t addr = ra->fetchaddr;
    unsigned blocks = 0;
    bool fetch_error = false, push_error = false;
    uint32_t mode, d[4];
    AES_KEY ek, dk;
    unsigned keybits;
    size_t pos = 0;
    uint32_t hdr_pos = 0, pay_pos = 0;

    /* Fetcher. */
    for (unsigned n = 0; n < MAX_DESCRIPTORS; n++) {
        uint32_t len, tag;
        uint8_t *buf;

        if (address_space_read(as, addr & ~3u, MEMTXATTRS_UNSPECIFIED, d,
                               sizeof(d)) != MEMTX_OK) {
            fetch_error = true;
            break;
        }
        len = le32_to_cpu(d[2]) & DESC_LEN_MASK;
        tag = le32_to_cpu(d[3]);
        len = MIN(len, MAX_STREAM);
        buf = g_malloc(len + 4);
        if (le32_to_cpu(d[2]) & DESC_LEN_CONSTADDR) {
            uint8_t w[4];

            if (address_space_read(as, le32_to_cpu(d[0]), MEMTXATTRS_UNSPECIFIED,
                                   w, 4) != MEMTX_OK) {
                fetch_error = true;
            }
            for (uint32_t i = 0; i < len; i++) {
                buf[i] = w[i & 3];
            }
        } else if (len && address_space_read(as, le32_to_cpu(d[0]),
                                             MEMTXATTRS_UNSPECIFIED, buf,
                                             len) != MEMTX_OK) {
            fetch_error = true;
        }
        if ((tag & TAG_ENGINE_MASK) == TAG_ENGINE_BYPASS) {
            stream_append(&out, buf, len);
        } else if ((tag & TAG_ENGINE_MASK) == TAG_ENGINE_BA411E) {
            if (tag & TAG_IS_CONFIG) {
                uint32_t off = TAG_OFFSET(tag);

                if (off < BA411E_CFG_SIZE) {
                    memcpy(cfg + off, buf, MIN(len, BA411E_CFG_SIZE - off));
                }
            } else {
                /*
                 * The invalid bytes are the padding of the 16-byte block in
                 * which this descriptor's data ends, counted back from that
                 * block's end, whether the descriptor delivers them (ECB,
                 * CMAC) or not (CCM, REALIGN). The next data of the same type
                 * starts a new block. Every descriptor sequence of
                 * sli_protocol_crypto_radioaes.c and of the open driver is
                 * consistent with this reading.
                 */
                bool header = tag & TAG_DATA_HEADER;
                uint32_t *pos = header ? &hdr_pos : &pay_pos;
                uint32_t valid = len;

                if (TAG_INVALID(tag) || (tag & TAG_IS_LAST)) {
                    uint32_t end = QEMU_ALIGN_UP(*pos + len, BLOCK);
                    int64_t v = (int64_t)end - TAG_INVALID(tag) - *pos;

                    valid = v < 0 ? 0 : MIN((uint32_t)v, len);
                    *pos = end;
                } else {
                    *pos += len;
                }
                stream_append(header ? &hdr : &pay, buf, valid);
            }
        } else {
            qemu_log_mask(LOG_UNIMP, "efr32 RADIOAES: engine %u\n",
                          tag & TAG_ENGINE_MASK);
        }
        g_free(buf);
        if (fetch_error || (le32_to_cpu(d[1]) & DESC_NEXT_STOP)) {
            break;
        }
        addr = le32_to_cpu(d[1]);
    }

    /* Engine. */
    mode = ldl_le_p(cfg + BA411E_OFFSET_CFG);
    keybits = (mode & MODE_KEYSIZE_MASK) == MODE_AES256 ? 256
              : (mode & MODE_KEYSIZE_MASK) == MODE_AES192 ? 192 : 128;
    AES_set_encrypt_key(cfg + BA411E_OFFSET_KEY, keybits, &ek);
    AES_set_decrypt_key(cfg + BA411E_OFFSET_KEY, keybits, &dk);
    if (!fetch_error) {
        if (mode & MODE_CMAC) {
            do_cmac(&ek, pay.data, pay.len, out.data);
            out.len = BLOCK;
            blocks = (pay.len ? (pay.len + BLOCK - 1) / BLOCK : 1) + 1;
        } else if (mode & MODE_CCM) {
            if (!do_ccm(&ek, mode & MODE_DECRYPT, &hdr, &pay, &out, &blocks)) {
                qemu_log_mask(LOG_GUEST_ERROR, "efr32 RADIOAES: bad CCM input\n");
            }
        } else if (mode & MODE_ECB) {
            for (size_t i = 0; i + BLOCK <= pay.len; i += BLOCK) {
                if (mode & MODE_DECRYPT) {
                    AES_decrypt(pay.data + i, out.data + i, &dk);
                } else {
                    AES_encrypt(pay.data + i, out.data + i, &ek);
                }
                blocks++;
            }
            out.len = pay.len;
        } else {
            qemu_log_mask(LOG_UNIMP, "efr32 RADIOAES: mode 0x%x\n", mode);
        }
    }

    /* Pusher. */
    addr = ra->pushaddr;
    for (unsigned n = 0; n < MAX_DESCRIPTORS && !push_error; n++) {
        uint32_t len, flags;

        if (address_space_read(as, addr & ~3u, MEMTXATTRS_UNSPECIFIED, d,
                               sizeof(d)) != MEMTX_OK) {
            push_error = true;
            break;
        }
        flags = le32_to_cpu(d[2]);
        len = flags & DESC_LEN_MASK;
        if (!(flags & DESC_LEN_DISCARD) && len) {
            uint8_t *buf = g_malloc0(len);

            if (pos < out.len) {
                memcpy(buf, out.data + pos, MIN(len, out.len - pos));
            }
            if (address_space_write(as, le32_to_cpu(d[0]), MEMTXATTRS_UNSPECIFIED,
                                    buf, len) != MEMTX_OK) {
                push_error = true;
            }
            g_free(buf);
        }
        pos += len;
        if (le32_to_cpu(d[1]) & DESC_NEXT_STOP) {
            break;
        }
        addr = le32_to_cpu(d[1]);
    }

    ra->if_flags |= IF_FETCHERSTOPPED | IF_PUSHERSTOPPED |
                    (fetch_error ? IF_FETCHERERROR : 0) |
                    (push_error ? IF_PUSHERERROR : 0);
    ra->busy_until_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
        ((mode & MODE_CCM) ? RADIOAES_CCM_START_NS + (int64_t)blocks * RADIOAES_CCM_BLOCK_NS
                           : RADIOAES_START_NS + (int64_t)blocks * RADIOAES_BLOCK_NS);
    ra->operations++;
    g_free(hdr.data);
    g_free(pay.data);
    g_free(out.data);
}

void efr32_radioaes_reset(Efr32RadioAes *a)
{
    memset(a, 0, sizeof(*a));
}

bool efr32_radioaes_read(Efr32RadioAes *a, uint32_t offset, uint32_t *value)
{
    switch (offset) {
    case RA_FETCHADDR: *value = a->fetchaddr; return true;
    case RA_FETCHLEN: *value = a->fetchlen; return true;
    case RA_FETCHTAG: *value = a->fetchtag; return true;
    case RA_PUSHADDR: *value = a->pushaddr; return true;
    case RA_PUSHLEN: *value = a->pushlen; return true;
    case RA_IEN: *value = a->ien; return true;
    case RA_IF: *value = a->if_flags; return true;
    case RA_IF_CLR: case RA_CMD: *value = 0; return true;
    case RA_CTRL: *value = a->ctrl; return true;
    case RA_STATUS:
        *value = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < a->busy_until_ns
                     ? STATUS_FETCHERBSY | STATUS_PUSHERBSY
                     : STATUS_IDLE;
        return true;
    case RA_INCL_IPS_HW_CFG: *value = INCL_IPS_HW_CFG_RESET; return true;
    case RA_BA411E_HW_CFG_1: *value = BA411E_HW_CFG_1_RESET; return true;
    case RA_BA411E_HW_CFG_2: *value = BA411E_HW_CFG_2_RESET; return true;
    default:
        return false;
    }
}

bool efr32_radioaes_write(Efr32RadioAes *a, AddressSpace *as, uint32_t offset,
                          uint32_t value)
{
    switch (offset) {
    case RA_FETCHADDR: a->fetchaddr = value; return true;
    case RA_FETCHLEN: a->fetchlen = value; return true;
    case RA_FETCHTAG: a->fetchtag = value; return true;
    case RA_PUSHADDR: a->pushaddr = value; return true;
    case RA_PUSHLEN: a->pushlen = value; return true;
    case RA_IEN: a->ien = value & 0x3fu; return true;
    case RA_IF_CLR: a->if_flags &= ~value; return true;
    case RA_CTRL: a->ctrl = value & 0x1fu; return true;
    case RA_CMD:
        if ((value & (CMD_STARTFETCHER | CMD_STARTPUSHER)) ==
                (CMD_STARTFETCHER | CMD_STARTPUSHER) &&
            (a->ctrl & (CTRL_FETCHERSCATTERGATHER | CTRL_PUSHERSCATTERGATHER)) ==
                (CTRL_FETCHERSCATTERGATHER | CTRL_PUSHERSCATTERGATHER)) {
            radioaes_run(a, as);
        } else if (value & (CMD_STARTFETCHER | CMD_STARTPUSHER)) {
            qemu_log_mask(LOG_UNIMP,
                          "efr32 RADIOAES: direct mode or split start (CTRL 0x%x CMD 0x%x)\n",
                          a->ctrl, value);
        }
        return true;
    case RA_STATUS:
        return true;
    default:
        return false;
    }
}
