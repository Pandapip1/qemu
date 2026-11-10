/*
 * Log every call into a ROM window from code outside it: caller (lr),
 * target, r0-r3 and the first four stack words, in order. Each (lr, target)
 * pair is logged for its first `each` occurrences only; at exit a summary of
 * every pair's total count is appended, so hot loops stay bounded.
 *
 * Arguments:
 *   lo=ADDR,hi=ADDR  ROM window [lo, hi) (default 0x400000-0x59b000)
 *   out=FILE         output TSV (default romcall.tsv)
 *   start=ADDR       begin logging when a TB at ADDR first runs (default 0)
 *   stop=ADDR        stop logging when a TB at ADDR runs (default none)
 *   each=N           log each (lr, target) pair at most N times (default 3)
 *
 * A call is a ROM TB entered from a TB outside the window whose lr does not
 * point back at it (that would be a RAM callee returning into ROM).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t lo = 0x400000, hi = 0x59b000, start, stop = UINT64_MAX;
static uint64_t each = 3;
static GHashTable *pairs;      /* (lr << 32 | target) -> count */
static bool logging;
static FILE *out;
static uint64_t prev_tb, seq;
static struct qemu_plugin_register *reg[16];


static bool in_rom(uint64_t a)
{
    return a >= lo && a < hi;
}

static uint32_t rd(int n)
{
    g_autoptr(GByteArray) buf = g_byte_array_new();
    uint32_t v = 0;

    if (reg[n] && qemu_plugin_read_register(reg[n], buf) && buf->len >= 4) {
        memcpy(&v, buf->data, 4);
    }
    return v;
}

static void vcpu_init(unsigned int vcpu_index, void *userdata)
{
    g_autoptr(GArray) regs = qemu_plugin_get_registers();

    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        int n = -1;

        if (d->name[0] == 'r' && g_ascii_isdigit(d->name[1])) {
            n = atoi(d->name + 1);
        } else if (!strcmp(d->name, "sp")) {
            n = 13;
        } else if (!strcmp(d->name, "lr")) {
            n = 14;
        }
        if (n >= 0 && n < 16) {
            reg[n] = d->handle;
        }
    }
}

static void check_bounds(uint64_t tb)
{
    if (!logging && tb == start) {
        logging = true;
    }
    if (logging && tb == stop) {
        logging = false;
        start = UINT64_MAX;             /* never restart */
    }
}

static void tb_outside(unsigned int vcpu_index, void *userdata)
{
    prev_tb = (uint64_t)userdata;
    check_bounds(prev_tb);
}

static void tb_rom(unsigned int vcpu_index, void *userdata)
{
    uint64_t tb = (uint64_t)userdata, from = prev_tb, key, n;
    uint32_t lr, sp, st[4] = { 0 };
    g_autoptr(GByteArray) buf = NULL;

    prev_tb = tb;
    check_bounds(tb);
    if (!logging || in_rom(from)) {
        return;
    }
    lr = rd(14);
    if ((lr & ~1u) == tb) {
        return;                         /* RAM callee returning into ROM */
    }
    seq++;
    key = (uint64_t)lr << 32 | tb;
    n = GPOINTER_TO_SIZE(g_hash_table_lookup(pairs, (gpointer)key)) + 1;
    g_hash_table_insert(pairs, (gpointer)key, GSIZE_TO_POINTER(n));
    if (n > each) {
        return;
    }
    sp = rd(13);
    buf = g_byte_array_new();
    if (qemu_plugin_read_memory_vaddr(sp, buf, 16) && buf->len >= 16) {
        memcpy(st, buf->data, 16);
    }
    fprintf(out, "%" PRIu64 "\t%08x\t%08" PRIx64 "\t%08" PRIx64 "\t%08x\t%08x"
            "\t%08x\t%08x\t%08x\t%08x\t%08x\t%08x\t%" PRIu64 "\n", seq, lr,
            from, tb, rd(0), rd(1), rd(2), rd(3), st[0], st[1], st[2], st[3], n);
}

static void vcpu_tb_trans(struct qemu_plugin_tb *tb, void *userdata)
{
    uint64_t va = qemu_plugin_tb_vaddr(tb);

    if (in_rom(va)) {
        qemu_plugin_register_vcpu_tb_exec_cb(tb, tb_rom, QEMU_PLUGIN_CB_R_REGS,
                                             (void *)va);
    } else {
        qemu_plugin_register_vcpu_tb_exec_cb(tb, tb_outside,
                                             QEMU_PLUGIN_CB_NO_REGS,
                                             (void *)va);
    }
}

static void plugin_exit(void *userdata)
{
    GHashTableIter it;
    gpointer k, v;

    fprintf(out, "# totals: lr\ttarget\tcount\n");
    g_hash_table_iter_init(&it, pairs);
    while (g_hash_table_iter_next(&it, &k, &v)) {
        fprintf(out, "#\t%08x\t%08x\t%zu\n", (uint32_t)((uint64_t)k >> 32),
                (uint32_t)(uint64_t)k, GPOINTER_TO_SIZE(v));
    }
    fclose(out);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *path = "romcall.tsv";

    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) kv = g_strsplit(argv[i], "=", 2);

        if (!kv[1]) {
            fprintf(stderr, "romcall: bad option %s\n", argv[i]);
            return -1;
        }
        if (!strcmp(kv[0], "lo")) {
            lo = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "hi")) {
            hi = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "start")) {
            start = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "stop")) {
            stop = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "each")) {
            each = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "out")) {
            path = argv[i] + 4;
        } else {
            fprintf(stderr, "romcall: unknown option %s\n", kv[0]);
            return -1;
        }
    }
    out = fopen(path, "w");
    if (!out) {
        perror(path);
        return -1;
    }
    setvbuf(out, NULL, _IOFBF, 1 << 20);
    fprintf(out, "seq\tlr\tprev_tb\ttarget\tr0\tr1\tr2\tr3\tsp0\tsp4\tsp8\tsp12"
            "\tnth\n");
    logging = start == 0;
    pairs = g_hash_table_new(g_direct_hash, g_direct_equal);
    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
