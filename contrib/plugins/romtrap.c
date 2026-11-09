/*
 * Trap execution inside an address window (an unavailable ROM), log each
 * entry with its caller, and optionally return to the caller with r0 = 0.
 *
 * Arguments:
 *   lo=ADDR,hi=ADDR  window [lo, hi) (default 0x400000-0x580000)
 *   stub=on|off      return to lr with r0 = 0 (default on)
 *   limit=N          stop logging individual calls after N (default 200)
 *
 * At exit, prints every distinct entry point with its first caller and count.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

typedef struct {
    uint64_t target;
    uint64_t first_lr;
    uint64_t count;
    uint64_t order;
} Entry;

static uint64_t lo = 0x400000, hi = 0x580000;
static bool stub = true;
static uint64_t limit = 200;
static uint64_t ncalls;
static GHashTable *entries;
static GMutex lock;
static struct qemu_plugin_register *reg[16];

static uint32_t rd(int n)
{
    g_autoptr(GByteArray) buf = g_byte_array_new();
    uint32_t v = 0;

    if (reg[n] && qemu_plugin_read_register(reg[n], buf) && buf->len >= 4) {
        memcpy(&v, buf->data, 4);
    }
    return v;
}

static void wr(int n, uint32_t v)
{
    g_autoptr(GByteArray) buf = g_byte_array_new();

    g_byte_array_append(buf, (guint8 *)&v, 4);
    qemu_plugin_write_register(reg[n], buf);
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
        } else if (!strcmp(d->name, "pc")) {
            n = 15;
        }
        if (n >= 0 && n < 16) {
            reg[n] = d->handle;
        }
    }
}

/*
 * One callback per executed instruction. We detect the *edge* into the ROM
 * window (previous insn outside, this insn inside) rather than trusting the
 * TB start address, so an indirect/relocated branch is attributed to the
 * instruction actually reached, with its true caller (the previous PC).
 */
static uint64_t prev_pc;

static void insn_exec(unsigned int vcpu_index, void *userdata)
{
    uint64_t pc = (uint64_t)userdata;
    bool in = pc >= lo && pc < hi;
    bool prev_in = prev_pc >= lo && prev_pc < hi;
    uint64_t caller, lr;
    Entry *e;

    if (!in || prev_in) {
        prev_pc = pc;
        return;
    }

    caller = prev_pc;
    lr = rd(14);
    g_mutex_lock(&lock);
    ncalls++;
    e = g_hash_table_lookup(entries, &pc);
    if (!e) {
        e = g_new0(Entry, 1);
        e->target = pc;
        e->first_lr = caller;
        e->order = g_hash_table_size(entries);
        g_hash_table_insert(entries, &e->target, e);
    }
    e->count++;
    if (ncalls <= limit) {
        g_autofree char *s = g_strdup_printf(
            "romtrap: #%" PRIu64 " 0x%06" PRIx64 " from 0x%08" PRIx64
            " lr=0x%08x r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x sp=0x%08x%s\n",
            ncalls, pc, caller, (uint32_t)lr, rd(0), rd(1), rd(2), rd(3),
            rd(13), e->count == 1 ? " NEW" : "");
        qemu_plugin_outs(s);
    }
    g_mutex_unlock(&lock);

    if (stub) {
        prev_pc = lr;
        wr(0, 0);
        qemu_plugin_set_pc(lr);
    }
    prev_pc = pc;
}

static void vcpu_tb_trans(struct qemu_plugin_tb *tb, void *userdata)
{
    size_t n = qemu_plugin_tb_n_insns(tb);

    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t vaddr = qemu_plugin_insn_vaddr(insn);

        qemu_plugin_register_vcpu_insn_exec_cb(insn, insn_exec,
                                               QEMU_PLUGIN_CB_RW_REGS_PC,
                                               (void *)vaddr);
    }
}

static gint by_order(gconstpointer a, gconstpointer b)
{
    const Entry *x = *(Entry **)a, *y = *(Entry **)b;

    return x->order < y->order ? -1 : x->order > y->order;
}

static void plugin_exit(void *userdata)
{
    g_autoptr(GPtrArray) all = g_ptr_array_new();
    GHashTableIter it;
    gpointer v;

    g_hash_table_iter_init(&it, entries);
    while (g_hash_table_iter_next(&it, NULL, &v)) {
        g_ptr_array_add(all, v);
    }
    g_ptr_array_sort(all, by_order);
    g_autofree char *hdr = g_strdup_printf("romtrap: %u distinct entries, %"
                                           PRIu64 " calls\n", all->len, ncalls);
    qemu_plugin_outs(hdr);
    for (guint i = 0; i < all->len; i++) {
        Entry *e = g_ptr_array_index(all, i);
        g_autofree char *s = g_strdup_printf(
            "romtrap: entry %3u 0x%06" PRIx64 " from=0x%08" PRIx64
            " count=%" PRIu64 "\n", i, e->target, e->first_lr, e->count);
        qemu_plugin_outs(s);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) kv = g_strsplit(argv[i], "=", 2);

        if (!kv[1]) {
            continue;
        }
        if (!strcmp(kv[0], "lo")) {
            lo = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "hi")) {
            hi = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "limit")) {
            limit = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "stub")) {
            if (!qemu_plugin_bool_parse(kv[0], kv[1], &stub)) {
                return -1;
            }
        }
    }
    entries = g_hash_table_new(g_int64_hash, g_int64_equal);
    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
