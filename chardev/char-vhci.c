/*
 * Linux virtual HCI (hci_vhci) as a character device: pass /dev/vhci
 * through to an emulated HCI UART transport.
 *
 * The backend owns the host kernel's virtual controller and is otherwise
 * silicon-independent.  Its frontend is whatever emulated UART carries the
 * guest controller's H4 stream (Bluetooth Core, Vol 4, Part A); the host
 * kernel then sees that controller exactly as it would see a real
 * UART-attached one (hciattach/btattach), as a new hciN.
 *
 * /dev/vhci is packet-oriented: every read() returns one packet and every
 * write() must carry one, each prefixed with its H4 packet indicator.  The
 * backend therefore splits the guest's byte stream into H4 packets on the
 * way to the host and serialises host packets into the byte stream on the
 * way to the guest, pacing them by what the frontend accepts.
 *
 * The host controller is created when the frontend can first receive (the
 * guest's UART receiver is up), the moment a host would run btattach on a
 * booted controller: the kernel sends HCI_Reset as soon as the controller
 * registers and gives it HCI_CMD_TIMEOUT to answer.
 *
 *   -chardev vhci,id=<id>[,path=/dev/vhci]
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/option.h"
#include "chardev/char.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_CHARDEV_VHCI "chardev-vhci"
OBJECT_DECLARE_SIMPLE_TYPE(VhciChardev, CHARDEV_VHCI)

#define VHCI_DEFAULT_PATH "/dev/vhci"

/* H4 packet indicators (Core Vol 4, Part A, 2) and the vhci vendor packet. */
enum {
    H4_COMMAND = 0x01,
    H4_ACL = 0x02,
    H4_SCO = 0x03,
    H4_EVENT = 0x04,
    H4_ISO = 0x05,
    /* include/net/bluetooth/hci.h HCI_VENDOR_PKT: vhci's control packet. */
    VHCI_VENDOR = 0xff,
};

/* drivers/bluetooth/hci_vhci.c: create opcode bits 1:0 are the type. */
#define VHCI_CREATE_PRIMARY 0x00
/* The kernel's reply: indicator, opcode, controller index (LE16). */
#define VHCI_CREATE_REPLY_LEN 4

/*
 * Largest H4 packet: indicator, the longest header (ACL and ISO, 4 bytes)
 * and the largest 16-bit payload length.
 */
#define H4_INDICATOR_LEN 1
#define H4_MAX_HEADER_LEN 4
#define H4_MAX_PACKET (H4_INDICATOR_LEN + H4_MAX_HEADER_LEN + UINT16_MAX)
/* ISO_Data_Load_Length is 14 bits (Core Vol 4, Part E, 5.4.5). */
#define H4_ISO_LENGTH_MASK 0x3fff

struct VhciChardev {
    Chardev parent;

    char *path;
    int fd;
    bool attached;
    /* Guest -> host: the H4 packet being assembled. */
    uint8_t *tx_packet;
    size_t tx_len;
    /* Host -> guest: the packet being serialised, and how far. */
    uint8_t *rx_packet;
    size_t rx_len;
    size_t rx_offset;
    bool reading;
};

/*
 * Bytes of header after the indicator, and the payload length once the
 * header is complete; false for an indicator H4 does not define.
 */
static bool vhci_h4_header(const uint8_t *packet, size_t *header_len)
{
    switch (packet[0]) {
    case H4_COMMAND:
    case H4_SCO:
        *header_len = 3;
        return true;
    case H4_EVENT:
        *header_len = 2;
        return true;
    case H4_ACL:
    case H4_ISO:
        *header_len = 4;
        return true;
    default:
        return false;
    }
}

static size_t vhci_h4_payload_len(const uint8_t *packet)
{
    const uint8_t *header = packet + H4_INDICATOR_LEN;

    switch (packet[0]) {
    case H4_COMMAND:
    case H4_SCO:
        return header[2];
    case H4_EVENT:
        return header[1];
    case H4_ACL:
        return lduw_le_p(header + 2);
    case H4_ISO:
        return lduw_le_p(header + 2) & H4_ISO_LENGTH_MASK;
    default:
        g_assert_not_reached();
    }
}

static void vhci_read(void *opaque);

static void vhci_set_reading(VhciChardev *s, bool reading)
{
    if (s->fd < 0 || s->reading == reading) {
        return;
    }
    s->reading = reading;
    qemu_set_fd_handler(s->fd, reading ? vhci_read : NULL, NULL, s);
}

/* Hand the frontend as much of the current host packet as it takes. */
static void vhci_flush_to_guest(VhciChardev *s)
{
    Chardev *chr = CHARDEV(s);

    while (s->rx_offset < s->rx_len) {
        int room = qemu_chr_be_can_write(chr);
        size_t chunk;

        if (room <= 0) {
            break;
        }
        chunk = MIN((size_t)room, s->rx_len - s->rx_offset);
        qemu_chr_be_write(chr, s->rx_packet + s->rx_offset, chunk);
        s->rx_offset += chunk;
    }
    /* Read the next packet only once this one is through. */
    vhci_set_reading(s, s->rx_offset == s->rx_len);
}

static void vhci_read(void *opaque)
{
    VhciChardev *s = opaque;
    ssize_t len = read(s->fd, s->rx_packet, H4_MAX_PACKET);

    if (len < 0) {
        if (errno != EAGAIN && errno != EINTR) {
            error_report("vhci %s: read: %s", s->path, strerror(errno));
            vhci_set_reading(s, false);
        }
        return;
    }
    if (len == 0) {
        return;
    }
    if (s->rx_packet[0] == VHCI_VENDOR) {
        if (len >= VHCI_CREATE_REPLY_LEN) {
            trace_vhci_created(s->path, lduw_le_p(s->rx_packet + 2));
        }
        return;
    }
    s->rx_len = len;
    s->rx_offset = 0;
    vhci_flush_to_guest(s);
}

static void vhci_attach(VhciChardev *s)
{
    static const uint8_t create[] = {VHCI_VENDOR, VHCI_CREATE_PRIMARY};
    Error *err = NULL;

    s->attached = true;
    s->fd = qemu_open(s->path, O_RDWR | O_CLOEXEC, &err);
    if (s->fd < 0) {
        error_report_err(err);
        return;
    }
    if (!g_unix_set_fd_nonblocking(s->fd, true, NULL) ||
        write(s->fd, create, sizeof(create)) != sizeof(create)) {
        error_report("vhci %s: cannot create a controller: %s", s->path,
                     strerror(errno));
        close(s->fd);
        s->fd = -1;
        return;
    }
    vhci_set_reading(s, true);
}

static void vhci_accept_input(Chardev *chr)
{
    VhciChardev *s = CHARDEV_VHCI(chr);

    if (!s->attached) {
        if (qemu_chr_be_can_write(chr) > 0) {
            vhci_attach(s);
        }
        return;
    }
    vhci_flush_to_guest(s);
}

static void vhci_send_packet(VhciChardev *s)
{
    ssize_t written;

    if (s->fd < 0) {
        trace_vhci_drop(s->path, s->tx_packet[0], s->tx_len);
        return;
    }
    do {
        written = write(s->fd, s->tx_packet, s->tx_len);
    } while (written < 0 && errno == EINTR);
    if (written != s->tx_len) {
        error_report("vhci %s: write of a %zu-byte packet: %s", s->path,
                     s->tx_len, written < 0 ? strerror(errno) : "short");
    }
}

static int vhci_chr_write(Chardev *chr, const uint8_t *buf, int len)
{
    VhciChardev *s = CHARDEV_VHCI(chr);

    for (int i = 0; i < len; i++) {
        size_t header_len;

        s->tx_packet[s->tx_len++] = buf[i];
        if (!vhci_h4_header(s->tx_packet, &header_len)) {
            /* H4 cannot resynchronise; skip to the next byte. */
            trace_vhci_bad_indicator(s->path, s->tx_packet[0]);
            s->tx_len = 0;
            continue;
        }
        if (s->tx_len < H4_INDICATOR_LEN + header_len ||
            s->tx_len < H4_INDICATOR_LEN + header_len +
                        vhci_h4_payload_len(s->tx_packet)) {
            continue;
        }
        vhci_send_packet(s);
        s->tx_len = 0;
    }
    return len;
}

static void vhci_open(Chardev *chr, ChardevBackend *backend, bool *be_opened,
                      Error **errp)
{
    VhciChardev *s = CHARDEV_VHCI(chr);
    ChardevHostdev *opts = backend->u.vhci.data;

    s->path = g_strdup(opts->device);
    s->fd = -1;
    s->tx_packet = g_malloc(H4_MAX_PACKET);
    s->rx_packet = g_malloc(H4_MAX_PACKET);
    *be_opened = true;
}

static void vhci_finalize(Object *obj)
{
    VhciChardev *s = CHARDEV_VHCI(obj);

    if (s->fd >= 0) {
        vhci_set_reading(s, false);
        close(s->fd);
    }
    g_free(s->path);
    g_free(s->tx_packet);
    g_free(s->rx_packet);
}

static void vhci_parse(QemuOpts *opts, ChardevBackend *backend, Error **errp)
{
    const char *path = qemu_opt_get(opts, "path");
    ChardevHostdev *vhci;

    backend->type = CHARDEV_BACKEND_KIND_VHCI;
    vhci = backend->u.vhci.data = g_new0(ChardevHostdev, 1);
    qemu_chr_parse_common(opts, qapi_ChardevHostdev_base(vhci));
    vhci->device = g_strdup(path ? path : VHCI_DEFAULT_PATH);
}

static void vhci_class_init(ObjectClass *oc, void *data)
{
    ChardevClass *cc = CHARDEV_CLASS(oc);

    cc->parse = vhci_parse;
    cc->open = vhci_open;
    cc->chr_write = vhci_chr_write;
    cc->chr_accept_input = vhci_accept_input;
    /* A frontend that is ready as soon as it attaches. */
    cc->chr_update_read_handler = vhci_accept_input;
}

static const TypeInfo vhci_type_info = {
    .name = TYPE_CHARDEV_VHCI,
    .parent = TYPE_CHARDEV,
    .instance_size = sizeof(VhciChardev),
    .instance_finalize = vhci_finalize,
    .class_init = vhci_class_init,
};

static void register_types(void)
{
    type_register_static(&vhci_type_info);
}

type_init(register_types);
