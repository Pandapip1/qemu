/*
 * USB gadget passthrough backend on Linux raw-gadget (see the header).
 *
 * Kernel interface: Documentation/usb/raw-gadget.rst and
 * include/uapi/linux/usb/raw_gadget.h.  raw-gadget calls block, so endpoint
 * 0 and every enabled endpoint get a thread; frontend callbacks run with
 * the BQL held.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/ioctl.h>
#include <linux/usb/ch9.h>
#include <linux/usb/raw_gadget.h>
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/cutils.h"
#include "qom/object_interfaces.h"
#include "hw/usb/gadget-host.h"

#define GADGET_DEFAULT_PATH "/dev/raw-gadget"
/* bMaxPower and USB_RAW_IOCTL_VBUS_DRAW are in 2 mA units (USB 2.0 9.6.3) */
#define GADGET_VBUS_UNIT_MA 2
/* IN threads wake this often to notice a disabled endpoint */
#define GADGET_IN_POLL_US (100 * 1000)
#define GADGET_EP0_MAX_DATA 0xffff
#define GADGET_EP_DESC_SIZE 7
#define GADGET_EP_ADDR_DIR_IN 0x80
#define GADGET_EP_DESC_ADDR 2
#define GADGET_EP_DESC_MAXPACKET 4

typedef struct GadgetEp {
    UsbGadgetHost *g;
    uint8_t addr;
    int handle;
    uint16_t maxpacket;
    unsigned generation;
    GAsyncQueue *queue;      /* IN: GByteArray buffers to send */
} GadgetEp;

struct UsbGadgetHost {
    Object parent_obj;

    char *path;
    char *udc_driver;
    char *udc_device;

    int fd;
    const UsbGadgetHostOps *ops;
    void *opaque;
    QemuThread ep0_thread;
    QemuMutex lock;
    GadgetEp eps[USB_RAW_EPS_NUM_MAX];
    unsigned num_eps;
    unsigned generation;
};

static bool gadget_ep_current(GadgetEp *ep, unsigned generation)
{
    bool current;

    qemu_mutex_lock(&ep->g->lock);
    current = ep->generation == generation && ep->handle >= 0;
    qemu_mutex_unlock(&ep->g->lock);
    return current;
}

typedef struct GadgetEpThreadArg {
    GadgetEp *ep;
    unsigned generation;
} GadgetEpThreadArg;

static void *gadget_ep_out_thread(void *opaque)
{
    GadgetEpThreadArg *arg = opaque;
    GadgetEp *ep = arg->ep;
    UsbGadgetHost *g = ep->g;
    unsigned generation = arg->generation;
    /* One packet per request: a transfer need not end in a short packet */
    g_autofree struct usb_raw_ep_io *io =
        g_malloc0(sizeof(*io) + ep->maxpacket);

    g_free(arg);
    while (gadget_ep_current(ep, generation)) {
        int n;

        io->ep = ep->handle;
        io->flags = 0;
        io->length = ep->maxpacket;
        n = ioctl(g->fd, USB_RAW_IOCTL_EP_READ, io);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        bql_lock();
        g->ops->ep_out(g->opaque, ep->addr, io->data, n);
        bql_unlock();
    }
    return NULL;
}

static void *gadget_ep_in_thread(void *opaque)
{
    GadgetEpThreadArg *arg = opaque;
    GadgetEp *ep = arg->ep;
    UsbGadgetHost *g = ep->g;
    unsigned generation = arg->generation;

    g_free(arg);
    while (gadget_ep_current(ep, generation)) {
        GByteArray *data = g_async_queue_timeout_pop(ep->queue,
                                                     GADGET_IN_POLL_US);
        g_autofree struct usb_raw_ep_io *io = NULL;

        if (!data) {
            continue;
        }
        io = g_malloc(sizeof(*io) + data->len);
        io->ep = ep->handle;
        /* End a transfer that fills whole packets with a zero-length one */
        io->flags = (data->len % ep->maxpacket) ? 0 : USB_RAW_IO_FLAGS_ZERO;
        io->length = data->len;
        memcpy(io->data, data->data, data->len);
        g_byte_array_unref(data);
        while (ioctl(g->fd, USB_RAW_IOCTL_EP_WRITE, io) < 0 && errno == EINTR) {
        }
    }
    return NULL;
}

static void gadget_disable_all(UsbGadgetHost *g)
{
    qemu_mutex_lock(&g->lock);
    g->generation++;
    for (unsigned i = 0; i < g->num_eps; i++) {
        GadgetEp *ep = &g->eps[i];

        if (ep->handle >= 0) {
            ioctl(g->fd, USB_RAW_IOCTL_EP_DISABLE, ep->handle);
            ep->handle = -1;
        }
        ep->generation = g->generation;
    }
    qemu_mutex_unlock(&g->lock);
}

bool usb_gadget_host_ep_enable(UsbGadgetHost *g, const uint8_t *desc)
{
    struct usb_endpoint_descriptor d = { 0 };
    GadgetEp *ep = NULL;
    GadgetEpThreadArg *arg;
    int handle;

    memcpy(&d, desc, GADGET_EP_DESC_SIZE);
    qemu_mutex_lock(&g->lock);
    for (unsigned i = 0; i < g->num_eps; i++) {
        if (g->eps[i].addr == d.bEndpointAddress) {
            ep = &g->eps[i];
        }
    }
    if (!ep) {
        if (g->num_eps == ARRAY_SIZE(g->eps)) {
            qemu_mutex_unlock(&g->lock);
            return false;
        }
        ep = &g->eps[g->num_eps++];
        ep->g = g;
        ep->addr = d.bEndpointAddress;
        ep->handle = -1;
        ep->queue = g_async_queue_new_full((GDestroyNotify)g_byte_array_unref);
    }
    if (ep->handle >= 0) {
        ioctl(g->fd, USB_RAW_IOCTL_EP_DISABLE, ep->handle);
    }
    handle = ioctl(g->fd, USB_RAW_IOCTL_EP_ENABLE, &d);
    if (handle < 0) {
        error_report("usb-gadget-host: the UDC cannot provide endpoint "
                     "0x%02x: %s", d.bEndpointAddress, strerror(errno));
        ep->handle = -1;
        qemu_mutex_unlock(&g->lock);
        return false;
    }
    ep->handle = handle;
    ep->maxpacket = lduw_le_p(desc + GADGET_EP_DESC_MAXPACKET);
    ep->generation = ++g->generation;
    qemu_mutex_unlock(&g->lock);

    arg = g_new(GadgetEpThreadArg, 1);
    arg->ep = ep;
    arg->generation = ep->generation;
    {
        QemuThread thread;

        qemu_thread_create(&thread, "usb-gadget-ep",
                           (ep->addr & GADGET_EP_ADDR_DIR_IN) ?
                           gadget_ep_in_thread : gadget_ep_out_thread,
                           arg, QEMU_THREAD_DETACHED);
    }
    return true;
}

void usb_gadget_host_configure(UsbGadgetHost *g, unsigned max_power_ma)
{
    ioctl(g->fd, USB_RAW_IOCTL_VBUS_DRAW, max_power_ma / GADGET_VBUS_UNIT_MA);
    ioctl(g->fd, USB_RAW_IOCTL_CONFIGURE, 0);
}

void usb_gadget_host_ep_in(UsbGadgetHost *g, uint8_t ep_addr,
                           const uint8_t *data, size_t len)
{
    GAsyncQueue *queue = NULL;

    qemu_mutex_lock(&g->lock);
    for (unsigned i = 0; i < g->num_eps; i++) {
        if (g->eps[i].addr == ep_addr && g->eps[i].handle >= 0) {
            queue = g->eps[i].queue;
        }
    }
    qemu_mutex_unlock(&g->lock);
    if (queue) {
        GByteArray *buf = g_byte_array_sized_new(len);

        g_byte_array_append(buf, data, len);
        g_async_queue_push(queue, buf);
    }
}

static void gadget_control(UsbGadgetHost *g, const struct usb_ctrlrequest *c)
{
    UsbGadgetSetup setup = {
        .bRequestType = c->bRequestType,
        .bRequest = c->bRequest,
        .wValue = le16_to_cpu(c->wValue),
        .wIndex = le16_to_cpu(c->wIndex),
        .wLength = le16_to_cpu(c->wLength),
    };
    g_autofree struct usb_raw_ep_io *io =
        g_malloc0(sizeof(*io) + setup.wLength);
    int n;

    io->ep = 0;
    if (setup.bRequestType & USB_DIR_IN) {
        bql_lock();
        n = g->ops->control(g->opaque, &setup, io->data);
        bql_unlock();
        if (n < 0) {
            ioctl(g->fd, USB_RAW_IOCTL_EP0_STALL, 0);
            return;
        }
        io->length = MIN(n, setup.wLength);
        ioctl(g->fd, USB_RAW_IOCTL_EP0_WRITE, io);
        return;
    }
    io->length = setup.wLength;
    if (setup.wLength && ioctl(g->fd, USB_RAW_IOCTL_EP0_READ, io) < 0) {
        return;
    }
    bql_lock();
    n = g->ops->control(g->opaque, &setup, io->data);
    bql_unlock();
    if (n < 0) {
        ioctl(g->fd, USB_RAW_IOCTL_EP0_STALL, 0);
    } else if (!setup.wLength) {
        /* Status stage of a request without data */
        io->length = 0;
        ioctl(g->fd, USB_RAW_IOCTL_EP0_READ, io);
    }
}

static void *gadget_ep0_thread(void *opaque)
{
    UsbGadgetHost *g = opaque;
    struct {
        struct usb_raw_event event;
        struct usb_ctrlrequest request;
    } ev;

    for (;;) {
        ev.event.type = USB_RAW_EVENT_INVALID;
        ev.event.length = sizeof(ev.request);
        if (ioctl(g->fd, USB_RAW_IOCTL_EVENT_FETCH, &ev) < 0) {
            if (errno == EINTR) {
                continue;
            }
            error_report("usb-gadget-host: event fetch failed: %s",
                         strerror(errno));
            return NULL;
        }
        switch (ev.event.type) {
        case USB_RAW_EVENT_CONTROL:
            gadget_control(g, &ev.request);
            break;
        case USB_RAW_EVENT_RESET:
        case USB_RAW_EVENT_DISCONNECT:
            gadget_disable_all(g);
            bql_lock();
            g->ops->reset(g->opaque);
            bql_unlock();
            break;
        default:
            break;
        }
    }
    return NULL;
}

bool usb_gadget_host_claim(UsbGadgetHost *g, const UsbGadgetHostOps *ops,
                           void *opaque, Error **errp)
{
    if (g->ops) {
        error_setg(errp, "usb-gadget-host: already given to another device");
        return false;
    }
    if (!g->udc_driver || !g->udc_device) {
        error_setg(errp, "usb-gadget-host: 'udc-driver' and 'udc-device' "
                   "name the UDC to bind (see /sys/class/udc)");
        return false;
    }
    g->ops = ops;
    g->opaque = opaque;
    return true;
}

bool usb_gadget_host_start(UsbGadgetHost *g, UsbGadgetSpeed speed,
                           Error **errp)
{
    struct usb_raw_init init = { .speed = speed };

    g->fd = open(g->path ? g->path : GADGET_DEFAULT_PATH, O_RDWR);
    if (g->fd < 0) {
        error_setg_errno(errp, errno, "usb-gadget-host: cannot open %s",
                         g->path ? g->path : GADGET_DEFAULT_PATH);
        return false;
    }
    pstrcpy((char *)init.driver_name, sizeof(init.driver_name),
            g->udc_driver);
    pstrcpy((char *)init.device_name, sizeof(init.device_name),
            g->udc_device);
    if (ioctl(g->fd, USB_RAW_IOCTL_INIT, &init) < 0 ||
        ioctl(g->fd, USB_RAW_IOCTL_RUN, 0) < 0) {
        error_setg_errno(errp, errno, "usb-gadget-host: cannot bind UDC "
                         "%s/%s", g->udc_driver, g->udc_device);
        close(g->fd);
        g->fd = -1;
        return false;
    }
    qemu_thread_create(&g->ep0_thread, "usb-gadget-ep0", gadget_ep0_thread,
                       g, QEMU_THREAD_DETACHED);
    return true;
}

static char *gadget_get_path(Object *obj, Error **errp)
{
    return g_strdup(USB_GADGET_HOST(obj)->path);
}

static void gadget_set_path(Object *obj, const char *value, Error **errp)
{
    UsbGadgetHost *g = USB_GADGET_HOST(obj);

    g_free(g->path);
    g->path = g_strdup(value);
}

static char *gadget_get_driver(Object *obj, Error **errp)
{
    return g_strdup(USB_GADGET_HOST(obj)->udc_driver);
}

static void gadget_set_driver(Object *obj, const char *value, Error **errp)
{
    UsbGadgetHost *g = USB_GADGET_HOST(obj);

    g_free(g->udc_driver);
    g->udc_driver = g_strdup(value);
}

static char *gadget_get_device(Object *obj, Error **errp)
{
    return g_strdup(USB_GADGET_HOST(obj)->udc_device);
}

static void gadget_set_device(Object *obj, const char *value, Error **errp)
{
    UsbGadgetHost *g = USB_GADGET_HOST(obj);

    g_free(g->udc_device);
    g->udc_device = g_strdup(value);
}

static void gadget_instance_init(Object *obj)
{
    UsbGadgetHost *g = USB_GADGET_HOST(obj);

    g->fd = -1;
    qemu_mutex_init(&g->lock);
}

static void gadget_class_init(ObjectClass *oc, void *data)
{
    object_class_property_add_str(oc, "path", gadget_get_path,
                                  gadget_set_path);
    object_class_property_set_description(oc, "path",
        "raw-gadget device node (default " GADGET_DEFAULT_PATH ")");
    object_class_property_add_str(oc, "udc-driver", gadget_get_driver,
                                  gadget_set_driver);
    object_class_property_set_description(oc, "udc-driver",
        "UDC driver name, as /sys/class/udc/<dev>/device/driver names it");
    object_class_property_add_str(oc, "udc-device", gadget_get_device,
                                  gadget_set_device);
    object_class_property_set_description(oc, "udc-device",
        "UDC device name, as listed in /sys/class/udc");
}

static const TypeInfo gadget_info = {
    .name = TYPE_USB_GADGET_HOST,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(UsbGadgetHost),
    .instance_init = gadget_instance_init,
    .class_init = gadget_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_USER_CREATABLE },
        { }
    },
};

static void gadget_register_types(void)
{
    type_register_static(&gadget_info);
}

type_init(gadget_register_types)
