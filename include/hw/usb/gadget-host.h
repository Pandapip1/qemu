/*
 * USB gadget passthrough: hand a host USB device controller (UDC) to an
 * emulated USB device.  The backend only owns the host kernel's gadget
 * interface (Linux raw-gadget, Documentation/usb/raw-gadget.rst) and moves
 * SETUP packets and endpoint data; everything the USB host sees (the
 * descriptors, the answers to requests, the endpoint data) is the emulated
 * device's behaviour, implemented by the frontend that claims the backend.
 *
 *   -object usb-gadget-host,id=udc0,udc-driver=<driver>,udc-device=<dev>
 *   -device <emulated usb device>,gadget=udc0
 *
 * On a board with a peripheral-mode USB port, pointing the backend at that
 * port's UDC makes the emulated device appear to whatever USB host is
 * plugged into it.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_GADGET_HOST_H
#define HW_USB_GADGET_HOST_H

#include "qom/object.h"

#define TYPE_USB_GADGET_HOST "usb-gadget-host"
OBJECT_DECLARE_SIMPLE_TYPE(UsbGadgetHost, USB_GADGET_HOST)

/* USB 2.0 SETUP packet (USB 2.0 spec, 9.3), little-endian fields */
typedef struct UsbGadgetSetup {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} UsbGadgetSetup;

/* Device speeds, as USB_SPEED_* in the Linux gadget API */
typedef enum {
    USB_GADGET_SPEED_FULL = 2,
    USB_GADGET_SPEED_HIGH = 3,
} UsbGadgetSpeed;

typedef struct UsbGadgetHostOps {
    /*
     * A control request on endpoint 0.  For a device-to-host request fill
     * @buf (at most @setup->wLength bytes) and return its length; for a
     * host-to-device request @buf holds the @setup->wLength bytes of the
     * data stage.  Return a negative value to stall.  May call
     * usb_gadget_host_ep_enable() and usb_gadget_host_configure().
     */
    int (*control)(void *opaque, const UsbGadgetSetup *setup, uint8_t *buf);
    /* Bus reset or disconnect: the configuration is gone. */
    void (*reset)(void *opaque);
    /* Data the host sent to OUT endpoint @ep_addr. */
    void (*ep_out)(void *opaque, uint8_t ep_addr, const uint8_t *data,
                   size_t len);
} UsbGadgetHostOps;

/*
 * Frontend calls.  All callbacks run with the BQL held, in the backend's
 * threads.  ep_enable/configure are for the control callback.
 */
bool usb_gadget_host_claim(UsbGadgetHost *g, const UsbGadgetHostOps *ops,
                           void *opaque, Error **errp);
bool usb_gadget_host_start(UsbGadgetHost *g, UsbGadgetSpeed speed,
                           Error **errp);
/* @desc is a 7-byte USB endpoint descriptor. */
bool usb_gadget_host_ep_enable(UsbGadgetHost *g, const uint8_t *desc);
void usb_gadget_host_configure(UsbGadgetHost *g, unsigned max_power_ma);
/* Queue @len bytes for IN endpoint @ep_addr (copied). */
void usb_gadget_host_ep_in(UsbGadgetHost *g, uint8_t ep_addr,
                           const uint8_t *data, size_t len);

#endif
