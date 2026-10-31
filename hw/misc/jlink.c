/*
 * SEGGER J-Link debug probe, as its USB host and its target see it: a USB
 * device speaking the J-Link command protocol on a vendor-specific bulk
 * interface (and, for the on-board J-Link OB, a CDC-ACM virtual COM port),
 * whose SWD pins drive an SWDTarget.  The USB side runs on a host UDC
 * handed over with a usb-gadget-host backend:
 *
 *   -object usb-gadget-host,id=udc0,udc-driver=<driver>,udc-device=<dev>
 *   -device jlink,model=ob,gadget=udc0[,serial=...][,vcom=<chardev>]
 *
 * Protocol reference: libjaylink (https://gitlab.zapb.de/libjaylink), the
 * open host library OpenOCD uses: command codes and formats in
 * libjaylink/device.c, target.c, swd.c; USB discovery and transport in
 * discovery_usb.c and transport_usb.c.  Probe-specific values are probe
 * captures where we have them (owo-open-fw docs/debug-port-model.md);
 * the rest is marked as such and listed in owo-open-fw TODO.md.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/misc/swd.h"
#include "hw/misc/jlink-swd-io.h"
#include "hw/usb/gadget-host.h"

#define TYPE_JLINK "jlink"
OBJECT_DECLARE_SIMPLE_TYPE(JLinkState, JLINK)

/* --- J-Link protocol (libjaylink) ------------------------------------ */

#define JL_CMD_GET_VERSION 0x01
#define JL_CMD_SET_SPEED 0x05
#define JL_CMD_GET_HW_STATUS 0x07
#define JL_CMD_SET_TARGET_POWER 0x08
#define JL_CMD_GET_SPEEDS 0xc0
#define JL_CMD_SELECT_TIF 0xc7
#define JL_CMD_HW_IO_V3 JLINK_CMD_SWD_IO /* SWD I/O, JTAG I/O version 3 */
#define JL_CMD_CLEAR_RESET 0xdc
#define JL_CMD_SET_RESET 0xdd
#define JL_CMD_JTAG_CLEAR_TRST 0xde
#define JL_CMD_JTAG_SET_TRST 0xdf
#define JL_CMD_GET_CAPS 0xe8
#define JL_CMD_GET_EXT_CAPS 0xed
#define JL_CMD_GET_HW_VERSION 0xf0
#define JL_TIF_GET_SELECTED 0xfe
#define JL_TIF_GET_AVAILABLE 0xff
#define JL_TIF_JTAG 0
#define JL_TIF_SWD 1
/* Capability bits (libjaylink.h, enum jaylink_device_capability) */
#define JL_CAP_GET_HW_VERSION 1
#define JL_CAP_GET_SPEEDS 9
#define JL_CAP_SELECT_TIF 17
#define JL_CAPS_SIZE 4
#define JL_HW_STATUS_SIZE 8
/* Hardware version encoding: type*10^6 + major*10^4 + minor*10^2 + rev */
#define JL_HW_VERSION(major, minor) ((major) * 10000 + (minor) * 100)
/* Speeds: base frequency (Hz) and minimum divider; max = freq / div */
#define JL_SPEEDS_DIVIDER 1
#define HZ_PER_KHZ 1000

/* --- USB ------------------------------------------------------------- */

/* SEGGER's USB vendor ID (libjaylink discovery_usb.c USB_VENDOR_ID) */
#define JL_USB_VID 0x1366
/* A serial number string has 12 digits (USB_SERIAL_NUMBER_LENGTH) */
#define JL_SERIAL_DIGITS 12
#define USB_LANGID_EN_US 0x0409
#define USB_DT_DEVICE 1
#define USB_DT_CONFIG 2
#define USB_DT_STRING 3
#define USB_DT_INTERFACE 4
#define USB_DT_ENDPOINT 5
#define USB_DT_IAD 11
#define USB_DT_CS_INTERFACE 0x24
#define USB_DT_DEVICE_SIZE 18
#define USB_DT_CONFIG_SIZE 9
#define USB_DT_INTERFACE_SIZE 9
#define USB_DT_ENDPOINT_SIZE 7
#define USB_DT_IAD_SIZE 8
#define USB_REQ_GET_STATUS 0
#define USB_REQ_CLEAR_FEATURE 1
#define USB_REQ_SET_FEATURE 3
#define USB_REQ_GET_DESCRIPTOR 6
#define USB_REQ_GET_CONFIGURATION 8
#define USB_REQ_SET_CONFIGURATION 9
#define USB_REQ_GET_INTERFACE 10
#define USB_REQ_SET_INTERFACE 11
#define USB_TYPE_MASK 0x60
#define USB_TYPE_STANDARD 0x00
#define USB_TYPE_CLASS 0x20
#define USB_DIR_IN 0x80
#define USB_CLASS_PER_INTERFACE 0x00
#define USB_CLASS_MISC 0xef
#define USB_MISC_SUBCLASS_COMMON 0x02
#define USB_MISC_PROTOCOL_IAD 0x01
#define USB_CLASS_VENDOR 0xff
#define USB_CLASS_CDC 0x02
#define USB_CLASS_CDC_DATA 0x0a
#define USB_CDC_SUBCLASS_ACM 0x02
#define USB_CDC_PROTOCOL_AT 0x01
#define USB_EP_BULK 0x02
#define USB_EP_INTERRUPT 0x03
#define USB_CONFIG_ATTR_ONE 0x80   /* bit 7 of bmAttributes is reserved, 1 */
#define USB_BCD_2_0 0x0200
#define USB_EP0_MAXPACKET 64
#define USB_FS_BULK_MAXPACKET 64
#define USB_HS_BULK_MAXPACKET 512
#define USB_CDC_NOTIFY_MAXPACKET 16
#define USB_CDC_NOTIFY_INTERVAL_FS 16 /* ms */
/* CDC 1.1 (PSTN) class requests and functional descriptors */
#define CDC_SET_LINE_CODING 0x20
#define CDC_GET_LINE_CODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK 0x23
#define CDC_LINE_CODING_SIZE 7
#define CDC_FD_HEADER 0x00
#define CDC_FD_CALL_MGMT 0x01
#define CDC_FD_ACM 0x02
#define CDC_FD_UNION 0x06
#define CDC_BCD_1_10 0x0110
#define CDC_ACM_CAP_LINE_CODING 0x02
#define CONFIG_VALUE 1
#define CONFIG_MAX_POWER_MA 100

/* Interfaces and endpoints of the modelled configuration */
#define JL_IF_JLINK 0
#define JL_IF_CDC_COMM 1
#define JL_IF_CDC_DATA 2
#define JL_EP_OUT 0x01
#define JL_EP_IN 0x81
#define JL_EP_CDC_DATA_OUT 0x02
#define JL_EP_CDC_DATA_IN 0x82
#define JL_EP_CDC_NOTIFY 0x83
#define JL_VCOM_RX_CHUNK USB_FS_BULK_MAXPACKET
/* The EK's VCOM default (Zephyr's console on USART1 runs at 115200 8N1) */
#define JL_VCOM_DEFAULT_BAUD 115200
#define JL_VCOM_DEFAULT_DATABITS 8
#define CDC_LINE_CODING_DATABITS 6

enum {
    JL_STR_LANGID,
    JL_STR_MANUFACTURER,
    JL_STR_PRODUCT,
    JL_STR_SERIAL,
    JL_STR_COUNT,
};

typedef struct JLinkModel {
    const char *name;
    uint16_t pid;
    /* Default serial: clearly fake, see jlink_default_serial below */
    const char *serial;
    const char *firmware;
    uint32_t hw_version;
    uint32_t max_khz;
    bool vcom;
} JLinkModel;

/*
 * Simulated probes use 10-digit serial numbers 4242000xxx.  SEGGER's
 * factory serials have 9 digits (the real probes: OB 000440200548, EDU
 * Mini 000801032573), so a simulated probe can never be mistaken for, or
 * selected instead of, real hardware.  libjaylink parses the last ten
 * digits into a 32-bit value; 4242000xxx fits.
 */
static const JLinkModel jlink_models[] = {
    {
        /* Evaluation kit's on-board probe; values from OpenOCD on the EK */
        .name = "ob",
        .pid = 0x0105,
        .serial = "004242000105",
        .firmware = "Silicon Labs J-Link OB compiled Oct 20 2020 14:45:20",
        .hw_version = JL_HW_VERSION(1, 0),
        /* "Reduced speed from 4000 kHz to 1900 kHz (maximum)" */
        .max_khz = 1900,
        .vcom = true,
    },
    {
        /*
         * J-Link EDU Mini on the suit.  Its firmware string and hardware
         * version were not captured (placeholders; TODO.md); maximum SWD
         * clock ~4 MHz per SEGGER's product description.
         */
        .name = "edu-mini",
        .pid = 0x0101,
        .serial = "004242000101",
        .firmware = "J-Link EDU Mini (simulated; firmware string not captured)",
        .hw_version = JL_HW_VERSION(1, 0),
        .max_khz = 4000,
        .vcom = false,
    },
};

struct JLinkState {
    DeviceState parent_obj;

    char *model_name;
    char *serial;
    Object *gadget_obj;
    Object *target_obj;
    CharBackend vcom;

    const JLinkModel *model;
    UsbGadgetHost *gadget;
    SWDTarget *target;
    GByteArray *cmd;
    uint32_t tif;
    uint16_t speed_khz;
    uint8_t configuration;
    uint8_t line_coding[CDC_LINE_CODING_SIZE];
    bool high_speed;
    GByteArray *config_desc;
};

/* --- Descriptors --------------------------------------------------- */

static uint16_t jlink_bulk_maxpacket(JLinkState *s)
{
    return s->high_speed ? USB_HS_BULK_MAXPACKET : USB_FS_BULK_MAXPACKET;
}

static void put_u16(GByteArray *a, uint16_t v)
{
    uint8_t b[2];

    stw_le_p(b, v);
    g_byte_array_append(a, b, sizeof(b));
}

static void put_bytes(GByteArray *a, const uint8_t *b, size_t n)
{
    g_byte_array_append(a, b, n);
}

static void put_endpoint(GByteArray *a, uint8_t addr, uint8_t type,
                         uint16_t maxpacket, uint8_t interval)
{
    uint8_t d[USB_DT_ENDPOINT_SIZE] = {
        USB_DT_ENDPOINT_SIZE, USB_DT_ENDPOINT, addr, type,
        maxpacket & 0xff, maxpacket >> 8, interval,
    };

    put_bytes(a, d, sizeof(d));
}

static void put_interface(GByteArray *a, uint8_t number, uint8_t endpoints,
                          uint8_t class, uint8_t subclass, uint8_t protocol)
{
    uint8_t d[USB_DT_INTERFACE_SIZE] = {
        USB_DT_INTERFACE_SIZE, USB_DT_INTERFACE, number, 0, endpoints,
        class, subclass, protocol, 0,
    };

    put_bytes(a, d, sizeof(d));
}

/*
 * Configuration: the J-Link vendor interface (two bulk endpoints, as
 * libjaylink requires: transport_usb.c initialize_handle()) and, for the
 * OB, a CDC-ACM function for the VCOM.  Interface order and endpoint
 * numbers of the real probes were not captured (TODO.md).
 */
static void jlink_build_config(JLinkState *s)
{
    GByteArray *a = g_byte_array_new();
    uint16_t mps = jlink_bulk_maxpacket(s);
    uint8_t interfaces = s->model->vcom ? 3 : 1;
    uint8_t header[USB_DT_CONFIG_SIZE] = {
        USB_DT_CONFIG_SIZE, USB_DT_CONFIG, 0, 0, interfaces, CONFIG_VALUE, 0,
        USB_CONFIG_ATTR_ONE, CONFIG_MAX_POWER_MA / 2,
    };

    put_bytes(a, header, sizeof(header));
    put_interface(a, JL_IF_JLINK, 2, USB_CLASS_VENDOR, USB_CLASS_VENDOR,
                  USB_CLASS_VENDOR);
    put_endpoint(a, JL_EP_IN, USB_EP_BULK, mps, 0);
    put_endpoint(a, JL_EP_OUT, USB_EP_BULK, mps, 0);
    if (s->model->vcom) {
        uint8_t iad[USB_DT_IAD_SIZE] = {
            USB_DT_IAD_SIZE, USB_DT_IAD, JL_IF_CDC_COMM, 2, USB_CLASS_CDC,
            USB_CDC_SUBCLASS_ACM, USB_CDC_PROTOCOL_AT, 0,
        };
        uint8_t fd_header[] = { 5, USB_DT_CS_INTERFACE, CDC_FD_HEADER,
                                CDC_BCD_1_10 & 0xff, CDC_BCD_1_10 >> 8 };
        uint8_t fd_call[] = { 5, USB_DT_CS_INTERFACE, CDC_FD_CALL_MGMT, 0,
                              JL_IF_CDC_DATA };
        uint8_t fd_acm[] = { 4, USB_DT_CS_INTERFACE, CDC_FD_ACM,
                             CDC_ACM_CAP_LINE_CODING };
        uint8_t fd_union[] = { 5, USB_DT_CS_INTERFACE, CDC_FD_UNION,
                               JL_IF_CDC_COMM, JL_IF_CDC_DATA };

        put_bytes(a, iad, sizeof(iad));
        put_interface(a, JL_IF_CDC_COMM, 1, USB_CLASS_CDC,
                      USB_CDC_SUBCLASS_ACM, USB_CDC_PROTOCOL_AT);
        put_bytes(a, fd_header, sizeof(fd_header));
        put_bytes(a, fd_call, sizeof(fd_call));
        put_bytes(a, fd_acm, sizeof(fd_acm));
        put_bytes(a, fd_union, sizeof(fd_union));
        put_endpoint(a, JL_EP_CDC_NOTIFY, USB_EP_INTERRUPT,
                     USB_CDC_NOTIFY_MAXPACKET, USB_CDC_NOTIFY_INTERVAL_FS);
        put_interface(a, JL_IF_CDC_DATA, 2, USB_CLASS_CDC_DATA, 0, 0);
        put_endpoint(a, JL_EP_CDC_DATA_IN, USB_EP_BULK, mps, 0);
        put_endpoint(a, JL_EP_CDC_DATA_OUT, USB_EP_BULK, mps, 0);
    }
    stw_le_p(a->data + 2, a->len);  /* wTotalLength */
    s->config_desc = a;
}

static int jlink_device_desc(JLinkState *s, uint8_t *buf)
{
    bool iad = s->model->vcom;
    uint8_t d[USB_DT_DEVICE_SIZE] = {
        USB_DT_DEVICE_SIZE, USB_DT_DEVICE,
        USB_BCD_2_0 & 0xff, USB_BCD_2_0 >> 8,
        iad ? USB_CLASS_MISC : USB_CLASS_PER_INTERFACE,
        iad ? USB_MISC_SUBCLASS_COMMON : 0,
        iad ? USB_MISC_PROTOCOL_IAD : 0,
        USB_EP0_MAXPACKET,
        JL_USB_VID & 0xff, JL_USB_VID >> 8,
        s->model->pid & 0xff, s->model->pid >> 8,
        0x00, 0x01,          /* bcdDevice 1.00: not captured (TODO.md) */
        JL_STR_MANUFACTURER, JL_STR_PRODUCT, JL_STR_SERIAL, 1,
    };

    memcpy(buf, d, sizeof(d));
    return sizeof(d);
}

static int jlink_string_desc(JLinkState *s, unsigned index, uint8_t *buf)
{
    const char *str;
    int len;

    switch (index) {
    case JL_STR_LANGID:
        buf[0] = 4;
        buf[1] = USB_DT_STRING;
        stw_le_p(buf + 2, USB_LANGID_EN_US);
        return 4;
    case JL_STR_MANUFACTURER:
        str = "SEGGER";
        break;
    case JL_STR_PRODUCT:
        /* libjaylink: "<product> [_<nickname>]" */
        str = "J-Link";
        break;
    case JL_STR_SERIAL:
        str = s->serial;
        break;
    default:
        return -1;
    }
    len = strlen(str);
    buf[0] = 2 + 2 * len;
    buf[1] = USB_DT_STRING;
    for (int i = 0; i < len; i++) {
        stw_le_p(buf + 2 + 2 * i, str[i]);
    }
    return buf[0];
}

/* --- Control requests ---------------------------------------------- */

static void jlink_set_configuration(JLinkState *s, uint8_t value)
{
    uint8_t *d = s->config_desc->data + USB_DT_CONFIG_SIZE;
    uint8_t *end = s->config_desc->data + s->config_desc->len;

    s->configuration = value;
    if (value != CONFIG_VALUE) {
        return;
    }
    for (; d < end; d += d[0]) {
        if (d[1] == USB_DT_ENDPOINT) {
            usb_gadget_host_ep_enable(s->gadget, d);
        }
    }
    usb_gadget_host_configure(s->gadget, CONFIG_MAX_POWER_MA);
}

static int jlink_control(void *opaque, const UsbGadgetSetup *req,
                         uint8_t *buf)
{
    JLinkState *s = opaque;
    uint8_t type = req->bRequestType & USB_TYPE_MASK;
    int n;

    if (type == USB_TYPE_CLASS && s->model->vcom) {
        switch (req->bRequest) {
        case CDC_SET_LINE_CODING:
            memcpy(s->line_coding, buf,
                   MIN(req->wLength, sizeof(s->line_coding)));
            return 0;
        case CDC_GET_LINE_CODING:
            memcpy(buf, s->line_coding, sizeof(s->line_coding));
            return sizeof(s->line_coding);
        case CDC_SET_CONTROL_LINE_STATE:
        case CDC_SEND_BREAK:
            return 0;
        }
        return -1;
    }
    if (type != USB_TYPE_STANDARD) {
        return -1;
    }
    switch (req->bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
        switch (req->wValue >> 8) {
        case USB_DT_DEVICE:
            return jlink_device_desc(s, buf);
        case USB_DT_CONFIG:
            n = MIN(req->wLength, s->config_desc->len);
            memcpy(buf, s->config_desc->data, n);
            return n;
        case USB_DT_STRING:
            {
                uint8_t tmp[2 + 2 * UINT8_MAX];

                n = jlink_string_desc(s, req->wValue & 0xff, tmp);
                if (n > 0) {
                    memcpy(buf, tmp, MIN(n, req->wLength));
                }
                return n < 0 ? n : MIN(n, req->wLength);
            }
        default:
            /* Full-speed device: no device qualifier, no BOS */
            return -1;
        }
    case USB_REQ_SET_CONFIGURATION:
        jlink_set_configuration(s, req->wValue & 0xff);
        return 0;
    case USB_REQ_GET_CONFIGURATION:
        buf[0] = s->configuration;
        return 1;
    case USB_REQ_GET_STATUS:
        buf[0] = 0;
        buf[1] = 0;
        return 2;
    case USB_REQ_GET_INTERFACE:
        buf[0] = 0;
        return 1;
    case USB_REQ_SET_INTERFACE:
        return req->wValue == 0 ? 0 : -1;
    case USB_REQ_CLEAR_FEATURE:
    case USB_REQ_SET_FEATURE:
        return 0;
    }
    return -1;
}

static void jlink_reset(void *opaque)
{
    JLinkState *s = opaque;

    s->configuration = 0;
    g_byte_array_set_size(s->cmd, 0);
}

/* --- J-Link commands ----------------------------------------------- */

static void jlink_reply(JLinkState *s, const uint8_t *data, size_t len)
{
    usb_gadget_host_ep_in(s->gadget, JL_EP_IN, data, len);
}

static void jlink_reply_u32(JLinkState *s, uint32_t value)
{
    uint8_t b[4];

    stl_le_p(b, value);
    jlink_reply(s, b, sizeof(b));
}

/* Bytes the command at the head of the buffer needs, or 0 if unknown. */
static size_t jlink_cmd_length(const uint8_t *c, size_t avail)
{
    switch (c[0]) {
    case JL_CMD_GET_VERSION:
    case JL_CMD_GET_HW_STATUS:
    case JL_CMD_GET_SPEEDS:
    case JL_CMD_CLEAR_RESET:
    case JL_CMD_SET_RESET:
    case JL_CMD_JTAG_CLEAR_TRST:
    case JL_CMD_JTAG_SET_TRST:
    case JL_CMD_GET_CAPS:
    case JL_CMD_GET_EXT_CAPS:
    case JL_CMD_GET_HW_VERSION:
        return 1;
    case JL_CMD_SELECT_TIF:
    case JL_CMD_SET_TARGET_POWER:
        return 2;
    case JL_CMD_SET_SPEED:
        return 3;
    case JL_CMD_HW_IO_V3:
        if (avail < JLINK_SWD_IO_HEADER) {
            return JLINK_SWD_IO_HEADER;
        }
        return jlink_swd_io_cmd_len(lduw_le_p(c + 2));
    default:
        return 0;
    }
}

/* SWD I/O: the wire format is in include/hw/misc/jlink-swd-io.h */
static void jlink_hw_io(JLinkState *s, const uint8_t *c)
{
    size_t len = jlink_swd_io_reply_len(lduw_le_p(c + 2));
    g_autofree uint8_t *in = g_malloc0(len);

    if (s->tif == JL_TIF_SWD) {
        jlink_swd_io(s->target, c, in);
    }
    jlink_reply(s, in, len);
}

static void jlink_execute(JLinkState *s, const uint8_t *c)
{
    switch (c[0]) {
    case JL_CMD_GET_VERSION: {
        size_t len = strlen(s->model->firmware) + 1;
        g_autofree uint8_t *r = g_malloc0(2 + len);

        stw_le_p(r, len);
        memcpy(r + 2, s->model->firmware, len - 1);
        jlink_reply(s, r, 2 + len);
        break;
    }
    case JL_CMD_GET_CAPS:
        jlink_reply_u32(s, (1u << JL_CAP_GET_HW_VERSION) |
                        (1u << JL_CAP_GET_SPEEDS) |
                        (1u << JL_CAP_SELECT_TIF));
        break;
    case JL_CMD_GET_HW_VERSION:
        jlink_reply_u32(s, s->model->hw_version);
        break;
    case JL_CMD_GET_HW_STATUS: {
        uint8_t r[JL_HW_STATUS_SIZE] = { 0 };

        /* Target voltage on VTref in mV; the pin levels are not modelled */
        stw_le_p(r, swd_target_vtref_mv(s->target));
        jlink_reply(s, r, sizeof(r));
        break;
    }
    case JL_CMD_GET_SPEEDS: {
        uint8_t r[6];

        stl_le_p(r, s->model->max_khz * HZ_PER_KHZ * JL_SPEEDS_DIVIDER);
        stw_le_p(r + 4, JL_SPEEDS_DIVIDER);
        jlink_reply(s, r, sizeof(r));
        break;
    }
    case JL_CMD_SELECT_TIF:
        if (c[1] == JL_TIF_GET_AVAILABLE) {
            /* SWD only: JTAG support on these probes was not checked */
            jlink_reply_u32(s, 1u << JL_TIF_SWD);
        } else if (c[1] == JL_TIF_GET_SELECTED) {
            jlink_reply_u32(s, s->tif);
        } else {
            jlink_reply_u32(s, s->tif);
            s->tif = c[1];
        }
        break;
    case JL_CMD_SET_SPEED:
        /* No SWCLK timing is modelled; the speed is only recorded */
        s->speed_khz = lduw_le_p(c + 1);
        break;
    case JL_CMD_HW_IO_V3:
        jlink_hw_io(s, c);
        break;
    default:
        /*
         * Reset lines (no nRESET is wired on the EK) and target power (not
         * advertised): accepted, no reply.
         */
        break;
    }
}

static void jlink_ep_out(void *opaque, uint8_t ep, const uint8_t *data,
                         size_t len)
{
    JLinkState *s = opaque;

    if (ep == JL_EP_CDC_DATA_OUT) {
        if (qemu_chr_fe_backend_connected(&s->vcom)) {
            qemu_chr_fe_write_all(&s->vcom, data, len);
        }
        return;
    }
    if (ep != JL_EP_OUT) {
        return;
    }
    g_byte_array_append(s->cmd, data, len);
    while (s->cmd->len) {
        size_t need = jlink_cmd_length(s->cmd->data, s->cmd->len);

        if (!need) {
            qemu_log_mask(LOG_UNIMP, "jlink: command 0x%02x\n",
                          s->cmd->data[0]);
            g_byte_array_set_size(s->cmd, 0);
            return;
        }
        if (s->cmd->len < need) {
            return;
        }
        jlink_execute(s, s->cmd->data);
        g_byte_array_remove_range(s->cmd, 0, need);
    }
}

static const UsbGadgetHostOps jlink_gadget_ops = {
    .control = jlink_control,
    .reset = jlink_reset,
    .ep_out = jlink_ep_out,
};

/* --- VCOM ---------------------------------------------------------- */

static int jlink_vcom_can_receive(void *opaque)
{
    JLinkState *s = opaque;

    return s->configuration == CONFIG_VALUE ? JL_VCOM_RX_CHUNK : 0;
}

static void jlink_vcom_receive(void *opaque, const uint8_t *buf, int size)
{
    JLinkState *s = opaque;

    usb_gadget_host_ep_in(s->gadget, JL_EP_CDC_DATA_IN, buf, size);
}

/* --- Device -------------------------------------------------------- */

static void jlink_realize(DeviceState *dev, Error **errp)
{
    JLinkState *s = JLINK(dev);
    bool ambiguous = false;

    s->model = NULL;
    for (unsigned i = 0; i < ARRAY_SIZE(jlink_models); i++) {
        if (!g_strcmp0(s->model_name, jlink_models[i].name)) {
            s->model = &jlink_models[i];
        }
    }
    if (!s->model) {
        error_setg(errp, "jlink: model must be 'ob' or 'edu-mini'");
        return;
    }
    if (!s->serial) {
        s->serial = g_strdup(s->model->serial);
    }
    if (strlen(s->serial) > JL_SERIAL_DIGITS ||
        strspn(s->serial, "0123456789") != strlen(s->serial)) {
        error_setg(errp, "jlink: serial must be up to %d digits",
                   JL_SERIAL_DIGITS);
        return;
    }
    if (!s->gadget_obj) {
        error_setg(errp, "jlink: 'gadget' must name a usb-gadget-host");
        return;
    }
    if (!s->target_obj) {
        s->target_obj = object_resolve_path_type("", TYPE_SWD_TARGET,
                                                 &ambiguous);
    }
    if (!s->target_obj) {
        error_setg(errp, "jlink: no unique SWD target; set 'target'");
        return;
    }
    if (s->model->vcom && qemu_chr_fe_backend_connected(&s->vcom)) {
        qemu_chr_fe_set_handlers(&s->vcom, jlink_vcom_can_receive,
                                 jlink_vcom_receive, NULL, NULL, s, NULL,
                                 true);
    }
    s->target = SWD_TARGET(s->target_obj);
    s->gadget = USB_GADGET_HOST(s->gadget_obj);
    s->cmd = g_byte_array_new();
    /* CDC line coding until the host sets one: 115200 8N1 */
    stl_le_p(s->line_coding, JL_VCOM_DEFAULT_BAUD);
    s->line_coding[CDC_LINE_CODING_DATABITS] = JL_VCOM_DEFAULT_DATABITS;
    jlink_build_config(s);
    if (!usb_gadget_host_claim(s->gadget, &jlink_gadget_ops, s, errp) ||
        !usb_gadget_host_start(s->gadget, s->high_speed ?
                               USB_GADGET_SPEED_HIGH : USB_GADGET_SPEED_FULL,
                               errp)) {
        return;
    }
}

static Property jlink_properties[] = {
    DEFINE_PROP_STRING("model", JLinkState, model_name),
    DEFINE_PROP_STRING("serial", JLinkState, serial),
    DEFINE_PROP_LINK("gadget", JLinkState, gadget_obj, TYPE_USB_GADGET_HOST,
                     Object *),
    DEFINE_PROP_LINK("target", JLinkState, target_obj, TYPE_SWD_TARGET,
                     Object *),
    DEFINE_PROP_CHR("vcom", JLinkState, vcom),
    /* Both probes enumerate full-speed until captured otherwise */
    DEFINE_PROP_BOOL("high-speed", JLinkState, high_speed, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void jlink_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = jlink_realize;
    dc->user_creatable = true;
    dc->desc = "SEGGER J-Link probe on a passed-through USB gadget";
    device_class_set_props(dc, jlink_properties);
}

static const TypeInfo jlink_info = {
    .name = TYPE_JLINK,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(JLinkState),
    .class_init = jlink_class_init,
};

static void jlink_register_types(void)
{
    type_register_static(&jlink_info);
}

type_init(jlink_register_types)
