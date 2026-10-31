/*
 * Host-device backends for emulated buses (Linux).
 *
 * The emulated SoC peripheral is the per-silicon translation layer (its
 * registers, timing and interrupts); these devices are the thin,
 * silicon-independent other side, handing each bus transaction to a host
 * device, much as a PCI device is passed through to a guest:
 *  - "spidev-host" (SPI peripheral): each byte the emulated controller
 *    clocks is one full-duplex SPI_IOC_MESSAGE transfer on /dev/spidevB.C,
 *    with the chip select held between bytes while the board's CS line is
 *    low ("cs" GPIO input), released when it rises.
 *    -device spidev-host,bus=usart1.ssi,path=/dev/spidev0.0
 *  - "i2c-dev-host" (I2C follower): the transaction is replayed through
 *    I2C_RDWR on /dev/i2c-N at the follower address: written bytes are
 *    sent at STOP or repeated START; a read fetches "read-ahead" bytes in
 *    one message at its first byte (an over-read of a FIFO register is the
 *    known limitation).  A host NACK becomes an address NACK.
 *    -device i2c-dev-host,bus=i2c0,address=0x20,path=/dev/i2c-3
 *  - "iio-analog" (analog-source): a net is read as
 *    (in_voltageN_raw + offset) x scale from an IIO device's sysfs
 *    directory, mapped by "map=PD02:in_voltage0,PD03:in_voltage1".
 * GPIO lines (gpio-sim, a GPIO chip) and UARTs (a tty, through the USART's
 * chardev) are the remaining classes; GPIO is not implemented here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/analog-source.h"
#include "qom/object_interfaces.h"
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* ------------------------------------------------------------- spidev */

#define TYPE_SPIDEV_HOST "spidev-host"
OBJECT_DECLARE_SIMPLE_TYPE(SpidevHostState, SPIDEV_HOST)

struct SpidevHostState {
    SSIPeripheral parent_obj;
    char *path;
    uint32_t speed_hz;
    int fd;
    bool selected;
};

static uint32_t spidev_host_transfer(SSIPeripheral *dev, uint32_t val)
{
    SpidevHostState *s = SPIDEV_HOST(dev);
    uint8_t tx = val, rx = 0;
    struct spi_ioc_transfer t = {
        .tx_buf = (uintptr_t)&tx,
        .rx_buf = (uintptr_t)&rx,
        .len = 1,
        .speed_hz = s->speed_hz,
        .bits_per_word = 8,
        /* Keep CS asserted after this byte while the board's CS is low. */
        .cs_change = s->selected,
    };

    if (!s->selected || ioctl(s->fd, SPI_IOC_MESSAGE(1), &t) < 0) {
        return 0;
    }
    return rx;
}

static void spidev_host_cs(void *opaque, int n, int level)
{
    SpidevHostState *s = opaque;
    bool was = s->selected;

    s->selected = !level;
    if (was && !s->selected) {
        /* End the held transfer with a zero-length message, CS released. */
        struct spi_ioc_transfer t = { .len = 0, .cs_change = 0 };

        ioctl(s->fd, SPI_IOC_MESSAGE(1), &t);
    }
}

static void spidev_host_realize(SSIPeripheral *dev, Error **errp)
{
    SpidevHostState *s = SPIDEV_HOST(dev);

    if (!s->path) {
        error_setg(errp, "spidev-host: path is required");
        return;
    }
    s->fd = qemu_open(s->path, O_RDWR, errp);
    if (s->fd < 0) {
        return;
    }
    qdev_init_gpio_in_named(DEVICE(dev), spidev_host_cs, "cs", 1);
}

static Property spidev_host_properties[] = {
    DEFINE_PROP_STRING("path", SpidevHostState, path),
    DEFINE_PROP_UINT32("speed-hz", SpidevHostState, speed_hz, 1000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void spidev_host_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = spidev_host_realize;
    k->transfer_raw = spidev_host_transfer;
    device_class_set_props(dc, spidev_host_properties);
    dc->user_creatable = true;
    dc->desc = "Host spidev behind an emulated SPI bus";
}

/* ------------------------------------------------------------ i2c-dev */

#define TYPE_I2C_DEV_HOST "i2c-dev-host"
OBJECT_DECLARE_SIMPLE_TYPE(I2cDevHostState, I2C_DEV_HOST)
#define I2C_DEV_HOST_BUF 64

struct I2cDevHostState {
    I2CSlave parent_obj;
    char *path;
    uint32_t read_ahead;
    int fd;
    uint8_t wbuf[I2C_DEV_HOST_BUF];
    unsigned wlen;
    uint8_t rbuf[I2C_DEV_HOST_BUF];
    unsigned rlen, rpos;
    bool reading;
};

static int i2c_dev_host_rdwr(I2cDevHostState *s, bool with_read)
{
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = 0 };
    uint16_t addr = s->parent_obj.address;

    if (s->wlen) {
        msgs[data.nmsgs++] = (struct i2c_msg){ addr, 0, s->wlen, s->wbuf };
    }
    if (with_read) {
        s->rlen = MIN(s->read_ahead, (uint32_t)I2C_DEV_HOST_BUF);
        s->rpos = 0;
        msgs[data.nmsgs++] = (struct i2c_msg){ addr, I2C_M_RD, s->rlen,
                                               s->rbuf };
    }
    s->wlen = 0;
    if (!data.nmsgs) {
        return 0;
    }
    return ioctl(s->fd, I2C_RDWR, &data) < 0 ? -1 : 0;
}

static int i2c_dev_host_event(I2CSlave *i2c, enum i2c_event event)
{
    I2cDevHostState *s = I2C_DEV_HOST(i2c);

    switch (event) {
    case I2C_START_RECV:
        s->reading = true;
        /* Pending writes and the read go out as one combined transfer. */
        return i2c_dev_host_rdwr(s, true);
    case I2C_START_SEND:
        if (s->wlen) {
            i2c_dev_host_rdwr(s, false);
        }
        s->reading = false;
        return 0;
    case I2C_FINISH:
        if (!s->reading) {
            return i2c_dev_host_rdwr(s, false);
        }
        return 0;
    default:
        return 0;
    }
}

static int i2c_dev_host_send(I2CSlave *i2c, uint8_t data)
{
    I2cDevHostState *s = I2C_DEV_HOST(i2c);

    if (s->wlen >= I2C_DEV_HOST_BUF) {
        return -1;
    }
    s->wbuf[s->wlen++] = data;
    return 0;
}

static uint8_t i2c_dev_host_recv(I2CSlave *i2c)
{
    I2cDevHostState *s = I2C_DEV_HOST(i2c);

    return s->rpos < s->rlen ? s->rbuf[s->rpos++] : 0xff;
}

static void i2c_dev_host_realize(DeviceState *dev, Error **errp)
{
    I2cDevHostState *s = I2C_DEV_HOST(dev);

    if (!s->path) {
        error_setg(errp, "i2c-dev-host: path is required");
        return;
    }
    s->fd = qemu_open(s->path, O_RDWR, errp);
}

static Property i2c_dev_host_properties[] = {
    DEFINE_PROP_STRING("path", I2cDevHostState, path),
    DEFINE_PROP_UINT32("read-ahead", I2cDevHostState, read_ahead, 2),
    DEFINE_PROP_END_OF_LIST(),
};

static void i2c_dev_host_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = i2c_dev_host_realize;
    k->event = i2c_dev_host_event;
    k->send = i2c_dev_host_send;
    k->recv = i2c_dev_host_recv;
    device_class_set_props(dc, i2c_dev_host_properties);
    dc->user_creatable = true;
    dc->desc = "Host i2c-dev follower behind an emulated I2C bus";
}

/* ---------------------------------------------------------------- IIO */

#define TYPE_IIO_ANALOG "iio-analog"
OBJECT_DECLARE_SIMPLE_TYPE(IioAnalogState, IIO_ANALOG)

struct IioAnalogState {
    Object parent_obj;
    char *dir;
    char *map;
};

static bool iio_read_double(const char *path, double *out)
{
    g_autofree char *text = NULL;

    if (!g_file_get_contents(path, &text, NULL, NULL)) {
        return false;
    }
    *out = g_ascii_strtod(text, NULL);
    return true;
}

static int64_t iio_analog_read_uv(AnalogSource *src, const char *net)
{
    IioAnalogState *s = IIO_ANALOG(src);
    g_auto(GStrv) entries = s->map ? g_strsplit(s->map, ",", -1) : NULL;

    for (unsigned i = 0; entries && entries[i]; i++) {
        g_auto(GStrv) kv = g_strsplit(entries[i], ":", 2);
        g_autofree char *raw_path = NULL, *scale_path = NULL;
        g_autofree char *offset_path = NULL;
        double raw, scale = 1, offset = 0;

        if (g_strv_length(kv) != 2 || strcmp(kv[0], net)) {
            continue;
        }
        raw_path = g_strdup_printf("%s/%s_raw", s->dir, kv[1]);
        scale_path = g_strdup_printf("%s/%s_scale", s->dir, kv[1]);
        offset_path = g_strdup_printf("%s/%s_offset", s->dir, kv[1]);
        if (!iio_read_double(raw_path, &raw)) {
            return -1;
        }
        iio_read_double(scale_path, &scale);
        iio_read_double(offset_path, &offset);
        /* IIO: (raw + offset) x scale is in millivolts. */
        return (int64_t)((raw + offset) * scale * 1000);
    }
    return -1;
}

static void iio_analog_class_init(ObjectClass *klass, void *data)
{
    AnalogSourceClass *ac = ANALOG_SOURCE_CLASS(klass);

    ac->read_uv = iio_analog_read_uv;
}

static char *iio_get_dir(Object *o, Error **e)
{
    return g_strdup(IIO_ANALOG(o)->dir);
}

static void iio_set_dir(Object *o, const char *v, Error **e)
{
    g_free(IIO_ANALOG(o)->dir);
    IIO_ANALOG(o)->dir = g_strdup(v);
}

static char *iio_get_map(Object *o, Error **e)
{
    return g_strdup(IIO_ANALOG(o)->map);
}

static void iio_set_map(Object *o, const char *v, Error **e)
{
    g_free(IIO_ANALOG(o)->map);
    IIO_ANALOG(o)->map = g_strdup(v);
}

static void iio_analog_init(Object *obj)
{
    object_property_add_str(obj, "dir", iio_get_dir, iio_set_dir);
    object_property_add_str(obj, "map", iio_get_map, iio_set_map);
}

static const TypeInfo host_passthrough_types[] = {
    {
        .name = TYPE_SPIDEV_HOST,
        .parent = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(SpidevHostState),
        .class_init = spidev_host_class_init,
    }, {
        .name = TYPE_I2C_DEV_HOST,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(I2cDevHostState),
        .class_init = i2c_dev_host_class_init,
    }, {
        .name = TYPE_IIO_ANALOG,
        .parent = TYPE_OBJECT,
        .instance_size = sizeof(IioAnalogState),
        .instance_init = iio_analog_init,
        .class_init = iio_analog_class_init,
        .interfaces = (InterfaceInfo[]) {
            { TYPE_ANALOG_SOURCE },
            { TYPE_USER_CREATABLE },
            { }
        },
    },
};

DEFINE_TYPES(host_passthrough_types)
