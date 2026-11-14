#include <string.h>

#include "bsp.h"
#include "tusb.h"

// pid.codes test VID/PID. Fine for a personal device; get a real PID before sharing.
#define USB_VID   0x1209
#define USB_PID   0x0001
#define USB_BCD   0x0200

// ---------------------------------------------------------------------------
// Device descriptor
// ---------------------------------------------------------------------------

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    // IAD required for CDC (two interfaces)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0001,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&desc_device;
}

// ---------------------------------------------------------------------------
// Configuration descriptor
// ---------------------------------------------------------------------------

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

// ---------------------------------------------------------------------------
// String descriptors
// ---------------------------------------------------------------------------

static const char *const string_desc[] = {
    NULL,                    // 0: language (handled below)
    "DIY",                   // 1: manufacturer
    "USB Motion Sensor",     // 2: product
    NULL,                    // 3: serial (from the BSP)
    "IMU stream",            // 4: CDC interface
};

static uint16_t desc_str[32 + 1];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t n;

    if (index == 0) {
        desc_str[1] = 0x0409;   // English (US)
        n = 1;
    } else {
        if (index >= sizeof string_desc / sizeof string_desc[0])
            return NULL;
        const char *s = (index == 3) ? bsp_serial_string() : string_desc[index];
        n = strlen(s);
        if (n > 32) n = 32;
        for (size_t i = 0; i < n; i++)
            desc_str[1 + i] = (uint8_t)s[i];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return desc_str;
}
