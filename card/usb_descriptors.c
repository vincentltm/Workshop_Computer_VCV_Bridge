/*
 * usb_descriptors.c  —  TinyUSB USB descriptors for VCV Bridge
 * Single CDC ACM interface: appears as a virtual COM port on all platforms.
 *
 * macOS:  /dev/cu.usbmodem*
 * Linux:  /dev/ttyACM0
 * Windows: COMx (usbser.sys, no driver install needed on Win10+)
 */

#include "tusb.h"

// ── Device descriptor ─────────────────────────────────────────────────────────
// VID/PID: use Music Thing Modular's assigned values if available,
// otherwise use TinyUSB's example values for development.
// !! Replace with your own VID/PID before production release !!

#define USBD_VID    0xCAFE  // TinyUSB example VID (replace!)
#define USBD_PID    0x4002  // Product ID for VCV Bridge
#define USBD_DESC_STR_MFR   "Music Thing Modular"
#define USBD_DESC_STR_PROD  "Workshop Computer VCV Bridge"
#define USBD_DESC_STR_SER   "01"

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

// ── Configuration descriptor ──────────────────────────────────────────────────

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define EPNUM_CDC_NOTIF     0x81
#define EPNUM_CDC_OUT       0x02
#define EPNUM_CDC_IN        0x82

uint8_t const desc_configuration[] = {
    // Config header
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // CDC: notification EP (64 bytes), data OUT + IN (64 bytes each)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// ── String descriptors ────────────────────────────────────────────────────────

char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: supported language (English)
    USBD_DESC_STR_MFR,              // 1: Manufacturer
    USBD_DESC_STR_PROD,             // 2: Product
    USBD_DESC_STR_SER,              // 3: Serial
    "CDC Interface",                 // 4: CDC Interface
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char* str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
