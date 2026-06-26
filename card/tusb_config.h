/**
 * tusb_config.h  —  TinyUSB configuration for VCV Bridge card
 * CDC ACM device on RP2040
 */
#pragma once

// Target MCU
#define CFG_TUSB_MCU    OPT_MCU_RP2040
#define CFG_TUSB_OS     OPT_OS_PICO

// Endpoint 0 size
#define CFG_TUSB_DEBUG  0

// Device only
#define CFG_TUD_ENABLED 1

// CDC (virtual COM port) — 1 interface
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  512

// Disable unused classes
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0
