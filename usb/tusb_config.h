// TinyUSB configuration: one CDC-ACM interface carrying the binary IMU stream.
// Kept small enough for the STM32F042 (6 KB RAM, 1 KB PMA).
#pragma once

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT        0
#endif

// CFG_TUSB_MCU is supplied by the build (pico-sdk sets OPT_MCU_RP2040).
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS             OPT_OS_NONE   // pico-sdk overrides with OPT_OS_PICO
#endif
#define CFG_TUSB_DEBUG          0
#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE  64

#define CFG_TUD_CDC             1
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

#define CFG_TUD_CDC_EP_BUFSIZE  64
#define CFG_TUD_CDC_RX_BUFSIZE  64
// ~38 KB/s at 1125 Hz = ~1.3 frames per ms. 512 B covers >10 ms of host stall.
#ifndef CFG_TUD_CDC_TX_BUFSIZE
#define CFG_TUD_CDC_TX_BUFSIZE  512
#endif
