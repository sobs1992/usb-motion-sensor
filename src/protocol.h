// Wire protocol, device -> host (see README.md for the full description).
//
//   A5 5A | type | len | payload[len] | crc16 (LE)
//
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload.
// All multi-byte payload fields are little-endian.
#pragma once

#include <stdint.h>

#define PROTO_VERSION   2
#define PROTO_SYNC0     0xA5
#define PROTO_SYNC1     0x5A
#define PROTO_OVERHEAD  6      // sync(2) type len crc(2)

enum {
    PKT_IMU    = 0x01,
    PKT_STATUS = 0x02,
};

typedef struct __attribute__((packed)) {
    uint32_t t_us;             // MCU timestamp of data-ready (wraps)
    uint16_t seq;              // +1 per sample read from the sensor; gaps = USB drops
    int16_t  accel[3];         // raw, see status.accel_fs_g
    int16_t  gyro[3];          // raw, see status.gyro_fs_dps
    int16_t  temp;             // raw: degC = temp / 333.87 + 21
    int16_t  mag[3];           // raw AK09916, 0.15 uT/LSB, AK axes
    uint8_t  mag_st1;          // bit0 new mag sample in THIS frame, bit1 AK skipped one (DOR)
    uint8_t  mag_st2;          // bit3 magnetic overflow
} pkt_imu_t;                   // 28 bytes

enum {
    STATUS_IMU_OK   = 1u << 0,
    STATUS_MAG_OK   = 1u << 1,
    STATUS_INT_PIN  = 1u << 2,
    STATUS_STREAM   = 1u << 3,
};

typedef struct __attribute__((packed)) {
    uint8_t  proto_version;
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  flags;            // STATUS_*
    uint32_t odr_mhz;          // sensor ODR in millihertz
    uint16_t gyro_fs_dps;      // full scale: raw 32768 == this many deg/s
    uint8_t  accel_fs_g;       // full scale: raw 32768 == this many g
    uint8_t  imu_addr;         // 7-bit I2C address in use
    uint32_t uptime_ms;
    uint32_t samples;          // total samples read from the sensor
    uint32_t usb_dropped;      // samples not sent because the CDC FIFO was full
    uint32_t i2c_errors;
    uint32_t overruns;         // data-ready intervals > 1.5 periods (missed sensor samples)
    uint8_t  mag_diag;         // 0 ok; else (step << 4) | reason. step: 1 bypass setup, 2 WIA, 4 CNTL2,
                               // 0xF not attempted. reason: 1 ICM I2C, 2 AK NACK, 4 bad ID
    uint8_t  mag_wia;          // AK09916 WIA2 as read (0x09 expected)
    uint8_t  reserved[2];
} pkt_status_t;                // 36 bytes

_Static_assert(sizeof(pkt_imu_t) == 28, "pkt_imu_t layout");
_Static_assert(sizeof(pkt_status_t) == 36, "pkt_status_t layout");

// Host -> device: single ASCII command bytes, anything else is ignored.
#define CMD_STATUS      'i'    // send a status packet now
#define CMD_STREAM_ON   's'
#define CMD_STREAM_OFF  'x'
#define CMD_REINIT      'r'    // re-initialise the sensor
#define CMD_BOOTLOADER  'B'    // reboot into the ROM bootloader

uint16_t proto_crc16(const uint8_t *data, uint32_t len, uint16_t crc);

// Builds a full frame into `out` (needs len + PROTO_OVERHEAD bytes). Returns frame size.
uint32_t proto_frame(uint8_t *out, uint8_t type, const void *payload, uint8_t len);
