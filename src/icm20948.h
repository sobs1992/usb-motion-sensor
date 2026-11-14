// Minimal ICM-20948 driver: raw accel/gyro/temp + AK09916 magnetometer read
// directly by the MCU (ICM in I2C bypass mode). No floating point, no DMP.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum { ICM_GYRO_250DPS = 0, ICM_GYRO_500DPS, ICM_GYRO_1000DPS, ICM_GYRO_2000DPS } icm_gyro_fs_t;
typedef enum { ICM_ACCEL_2G = 0, ICM_ACCEL_4G, ICM_ACCEL_8G, ICM_ACCEL_16G } icm_accel_fs_t;

typedef struct {
    uint8_t        smplrt_div;   // ODR = 1125 Hz / (1 + div), applied to both gyro and accel
    icm_gyro_fs_t  gyro_fs;
    icm_accel_fs_t accel_fs;
    uint8_t        gyro_dlpf;    // GYRO_DLPFCFG 0..7 (DLPF always enabled)
    uint8_t        accel_dlpf;   // ACCEL_DLPFCFG 0..7
    bool           enable_mag;
} icm20948_cfg_t;

typedef struct {
    uint8_t addr;
    uint8_t bank;                // cached REG_BANK_SEL, 0xFF = unknown
    bool    mag_ok;
    uint8_t mag_diag;            // 0 = ok, else (failed step << 4) | reason, see icm20948.c
    uint8_t mag_wia;             // last AK09916 WIA2 value read (expect 0x09)
} icm20948_t;

// Raw accel/gyro/temp frame (big-endian ICM values decoded).
typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
} icm20948_sample_t;

typedef enum {
    ICM_OK = 0,
    ICM_ERR_I2C = -1,
    ICM_ERR_WHOAMI = -2,
} icm_err_t;

icm_err_t icm20948_init(icm20948_t *dev, uint8_t addr, const icm20948_cfg_t *cfg);

// Reads INT_STATUS_1 (clears it). Returns false on I2C error.
bool icm20948_poll_ready(icm20948_t *dev, bool *ready);

// One burst read of accel+gyro+temp (14 bytes). Returns false on I2C error.
bool icm20948_read(icm20948_t *dev, icm20948_sample_t *out);

// Checks AK09916 ST1; if a new sample is ready, reads it into mag[] (AK axes:
// x = accel x, y = -accel y, z = -accel z) and sets *st1 (bit0 DRDY, bit1 DOR)
// and *st2 (bit3 HOFL). With nothing new, *st1 = *st2 = 0 and mag[] is untouched.
// Call every ~2 ms: reading often is cheap (1 byte) but never mid-measurement-heavy.
bool icm20948_mag_poll(icm20948_t *dev, int16_t mag[3], uint8_t *st1, uint8_t *st2);

// ODR in millihertz for a given divider (1125000 / (1 + div)).
static inline uint32_t icm20948_odr_mhz(uint8_t div) { return 1125000u / (1u + div); }
