#include "icm20948.h"

#include "bsp.h"
#include "icm20948_regs.h"

// ---------------------------------------------------------------------------
// Low-level register access with bank switching
// ---------------------------------------------------------------------------

static bool select_bank(icm20948_t *d, uint8_t bank)
{
    if (d->bank == bank)
        return true;
    uint8_t buf[2] = { ICM_REG_BANK_SEL, (uint8_t)(bank << 4) };
    if (!bsp_i2c_write(d->addr, buf, 2)) {
        d->bank = 0xFF;
        return false;
    }
    d->bank = bank;
    return true;
}

static bool wr(icm20948_t *d, uint16_t reg, uint8_t val)
{
    if (!select_bank(d, reg >> 8))
        return false;
    uint8_t buf[2] = { (uint8_t)reg, val };
    return bsp_i2c_write(d->addr, buf, 2);
}

static bool rd(icm20948_t *d, uint16_t reg, uint8_t *buf, size_t len)
{
    if (!select_bank(d, reg >> 8))
        return false;
    uint8_t r = (uint8_t)reg;
    return bsp_i2c_write_read(d->addr, &r, 1, buf, len);
}

static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static inline int16_t le16(const uint8_t *p) { return (int16_t)((p[1] << 8) | p[0]); }

// ---------------------------------------------------------------------------
// AK09916, accessed directly on the host bus (ICM in I2C bypass mode)
//
// Why not the ICM's internal I2C master: it re-reads the AK on every sample
// (~1 kHz) and the AK drops any measurement that completes during a read, so
// ~25 % of samples were lost (DOR). Slowing the master down makes the shadowed
// EXT_SLV data go stale between our reads, so DRDY shows up in several frames
// and "new sample" can't be told apart. Reading the AK ourselves at a pace we
// control gives exact new-sample flags and less bus traffic.
// ---------------------------------------------------------------------------

static bool ak_wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return bsp_i2c_write(AK_ADDR, buf, 2);
}

static bool ak_rd(uint8_t reg, uint8_t *buf, size_t len)
{
    return bsp_i2c_write_read(AK_ADDR, &reg, 1, buf, len);
}

// mag_diag reasons (low nibble)
enum { MAG_ERR_ICM_I2C = 1, MAG_ERR_AK_NACK = 2, MAG_ERR_ID = 4 };

#define MAG_FAIL(step, code) do { d->mag_diag = (uint8_t)(((step) << 4) | (code)); return false; } while (0)

static bool mag_init(icm20948_t *d)
{
    // Step 1: internal master off, bypass on -> AK09916 appears at 0x0C on our bus.
    if (!(wr(d, ICM_USER_CTRL, 0x00)
          && wr(d, ICM_INT_PIN_CFG, INT_PIN_CFG_BYPASS_EN)))
        MAG_FAIL(1, MAG_ERR_ICM_I2C);
    bsp_delay_ms(1);

    // Step 2: identify
    d->mag_wia = 0;
    if (!ak_rd(AK_WIA2, &d->mag_wia, 1)) MAG_FAIL(2, MAG_ERR_AK_NACK);
    if (d->mag_wia != AK_WIA2_VALUE)     MAG_FAIL(2, MAG_ERR_ID);

    // Step 3: soft reset (may NACK while resetting), step 4: continuous 100 Hz
    ak_wr(AK_CNTL3, AK_CNTL3_SRST);
    bsp_delay_ms(5);
    if (!ak_wr(AK_CNTL2, AK_CNTL2_CONT_100HZ)) MAG_FAIL(4, MAG_ERR_AK_NACK);

    d->mag_diag = 0;
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

icm_err_t icm20948_init(icm20948_t *d, uint8_t addr, const icm20948_cfg_t *cfg)
{
    d->addr = addr;
    d->bank = 0xFF;
    d->mag_ok = false;
    d->mag_diag = 0xF0;                                 // "not attempted"
    d->mag_wia = 0;

    uint8_t who = 0;
    if (!rd(d, ICM_WHO_AM_I, &who, 1))
        return ICM_ERR_I2C;
    if (who != ICM_WHO_AM_I_VALUE)
        return ICM_ERR_WHOAMI;

    if (!wr(d, ICM_PWR_MGMT_1, PWR_MGMT_1_RESET))
        return ICM_ERR_I2C;
    bsp_delay_ms(100);
    d->bank = 0xFF;                                    // reset puts it back to bank 0, be safe

    if (!wr(d, ICM_PWR_MGMT_1, PWR_MGMT_1_CLK_AUTO)) return ICM_ERR_I2C;   // wake, PLL clock
    bsp_delay_ms(10);
    if (!wr(d, ICM_PWR_MGMT_2, 0x00)) return ICM_ERR_I2C;                  // all axes on

    // Sample rate and full scale. FCHOICE = 1 (DLPF path) for both.
    if (!wr(d, ICM_GYRO_SMPLRT_DIV, cfg->smplrt_div)) return ICM_ERR_I2C;
    if (!wr(d, ICM_GYRO_CONFIG_1,
            (uint8_t)(((cfg->gyro_dlpf & 7) << 3) | ((cfg->gyro_fs & 3) << 1) | 1)))
        return ICM_ERR_I2C;
    if (!wr(d, ICM_ACCEL_SMPLRT_DIV_1, 0)) return ICM_ERR_I2C;
    if (!wr(d, ICM_ACCEL_SMPLRT_DIV_2, cfg->smplrt_div)) return ICM_ERR_I2C;
    if (!wr(d, ICM_ACCEL_CONFIG,
            (uint8_t)(((cfg->accel_dlpf & 7) << 3) | ((cfg->accel_fs & 3) << 1) | 1)))
        return ICM_ERR_I2C;
    if (!wr(d, ICM_ODR_ALIGN_EN, 0x01)) return ICM_ERR_I2C;

    // INT: active high, push-pull, 50 us pulse, cleared by reading INT_STATUS.
    // (mag_init() adds BYPASS_EN to this register.)
    if (!wr(d, ICM_INT_PIN_CFG, 0x00)) return ICM_ERR_I2C;
    if (!wr(d, ICM_INT_ENABLE_1, INT_ENABLE_1_RAW_RDY)) return ICM_ERR_I2C;

    if (cfg->enable_mag)
        d->mag_ok = mag_init(d);

    bsp_delay_ms(20);                                   // let filters settle
    uint8_t dummy;
    if (!rd(d, ICM_INT_STATUS_1, &dummy, 1))
        return ICM_ERR_I2C;
    return ICM_OK;
}

bool icm20948_poll_ready(icm20948_t *d, bool *ready)
{
    uint8_t st;
    if (!rd(d, ICM_INT_STATUS_1, &st, 1))
        return false;
    *ready = (st & INT_STATUS_1_RAW_RDY) != 0;
    return true;
}

bool icm20948_read(icm20948_t *d, icm20948_sample_t *s)
{
    // 0x2D..0x3A: accel(6) gyro(6) temp(2)
    uint8_t b[14];
    if (!rd(d, ICM_ACCEL_XOUT_H, b, sizeof b))
        return false;
    for (int i = 0; i < 3; i++) {
        s->accel[i] = be16(&b[0 + 2 * i]);
        s->gyro[i]  = be16(&b[6 + 2 * i]);
    }
    s->temp = be16(&b[12]);
    return true;
}

bool icm20948_mag_poll(icm20948_t *d, int16_t mag[3], uint8_t *st1, uint8_t *st2)
{
    uint8_t s1;
    *st1 = 0;
    *st2 = 0;
    if (!ak_rd(AK_ST1, &s1, 1))
        return false;
    if (!(s1 & AK_ST1_DRDY))
        return true;                                    // nothing new yet
    // HXL..HZH, TMPS (dummy), ST2. Reading ST2 releases the data latch.
    uint8_t b[8];
    if (!ak_rd(AK_HXL, b, sizeof b))
        return false;
    for (int i = 0; i < 3; i++)
        mag[i] = le16(&b[2 * i]);
    *st1 = s1;
    *st2 = b[7];
    return true;
}
