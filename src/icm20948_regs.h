// ICM-20948 / AK09916 register map (subset). Address = (bank << 8) | reg.
#pragma once

#define ICM_REG(bank, reg) ((uint16_t)(((bank) << 8) | (reg)))

// ---- Bank 0 ----
#define ICM_WHO_AM_I            ICM_REG(0, 0x00)   // = 0xEA
#define ICM_USER_CTRL           ICM_REG(0, 0x03)
#define   USER_CTRL_I2C_MST_EN    0x20
#define   USER_CTRL_I2C_MST_RST   0x02
#define ICM_LP_CONFIG           ICM_REG(0, 0x05)
#define ICM_PWR_MGMT_1          ICM_REG(0, 0x06)
#define   PWR_MGMT_1_RESET        0x80
#define   PWR_MGMT_1_CLK_AUTO     0x01
#define ICM_PWR_MGMT_2          ICM_REG(0, 0x07)
#define ICM_INT_PIN_CFG         ICM_REG(0, 0x0F)
#define   INT_PIN_CFG_BYPASS_EN   0x02
#define ICM_INT_ENABLE_1        ICM_REG(0, 0x11)
#define   INT_ENABLE_1_RAW_RDY    0x01
#define ICM_I2C_MST_STATUS      ICM_REG(0, 0x17)
#define   I2C_MST_STATUS_SLV4_DONE 0x40
#define   I2C_MST_STATUS_SLV4_NACK 0x10
#define ICM_INT_STATUS_1        ICM_REG(0, 0x1A)
#define   INT_STATUS_1_RAW_RDY    0x01
#define ICM_ACCEL_XOUT_H        ICM_REG(0, 0x2D)   // accel(6) gyro(6) temp(2) ext(24) contiguous
#define ICM_EXT_SLV_SENS_DATA_00 ICM_REG(0, 0x3B)
#define ICM_REG_BANK_SEL        0x7F               // same address in every bank

// ---- Bank 2 ----
#define ICM_GYRO_SMPLRT_DIV     ICM_REG(2, 0x00)
#define ICM_GYRO_CONFIG_1       ICM_REG(2, 0x01)
#define ICM_ODR_ALIGN_EN        ICM_REG(2, 0x09)
#define ICM_ACCEL_SMPLRT_DIV_1  ICM_REG(2, 0x10)
#define ICM_ACCEL_SMPLRT_DIV_2  ICM_REG(2, 0x11)
#define ICM_ACCEL_CONFIG        ICM_REG(2, 0x14)

// ---- Bank 3 (internal I2C master for the magnetometer) ----
#define ICM_I2C_MST_CTRL        ICM_REG(3, 0x01)
#define ICM_I2C_MST_DELAY_CTRL  ICM_REG(3, 0x02)
#define   I2C_MST_DELAY_ES_SHADOW 0x80
#define ICM_I2C_SLV0_ADDR       ICM_REG(3, 0x03)
#define ICM_I2C_SLV0_REG        ICM_REG(3, 0x04)
#define ICM_I2C_SLV0_CTRL       ICM_REG(3, 0x05)
#define ICM_I2C_SLV4_ADDR       ICM_REG(3, 0x13)
#define ICM_I2C_SLV4_REG        ICM_REG(3, 0x14)
#define ICM_I2C_SLV4_CTRL       ICM_REG(3, 0x15)
#define ICM_I2C_SLV4_DO         ICM_REG(3, 0x16)
#define ICM_I2C_SLV4_DI         ICM_REG(3, 0x17)
#define   I2C_SLV_EN              0x80
#define   I2C_SLV_READ            0x80             // bit 7 of SLVx_ADDR

#define ICM_WHO_AM_I_VALUE      0xEA

// ---- AK09916 (on the ICM's auxiliary I2C bus) ----
#define AK_ADDR                 0x0C
#define AK_WIA2                 0x01               // = 0x09
#define AK_ST1                  0x10               // bit0 DRDY, bit1 DOR
#define   AK_ST1_DRDY             0x01
#define AK_HXL                  0x11
#define AK_ST2                  0x18               // bit3 HOFL; reading ST2 releases the data latch
#define AK_CNTL2                0x31
#define   AK_CNTL2_CONT_100HZ     0x08
#define AK_CNTL3                0x32
#define   AK_CNTL3_SRST           0x01
#define AK_WIA2_VALUE           0x09
