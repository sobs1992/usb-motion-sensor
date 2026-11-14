// Board support interface. Everything MCU-specific lives behind this header,
// so the rest of the firmware (sensor driver, protocol, main loop) is portable
// between RP2040 (prototype) and STM32F042 (target). Integer-only on purpose:
// the F042 has no FPU.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void bsp_init(void);

// Free-running microsecond counter (wraps at 2^32, ~71 min).
uint32_t bsp_time_us(void);
// Busy-waits, calling app_idle() in the loop so USB keeps being serviced
// (sensor init takes ~150 ms, too long to stall enumeration).
void bsp_delay_ms(uint32_t ms);

// Provided by the application: work that must keep running during busy-waits.
void app_idle(void);

// I2C master, 7-bit address. Return true on success (all bytes ACKed).
bool bsp_i2c_write(uint8_t addr, const uint8_t *buf, size_t len);
// Write `wlen` bytes, repeated START, read `rlen` bytes.
bool bsp_i2c_write_read(uint8_t addr, const uint8_t *w, size_t wlen, uint8_t *r, size_t rlen);

// IMU INT line. If the board has it wired, bsp_imu_int_take() returns true once
// per rising edge and gives the edge timestamp captured in the ISR.
bool bsp_imu_int_supported(void);
bool bsp_imu_int_take(uint32_t *ts_us);

void bsp_led(bool on);

// NUL-terminated unique serial for the USB string descriptor.
const char *bsp_serial_string(void);

// Reboot into the ROM USB bootloader (BOOTSEL on RP2040, DFU on STM32).
void bsp_enter_bootloader(void) __attribute__((noreturn));
