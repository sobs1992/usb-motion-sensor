#include "bsp.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#define IMU_I2C      i2c0
#define IMU_I2C_HZ   400000
#define I2C_TIMEOUT_PER_BYTE_US 100   // 22.5 us/byte at 400 kHz, generous margin

#if defined(PICO_DEFAULT_LED_PIN)
#define HAS_LED 1                      // plain Pico: GPIO25
#else
#define HAS_LED 0                      // Pico W: LED is behind the CYW43, not worth the blob
#endif

static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

#if IMU_INT_GPIO >= 0
static volatile uint32_t int_ts_us;
static volatile bool int_pending;

static void imu_int_isr(uint gpio, uint32_t events)
{
    if (gpio == IMU_INT_GPIO && (events & GPIO_IRQ_EDGE_RISE)) {
        int_ts_us = time_us_32();
        int_pending = true;
    }
}
#endif

void bsp_init(void)
{
    // Default clocks from the SDK runtime already give 48 MHz on clk_usb.
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(IMU_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL, GPIO_FUNC_I2C);
    // The Adafruit breakout has 10k pull-ups; the internal ones are a harmless extra.
    gpio_pull_up(IMU_I2C_SDA);
    gpio_pull_up(IMU_I2C_SCL);

#if IMU_INT_GPIO >= 0
    gpio_init(IMU_INT_GPIO);
    gpio_set_dir(IMU_INT_GPIO, GPIO_IN);
    gpio_pull_down(IMU_INT_GPIO);
    gpio_set_irq_enabled_with_callback(IMU_INT_GPIO, GPIO_IRQ_EDGE_RISE, true, imu_int_isr);
#endif

#if HAS_LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    pico_get_unique_board_id_string(serial_str, sizeof serial_str);
}

uint32_t bsp_time_us(void) { return time_us_32(); }

void bsp_delay_ms(uint32_t ms)
{
    uint32_t t0 = time_us_32();
    while (time_us_32() - t0 < ms * 1000u)
        app_idle();
}

bool bsp_i2c_write(uint8_t addr, const uint8_t *buf, size_t len)
{
    int n = i2c_write_timeout_us(IMU_I2C, addr, buf, len, false,
                                 (uint)(len + 2) * I2C_TIMEOUT_PER_BYTE_US);
    return n == (int)len;
}

bool bsp_i2c_write_read(uint8_t addr, const uint8_t *w, size_t wlen, uint8_t *r, size_t rlen)
{
    int n = i2c_write_timeout_us(IMU_I2C, addr, w, wlen, true,
                                 (uint)(wlen + 2) * I2C_TIMEOUT_PER_BYTE_US);
    if (n != (int)wlen)
        return false;
    n = i2c_read_timeout_us(IMU_I2C, addr, r, rlen, false,
                            (uint)(rlen + 2) * I2C_TIMEOUT_PER_BYTE_US);
    return n == (int)rlen;
}

bool bsp_imu_int_supported(void) { return IMU_INT_GPIO >= 0; }

bool bsp_imu_int_take(uint32_t *ts_us)
{
#if IMU_INT_GPIO >= 0
    bool got = false;
    uint32_t irq = save_and_disable_interrupts();
    if (int_pending) {
        int_pending = false;
        *ts_us = int_ts_us;
        got = true;
    }
    restore_interrupts(irq);
    return got;
#else
    (void)ts_us;
    return false;
#endif
}

void bsp_led(bool on)
{
#if HAS_LED
    gpio_put(PICO_DEFAULT_LED_PIN, on);
#else
    (void)on;
#endif
}

const char *bsp_serial_string(void) { return serial_str; }

void bsp_enter_bootloader(void)
{
    rom_reset_usb_boot(0, 0);
}
