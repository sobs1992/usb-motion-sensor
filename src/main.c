// USB motion sensor: ICM-20948 -> raw 9-axis stream over USB CDC.
// All fusion/calibration happens on the host; the MCU only timestamps and ships
// samples. Portable: only bsp.h and TinyUSB are touched here.

#include <string.h>

#include "bsp.h"
#include "icm20948.h"
#include "protocol.h"
#include "tusb.h"

#define FW_MAJOR 0
#define FW_MINOR 1

#define IMU_ADDR_PRIMARY   0x69   // Adafruit 4554 default
#define IMU_ADDR_ALT       0x68   // with the ADR jumper closed

static const icm20948_cfg_t imu_cfg = {
    .smplrt_div = 0,              // 1125 Hz
    .gyro_fs    = ICM_GYRO_2000DPS,
    .accel_fs   = ICM_ACCEL_4G,
    .gyro_dlpf  = 1,              // ~197 Hz bandwidth
    .accel_dlpf = 2,              // ~111 Hz bandwidth
    .enable_mag = true,
};
static const uint16_t gyro_fs_dps[] = { 250, 500, 1000, 2000 };
static const uint8_t  accel_fs_g[]  = { 2, 4, 8, 16 };

#define REINIT_INTERVAL_US   1000000u
#define STATUS_INTERVAL_US   1000000u
#define DATA_TIMEOUT_US      100000u   // no data-ready for this long -> re-init sensor
#define MAX_CONSEC_I2C_ERR   10
#define MAG_POLL_US          2000u     // AK09916 runs at 100 Hz; check ST1 5x per period

static struct {
    icm20948_t dev;
    bool       ok;
    uint32_t   period_us;
    uint32_t   last_try_us;
    uint32_t   last_sample_us;
    uint32_t   last_poll_us;
    uint8_t    consec_err;
    bool       have_last;
    uint32_t   last_mag_poll_us;
    uint8_t    mag_consec_err;
    int16_t    mag[3];            // last mag reading, repeated until a new one arrives
} imu;

static struct {
    bool     stream;              // host may turn the stream off; DTR also gates it
    bool     was_connected;
    uint16_t seq;
    uint32_t samples;
    uint32_t usb_dropped;
    uint32_t i2c_errors;
    uint32_t overruns;
    uint32_t last_status_us;
    bool     status_requested;
} app = { .stream = true };

// ---------------------------------------------------------------------------

static void send_frame(uint8_t type, const void *payload, uint8_t len, bool count_drop)
{
    uint8_t buf[64];
    if (!tud_cdc_connected()) return;
    if (tud_cdc_write_available() < (uint32_t)len + PROTO_OVERHEAD) {
        if (count_drop) app.usb_dropped++;
        return;
    }
    uint32_t n = proto_frame(buf, type, payload, len);
    tud_cdc_write(buf, n);
    tud_cdc_write_flush();
}

static void send_status(void)
{
    pkt_status_t st = {
        .proto_version = PROTO_VERSION,
        .fw_major      = FW_MAJOR,
        .fw_minor      = FW_MINOR,
        .flags         = (uint8_t)((imu.ok ? STATUS_IMU_OK : 0)
                       | (imu.ok && imu.dev.mag_ok ? STATUS_MAG_OK : 0)
                       | (bsp_imu_int_supported() ? STATUS_INT_PIN : 0)
                       | (app.stream ? STATUS_STREAM : 0)),
        .odr_mhz       = icm20948_odr_mhz(imu_cfg.smplrt_div),
        .gyro_fs_dps   = gyro_fs_dps[imu_cfg.gyro_fs],
        .accel_fs_g    = accel_fs_g[imu_cfg.accel_fs],
        .imu_addr      = imu.dev.addr,
        .uptime_ms     = bsp_time_us() / 1000u,   // wraps with the us timer; good enough
        .samples       = app.samples,
        .usb_dropped   = app.usb_dropped,
        .i2c_errors    = app.i2c_errors,
        .overruns      = app.overruns,
        .mag_diag      = imu.dev.mag_diag,
        .mag_wia       = imu.dev.mag_wia,
    };
    send_frame(PKT_STATUS, &st, sizeof st, false);
}

// ---------------------------------------------------------------------------

static void imu_try_init(void)
{
    static const uint8_t addrs[] = { IMU_ADDR_PRIMARY, IMU_ADDR_ALT };
    imu.ok = false;
    for (unsigned i = 0; i < sizeof addrs; i++) {
        if (icm20948_init(&imu.dev, addrs[i], &imu_cfg) == ICM_OK) {
            imu.ok = true;
            break;
        }
    }
    imu.period_us = 1000000000u / icm20948_odr_mhz(imu_cfg.smplrt_div);
    imu.last_try_us = bsp_time_us();
    imu.last_sample_us = imu.last_try_us;
    imu.consec_err = 0;
    imu.have_last = false;
    imu.mag_consec_err = 0;
    memset(imu.mag, 0, sizeof imu.mag);
    bsp_led(imu.ok);
    app.status_requested = true;          // tell the host what happened
}

static void imu_i2c_error(void)
{
    app.i2c_errors++;
    if (++imu.consec_err >= MAX_CONSEC_I2C_ERR) {
        imu.ok = false;                   // re-init on the next retry tick
        bsp_led(false);
        app.status_requested = true;
    }
}

static void imu_task(void)
{
    uint32_t now = bsp_time_us();

    if (!imu.ok) {
        if (now - imu.last_try_us >= REINIT_INTERVAL_US)
            imu_try_init();
        return;
    }

    uint32_t ts;
    if (bsp_imu_int_supported()) {
        if (!bsp_imu_int_take(&ts)) {
            if (now - imu.last_sample_us > DATA_TIMEOUT_US) imu.ok = false;
            return;
        }
    } else {
        // Don't hammer the bus right after a sample: the next one is ~period away.
        // Polling is ~100 us per check at 400 kHz, which also bounds timestamp jitter.
        if (now - imu.last_sample_us < imu.period_us * 3 / 4) return;
        if (now - imu.last_poll_us < 50) return;
        imu.last_poll_us = now;
        bool ready;
        if (!icm20948_poll_ready(&imu.dev, &ready)) { imu_i2c_error(); return; }
        if (!ready) {
            if (now - imu.last_sample_us > DATA_TIMEOUT_US) imu.ok = false;
            return;
        }
        ts = now;
    }

    icm20948_sample_t s;
    if (!icm20948_read(&imu.dev, &s)) { imu_i2c_error(); return; }
    imu.consec_err = 0;

    if (imu.have_last && (ts - imu.last_sample_us) > imu.period_us * 3 / 2)
        app.overruns++;
    imu.last_sample_us = ts;
    imu.have_last = true;
    app.samples++;

    // Magnetometer: polled right after the ICM read, in the idle part of the
    // sample period, so it never delays data-ready detection.
    uint8_t st1 = 0, st2 = 0;
    if (imu.dev.mag_ok && ts - imu.last_mag_poll_us >= MAG_POLL_US) {
        imu.last_mag_poll_us = ts;
        if (icm20948_mag_poll(&imu.dev, imu.mag, &st1, &st2)) {
            imu.mag_consec_err = 0;
        } else {
            app.i2c_errors++;
            if (++imu.mag_consec_err >= MAX_CONSEC_I2C_ERR) {
                imu.dev.mag_ok = false;   // keep streaming accel/gyro; 'r' retries the mag
                app.status_requested = true;
            }
        }
    }

    pkt_imu_t p;
    p.t_us = ts;
    p.seq  = app.seq++;
    memcpy(p.accel, s.accel, sizeof p.accel);
    memcpy(p.gyro,  s.gyro,  sizeof p.gyro);
    p.temp = s.temp;
    memcpy(p.mag, imu.mag, sizeof p.mag);
    p.mag_st1 = st1;
    p.mag_st2 = st2;

    if (app.stream)
        send_frame(PKT_IMU, &p, sizeof p, true);
}

// ---------------------------------------------------------------------------

static void host_task(void)
{
    bool connected = tud_cdc_connected();
    if (connected && !app.was_connected)
        app.status_requested = true;      // greet a freshly opened port
    app.was_connected = connected;

    while (tud_cdc_available()) {
        int c = tud_cdc_read_char();
        switch (c) {
        case CMD_STATUS:     app.status_requested = true; break;
        case CMD_STREAM_ON:  app.stream = true;  app.status_requested = true; break;
        case CMD_STREAM_OFF: app.stream = false; app.status_requested = true; break;
        case CMD_REINIT:     imu_try_init(); break;
        case CMD_BOOTLOADER: bsp_enter_bootloader();
        default: break;                   // ignore noise (e.g. ModemManager AT probes)
        }
    }

    uint32_t now = bsp_time_us();
    if (app.status_requested || now - app.last_status_us >= STATUS_INTERVAL_US) {
        app.status_requested = false;
        app.last_status_us = now;
        send_status();
    }
}

void app_idle(void)
{
    tud_task();
}

int main(void)
{
    bsp_init();
    tud_init(BOARD_TUD_RHPORT);
    imu_try_init();

    for (;;) {
        tud_task();
        host_task();
        imu_task();
    }
}
