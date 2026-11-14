# USB Motion Sensor

ICM-20948 (Adafruit 4554) → Raspberry Pi Pico W → USB → ODROID Go Ultra.

The MCU only reads raw 9-axis data, timestamps it and streams it over USB CDC.
Calibration, fusion (Madgwick/EKF) and gyro-to-mouse/stick happen in a host daemon (not written yet).
The firmware is integer-only and keeps everything MCU-specific behind `bsp/bsp.h`, so it can be ported to the STM32F042.

## Wiring

| ICM-20948 (STEMMA QT) | Pico W            |
|-----------------------|-------------------|
| VIN (red)             | 3V3 (pin 36)      |
| GND (black)           | GND (pin 38)      |
| SDA (blue)            | GP4 (pin 6)       |
| SCL (yellow)          | GP5 (pin 7)       |
| INT (optional)        | any GPIO, set `-DIMU_INT_GPIO=<n>` |

I2C address: 0x69 by default, 0x68 with the ADR jumper. The firmware tries both.
If INT isn't wired, the firmware polls `INT_STATUS_1`. That's fine at 1125 Hz, but timestamps get ~100 µs of jitter.

## Build & flash

```sh
cmake -S . -B build -G Ninja          # or open the folder with the Pico VS Code extension
ninja -C build
# hold BOOTSEL, plug in, then:
cp build/usb_motion_sensor.uf2 /media/$USER/RPI-RP2/
```

After the first flash, `tools/imu_view.py --send B` reboots the device into BOOTSEL, so you don't have to press the button.

Build options: `-DPICO_BOARD=pico` (plain Pico, enables the LED on GP25), `-DIMU_INT_GPIO=6`, `-DIMU_I2C_SDA=..`, `-DIMU_I2C_SCL=..`.

## Host side

```sh
sudo cp tools/99-usb-motion-sensor.rules /etc/udev/rules.d/ && sudo udevadm control --reload
tools/imu_view.py                 # live view: rates, drops, gyro bias/noise, tilt, raw heading
tools/imu_view.py --csv log.csv   # also log every sample
tools/imu_3d.py                   # 3D orientation viewer (tkinter, no other deps)
```

`imu_3d.py` runs a Mahony AHRS on every sample, using dt = seq × estimated period, which removes timestamp jitter. It draws the board in 3D.
- At startup it estimates the gyro bias, so keep the sensor still for about 2 s.
- Keys:
  - `b` re-estimates the gyro bias.
  - `c` starts and finishes mag calibration: rotate the sensor through all directions in between. The hard and soft-iron result is saved to `~/.config/usb_motion_sensor/mag_cal.json`.
  - `m` toggles absolute heading from the magnetometer.
  - `z` zeroes the yaw.
  - `s` saves the current pose. It's then drawn as a dashed ghost, with the angle Δ to it shown at the top right.
  - `x` clears the saved pose.
  - `r` resets the filter.
- Drag the mouse to orbit the camera; use the wheel to zoom.
- Without the magnetometer, yaw drifts slowly (residual gyro bias). With a calibrated mag, the heading is absolute but sensitive to nearby iron.

## Sensor configuration (src/main.c)

| | |
|---|---|
| ODR | 1125 Hz for accel and gyro (`smplrt_div = 0`), aligned |
| Gyro | ±2000 dps, DLPF cfg 1 (~197 Hz) |
| Accel | ±4 g, DLPF cfg 2 (~111 Hz) |
| Mag | AK09916 continuous at 100 Hz, ICM in I2C **bypass**: the MCU reads the AK at 0x0C itself |

Each sample is one 14-byte burst (accel, gyro, temp). Right after it, every 2 ms, the MCU reads the AK's ST1 and, when DRDY is set, the 8 data bytes.

Why bypass instead of the ICM's internal I2C master (tested on hardware):
- The master re-reads the AK on every sample. The AK discards any measurement that completes during a read, so ~25 % of measurements were lost (DOR).
- A slower master leaves the shadowed data stale between our reads, so the same sample was flagged "new" in several frames.
- With `LP_CONFIG.I2C_MST_CYCLE = 0` the master doesn't run at all.

Measured with bypass: mag at 99.4 Hz, 0 DOR, 0 duplicates.

**The real ODR differs from the nominal 1125 Hz.** The ICM runs on its own oscillator: one unit measured 1114 Hz in one configuration and 1139 Hz in another. Estimate the rate from `t_us` and `seq` on the host. Don't assume 1/1125 s.

**Timestamp jitter.** Without the INT pin, `t_us` is the time the MCU noticed data-ready, which jitters by about ±200 µs (worst on samples where the mag is read). No samples are lost. For integration, either fit `t = t0 + seq·T` on the host, or wire INT, which gives ISR timestamps.

## Protocol (v2)

Device → host, all little-endian:

```
A5 5A | type u8 | len u8 | payload[len] | crc16 u16
```

The CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), computed over `type`, `len` and `payload`.

**0x01 IMU** (28 bytes), one per sensor sample:

| field | type | notes |
|---|---|---|
| t_us | u32 | MCU time of data-ready, wraps every ~71 min |
| seq | u16 | +1 per sample. A gap means a frame was dropped on USB |
| accel[3] | i16 | raw, 32768 = `accel_fs_g` |
| gyro[3] | i16 | raw, 32768 = `gyro_fs_dps` |
| temp | i16 | °C = raw / 333.87 + 21 |
| mag[3] | i16 | raw AK09916, 0.15 µT/LSB, **AK axes**: in the accel frame it's (x, −y, −z) |
| mag_st1 | u8 | bit0: this frame carries a new mag sample (each sample appears exactly once). bit1: the AK skipped a measurement (DOR) |
| mag_st2 | u8 | bit3: magnetic overflow |

The mag fields repeat the last reading between new samples. Use only frames with `mag_st1 & 1`.

**0x02 STATUS** (36 bytes). Sent when the port is opened, after a command or state change, and once per second:
`proto_version u8, fw_major u8, fw_minor u8, flags u8 (bit0 IMU ok, bit1 mag ok, bit2 INT pin, bit3 streaming), odr_mhz u32, gyro_fs_dps u16, accel_fs_g u8, imu_addr u8, uptime_ms u32, samples u32, usb_dropped u32, i2c_errors u32, overruns u32, mag_diag u8, mag_wia u8, reserved u8[2]`

`mag_diag` is 0 when the mag is OK. Otherwise it's `(step << 4) | reason`. Steps: 1 bypass setup, 2 WIA2 read, 4 CNTL2 write, 0xF not attempted. Reasons: 1 ICM I2C error, 2 AK NACK, 4 wrong ID.

`overruns` counts data-ready intervals longer than 1.5 periods, i.e. sensor samples the MCU missed.

Host → device: single ASCII bytes. `i` sends status, `s`/`x` turn streaming on/off, `r` re-initialises the sensor, `B` reboots into the bootloader. Any other byte is ignored.
The stream is only sent while the port is open (DTR set).

## Layout

```
src/        portable: main loop, ICM-20948 driver, protocol
usb/        TinyUSB config + descriptors (portable to STM32F0)
bsp/        bsp.h interface; rp2040/ implementation
tools/      imu_view.py (stdlib-only viewer/logger), udev rule
```

## Porting to the STM32F042

Write `bsp/stm32f0/bsp_stm32f0.c`. It needs I2C1, TIM2 as the µs clock, EXTI for INT, CRS for crystal-less USB, and a jump to the system-memory DFU for the bootloader.
Build TinyUSB with `CFG_TUSB_MCU=OPT_MCU_STM32F0`.
RAM use is currently ~3.7 KB on the RP2040, most of it TinyUSB buffers. The F042 has 6 KB, so `CFG_TUD_CDC_TX_BUFSIZE` can drop to 256 if needed.

## Open items

- Check that the Go Ultra's USB-C port works in host/OTG mode.
- Host daemon: calibration (gyro bias, accel 6-point, mag hard/soft iron), fusion, uinput mouse/stick, DSU server.
