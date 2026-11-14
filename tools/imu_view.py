#!/usr/bin/env python3
"""Live viewer / logger for the USB Motion Sensor raw IMU stream.

Stdlib only (no pyserial), so it runs as-is on the ODROID.

  tools/imu_view.py                 # auto-detect port, live view
  tools/imu_view.py --csv log.csv   # also log every sample
  tools/imu_view.py --send B        # reboot device into BOOTSEL and exit
"""
import argparse
import glob
import math
import os
import select
import struct
import sys
import termios
import time
import tty

SYNC = b"\xA5\x5A"
PKT_IMU, PKT_STATUS = 0x01, 0x02
IMU_FMT = "<IH3h3hh3hBB"          # 28 bytes
STATUS_FMT = "<BBBBIHBBIIIIIBBxx" # 36 bytes
MAG_UT_PER_LSB = 0.15
TEMP_SENS = 333.87


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def find_port():
    hits = sorted(glob.glob("/dev/serial/by-id/*USB_Motion_Sensor*"))
    if hits:
        return hits[0]
    hits = sorted(glob.glob("/dev/ttyACM*"))
    if hits:
        return hits[0]
    sys.exit("no device found (looked for /dev/serial/by-id/*USB_Motion_Sensor* and /dev/ttyACM*)")


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)                       # no echo / line discipline; DTR is raised on open
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]        # keep a possible half sync byte
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ln = self.buf[3]
            total = 4 + ln + 2
            if len(self.buf) < total:
                return out
            frame = bytes(self.buf[:total])
            want = frame[-2] | (frame[-1] << 8)
            if crc16(frame[2:-2]) != want:
                self.crc_errors += 1
                del self.buf[:2]         # resync past this false sync
                continue
            del self.buf[:total]
            out.append((frame[2], frame[4:-2]))


class Stats:
    """Per-window accumulator for the live display."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.g_sum = [0.0] * 3
        self.g_sq = [0.0] * 3
        self.mag_new = 0

    def add(self, gyro_dps, mag_new):
        self.n += 1
        for i in range(3):
            self.g_sum[i] += gyro_dps[i]
            self.g_sq[i] += gyro_dps[i] ** 2
        self.mag_new += mag_new

    def gyro_mean_std(self):
        if not self.n:
            return [0.0] * 3, [0.0] * 3
        mean = [s / self.n for s in self.g_sum]
        std = [math.sqrt(max(q / self.n - m * m, 0.0)) for q, m in zip(self.g_sq, mean)]
        return mean, std


def tilt_heading(acc, mag):
    ax, ay, az = acc
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    mx, my, mz = mag
    # tilt-compensated heading (uncalibrated mag -> only a rough indication)
    xh = mx * math.cos(pitch) + my * math.sin(roll) * math.sin(pitch) + mz * math.cos(roll) * math.sin(pitch)
    yh = my * math.cos(roll) - mz * math.sin(roll)
    heading = math.degrees(math.atan2(-yh, xh)) % 360.0
    return math.degrees(roll), math.degrees(pitch), heading


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (default: auto)")
    ap.add_argument("--csv", help="write every IMU sample (scaled) to this CSV file")
    ap.add_argument("--send", help="send these command bytes (i s x r B) and continue; with B exits")
    ap.add_argument("--hz", type=float, default=10.0, help="display refresh rate")
    args = ap.parse_args()

    port = args.port or find_port()
    fd = open_port(port)

    if args.send:
        os.write(fd, args.send.encode())
        if "B" in args.send:
            print("bootloader command sent")
            return

    os.write(fd, b"i")                    # ask for status right away
    parser = Parser()
    stats = Stats()
    status = None
    gyro_scale = 2000 / 32768.0
    acc_scale = 4 / 32768.0
    last_seq = None
    seq_gaps = 0
    total = 0
    t_first = None
    last = None
    mag_ut = (0.0, 0.0, 0.0)
    last_t_us = None
    dt_min = dt_max = None
    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("t_us,seq,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,mx_ut,my_ut,mz_ut,mag_new\n")

    period = 1.0 / args.hz
    next_draw = time.monotonic() + period
    print("\x1b[2J", end="")
    try:
        while True:
            r, _, _ = select.select([fd], [], [], period)
            if r:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                except OSError:                  # EIO when the device goes away
                    data = b""
                if not data:
                    sys.exit("\ndevice disconnected")
                for typ, payload in parser.feed(data):
                    if typ == PKT_STATUS and len(payload) == struct.calcsize(STATUS_FMT):
                        f = struct.unpack(STATUS_FMT, payload)
                        status = dict(zip(
                            ("proto", "fw_major", "fw_minor", "flags", "odr_mhz", "gyro_fs", "accel_fs",
                             "addr", "uptime_ms", "samples", "usb_dropped", "i2c_errors", "overruns",
                             "mag_diag", "mag_wia"), f))
                        gyro_scale = status["gyro_fs"] / 32768.0
                        acc_scale = status["accel_fs"] / 32768.0
                    elif typ == PKT_IMU and len(payload) == struct.calcsize(IMU_FMT):
                        f = struct.unpack(IMU_FMT, payload)
                        t_us, seq = f[0], f[1]
                        acc = [v * acc_scale for v in f[2:5]]
                        gyr = [v * gyro_scale for v in f[5:8]]
                        temp = f[8] / TEMP_SENS + 21.0
                        st1 = f[12]
                        mag_new = st1 & 1
                        if mag_new:
                            mx, my, mz = f[9:12]
                            # AK09916 axes -> accel/gyro frame: (x, -y, -z)
                            mag_ut = (mx * MAG_UT_PER_LSB, -my * MAG_UT_PER_LSB, -mz * MAG_UT_PER_LSB)
                        if last_seq is not None and seq != (last_seq + 1) & 0xFFFF:
                            seq_gaps += (seq - last_seq - 1) & 0xFFFF
                        last_seq = seq
                        if last_t_us is not None:
                            dt = (t_us - last_t_us) & 0xFFFFFFFF
                            dt_min = dt if dt_min is None else min(dt_min, dt)
                            dt_max = dt if dt_max is None else max(dt_max, dt)
                        last_t_us = t_us
                        total += 1
                        if t_first is None:
                            t_first = time.monotonic()
                        stats.add(gyr, mag_new)
                        last = (acc, gyr, temp)
                        if csv:
                            csv.write("%d,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%d\n" % (
                                t_us, seq, *acc, *gyr, temp, *mag_ut, mag_new))

            now = time.monotonic()
            if now >= next_draw:
                next_draw = now + period
                draw(port, status, stats, last, mag_ut, total, seq_gaps, parser.crc_errors,
                     t_first, dt_min, dt_max, period)
                stats.reset()
                dt_min = dt_max = None
    except KeyboardInterrupt:
        print()
    finally:
        if csv:
            csv.close()
        os.close(fd)


def draw(port, status, stats, last, mag_ut, total, seq_gaps, crc_errors, t_first, dt_min, dt_max, period):
    L = ["\x1b[H\x1b[0J" + f"port: {port}"]
    if status:
        fl = status["flags"]
        L.append("fw %d.%d proto %d | IMU %s @0x%02X | mag %s | INT pin %s | stream %s" % (
            status["fw_major"], status["fw_minor"], status["proto"],
            "OK" if fl & 1 else "MISSING", status["addr"], "OK" if fl & 2 else "--",
            "yes" if fl & 4 else "no (polling)", "on" if fl & 8 else "OFF"))
        if not fl & 2:
            steps = {1: "bypass setup", 2: "WIA2 read", 4: "CNTL2 write", 15: "not attempted"}
            why = {0: "-", 1: "ICM I2C error", 2: "AK NACK", 4: "wrong ID"}
            d = status["mag_diag"]
            L.append("mag init failed: step %s, reason %s, WIA2=0x%02X (diag 0x%02X)" % (
                steps.get(d >> 4, d >> 4), why.get(d & 15, d & 15), status["mag_wia"], d))
        L.append("ODR %.1f Hz | FS ±%d dps ±%d g | dev: samples %d, usb_dropped %d, i2c_err %d, overruns %d" % (
            status["odr_mhz"] / 1000, status["gyro_fs"], status["accel_fs"], status["samples"],
            status["usb_dropped"], status["i2c_errors"], status["overruns"]))
    else:
        L.append("waiting for status packet...")
    rate = stats.n / period
    L.append("host: rx rate %.0f Hz, total %d, seq gaps %d, crc errors %d, mag %.0f Hz, dt %s..%s us" % (
        rate, total, seq_gaps, crc_errors, stats.mag_new / period,
        dt_min if dt_min is not None else "-", dt_max if dt_max is not None else "-"))
    L.append("")
    if last:
        acc, gyr, temp = last
        mean, std = stats.gyro_mean_std()
        roll, pitch, heading = tilt_heading(acc, mag_ut)
        L.append("accel  [g]   x %+7.3f  y %+7.3f  z %+7.3f   |a| %.3f" % (*acc, math.sqrt(sum(a * a for a in acc))))
        L.append("gyro [dps]   x %+8.2f y %+8.2f z %+8.2f" % tuple(gyr))
        L.append("  window mean x %+8.3f y %+8.3f z %+8.3f   (bias when still)" % tuple(mean))
        L.append("  window std  x %8.3f  y %8.3f  z %8.3f   (noise)" % tuple(std))
        L.append("mag   [uT]   x %+7.1f  y %+7.1f  z %+7.1f   |B| %.1f (uncalibrated)" % (
            *mag_ut, math.sqrt(sum(m * m for m in mag_ut))))
        L.append("temp  %.2f C" % temp)
        L.append("roll %+7.1f  pitch %+7.1f  heading %5.1f (from accel+raw mag, rough)" % (roll, pitch, heading))
    L.append("\nCtrl+C to quit")
    sys.stdout.write("\n".join(L) + "\n")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
