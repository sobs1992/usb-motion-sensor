#!/usr/bin/env python3
"""3D orientation viewer for the USB Motion Sensor.

Runs a Mahony AHRS filter on every sample (~1.1 kHz) and draws the sensor as a
3D board. Needs only the stdlib + tkinter (Debian/Ubuntu: python3-tk).

Keys:
  b  re-estimate gyro bias (keep the sensor still ~2 s; also done at start)
  m  toggle magnetometer (absolute heading; needs mag calibration first)
  c  start / finish mag calibration (rotate the sensor through all directions)
  z  zero heading (current yaw becomes 0)
  s  save current pose as target     x  clear target
  r  reset filter                    q / Esc  quit
Mouse: drag to orbit the camera, wheel to zoom.
"""
import argparse
import glob
import json
import math
import os
import select
import struct
import sys
import threading
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_view import (IMU_FMT, MAG_UT_PER_LSB, PKT_IMU, PKT_STATUS,  # noqa: E402
                      STATUS_FMT, Parser, open_port)

CAL_PATH = os.path.expanduser("~/.config/usb_motion_sensor/mag_cal.json")
DEG = math.pi / 180.0

# ---------------------------------------------------------------------------
# Quaternion helpers, (w, x, y, z), q rotates body -> world, world Z up
# ---------------------------------------------------------------------------


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qnormalize(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def qrot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # v' = v + 2w(u x v) + 2 u x (u x v), u = (x, y, z)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def q_axis_angle(axis, ang):
    s = math.sin(ang / 2)
    return (math.cos(ang / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def q_angle_between(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2 * math.acos(min(1.0, d))


def q_euler(q):
    """ZYX: yaw, pitch, roll in degrees."""
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw / DEG, pitch / DEG, roll / DEG


def q_from_accel(a):
    ax, ay, az = a
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return qmul(q_axis_angle((0, 1, 0), pitch), q_axis_angle((1, 0, 0), roll))


# ---------------------------------------------------------------------------
# Mahony AHRS
# ---------------------------------------------------------------------------


class Mahony:
    KP, KI, KP_MAG = 0.6, 0.01, 1.5
    KP_STARTUP, STARTUP_S = 8.0, 1.5

    def __init__(self):
        self.reset()

    def reset(self):
        self.q = None
        self.ei = [0.0, 0.0, 0.0]
        self.t_run = 0.0

    def update(self, g, a, m, dt):
        """g: rad/s body, a: g units body, m: unit-less body vector or None."""
        if self.q is None:
            self.q = q_from_accel(a)
            return
        self.t_run += dt
        kp = self.KP_STARTUP if self.t_run < self.STARTUP_S else self.KP
        q0, q1, q2, q3 = self.q
        ex = ey = ez = 0.0
        # estimated gravity ("up") direction in body frame = R^T * ez
        vx = 2 * (q1 * q3 - q0 * q2)
        vy = 2 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        an = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        if 0.85 < an < 1.15:                      # skip accel during strong linear motion
            ax, ay, az = a[0] / an, a[1] / an, a[2] / an
            ex += ay * vz - az * vy
            ey += az * vx - ax * vz
            ez += ax * vy - ay * vx

        if m is not None:
            mn = math.sqrt(m[0] ** 2 + m[1] ** 2 + m[2] ** 2)
            if mn > 0:
                mx, my, mz = m[0] / mn, m[1] / mn, m[2] / mn
                hx, hy, hz = qrot(self.q, (mx, my, mz))          # mag in world
                bx, bz = math.hypot(hx, hy), hz                   # reference: north + down
                wx, wy, wz = qrot(qconj(self.q), (bx, 0.0, bz))   # back to body
                cx = my * wz - mz * wy
                cy = mz * wx - mx * wz
                cz = mx * wy - my * wx
                # only correct heading: project the error on the body "up" axis
                d = (cx * vx + cy * vy + cz * vz) * (self.KP_MAG / self.KP)   # boosted at startup too
                ex += d * vx
                ey += d * vy
                ez += d * vz

        gx, gy, gz = g
        if self.t_run >= self.STARTUP_S:
            self.ei[0] += self.KI * ex * dt
            self.ei[1] += self.KI * ey * dt
            self.ei[2] += self.KI * ez * dt
        gx += kp * ex + self.ei[0]
        gy += kp * ey + self.ei[1]
        gz += kp * ez + self.ei[2]

        h = 0.5 * dt
        self.q = qnormalize((q0 + h * (-q1 * gx - q2 * gy - q3 * gz),
                             q1 + h * (q0 * gx + q2 * gz - q3 * gy),
                             q2 + h * (q0 * gy - q1 * gz + q3 * gx),
                             q3 + h * (q0 * gz + q1 * gy - q2 * gx)))


# ---------------------------------------------------------------------------
# Sensor state, fed by the reader thread
# ---------------------------------------------------------------------------


class State:
    BIAS_CAL_S = 2.0
    BIAS_STILL_DPS = 1.5      # max per-axis std during bias calibration

    def __init__(self, use_mag):
        self.lock = threading.Lock()
        self.ahrs = Mahony()
        self.connected = False
        self.port = None
        self.status = None
        self.gyro_scale = 2000 / 32768.0
        self.acc_scale = 4 / 32768.0
        self.T = 1 / 1125.0       # sample period estimate (actual ODR differs from nominal)
        self.last_seq = None
        self.last_t = None
        self.seq_gaps = 0
        self.rate = 0.0
        self._rate_n, self._rate_t0 = 0, time.monotonic()
        self.bias = [0.0, 0.0, 0.0]
        self.bias_ok = False
        self.start_bias_cal()
        self.mag_raw = None
        self.mag_new_cnt = 0
        self._mag_n0 = 0
        self.mag_rate = 0.0
        self.mag_corr = None       # calibrated mag, body frame, uT
        self.mag_state = "no data from device"
        self.mag_cal = self.load_cal()
        self.use_mag = use_mag and self.mag_cal is not None
        self.mag_collect = None    # [mins, maxs, n] while calibrating
        self.yaw_offset = (1.0, 0.0, 0.0, 0.0)
        self.target = None
        self.last_acc = (0.0, 0.0, 1.0)
        self.last_gyro = (0.0, 0.0, 0.0)
        self.message = ""

    # -- calibration -------------------------------------------------------
    def start_bias_cal(self):
        self.bias_acc = [0.0] * 3
        self.bias_sq = [0.0] * 3
        self.bias_n = 0
        self.bias_t = 0.0
        self.bias_active = True

    @staticmethod
    def load_cal():
        try:
            with open(CAL_PATH) as f:
                c = json.load(f)
            return {"offset": c["offset"], "scale": c["scale"], "radius": c["radius"]}
        except (OSError, ValueError, KeyError):
            return None

    def toggle_mag_cal(self):
        if self.mag_collect is None:
            self.mag_collect = [[1e9] * 3, [-1e9] * 3, 0]
            self.message = "mag cal: rotate the sensor slowly through ALL directions, then press c"
            return
        mins, maxs, n = self.mag_collect
        self.mag_collect = None
        span = [maxs[i] - mins[i] for i in range(3)]
        if n < 200 or min(span) < 20.0:
            self.message = "mag cal rejected: not enough rotation (span %s uT)" % ["%.0f" % s for s in span]
            return
        offset = [(maxs[i] + mins[i]) / 2 for i in range(3)]
        radii = [s / 2 for s in span]
        r = sum(radii) / 3
        cal = {"offset": offset, "scale": [r / ri for ri in radii], "radius": r}
        os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)
        with open(CAL_PATH, "w") as f:
            json.dump(cal, f, indent=1)
        self.mag_cal = cal
        self.use_mag = True
        self.message = "mag cal saved, mag ON: offset %s uT, field %.1f uT" % (["%.1f" % o for o in offset], r)

    # -- per-sample processing (reader thread) -------------------------------
    def on_status(self, st):
        with self.lock:
            self.status = st
            self.gyro_scale = st["gyro_fs"] / 32768.0
            self.acc_scale = st["accel_fs"] / 32768.0
            if self.last_t is None and st["odr_mhz"]:
                self.T = 1000.0 / st["odr_mhz"]

    def on_imu(self, f):
        t_us, seq = f[0], f[1]
        acc = tuple(v * self.acc_scale for v in f[2:5])
        gyr_dps = tuple(v * self.gyro_scale for v in f[5:8])
        with self.lock:
            # dt from seq * estimated period: immune to the ~+-200 us detection jitter
            nseq = 1
            if self.last_seq is not None:
                nseq = (seq - self.last_seq) & 0xFFFF
                if nseq != 1:
                    self.seq_gaps += 1
                if 1 <= nseq <= 50:
                    dt_meas = ((t_us - self.last_t) & 0xFFFFFFFF) * 1e-6 / nseq
                    if 0.5 * self.T < dt_meas < 2 * self.T:
                        self.T += 0.002 * (dt_meas - self.T)
                else:
                    nseq = 1
            first = self.last_seq is None
            self.last_seq, self.last_t = seq, t_us
            dt = 0.0 if first else self.T * nseq

            if self.bias_active:
                self.bias_n += 1
                self.bias_t += dt
                for i in range(3):
                    self.bias_acc[i] += gyr_dps[i]
                    self.bias_sq[i] += gyr_dps[i] ** 2
                if self.bias_t >= self.BIAS_CAL_S:
                    n = self.bias_n
                    mean = [s / n for s in self.bias_acc]
                    std = [math.sqrt(max(q / n - m * m, 0)) for q, m in zip(self.bias_sq, mean)]
                    if max(std) < self.BIAS_STILL_DPS:
                        if not self.bias_ok and self.ahrs.q:   # first cal: start with heading 0
                            yaw = q_euler(self.ahrs.q)[0] * DEG
                            self.yaw_offset = q_axis_angle((0, 0, 1), -yaw)
                        self.bias = mean
                        self.bias_ok = True
                        self.bias_active = False
                        self.message = "gyro bias %+.3f %+.3f %+.3f dps" % tuple(mean)
                    else:
                        self.message = "keep the sensor still for gyro bias calibration..."
                        self.start_bias_cal()

            if f[12] & 1:                                 # new mag sample
                mx, my, mz = f[9:12]
                m = (mx * MAG_UT_PER_LSB, -my * MAG_UT_PER_LSB, -mz * MAG_UT_PER_LSB)
                self.mag_raw = m
                self.mag_new_cnt += 1
                if self.mag_collect is not None:
                    mins, maxs, _ = self.mag_collect
                    for i in range(3):
                        mins[i] = min(mins[i], m[i])
                        maxs[i] = max(maxs[i], m[i])
                    self.mag_collect[2] += 1

            mag = None
            self.mag_corr = None
            if self.mag_cal and self.mag_raw:
                c = self.mag_cal
                self.mag_corr = tuple((self.mag_raw[i] - c["offset"][i]) * c["scale"][i] for i in range(3))
            if self.mag_collect is not None:
                self.mag_state = "calibrating"
            elif not self.mag_raw:
                self.mag_state = "no data from device"
            elif not self.mag_cal:
                self.mag_state = "not used: no calibration (press c)"
            elif not self.use_mag:
                self.mag_state = "not used: off (press m)"
            else:
                mn = math.sqrt(sum(v * v for v in self.mag_corr))
                if 0.6 * self.mag_cal["radius"] < mn < 1.4 * self.mag_cal["radius"]:
                    mag = self.mag_corr
                    self.mag_state = "USED for heading"
                else:                                     # local disturbance, don't trust it
                    self.mag_state = "IGNORED: |B| %.0f uT vs calibrated %.0f" % (mn, self.mag_cal["radius"])

            g = tuple((gyr_dps[i] - self.bias[i]) * DEG for i in range(3))
            self.ahrs.update(g, acc, mag, dt)
            self.last_acc, self.last_gyro = acc, tuple(gyr_dps[i] - self.bias[i] for i in range(3))

            self._rate_n += 1
            now = time.monotonic()
            if now - self._rate_t0 >= 1.0:
                self.rate = self._rate_n / (now - self._rate_t0)
                self.mag_rate = (self.mag_new_cnt - self._mag_n0) / (now - self._rate_t0)
                self._mag_n0 = self.mag_new_cnt
                self._rate_n, self._rate_t0 = 0, now

    # -- GUI-side helpers --------------------------------------------------
    def snapshot(self):
        with self.lock:
            q = self.ahrs.q
            return {
                "q": qmul(self.yaw_offset, q) if q else None,
                "target": qmul(self.yaw_offset, self.target) if self.target else None,
                "q_raw": q,
            }

    def zero_yaw(self):
        with self.lock:
            if self.ahrs.q:
                yaw = q_euler(self.ahrs.q)[0] * DEG
                self.yaw_offset = q_axis_angle((0, 0, 1), -yaw)

    def save_target(self):
        with self.lock:
            self.target = self.ahrs.q

    def reset(self):
        with self.lock:
            self.ahrs.reset()
            self.yaw_offset = (1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Reader thread (reconnects automatically)
# ---------------------------------------------------------------------------


def discover(port_arg):
    if port_arg:
        return port_arg if os.path.exists(port_arg) else None
    hits = sorted(glob.glob("/dev/serial/by-id/*USB_Motion_Sensor*"))
    return hits[0] if hits else None


STATUS_KEYS = ("proto", "fw_major", "fw_minor", "flags", "odr_mhz", "gyro_fs", "accel_fs", "addr",
               "uptime_ms", "samples", "usb_dropped", "i2c_errors", "overruns", "mag_diag", "mag_wia")


def reader(state, port_arg, stop):
    imu_len, st_len = struct.calcsize(IMU_FMT), struct.calcsize(STATUS_FMT)
    while not stop.is_set():
        port = discover(port_arg)
        if not port:
            state.connected = False
            time.sleep(0.5)
            continue
        try:
            fd = open_port(port)
        except OSError:
            time.sleep(0.5)
            continue
        state.port, state.connected = port, True
        with state.lock:
            state.last_seq = state.last_t = None
        parser = Parser()
        try:
            os.write(fd, b"i")
            while not stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.2)
                if not r:
                    continue
                try:
                    data = os.read(fd, 8192)
                except BlockingIOError:
                    continue
                if not data:
                    break
                for typ, pl in parser.feed(data):
                    if typ == PKT_IMU and len(pl) == imu_len:
                        state.on_imu(struct.unpack(IMU_FMT, pl))
                    elif typ == PKT_STATUS and len(pl) == st_len:
                        state.on_status(dict(zip(STATUS_KEYS, struct.unpack(STATUS_FMT, pl))))
        except OSError:
            pass
        finally:
            os.close(fd)
            state.connected = False


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

# Board model in sensor axes (units ~ cm). Adafruit 4554 is 25.4 x 17.8 mm.
BX, BY, BZ = 1.27, 0.89, 0.08
BOX_V = [(sx * BX, sy * BY, sz * BZ) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
# faces: vertex indices (CCW seen from outside), outward normal, colour
BOX_F = [
    ((1, 3, 7, 5), (0, 0, 1), "#1f5fae"),    # +Z top (blue PCB)
    ((0, 4, 6, 2), (0, 0, -1), "#2a3550"),   # -Z bottom
    ((4, 5, 7, 6), (1, 0, 0), "#c0392b"),    # +X edge (red)
    ((0, 2, 3, 1), (-1, 0, 0), "#3a4a6a"),
    ((2, 6, 7, 3), (0, 1, 0), "#27ae60"),    # +Y edge (green)
    ((0, 1, 5, 4), (0, -1, 0), "#3a4a6a"),
]
BOX_E = [(a, b) for a in range(8) for b in range(a + 1, 8)
         if sum(1 for i in range(3) if (BOX_V[a][i] != BOX_V[b][i])) == 1]
CHIP = [(-0.25, -0.25, BZ), (0.25, -0.25, BZ), (0.25, 0.25, BZ), (-0.25, 0.25, BZ)]
AXES = [((1, 0, 0), "#ff5555", "X"), ((0, 1, 0), "#55dd55", "Y"), ((0, 0, 1), "#5599ff", "Z")]


def shade(hexcol, k):
    r, g, b = (int(hexcol[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % tuple(min(255, int(c * k)) for c in (r, g, b))


class App:
    def __init__(self, root, state):
        self.root, self.state = root, state
        root.title("USB Motion Sensor — orientation")
        self.cv = tk.Canvas(root, width=960, height=680, bg="#14161c", highlightthickness=0)
        self.cv.pack(fill="both", expand=True)
        self.az, self.el, self.dist = -55 * DEG, 25 * DEG, 7.0
        self._drag = None
        self.cv.bind("<ButtonPress-1>", lambda e: setattr(self, "_drag", (e.x, e.y)))
        self.cv.bind("<B1-Motion>", self.on_drag)
        self.cv.bind("<MouseWheel>", lambda e: self.zoom(-e.delta))
        self.cv.bind("<Button-4>", lambda e: self.zoom(-1))
        self.cv.bind("<Button-5>", lambda e: self.zoom(1))
        for key, fn in {"b": self.k_bias, "m": self.k_mag, "c": state.toggle_mag_cal, "z": state.zero_yaw,
                        "s": state.save_target, "x": self.k_clear, "r": state.reset,
                        "q": root.destroy, "Escape": root.destroy}.items():
            root.bind("<%s>" % key if len(key) > 1 else key, lambda e, f=fn: f())
        self.tick()

    # -- input -------------------------------------------------------------
    def on_drag(self, e):
        if self._drag:
            dx, dy = e.x - self._drag[0], e.y - self._drag[1]
            self.az -= dx * 0.01
            self.el = max(-85 * DEG, min(85 * DEG, self.el + dy * 0.01))
        self._drag = (e.x, e.y)

    def zoom(self, d):
        self.dist = max(3.0, min(20.0, self.dist * (1.1 if d > 0 else 1 / 1.1)))

    def k_bias(self):
        with self.state.lock:
            self.state.start_bias_cal()
            self.state.message = "gyro bias: keep still..."

    def k_mag(self):
        s = self.state
        if s.mag_cal is None:
            s.message = "no mag calibration yet: press c and rotate the sensor"
            return
        s.use_mag = not s.use_mag
        s.message = "magnetometer " + ("ON (absolute heading)" if s.use_mag else "OFF (6-axis, yaw drifts)")

    def k_clear(self):
        self.state.target = None

    # -- camera ------------------------------------------------------------
    def setup_camera(self):
        ce, se, ca, sa = math.cos(self.el), math.sin(self.el), math.cos(self.az), math.sin(self.az)
        self.cam = (self.dist * ce * ca, self.dist * ce * sa, self.dist * se)
        f = (-ce * ca, -ce * sa, -se)
        r = (f[1], -f[0], 0.0)                      # f x z
        rn = math.hypot(r[0], r[1]) or 1.0
        r = (r[0] / rn, r[1] / rn, 0.0)
        u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
        self.basis = (r, u, f)
        w, h = self.cv.winfo_width(), self.cv.winfo_height()
        self.cx, self.cy, self.fov = w / 2, h / 2 + 30, min(w, h) * 1.1

    def proj(self, p):
        r, u, f = self.basis
        d = (p[0] - self.cam[0], p[1] - self.cam[1], p[2] - self.cam[2])
        z = d[0] * f[0] + d[1] * f[1] + d[2] * f[2]
        z = max(z, 0.1)
        x = d[0] * r[0] + d[1] * r[1] + d[2] * r[2]
        y = d[0] * u[0] + d[1] * u[1] + d[2] * u[2]
        return self.cx + self.fov * x / z, self.cy - self.fov * y / z, z

    # -- drawing -----------------------------------------------------------
    def draw_floor(self, use_mag):
        c = self.cv
        z0 = -1.6
        for i in range(-3, 4):
            for a, b in (((i, -3, z0), (i, 3, z0)), ((-3, i, z0), (3, i, z0))):
                pa, pb = self.proj(a), self.proj(b)
                c.create_line(pa[0], pa[1], pb[0], pb[1], fill="#262a35")
        for v, col, lab in (((3.4, 0, z0), "#aa5555", "N (+X)" if use_mag else "+X ref"),
                            ((0, 3.4, z0), "#55aa55", "W (+Y)" if use_mag else "+Y ref")):
            o, p = self.proj((0, 0, z0)), self.proj(v)
            c.create_line(o[0], o[1], p[0], p[1], fill=col, width=2, arrow="last")
            c.create_text(p[0], p[1] - 10, text=lab, fill=col, font=("Sans", 10))

    def draw_board(self, q, ghost=False):
        c = self.cv
        wv = [qrot(q, v) for v in BOX_V]
        pv = [self.proj(v) for v in wv]
        if ghost:
            for a, b in BOX_E:
                c.create_line(pv[a][0], pv[a][1], pv[b][0], pv[b][1], fill="#e0b040", dash=(4, 3))
            p1, p2 = self.proj((0, 0, 0)), self.proj(qrot(q, (1.9, 0, 0)))
            c.create_line(p1[0], p1[1], p2[0], p2[1], fill="#e0b040", dash=(4, 3), arrow="last")
            return
        center_depth = self.proj((0, 0, 0))[2]
        light = qnormalize((0, 0.3, 0.5, 0.8))[1:]
        tips = [(qrot(q, tuple(1.9 * a for a in ax)), col, lab) for ax, col, lab in AXES]

        def draw_axis(tip, col, lab):
            o, p = self.proj((0, 0, 0)), self.proj(tip)
            c.create_line(o[0], o[1], p[0], p[1], fill=col, width=3, arrow="last", arrowshape=(12, 14, 5))
            c.create_text(p[0] + 10, p[1] - 10, text=lab, fill=col, font=("Sans", 12, "bold"))

        behind = [t for t in tips if self.proj(t[0])[2] > center_depth]
        front = [t for t in tips if self.proj(t[0])[2] <= center_depth]
        for t in behind:
            draw_axis(*t)
        for idx, n, col in BOX_F:
            nw = qrot(q, n)
            cen = [sum(wv[i][k] for i in idx) / 4 for k in range(3)]
            view = [cen[k] - self.cam[k] for k in range(3)]
            if sum(nw[k] * view[k] for k in range(3)) >= 0:
                continue                                    # back face
            k = 0.45 + 0.65 * max(0.0, sum(nw[i] * light[i] for i in range(3)))
            pts = [coord for i in idx for coord in pv[i][:2]]
            c.create_polygon(pts, fill=shade(col, k), outline="#0b0d12")
            if n == (0, 0, 1):                              # ICM chip on top
                cp = [coord for v in CHIP for coord in self.proj(qrot(q, v))[:2]]
                c.create_polygon(cp, fill="#111111", outline="#555555")
        for t in front:
            draw_axis(*t)

    def draw_hud(self, snap):
        s = self.state
        L = []
        if not s.connected:
            L.append("waiting for device (/dev/serial/by-id/*USB_Motion_Sensor*)...")
        else:
            st = s.status or {}
            L.append("%s   rx %.0f Hz   ODR est %.1f Hz   seq gaps %d" % (
                os.path.basename(s.port or ""), s.rate, 1 / s.T, s.seq_gaps))
            if st:
                L.append("device: usb_dropped %d  i2c_err %d  overruns %d  mag %s" % (
                    st["usb_dropped"], st["i2c_errors"], st["overruns"], "ok" if st["flags"] & 2 else "MISSING"))
        q = snap["q"]
        if q:
            yaw, pitch, roll = q_euler(q)
            L.append("yaw %+7.1f°   pitch %+6.1f°   roll %+7.1f°" % (yaw, pitch, roll))
        g = s.last_gyro
        L.append("gyro %+7.1f %+7.1f %+7.1f °/s   |a| %.3f g" % (*g, math.sqrt(sum(a * a for a in s.last_acc))))
        bias = "calibrating (keep still)..." if s.bias_active else ("%+.2f %+.2f %+.2f °/s" % tuple(s.bias))
        L.append("gyro bias: " + bias)
        if s.mag_collect is not None:
            mins, maxs, n = s.mag_collect
            L.append("MAG CAL: %d samples, span x %.0f y %.0f z %.0f uT — press c to finish" % (
                n, *(max(0, maxs[i] - mins[i]) for i in range(3))))
        mr = s.mag_raw
        if mr:
            L.append("mag raw  %+7.1f %+7.1f %+7.1f uT  |B| %6.1f   %3.0f Hz" % (
                *mr, math.sqrt(sum(v * v for v in mr)), s.mag_rate))
        mc = s.mag_corr
        if mc:
            L.append("mag cal  %+7.1f %+7.1f %+7.1f uT  |B| %6.1f" % (*mc, math.sqrt(sum(v * v for v in mc))))
        L.append("mag: " + s.mag_state)
        if s.mag_collect is None:
            L.append("heading: " + ("magnetometer (absolute)" if s.use_mag and s.mag_cal else
                                    "gyro only — drifts; press z to zero"))
        self.cv.create_text(14, 12, anchor="nw", text="\n".join(L), fill="#d0d4dc", font=("Monospace", 11))

        if snap["target"] and q:
            ang = q_angle_between(snap["q_raw"], s.target) / DEG
            col = "#55dd55" if ang < 2 else "#e0b040" if ang < 10 else "#ff7755"
            self.cv.create_text(self.cv.winfo_width() - 14, 12, anchor="ne", fill=col,
                                font=("Sans", 22, "bold"), text="Δ %.1f°" % ang)
            self.cv.create_text(self.cv.winfo_width() - 14, 48, anchor="ne", fill="#8a8f99",
                                text="to saved pose (dashed)", font=("Sans", 10))
        if s.message:
            self.cv.create_text(14, self.cv.winfo_height() - 34, anchor="sw", text=s.message, fill="#e0b040",
                                font=("Sans", 11))
        self.cv.create_text(14, self.cv.winfo_height() - 10, anchor="sw", fill="#6a6f79",
                            font=("Sans", 9),
                            text="b bias · m mag · c mag cal · z zero yaw · s save pose · x clear · r reset · "
                                 "drag/wheel camera · q quit")

    def draw_field(self, q):
        """Magnetic field direction as a purple arrow (calibrated if available, else raw)."""
        s = self.state
        m = s.mag_corr or s.mag_raw
        if not m:
            return
        n = math.sqrt(sum(v * v for v in m)) or 1.0
        tip = qrot(q, tuple(2.2 * v / n for v in m))
        o, p = self.proj((0, 0, 0)), self.proj(tip)
        col = "#c070ff" if s.mag_corr else "#806090"
        self.cv.create_line(o[0], o[1], p[0], p[1], fill=col, width=2, dash=(6, 3), arrow="last",
                            arrowshape=(10, 12, 4))
        self.cv.create_text(p[0] + 10, p[1] + 10, text="B" if s.mag_corr else "B raw", fill=col,
                            font=("Sans", 11, "bold"))

    def tick(self):
        self.cv.delete("all")
        self.setup_camera()
        snap = self.state.snapshot()
        self.draw_floor(self.state.use_mag)
        if snap["target"]:
            self.draw_board(snap["target"], ghost=True)
        if snap["q"]:
            self.draw_board(snap["q"])
            self.draw_field(snap["q"])
        self.draw_hud(snap)
        self.root.after(16, self.tick)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (default: auto)")
    ap.add_argument("--no-mag", action="store_true", help="start with the magnetometer disabled")
    ap.add_argument("--snapshot", metavar="FILE.ps", help=argparse.SUPPRESS)  # testing: dump canvas, exit
    ap.add_argument("--snapshot-after", type=float, default=4.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    state = State(use_mag=not args.no_mag)
    stop = threading.Event()
    threading.Thread(target=reader, args=(state, args.port, stop), daemon=True).start()
    root = tk.Tk()
    app = App(root, state)
    if args.snapshot:
        def fake_target():                     # testing: a target 30 deg off in yaw and 20 in roll
            state.save_target()
            state.target = qmul(q_axis_angle((0, 0, 1), 0.5), qmul(state.target, q_axis_angle((1, 0, 0), 0.35)))

        def snap():
            app.cv.postscript(file=args.snapshot, colormode="color")
            root.destroy()
        if os.environ.get("IMU3D_TARGET"):
            root.after(int(args.snapshot_after * 1000) - 500, fake_target)
        root.after(int(args.snapshot_after * 1000), snap)
    try:
        root.mainloop()
    finally:
        stop.set()


if __name__ == "__main__":
    main()
