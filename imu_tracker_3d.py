#!/usr/bin/env python3
"""BarTone 3D tracker: where is the M5StickC Plus in 3D space?

Reads raw IMU data from the stick (BarTone_IMUStream sketch) and shows a live
3D view of where the stick is and which way it's pointing, using dead reckoning.

How it works (the method from github.com/xioTechnologies/Gait-Tracking):
  1. Fusion AHRS (the imufusion library) tracks the stick's orientation from
     the gyroscope, so the accelerometer readings can be turned into room
     directions and gravity can be subtracted.
  2. What's left is the stick's own acceleration. Adding it up gives velocity,
     and adding velocity up gives position.
  3. On its own that drifts away very quickly. But whenever the stick is held
     still, we know its velocity is really 0. So each movement (from one still
     moment to the next) is corrected afterwards: any velocity left over at the
     end is drift, and it's removed, spread evenly over the movement. Then the
     tilt is re-levelled from gravity too.

So: move the stick, then hold it still for a moment. While it's moving you see
a live estimate (orange). When it stops, that movement is corrected and added
to the blue trail. Short moves with a stop at the end are the most accurate;
long moves without stopping drift more.

Room axes: "forward" is the way the top of the stick (the end away from the
USB port) pointed, level with the floor, when it calibrated. "Up" is up.
There's no compass in the stick, so this is relative, not north/south.

Setup (once):
    pip3 install imufusion pyserial matplotlib numpy

Live (close the Arduino Serial Monitor / Chrome / bar_tracker.py first):
    python3 imu_tracker_3d.py
  Put the stick down still for a second to calibrate, then move it around.
  Keys in the window: z = re-zero (start again from here), c = clear the trail,
  scroll or +/- = zoom, f = fit the whole trail. The view follows the stick.
  Button A on the stick also re-zeros.
  Everything received is saved in recordings/ (next to this script).

Replay a saved recording:
    python3 imu_tracker_3d.py --replay recordings/imu3d_raw_20261107_170000.csv
    python3 imu_tracker_3d.py --replay FILE --realtime         # play it back live
    python3 imu_tracker_3d.py --replay FILE --save path.png --no-gui
"""

import argparse
import glob
import math
import os
import queue
import sys
import threading
import time
import warnings
from collections import deque

import imufusion
import numpy as np

G = 9.80665       # 1 g in m/s^2
RATE = 100.0      # samples per second sent by the stick

# ---- Calibration: the stick sits still for a second at the start ----
CAL_S = 1.0
CAL_GYRO_STD = 1.5    # deg/s, maximum wobble allowed while calibrating
CAL_ACC_STD = 0.02    # g

# ---- When is the stick "still"? ----
ACC_MOVING = 0.35     # m/s^2, acceleration (gravity removed) above this = moving
GYRO_MOVING = 8.0     # deg/s, rotating faster than this = moving
STILL_MAX_V = 0.12    # m/s, the (uncorrected) speed must also be below this. Halfway
                      # through a smooth move the stick is fast but hardly
                      # accelerating, which would otherwise look still.
DRIFT_ALLOW = 0.02    # m/s per second: that speed limit grows the longer a move lasts,
                      # because the uncorrected speed drifts more over time
STILL_S = 0.25        # must look still for this long
# Backup check that doesn't rely on the orientation or speed being right. It
# looks at the last STILL_WIN_S as a whole, so a hand that's holding the stick
# (and shakes a little) still counts as still: one shaky reading doesn't reset it.
STILL_WIN_S = 0.4      # length of that window
STILL_GYRO_MEAN = 8.0  # deg/s, average rotation in the window must be below this
STILL_GYRO_PEAK = 20.0 # deg/s, and no reading above this
STILL_ACC_STD = 0.015  # g, total acceleration barely changing...
STILL_ACC_G = 0.04     # g, ...and on average within this of 1 g (only gravity)
LEAD_S = 0.10         # a move is taken to start this much before it was noticed
PREVIEW_RAW_S = 2.0   # live preview: plain integration for this long, then filtered (see _live_step)
PREVIEW_HP_HZ = 0.3   # high-pass cut-off for the filtered part of the live preview

LONG_MOVE_S = 5.0     # moves longer than this without a stop get a "probably drifted" warning

TRAIL_S = 120         # s of history shown
AXIS_LEN = 0.08       # m, length of the stick's axes drawn in the 3D view
VIEW_SIZE = 1.0       # m, width of the 3D view. It follows the stick instead of zooming out.
VIEW_EDGE = 0.2       # the view slides along when the stick gets this close to an edge (fraction of the width)

# Columns of the history table
T, DT, AX, AY, AZ, PX, PY, PZ = range(8)


def quat_from_accel(a):
    """Orientation (w, x, y, z) that turns the measured "up" into +Z, from gravity alone."""
    ax, ay, az = a / np.linalg.norm(a)
    if az < -0.999:
        return np.array([0.0, 1.0, 0.0, 0.0])
    q = np.array([1 + az, ay, -ax, 0.0])
    return q / np.linalg.norm(q)


def matrix_to_quat(R):
    """Rotation matrix -> quaternion (w, x, y, z)."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


class History:
    """A growable table of samples (one row per sample)."""

    def __init__(self, cols):
        self.a = np.zeros((4096, cols))
        self.n = 0

    def append(self, row):
        if self.n == len(self.a):
            self.a = np.concatenate([self.a, np.zeros_like(self.a)])
        self.a[self.n] = row
        self.n += 1


class Tracker3D:
    """All the maths. Feed it samples with add(); it works the same live or from a file."""

    def __init__(self, rate=RATE):
        self.rate = rate
        self.state = "calibrating"   # then "still" or "moving"
        self.cal = deque(maxlen=int(CAL_S * rate))
        self.recent_acc = deque(maxlen=int(STILL_S * rate))
        self.win_gyro = deque(maxlen=int(STILL_WIN_S * rate))  # for the backup still check
        self.win_acc = deque(maxlen=int(STILL_WIN_S * rate))
        self.t_prev_us = None
        self.t = 0.0
        self.h = History(8)
        self.done = -1               # positions are final (corrected) up to this row
        self.last_move_end = -1
        self.move_start = 0
        self.quiet = 0.0
        self.v_live = np.zeros(3)    # live (uncorrected) velocity while moving
        self.v_hp, self.p_int, self.p_hp, self.p_start, self.p_raw = (np.zeros(3) for _ in range(5))  # live preview
        self.live_t, self.handover = 0.0, None
        self.p_live = np.zeros(3)
        self.live_path = []
        self.R = np.eye(3)           # orientation: stick axes -> room axes
        self.F = np.eye(3)           # room axes -> display axes (right, forward, up)
        self.origin = np.zeros(3)    # subtracted from positions (z key)
        self.trail_start = 0
        self.moves = 0

    # ---------- Input ----------

    def add(self, t_us, gyro_dps, acc_g):
        gyro = np.asarray(gyro_dps, float)
        acc = np.asarray(acc_g, float)
        dt = 1.0 / self.rate if self.t_prev_us is None else ((t_us - self.t_prev_us) & 0xFFFFFFFF) / 1e6
        self.t_prev_us = t_us
        if dt <= 0 or dt > 0.5:
            dt = 1.0 / self.rate  # missing data: carry on as best we can

        if self.state == "calibrating":
            self._calibrate(gyro, acc)
            return

        self.t += dt
        gyro = self.bias.update(gyro - self.gyro_bias)  # Fusion keeps fine-tuning the bias when still
        acc = acc * self.acc_scale

        # Orientation. The accelerometer is only used to correct the tilt while
        # the stick is still; while moving, its own acceleration would be
        # mistaken for gravity. Giving Fusion zeros makes it use the gyro alone.
        self.ahrs.set_sample_period(dt)
        self.ahrs.update_no_magnetometer(gyro, acc if self.state == "still" else np.zeros(3))
        self.R = imufusion.quaternion_to_matrix(self.ahrs.get_quaternion())
        # Acceleration in room axes, minus gravity, in m/s^2
        a = self.F @ ((self.R @ acc - np.array([0, 0, 1.0])) * G)

        moving_now = np.linalg.norm(a) > ACC_MOVING or np.linalg.norm(gyro) > GYRO_MOVING
        self.recent_acc.append(acc)
        self.win_gyro.append(np.linalg.norm(gyro))
        self.win_acc.append(np.linalg.norm(acc))

        i = self.h.n
        self.h.append([self.t, dt, *a, 0, 0, 0])
        H = self.h.a

        if self.state == "still":
            if moving_now:
                # A move started. It probably began a moment before we noticed.
                self.state = "moving"
                self.quiet = 0.0
                self.move_start = max(i - int(LEAD_S * self.rate), self.last_move_end + 1, 0)
                self.done = self.move_start - 1
                self.p_start = H[self.done, PX:PZ + 1].copy() if self.done >= 0 else np.zeros(3)
                self.p_live = self.p_start.copy()
                self.v_live = np.zeros(3)
                self.v_hp = np.zeros(3)
                self.p_int = np.zeros(3)
                self.p_hp = np.zeros(3)
                self.p_raw = np.zeros(3)
                self.live_t = 0.0
                self.handover = None
                self.live_path = []
                for k in range(self.move_start, i + 1):
                    self._live_step(H[k])
            else:
                if i > 0:
                    H[i, PX:PZ + 1] = H[i - 1, PX:PZ + 1]  # still: position doesn't change
                self.done = i
        else:  # moving
            self._live_step(H[i])
            moving_for = H[i, T] - H[self.move_start, T]
            slow = np.linalg.norm(self.v_live) < STILL_MAX_V + DRIFT_ALLOW * moving_for
            self.quiet = self.quiet + dt if (not moving_now and slow) else 0.0
            if self.quiet >= STILL_S or self._window_still(i):
                self._finish_move(self.move_start, i)
                self._level_from_gravity()
                self.state = "still"

    def _window_still(self, i):
        """Backup still check over the last STILL_WIN_S, all of it since the move began."""
        n = self.win_gyro.maxlen
        if i - self.move_start < n or len(self.win_gyro) < n:
            return False
        g, a = np.array(self.win_gyro), np.array(self.win_acc)
        return (g.mean() < STILL_GYRO_MEAN and g.max() < STILL_GYRO_PEAK
                and a.std() < STILL_ACC_STD and abs(a.mean() - 1) < STILL_ACC_G)

    def _live_step(self, row):
        dt = row[DT]
        v_before = self.v_live.copy()
        self.v_live += row[AX:AZ + 1] * dt  # uncorrected velocity (used to decide when it's stopped)

        # ---- Live preview for the screen only ----
        # Plain integration is accurate for the first couple of seconds, but then
        # "falls" away: any small error in the acceleration keeps adding speed.
        # So after PREVIEW_RAW_S we switch to Oscillatory-Motion-Tracking's
        # approach (high-pass filter the velocity, integrate, high-pass filter the
        # position). That bleeds the drift away, so the dot still follows the
        # stick's movements but can't run off (it also shrinks slow movements,
        # which is why it isn't used from the start). When the stick stops, the
        # properly corrected move replaces the preview.
        self.p_raw += self.v_live * dt
        tau = 1 / (2 * math.pi * PREVIEW_HP_HZ)
        alpha = tau / (tau + dt)
        self.v_hp = alpha * (self.v_hp + self.v_live - v_before)
        p_int_before = self.p_int.copy()
        self.p_int += self.v_hp * dt
        self.p_hp = alpha * (self.p_hp + self.p_int - p_int_before)
        self.live_t += dt
        if self.live_t <= PREVIEW_RAW_S:
            self.p_live = self.p_start + self.p_raw
        else:
            if self.handover is None:  # carry on from where plain integration got to
                self.handover = self.p_raw - self.p_hp
            self.p_live = self.p_start + self.handover + self.p_hp
        self.live_path.append(self.p_live.copy())

    # ---------- Calibration ----------

    def _calibrate(self, gyro, acc):
        self.cal.append(np.concatenate([gyro, acc]))
        if len(self.cal) < self.cal.maxlen:
            return
        c = np.array(self.cal)
        if (c[:, :3].std(axis=0) > CAL_GYRO_STD).any() or (c[:, 3:].std(axis=0) > CAL_ACC_STD).any():
            return  # moved: keep waiting for a still second
        self.gyro_bias = c[:, :3].mean(axis=0)
        m = c[:, 3:].mean(axis=0)
        self.acc_scale = 1.0 / np.linalg.norm(m)

        s = imufusion.AhrsSettings()
        s.convention = imufusion.CONVENTION_NWU
        s.gain = 0.5
        s.gyroscope_range = 2000
        s.acceleration_rejection = 10
        s.magnetic_rejection = 0
        s.rejection_timeout = int(10 * self.rate)
        s.sample_rate = self.rate
        self.ahrs = imufusion.Ahrs()
        self.ahrs.set_settings(s)
        self.ahrs.skip_startup()
        q = quat_from_accel(m)
        self.ahrs.set_quaternion(q)
        self.R = imufusion.quaternion_to_matrix(q)

        bs = imufusion.BiasSettings()
        bs.sample_rate = self.rate
        bs.stationary_threshold = 3
        bs.stationary_period = 3
        self.bias = imufusion.Bias()
        self.bias.set_settings(bs)

        # Display axes: forward = where the top of the stick (+Y) points, made
        # level. If the stick is standing upright, use its +X instead.
        fwd = self.R @ np.array([0, 1.0, 0])
        if math.hypot(fwd[0], fwd[1]) < 0.3:
            fwd = self.R @ np.array([1.0, 0, 0])
        fwd = np.array([fwd[0], fwd[1], 0.0]) / math.hypot(fwd[0], fwd[1])
        up = np.array([0, 0, 1.0])
        right = np.cross(fwd, up)
        self.F = np.vstack([right, fwd, up])

        self.state = "still"
        print(f"Calibrated (gyro bias {np.round(self.gyro_bias, 2)} deg/s). Move the stick, then hold it still.")

    # ---------- Correcting a finished move (Gait-Tracking) ----------

    def _finish_move(self, s, e):
        H = self.h.a
        dt = H[s:e + 1, DT]
        v = np.cumsum(H[s:e + 1, AX:AZ + 1] * dt[:, None], axis=0)  # velocity, with drift
        t = np.cumsum(dt)
        # The stick was still at both ends, so velocity must be 0 there. Whatever
        # is left is drift: remove it, assuming it built up evenly over time.
        frac = (t - t[0]) / (t[-1] - t[0]) if t[-1] > t[0] else np.zeros_like(t)
        v = v - v[0] - (v[-1] - v[0]) * frac[:, None]
        p0 = H[s - 1, PX:PZ + 1] if s > 0 else np.zeros(3)
        H[s:e + 1, PX:PZ + 1] = p0 + np.cumsum(v * dt[:, None], axis=0)
        self.done = self.last_move_end = e
        self.moves += 1

        moved = H[e, PX:PZ + 1] - p0
        here = H[e, PX:PZ + 1] - self.origin
        if np.linalg.norm(moved) > 0.02:  # don't report tiny nudges
            print(f"Moved {np.linalg.norm(moved) * 100:5.1f} cm in {t[-1]:4.1f} s  ->  now at "
                  f"right {here[0] * 100:+6.1f}, forward {here[1] * 100:+6.1f}, up {here[2] * 100:+6.1f} cm")
        if t[-1] > LONG_MOVE_S:
            print(f"   (that move went on for {t[-1]:.0f} s without stopping, so it has probably drifted: "
                  f"stop for a moment every few seconds for accurate tracking, or press z to re-zero)")

    def _level_from_gravity(self):
        """Just stopped: re-level the tilt from gravity, keeping the heading."""
        g = np.mean(self.recent_acc, axis=0)
        # The accelerometer reads gravity slightly differently in each orientation
        # (small per-axis errors). Re-scale so gravity is exactly 1 g again here,
        # otherwise the difference shows up as the stick "falling" or "rising".
        size = np.linalg.norm(g)
        if 0.95 < size < 1.05:
            self.acc_scale /= size
        up_now = self.R @ (g / size)  # where we think "up" is
        axis = np.cross(up_now, [0, 0, 1.0])
        s, c = np.linalg.norm(axis), up_now[2]
        if s < 1e-6:
            return
        k = axis / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        self.R = (np.eye(3) + s * K + (1 - c) * K @ K) @ self.R  # smallest rotation onto true "up"
        self.ahrs.set_quaternion(matrix_to_quat(self.R))

    # ---------- For the display ----------

    def rezero(self):
        self.origin = self.current_position()[0] + self.origin
        self.clear_trail()
        print("Re-zeroed: this is now (0, 0, 0).")

    def clear_trail(self):
        self.trail_start = max(self.done, 0)

    def trail(self):
        start = max(self.trail_start, self.done - int(TRAIL_S * self.rate), 0)
        return self.h.a[start:self.done + 1, PX:PZ + 1] - self.origin

    def current_position(self):
        """(position, is_live_estimate)"""
        if self.state == "moving":
            return self.p_live - self.origin, True
        if self.done >= 0:
            return self.h.a[self.done, PX:PZ + 1] - self.origin, False
        return np.zeros(3), False

    def stick_axes(self):
        """The stick's own X, Y, Z axes in display axes (as columns)."""
        return self.F @ self.R


# ---------- Drawing ----------

class View:
    def __init__(self, tracker):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.tr = tracker
        # Free up keys we use that matplotlib also has shortcuts for (f = fullscreen, c = back)
        for name, key in (("keymap.fullscreen", "f"), ("keymap.back", "c")):
            if key in plt.rcParams[name]:
                plt.rcParams[name] = [k for k in plt.rcParams[name] if k != key]
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(projection="3d")
        # axlim_clip: anything outside the view is hidden instead of spilling over the edges
        (self.trail_line,) = self.ax.plot([], [], [], color="tab:blue", lw=1.5, label="corrected path", axlim_clip=True)
        (self.live_line,) = self.ax.plot([], [], [], "--", color="tab:orange", lw=1.5,
                                         label="moving (live estimate)", axlim_clip=True)
        (self.dot,) = self.ax.plot([], [], [], "o", ms=8, color="tab:green")
        self.axis_lines = [self.ax.plot([], [], [], color=c, lw=3)[0] for c in ("red", "green", "blue")]
        self.ax.plot([], [], [], color="grey", lw=3, label="stick axes: X red, Y green, Z blue")
        self.ax.set_xlabel("right (m)")
        self.ax.set_ylabel("forward (m)")
        self.ax.set_zlabel("up (m)")
        self.ax.legend(loc="upper left", fontsize=8)
        self.info = self.fig.text(0.02, 0.02, "", family="monospace", fontsize=9)
        # The view is a cube of width self.size centred on self.centre. It doesn't
        # zoom out by itself: it slides along to follow the stick, and snaps back
        # onto it after a re-zero. Scroll or +/- to zoom, f to fit the whole trail.
        self.size = VIEW_SIZE
        self.centre = None
        self.seen_reset = None
        self.fig.canvas.mpl_connect("key_press_event", self._key)
        self.fig.canvas.mpl_connect("scroll_event", self._scroll)

    def _key(self, event):
        if event.key == "z":
            self.tr.rezero()
        elif event.key == "c":
            self.tr.clear_trail()
        elif event.key in ("+", "="):
            self.size /= 1.25
        elif event.key in ("-", "_"):
            self.size *= 1.25
        elif event.key == "f":
            self._fit_trail()

    def _scroll(self, event):
        self.size *= 1 / 1.15 if event.button == "up" else 1.15

    def _fit_trail(self):
        """Zoom to show the whole trail once (it goes back to following the stick after)."""
        cur, _ = self.tr.current_position()
        pts = np.vstack([self.tr.trail(), cur[None]])
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        self.centre = (lo + hi) / 2
        self.size = max((hi - lo).max() * 1.2, 0.2)

    def update(self):
        tr = self.tr
        if tr.state == "calibrating":
            self.ax.set_title("Calibrating: put the stick down and keep it still")
            return
        pts = tr.trail()
        cur, live = tr.current_position()
        self.trail_line.set_data_3d(*(pts.T if len(pts) else ([], [], [])))
        lp = np.array(tr.live_path) - tr.origin if live and tr.live_path else np.empty((0, 3))
        self.live_line.set_data_3d(*(lp.T if len(lp) else ([], [], [])))
        self.dot.set_data_3d([cur[0]], [cur[1]], [cur[2]])
        self.dot.set_color("tab:orange" if live else "tab:green")
        axes = tr.stick_axes()
        for k, line in enumerate(self.axis_lines):
            end = cur + axes[:, k] * AXIS_LEN
            line.set_data_3d([cur[0], end[0]], [cur[1], end[1]], [cur[2], end[2]])

        # Follow the stick at a fixed zoom. Re-zeroing (z or button A) or clearing
        # the trail snaps the view back onto it.
        reset = (tr.trail_start, tuple(tr.origin))
        if self.centre is None or reset != self.seen_reset:
            self.centre = cur.copy()
            self.seen_reset = reset
        # Slide along when the stick gets near an edge (keeps it from being jumpy)
        room = self.size * (0.5 - VIEW_EDGE)
        self.centre = np.clip(self.centre, cur - room, cur + room)
        lo, hi = self.centre - self.size / 2, self.centre + self.size / 2
        self.ax.set_xlim(lo[0], hi[0])
        self.ax.set_ylim(lo[1], hi[1])
        self.ax.set_zlim(lo[2], hi[2])
        self.ax.set_box_aspect((1, 1, 1))

        speed = np.linalg.norm(tr.v_hp) if live else 0.0
        self.ax.set_title("MOVING (live estimate, corrected when it stops)" if live else "Still (corrected)",
                          color="tab:orange" if live else "tab:green")
        self.info.set_text(f"position  right {cur[0] * 100:+7.1f} cm   forward {cur[1] * 100:+7.1f} cm   "
                           f"up {cur[2] * 100:+7.1f} cm\nspeed {speed:4.2f} m/s   moves corrected: {tr.moves}"
                           f"\nkeys: z = re-zero, c = clear trail, scroll or +/- = zoom ({self.size * 100:.0f} cm wide), "
                           f"f = fit trail")


# ---------- Input ----------

def handle_line(tracker, line):
    parts = line.strip().split(",")
    try:
        if parts[0] == "D" and len(parts) == 8:
            v = list(map(float, parts[2:8]))
            tracker.add(int(parts[1]), v[0:3], v[3:6])
        elif parts[0] == "EV" and parts[1:2] == ["A"] and tracker.state != "calibrating":
            tracker.rezero()
    except (ValueError, IndexError):
        pass  # half a line (e.g. plugged in mid-message): ignore it


def find_port():
    ports = glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.wchusbserial*")
    if not ports:
        sys.exit("Couldn't find the stick. Plug it in, or give the port, e.g. /dev/cu.usbserial-XXXX")
    return ports[0]


def serial_reader(port, baud, lines, stop):
    """Background thread: reads lines from the stick, reconnecting if the cable drops out."""
    import serial
    ser = None
    while not stop.is_set():
        if ser is None:
            try:
                ser = serial.Serial()
                ser.port, ser.baudrate, ser.timeout = port, baud, 0.1
                ser.dtr = ser.rts = False  # don't reset the stick when connecting
                ser.open()
                print("Connected.")
            except (serial.SerialException, OSError):
                ser = None
                time.sleep(1)
                continue
        try:
            line = ser.readline().decode(errors="ignore")
        except (serial.SerialException, OSError):
            print("Lost the stick (USB unplugged or loose cable). Waiting for it to come back...")
            ser.close()
            ser = None
            continue
        if line:
            lines.put(line)
    if ser:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port (found automatically if left out)")
    ap.add_argument("--replay", help="process a saved recording instead of the stick")
    ap.add_argument("--realtime", action="store_true", help="with --replay: play it back at real speed")
    ap.add_argument("--save", help="save the final picture to this file (e.g. path.png)")
    ap.add_argument("--no-gui", action="store_true", help="with --replay: don't open a window")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    tracker = Tracker3D()

    if args.replay and not args.realtime:
        if args.no_gui:
            import matplotlib
            matplotlib.use("Agg")
        with open(args.replay) as f:
            for line in f:
                handle_line(tracker, line)
        view = View(tracker)
        view.update()
        if args.save:
            view.fig.savefig(args.save, dpi=120)
            print("Saved", args.save)
        if not args.no_gui:
            view.plt.show()
        return

    view = View(tracker)
    view.plt.ion()
    view.plt.show()
    lines = queue.Queue()
    stop = threading.Event()
    log = None

    if args.replay:  # play a file back at real speed
        def player():
            start, first = time.time(), None
            with open(args.replay) as f:
                for line in f:
                    parts = line.split(",")
                    if parts[0] == "D" and len(parts) == 8:
                        t = int(parts[1]) / 1e6
                        first = t if first is None else first
                        wait = (t - first) - (time.time() - start)
                        if wait > 0:
                            time.sleep(wait)
                    lines.put(line)
                    if stop.is_set():
                        return
        threading.Thread(target=player, daemon=True).start()
    else:
        port = args.port or find_port()
        rec_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
        os.makedirs(rec_dir, exist_ok=True)
        log_name = os.path.join(rec_dir, time.strftime("imu3d_raw_%Y%m%d_%H%M%S.csv"))
        log = open(log_name, "w")
        print(f"Reading {port}, saving to {log_name}.\nKeys: z = re-zero, c = clear trail, scroll or +/- = zoom, f = fit trail. Close the window to stop.")
        threading.Thread(target=serial_reader, args=(port, args.baud, lines, stop), daemon=True).start()

    try:
        while view.plt.fignum_exists(view.fig.number):
            while not lines.empty():
                line = lines.get()
                if log and line[:2] in ("D,", "EV"):
                    log.write(line)
                handle_line(tracker, line)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                view.update()
            view.plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if log:
            log.close()
        if args.save:
            view.fig.savefig(args.save, dpi=120)


if __name__ == "__main__":
    main()
