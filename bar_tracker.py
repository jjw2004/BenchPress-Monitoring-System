#!/usr/bin/env python3
"""BarTone tracker: 3D position of the stick + 2D bench press bar path.

Reads raw IMU data from the M5StickC Plus (BarTone_IMUStream sketch) and shows:
  - left:   a live 3D view of where the stick is (dead reckoning)
  - middle: the side-on bar path of each rep (the J-curve)
  - right:  the mean concentric velocity of each rep

The method combines two open-source x-io Technologies projects:

  Gait-Tracking (github.com/xioTechnologies/Gait-Tracking)
    - Fusion AHRS (imufusion) works out the orientation, so the acceleration
      can be turned into room axes with gravity removed.
    - Moments when the sensor is still are detected (with margins either side).
    - Each movement between still moments is integrated to velocity, and the
      drift is removed so the velocity is 0 at both ends (zero-velocity update),
      then integrated again to get position.

  Oscillatory-Motion-Tracking (github.com/xioTechnologies/Oscillatory-Motion-Tracking-With-x-IMU)
    - Velocity is high-pass filtered (1st order Butterworth, 0.1 Hz, run
      forwards and backwards so it doesn't shift anything in time) to remove
      slowly-changing drift, without needing any still moments.
    - Used here for long movements, e.g. touch-and-go reps with no pause at the
      top. The filtered velocity shows where the bar turns round at each top
      and bottom; it stops for an instant there, so those become extra
      zero-velocity moments for the Gait-Tracking drift removal. (Using the
      filtered velocity directly was tested and is worse: it also removes some
      of the real movement, so the 3D position ends up in the wrong place.)

While the bar is moving, the 3D view shows a live estimate (orange) that is
not corrected yet. As soon as the bar is still, that movement is corrected and
added to the blue trail, and any finished reps are drawn.

Setup (once):
    pip3 install imufusion pyserial matplotlib scipy numpy

Live (close the Arduino Serial Monitor / Chrome first):
    python3 bar_tracker.py                 # finds the stick's port automatically
    python3 bar_tracker.py /dev/cu.usbserial-XXXX
  Keys in the window: n = new set, z = re-zero the 3D position.
  Button A on the stick also starts a new set.
  Everything received is saved to recordings/bartone_raw_<date>_<time>.csv
  (next to this script) for replaying.

Replay a saved recording:
    python3 bar_tracker.py --replay recordings/bartone_raw_20261107_101500.csv            # final result
    python3 bar_tracker.py --replay recordings/bartone_raw_20261107_101500.csv --realtime # play it back live
    python3 bar_tracker.py --replay FILE --save result.png --no-gui             # for the report
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
from scipy.signal import butter, filtfilt, find_peaks

G = 9.80665  # 1 g in m/s^2

# ---- Calibration (bar sitting in the rack at the start) ----
CAL_S = 1.0           # s of stillness needed
CAL_GYRO_STD = 1.5    # deg/s, maximum wobble allowed while calibrating
CAL_ACC_STD = 0.02    # g

# ---- Still / moving detection (Gait-Tracking idea, retuned for slow bar movements) ----
# Gait-Tracking uses 3 m/s^2 because a foot hits the ground hard. A bar moves
# much more gently, so the thresholds are far lower here.
ACC_MOVING = 0.35     # m/s^2, acceleration (gravity removed) above this = moving
GYRO_MOVING = 8.0     # deg/s, rotation above this = moving
STILL_MAX_V = 0.12    # m/s, and the (uncorrected) speed must be below this to count as still.
                      # Halfway through a rep the bar is fast but hardly accelerating, which
                      # would otherwise look still for a moment and cut the rep in half.
DRIFT_ALLOW = 0.02    # m/s per second of movement: the speed limit above grows by this much,
                      # because the uncorrected speed drifts more the longer a movement goes on
STILL_S = 0.25        # must look still for this long to count as still (trailing margin)
STILL_ANYWAY_S = 1.5  # no acceleration or rotation for this long = still, whatever the speed says
STILL_ACC_G = 0.03    # g, for that: total acceleration within this of 1 g (just gravity)
LEAD_S = 0.10         # a movement is taken to start this much before it was noticed (leading margin)

# ---- Drift removal ----
HP_MIN_S = 4.0        # movements longer than this get extra zero-velocity moments at each turnaround
HP_CUTOFF_HZ = 0.1    # high-pass cut-off for finding them, same as Oscillatory-Motion-Tracking
MIN_STROKE = 0.08     # m, a turnaround only counts after the bar has moved at least this far

# ---- Rep detection ----
MIN_ROM = 0.12        # m, the bar must go down at least this far to count as a rep
RETURN_FRACTION = 0.75  # ...and come back up at least this much of the way
V_MOVING = 0.03       # m/s, slower than this is "not moving" when timing up/down
# Anything outside these isn't a bench rep (e.g. waving the stick around by hand)
MAX_ROM = 0.90        # m, deepest possible rep
MAX_PHASE_S = 6.0     # s, longest possible way down or way up
MAX_SWAY = 0.15       # m, furthest the bar can slide sideways (along itself)
MAX_HORIZONTAL = 0.30 # m, furthest the bottom can be from lockout towards head/feet

TRAIL_S = 60          # s of history shown in the 3D view

# Columns of the history array
T, DT, AX, AY, AZ, PX, PY, PZ, VX, VY, VZ = range(11)
NCOLS = 11


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

    def __init__(self):
        self.a = np.zeros((4096, NCOLS))
        self.n = 0

    def append(self, row):
        if self.n == len(self.a):
            self.a = np.concatenate([self.a, np.zeros_like(self.a)])
        self.a[self.n] = row
        self.n += 1

    def rows(self, start=0, stop=None):
        stop = self.n if stop is None else stop
        return self.a[start:stop]


class Tracker:
    """All the maths. Feed it samples with add(); it works the same live or from a file."""

    def __init__(self, rate=100.0, bar_axis=(0, 1, 0), head_sign=1.0):
        self.rate = rate
        self.bar_axis = np.array(bar_axis, float)
        self.head_sign = head_sign
        self.state = "calibrating"   # then "still" or "moving"
        self.cal = deque(maxlen=int(CAL_S * rate))
        self.recent_acc = deque(maxlen=int(STILL_S * rate))  # last few accel readings (stick axes, g)
        self.t_prev_us = None
        self.t = 0.0
        self.h = History()
        self.done = -1              # positions are final up to this row
        self.last_move_end = -1
        self.move_start = 0
        self.quiet = 0.0
        self.quiet_anyway = 0.0
        self.v_live = np.zeros(3)   # live (uncorrected) estimate while moving
        self.p_live = np.zeros(3)
        self.R = np.eye(3)          # current orientation: stick axes -> room axes
        self.F = np.eye(3)          # room axes -> bar axes (x along bar, y towards head, z up)
        self.origin = np.zeros(3)   # subtracted from positions for display (z key)
        self.set_number = 0
        self.set_start = 0
        self.reps = []
        self.new_reps = []
        self.seen_bottoms = set()
        self.moves = 0
        self.hp_moves = 0
        self.warning = ""

    # ---------- Input ----------

    def add(self, t_us, gyro_dps, acc_g):
        gyro = np.asarray(gyro_dps, float)
        acc = np.asarray(acc_g, float)
        if self.t_prev_us is None:
            dt = 1.0 / self.rate
        else:
            dt = ((t_us - self.t_prev_us) & 0xFFFFFFFF) / 1e6  # (handles the timer wrapping round)
        self.t_prev_us = t_us
        if dt <= 0 or dt > 0.5:
            dt = 1.0 / self.rate  # missing data: carry on as best we can

        if self.state == "calibrating":
            self._calibrate(gyro, acc)
            return

        self.t += dt
        # Remove gyro bias (our startup value, then Fusion keeps fine-tuning it when still)
        gyro = self.bias.update(gyro - self.gyro_bias)
        acc = acc * self.acc_scale

        # Orientation (Fusion AHRS). The accelerometer is only used to correct the
        # tilt while the bar is still: during a rep the bar's own (gentle)
        # acceleration would be mistaken for gravity and cancel out part of the
        # real movement. Giving Fusion zeros makes it use the gyro alone.
        self.ahrs.set_sample_period(dt)
        self.ahrs.update_no_magnetometer(gyro, acc if self.state == "still" else np.zeros(3))
        self.R = imufusion.quaternion_to_matrix(self.ahrs.get_quaternion())
        # Acceleration in room axes, minus gravity (1 g straight down), in m/s^2
        a = self.F @ ((self.R @ acc - np.array([0, 0, 1.0])) * G)

        moving_now = np.linalg.norm(a) > ACC_MOVING or np.linalg.norm(gyro) > GYRO_MOVING
        # The same check, but one that doesn't depend on the orientation being
        # right: barely rotating, and feeling only gravity (1 g in total). If
        # the stick is turned a long way by hand, the orientation can be off by
        # many degrees, so gravity leaks into `a` and the check above would
        # think it's moving forever.
        quiet_raw = (np.linalg.norm(gyro) < GYRO_MOVING
                     and abs(np.linalg.norm(acc) - 1.0) < STILL_ACC_G)
        self.recent_acc.append(acc)
        i = self.h.n
        self.h.append([self.t, dt, *a, 0, 0, 0, 0, 0, 0])
        H = self.h.a

        if self.state == "still":
            if moving_now:
                # Movement started. Like Gait-Tracking's leading margin, assume it
                # really began a little earlier (but never re-open an old movement).
                self.state = "moving"
                self.quiet = 0.0
                self.move_start = max(i - int(LEAD_S * self.rate), self.last_move_end + 1, 0)
                self.done = self.move_start - 1
                self.p_live = H[self.done, PX:PZ + 1].copy() if self.done >= 0 else np.zeros(3)
                self.v_live = np.zeros(3)
                self.quiet_anyway = 0.0
                for k in range(self.move_start, i + 1):
                    self._live_step(H[k])
            else:
                if i > 0:
                    H[i, PX:PZ + 1] = H[i - 1, PX:PZ + 1]
                self.done = i
        else:  # moving
            self._live_step(H[i])
            # Still = no acceleration or rotation, AND not moving at speed (the
            # speed limit grows with time to allow for drift). Or, as a backup
            # that doesn't trust the orientation at all: barely rotating and
            # feeling only gravity for a long time.
            moving_for = H[i, T] - H[self.move_start, T]
            slow = np.linalg.norm(self.v_live) < STILL_MAX_V + DRIFT_ALLOW * moving_for
            self.quiet = self.quiet + dt if (not moving_now and slow) else 0.0
            self.quiet_anyway = self.quiet_anyway + dt if quiet_raw else 0.0
            if self.quiet >= STILL_S or self.quiet_anyway >= STILL_ANYWAY_S:
                self._finish_movement(self.move_start, i)
                self._level_from_gravity()
                self.state = "still"

    def _level_from_gravity(self):
        """The stick has just stopped: set the tilt straight from gravity.

        While moving, the orientation comes from the gyro alone, so small errors
        build up. Now that it's still, the accelerometer only feels gravity, so
        we rotate the orientation (by the smallest amount, keeping the heading)
        until "down" matches. Same idea as the hold-still step in the sketches.
        """
        g = np.mean(self.recent_acc, axis=0)
        up_now = self.R @ (g / np.linalg.norm(g))  # where we think "up" is
        z = np.array([0, 0, 1.0])
        axis = np.cross(up_now, z)
        s, c = np.linalg.norm(axis), float(np.dot(up_now, z))
        if s < 1e-6:
            return
        k = axis / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        C = np.eye(3) + s * K + (1 - c) * K @ K  # rotation taking up_now onto z
        self.R = C @ self.R
        self.ahrs.set_quaternion(matrix_to_quat(self.R))

    def _live_step(self, row):
        # Live (uncorrected) estimate while moving
        self.v_live += row[AX:AZ + 1] * row[DT]
        self.p_live += self.v_live * row[DT]

    def new_set(self):
        if self.state == "calibrating":
            return
        self.set_number += 1
        self.set_start = self.h.n
        self.seen_bottoms = set()
        print(f"--- Set {self.set_number} ---")

    def rezero(self):
        self.origin = self.current_position()[0] + self.origin

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

        # Fusion AHRS. The accelerometer is only trusted when it's within 3
        # degrees of gravity, i.e. when the bar is nearly still.
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

        # Bar axes: x = along the bar, y = level and towards the head, z = up.
        # (No compass on the M5StickC Plus, so this comes from the bar itself.)
        bar = self.R @ self.bar_axis
        tilt = math.degrees(math.asin(min(abs(bar[2]), 1.0)))
        if tilt > 30:
            self.warning = f"bar axis is {tilt:.0f} deg from level: check mounting / --bar-axis"
            print("WARNING:", self.warning)
        bar_h = np.array([bar[0], bar[1], 0.0])
        bar_h = bar_h / np.linalg.norm(bar_h) if np.linalg.norm(bar_h) > 0.3 else np.array([1.0, 0, 0])
        up = np.array([0, 0, 1.0])
        head = np.cross(up, bar_h) * self.head_sign
        self.F = np.vstack([bar_h * self.head_sign, head, up])  # (flip x too so it stays right-handed)

        self.state = "still"
        self.set_number = 1
        print(f"Calibrated. Gyro bias {np.round(self.gyro_bias, 2)} deg/s, bar tilt {tilt:.1f} deg")
        print(f"--- Set {self.set_number} ---")

    # ---------- Drift removal (the two methods combined) ----------

    def _finish_movement(self, s, e):
        H = self.h.a
        dt = H[s:e + 1, DT]
        a = H[s:e + 1, AX:AZ + 1]
        v = np.cumsum(a * dt[:, None], axis=0)  # velocity, including drift
        t = np.cumsum(dt)
        duration = t[-1]

        # Moments when we know the velocity is 0: the start and end of the movement
        knots = [0, len(v) - 1]

        # Long movement (e.g. touch-and-go reps, no pause at the top): the bar
        # also stops for an instant at every top and bottom. Find those
        # turnarounds with Oscillatory-Motion-Tracking's high-pass filter, which
        # removes the drift so the vertical velocity crosses zero in the right
        # places, and use them as extra zero-velocity moments.
        if duration > HP_MIN_S and len(v) > 12:
            fs = 1.0 / dt.mean()
            b, a_ = butter(1, HP_CUTOFF_HZ / (fs / 2), "high")
            vz_hp = filtfilt(b, a_, v[:, 2])
            last, moved = 0, 0.0
            for k in range(1, len(v) - 1):
                moved += vz_hp[k] * dt[k]
                crossed = np.sign(vz_hp[k]) != np.sign(vz_hp[k + 1])
                if crossed and abs(moved) > MIN_STROKE and t[-1] - t[k] > 0.3:
                    knots.insert(-1, k)
                    last, moved = k, 0.0
            self.hp_moves += 1

        # Gait-Tracking: velocity must be 0 at each of those moments. Whatever is
        # left there is drift: remove it, assuming it changed in a straight line
        # from one zero-velocity moment to the next.
        for axis in range(3):
            v[:, axis] -= np.interp(t, t[knots], v[knots, axis])

        p0 = H[s - 1, PX:PZ + 1] if s > 0 else np.zeros(3)
        H[s:e + 1, VX:VZ + 1] = v
        H[s:e + 1, PX:PZ + 1] = p0 + np.cumsum(v * dt[:, None], axis=0)
        self.done = e
        self.last_move_end = e
        self.moves += 1
        self._find_reps()

    # ---------- Reps ----------

    def _find_reps(self):
        s0, e0 = self.set_start, self.done
        if e0 - s0 < 10:
            return
        H = self.h.a
        z = H[s0:e0 + 1, PZ]
        bottoms, _ = find_peaks(-z, prominence=MIN_ROM)
        for k, b in enumerate(bottoms):
            if s0 + b in self.seen_bottoms:
                continue
            lo = bottoms[k - 1] if k > 0 else 0
            hi = bottoms[k + 1] if k + 1 < len(bottoms) else len(z) - 1
            s = lo + int(np.argmax(z[lo:b + 1]))   # lockout before the rep
            e = b + int(np.argmax(z[b:hi + 1]))    # lockout after it
            if z[e] - z[b] < RETURN_FRACTION * (z[s] - z[b]):
                continue  # hasn't come back up yet (e.g. paused on the chest)
            self.seen_bottoms.add(s0 + b)
            rep = self._make_rep(s0 + s, s0 + b, s0 + e)
            if self._looks_like_a_bench_rep(rep):
                self.reps.append(rep)
                self.new_reps.append(rep)

    @staticmethod
    def _looks_like_a_bench_rep(r):
        """Throw away "reps" that can't be a bench press, e.g. waving the stick about."""
        problems = []
        if r["rom"] > MAX_ROM:
            problems.append(f"went down {r['rom'] * 100:.0f} cm")
        if r["ecc_s"] > MAX_PHASE_S or r["conc_s"] > MAX_PHASE_S:
            problems.append(f"took {r['ecc_s']:.1f} s down / {r['conc_s']:.1f} s up")
        if r["sway"] > MAX_SWAY:
            problems.append(f"moved {r['sway'] * 100:.0f} cm along the bar")
        if abs(r["h_bottom"]) > MAX_HORIZONTAL:
            problems.append(f"bottom {r['h_bottom'] * 100:+.0f} cm from lockout horizontally")
        if problems:
            print("Ignored a movement that isn't a bench rep:", ", ".join(problems))
        return not problems

    def _make_rep(self, s, b, e):
        H = self.h.a
        rows = H[s:e + 1]
        p = rows[:, PX:PZ + 1] - rows[0, PX:PZ + 1]  # relative to the starting lockout
        vz, dt = rows[:, VZ], rows[:, DT]
        ib = b - s
        down, up = vz[:ib] < -V_MOVING, vz[ib:] > V_MOVING
        conc_time = dt[ib:][up].sum()
        set_reps = [r for r in self.reps if r["set"] == self.set_number]
        rep = {
            "set": self.set_number,
            "rep": len(set_reps) + 1,
            "rom": -p[ib, 2],
            "mcv": (vz[ib:][up] * dt[ib:][up]).sum() / conc_time if conc_time > 0 else 0.0,
            "pcv": vz[ib:].max(),
            "ecc_s": dt[:ib][down].sum(),
            "conc_s": conc_time,
            "bottom_s": dt[p[:, 2] <= p[ib, 2] + 0.01].sum(),  # time within 1 cm of the bottom
            "h_bottom": p[ib, 1],
            "sway": np.ptp(p[:, 0]),
            "path": p,
            "bottom_index": ib,
        }
        best = max([r["mcv"] for r in set_reps] + [rep["mcv"]])
        rep["loss"] = 100 * (1 - rep["mcv"] / best) if best > 0 else 0.0
        return rep

    # ---------- For the display ----------

    def trail(self):
        start = max(0, self.done - int(TRAIL_S * self.rate))
        return self.h.a[start:self.done + 1, PX:PZ + 1] - self.origin

    def current_position(self):
        """(position, is_live_estimate)"""
        if self.state == "moving":
            return self.p_live - self.origin, True
        if self.done >= 0:
            return self.h.a[self.done, PX:PZ + 1] - self.origin, False
        return np.zeros(3), False

    def bar_direction(self):
        return self.F @ self.R @ self.bar_axis


def describe(r):
    return (f"Set {r['set']} rep {r['rep']}: ROM {r['rom'] * 100:.0f} cm, mean up {r['mcv']:.2f} m/s, "
            f"peak {r['pcv']:.2f} m/s, down {r['ecc_s']:.1f} s, up {r['conc_s']:.1f} s, "
            f"at bottom {r['bottom_s']:.1f} s, bottom {r['h_bottom'] * 100:+.1f} cm towards head, "
            f"sway {r['sway'] * 100:.1f} cm, loss {r['loss']:.0f}%")


# ---------- Drawing ----------

class View:
    def __init__(self, tracker):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.tr = tracker
        self.fig = plt.figure(figsize=(15, 5.5))
        gs = self.fig.add_gridspec(1, 3, width_ratios=[1.25, 1, 0.8])
        self.ax3 = self.fig.add_subplot(gs[0], projection="3d")
        self.ax2 = self.fig.add_subplot(gs[1])
        self.axv = self.fig.add_subplot(gs[2])
        (self.trail_line,) = self.ax3.plot([], [], [], color="tab:blue", lw=1, label="corrected path")
        (self.bar_line,) = self.ax3.plot([], [], [], color="k", lw=3, label="bar direction")
        (self.dot,) = self.ax3.plot([], [], [], "o", ms=9, color="tab:green")
        self.ax3.set_xlabel("along bar (m)")
        self.ax3.set_ylabel("towards head (m)")
        self.ax3.set_zlabel("up (m)")
        self.ax3.legend(loc="upper left", fontsize=8)
        self.drawn = (-1, -1)
        self.fig.canvas.mpl_connect("key_press_event", self._key)

    def _key(self, event):
        if event.key == "n":
            self.tr.new_set()
        elif event.key == "z":
            self.tr.rezero()

    def update(self):
        tr = self.tr
        if tr.state == "calibrating":
            self.fig.suptitle("Calibrating: leave the bar in the rack, still")
            return
        pts = tr.trail()
        cur, live = tr.current_position()
        if len(pts):
            self.trail_line.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
        self.dot.set_data_3d([cur[0]], [cur[1]], [cur[2]])
        self.dot.set_color("tab:orange" if live else "tab:green")
        d = tr.bar_direction() * 0.3
        self.bar_line.set_data_3d([cur[0] - d[0], cur[0] + d[0]], [cur[1] - d[1], cur[1] + d[1]],
                                  [cur[2] - d[2], cur[2] + d[2]])
        allp = np.vstack([pts, cur]) if len(pts) else cur[None]
        centre = (allp.min(axis=0) + allp.max(axis=0)) / 2
        span = max(np.ptp(allp, axis=0).max() * 1.1, 0.6)
        self.ax3.set_xlim(centre[0] - span / 2, centre[0] + span / 2)
        self.ax3.set_ylim(centre[1] - span / 2, centre[1] + span / 2)
        self.ax3.set_zlim(centre[2] - span / 2, centre[2] + span / 2)
        self.ax3.set_box_aspect((1, 1, 1))
        self.ax3.set_title(f"Where the stick is ({'moving: live estimate' if live else 'still: corrected'})",
                           fontsize=10)

        set_reps = [r for r in tr.reps if r["set"] == tr.set_number]
        if (tr.set_number, len(set_reps)) != self.drawn:
            self.drawn = (tr.set_number, len(set_reps))
            self._draw_reps(set_reps)
        self.fig.suptitle(f"BarTone  |  set {tr.set_number}  |  {len(set_reps)} reps  |  "
                          f"{tr.moves} movements corrected ({tr.hp_moves} long ones using turnarounds)"
                          + (f"  |  WARNING: {tr.warning}" if tr.warning else ""), fontsize=10)

    def _draw_reps(self, reps):
        ax, axv = self.ax2, self.axv
        ax.clear()
        axv.clear()
        for r in reps[:-1]:
            ax.plot(r["path"][:, 1] * 100, r["path"][:, 2] * 100, color="grey", alpha=0.4, lw=1)
        if reps:
            r = reps[-1]
            p, ib = r["path"] * 100, r["bottom_index"]
            ax.plot(p[:ib + 1, 1], p[:ib + 1, 2], color="tab:red", lw=2, label="down")
            ax.plot(p[ib:, 1], p[ib:, 2], color="tab:green", lw=2, label="up")
            ax.legend(fontsize=8, loc="lower right")
            ax.set_title(f"Bar path, rep {r['rep']}: ROM {r['rom'] * 100:.0f} cm, "
                         f"{r['mcv']:.2f} m/s", fontsize=10)
        else:
            ax.set_title("Bar path (side view)", fontsize=10)
        ax.plot(0, 0, "ko", ms=4)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("feet ←  horizontal (cm)  → head")
        ax.set_ylabel("height from lockout (cm)")
        ax.grid(alpha=0.3)

        mcv = [r["mcv"] for r in reps]
        axv.bar([r["rep"] for r in reps], mcv, color="tab:green")
        for r in reps:
            axv.text(r["rep"], r["mcv"], f"{r['mcv']:.2f}\n" + (f"-{r['loss']:.0f}%" if r['loss'] >= 0.5 else "best"), ha="center",
                     va="bottom", fontsize=8)
        axv.set_ylim(0, max(mcv + [0.5]) * 1.3)
        axv.set_xlabel("rep")
        axv.set_ylabel("mean concentric velocity (m/s)")
        axv.set_title("Speed per rep (and loss vs fastest)", fontsize=10)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # window too small to fit everything: harmless
            self.fig.tight_layout(rect=(0, 0, 1, 0.93))


# ---------- Input sources ----------

def handle_line(tracker, line):
    parts = line.strip().split(",")
    try:
        if parts[0] == "D" and len(parts) == 8:
            v = list(map(float, parts[2:8]))
            tracker.add(int(parts[1]), v[0:3], v[3:6])
        elif parts[0] == "EV" and parts[1:2] == ["A"]:
            tracker.new_set()
    except (ValueError, IndexError):
        pass  # half a line (e.g. plugged in mid-message): ignore it
    for r in tracker.new_reps:
        print(describe(r))
    tracker.new_reps.clear()


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
    ap.add_argument("--replay", help="process a saved bartone_raw_*.csv instead of the stick")
    ap.add_argument("--realtime", action="store_true", help="with --replay: play it back at real speed")
    ap.add_argument("--save", help="save the final figure to this file (e.g. result.png)")
    ap.add_argument("--no-gui", action="store_true", help="with --replay: don't open a window")
    ap.add_argument("--bar-axis", default="y", choices=["x", "y", "z", "-x", "-y", "-z"],
                    help="which of the stick's axes runs along the bar (default y, the long side)")
    ap.add_argument("--flip", action="store_true", help="mirror the head/feet direction")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    axis = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[args.bar_axis[-1]]
    axis = tuple(-c for c in axis) if args.bar_axis.startswith("-") else axis
    tracker = Tracker(bar_axis=axis, head_sign=-1.0 if args.flip else 1.0)

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
        log_name = os.path.join(rec_dir, time.strftime("bartone_raw_%Y%m%d_%H%M%S.csv"))
        log = open(log_name, "w")
        print(f"Reading {port}, saving to {log_name}. Keys: n = new set, z = re-zero. Close the window to stop.")
        threading.Thread(target=serial_reader, args=(port, args.baud, lines, stop), daemon=True).start()

    try:
        while view.plt.fignum_exists(view.fig.number):
            while not lines.empty():
                line = lines.get()
                if log and line[:2] in ("D,", "EV"):
                    log.write(line)
                handle_line(tracker, line)
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
