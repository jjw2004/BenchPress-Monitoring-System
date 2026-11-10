"""Fake BarTone_IMUStream recording of a bench session, with known true answers.

Writes sim_raw.csv. Test the tracker with:
    python3 simulate_bench.py
    python3 bar_tracker.py --replay sim_raw.csv
"""
import numpy as np
rng = np.random.default_rng(1)
FS = 1000  # simulate at 1 kHz, average to 100 Hz like the stick
G = 9.80665
segs = []  # (duration, dy, dz, kind)  y = towards head, z = up, in "bar frame"
def move(T, dy, dz): segs.append((T, dy, dz, 'move'))
def still(T): segs.append((T, 0, 0, 'still'))
def event(): segs.append((0, 0, 0, 'A'))
still(2.0)                    # calibration in the rack
move(1.0, -0.15, 0.08)        # unrack
still(1.0)
truth = []
for k, (Td, Tu, pause) in enumerate([(1.2, 1.0, 0), (1.2, 1.2, 1.0), (1.2, 1.4, 0)]):
    move(Td, -0.07, -0.40)     # down, J-curve: bottom is towards the feet
    if pause: still(pause)     # paused on the chest
    move(Tu, 0.07, 0.40)      # up
    still(0.7)                 # pause at lockout
    truth.append((1, k + 1, 0.40, 0.40 / Tu, -0.07))
event()
still(1.0)
# touch and go: no stops at the top or bottom
tg = []
for k, Tu in enumerate([0.9, 1.0, 1.1, 1.3]):
    tg.append((1.1, -0.06, -0.38)); tg.append((Tu, 0.06, 0.38))
    truth.append((2, k + 1, 0.38, 0.38 / Tu, -0.06))
segs.append((None, tg, None, 'chain'))
still(1.0)
move(1.0, 0.15, -0.08)        # rack it
still(2.0)

# Build position (bar frame) at 1 kHz with smooth (1-cos) profiles
ys, zs, ev = [], [], []
y = z = 0.0
def profile(T, dy, dz):
    n = int(round(T * FS)); s = (1 - np.cos(np.pi * (np.arange(1, n + 1) / n))) / 2
    return y + dy * s, z + dz * s
for T, dy, dz, kind in segs:
    if kind == 'A': ev.append(len(ys)); continue
    if kind == 'still':
        n = int(T * FS); ys += [y] * n; zs += [z] * n
    elif kind == 'move':
        py, pz = profile(T, dy, dz); ys += list(py); zs += list(pz); y, z = py[-1], pz[-1]
    else:  # chain of half-reps, velocity continuous only at turnaround points (v=0 there too)
        for T2, dy2, dz2 in dy:
            py, pz = profile(T2, dy2, dz2); ys += list(py); zs += list(pz); y, z = py[-1], pz[-1]
N = len(ys); t = np.arange(N) / FS
pos = np.stack([np.zeros(N), np.array(ys), np.array(zs)], 1)
acc = np.gradient(np.gradient(pos, 1 / FS, axis=0), 1 / FS, axis=0)  # m/s^2, bar frame

# Bar rolls in the hands +-3 deg while moving (rotation about the bar = sensor y)
moving = np.linalg.norm(np.gradient(pos, 1 / FS, axis=0), axis=1) > 1e-4
roll = np.deg2rad(3) * np.sin(2 * np.pi * 0.7 * t) * moving
roll = np.convolve(roll, np.ones(200) / 200, mode='same')
roll_rate = np.gradient(roll, 1 / FS)

def rx(a):  # rotation about x
    c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def rz(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
# Mount: sensor y along the bar (bar x); sensor z tilted 40 deg up from "towards head"
sz = np.array([0, np.cos(np.radians(40)), np.sin(np.radians(40))]); sy = np.array([1.0, 0, 0]); sx = np.cross(sy, sz)
R0 = np.stack([sx, sy, sz], 1)               # sensor -> bar frame
W = rz(np.radians(30))                        # bar frame -> room (the bench points 30 deg off "north")
gb = np.array([0.8, -0.5, 0.3]); ab = np.array([0.010, -0.008, 0.015])

lines = []
for i0 in range(0, N - 10, 10):
    gs, as_ = np.zeros(3), np.zeros(3)
    for i in range(i0, i0 + 10):
        R = W @ rx(roll[i]) @ R0                # sensor -> room
        f = (W @ acc[i]) / G + np.array([0, 0, 1.0])
        as_ += R.T @ f
        gs += np.array([0, np.degrees(roll_rate[i]), 0])
    g_ = gs / 10 + gb + rng.normal(0, 0.05, 3)
    a_ = as_ / 10 + ab + rng.normal(0, 0.002, 3)
    tus = int(t[i0 + 9] * 1e6) + 123456
    lines.append(f"D,{tus},{g_[0]:.2f},{g_[1]:.2f},{g_[2]:.2f},{a_[0]:.4f},{a_[1]:.4f},{a_[2]:.4f}\n")
    if any(i0 <= e < i0 + 10 for e in ev): lines.append("EV,A\n")
open('sim_raw.csv', 'w').writelines(lines)
print(f"{len(lines)} lines, {N / FS:.1f} s")
print("TRUE reps (set, rep, ROM m, MCV m/s (simple avg), bottom towards head m):")
for r in truth: print("  ", r[0], r[1], f"ROM {r[2]*100:.0f} cm  MCV~{r[3]:.2f}  bottom {r[4]*100:+.0f} cm")
