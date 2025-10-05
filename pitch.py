import sys
import math
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from collections import deque

# ----- CONFIG -----
PORT = '/dev/cu.usbmodem1101'  # <- change if needed
BAUD = 115200
WINDOW = 200                   # number of samples shown

ser = serial.Serial(PORT, BAUD, timeout=1)

pitch_buf = deque(maxlen=WINDOW)
roll_buf  = deque(maxlen=WINDOW)
x_idx     = deque(maxlen=WINDOW)

fig = plt.figure(figsize=(9,5))

# Top: time-series lines
ax1 = fig.add_subplot(2,1,1)
(line_pitch,) = ax1.plot([], [], label="Pitch (°)")
(line_roll,)  = ax1.plot([], [], label="Roll (°)")
ax1.set_xlim(0, WINDOW)
ax1.set_ylim(-90, 90)
ax1.set_xlabel("Samples")
ax1.set_ylabel("Angle (°)")
ax1.set_title("MPU6050 Pitch (Y) & Roll (X)")
ax1.legend(loc="upper right")

# Bottom: "seesaw" driven by Pitch
ax2 = fig.add_subplot(2,1,2)
ax2.set_xlim(-2, 2)
ax2.set_ylim(-1.2, 1.2)
ax2.set_aspect('equal', adjustable='box')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title("Pitch-driven Tilt")

# A rectangle centered at (0,0), width=3.0, height=0.2
bar = Rectangle((-1.5, -0.1), 3.0, 0.2, angle=0)
ax2.add_patch(bar)

def update_bar_angle(angle_deg):
    # matplotlib Rectangle rotates around lower-left corner by default.
    # We'll manually set transform around center.
    t = plt.matplotlib.transforms.Affine2D() \
        .rotate_deg_around(0, 0, angle_deg) + ax2.transData
    bar.set_transform(t)

def parse_line(line):
    # expecting "pitch,roll"
    try:
        parts = line.strip().split(',')
        if len(parts) != 2:
            return None, None
        pitch = float(parts[0])
        roll = float(parts[1])
        return pitch, roll
    except:
        return None, None

def init():
    line_pitch.set_data([], [])
    line_roll.set_data([], [])
    update_bar_angle(0)
    return (line_pitch, line_roll, bar)

def update(frame):
    # Read as many lines as available quickly
    for _ in range(5):
        raw = ser.readline().decode(errors='ignore')
        if not raw:
            break
        pitch, roll = parse_line(raw)
        if pitch is None:
            continue
        pitch_buf.append(pitch)
        roll_buf.append(roll)
        x_idx.append(len(x_idx) + 1 if x_idx else 1)

    # Update time-series
    xs = list(range(len(x_idx)))
    line_pitch.set_data(xs, list(pitch_buf))
    line_roll.set_data(xs, list(roll_buf))

    # Keep x-limits locked to WINDOW
    ax1.set_xlim(max(0, len(xs)-WINDOW), max(WINDOW, len(xs)))

    # Update tilt bar from Pitch
    if pitch_buf:
        update_bar_angle(pitch_buf[-1])

    return (line_pitch, line_roll, bar)

ani = animation.FuncAnimation(fig, update, init_func=init, interval=30, blit=True)
plt.tight_layout()
plt.show()
