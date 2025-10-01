import math
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from collections import deque

# ----- CONFIG -----
WINDOW = 200  # number of samples shown

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
ax1.set_title("Simulated Pitch (Y) & Roll (X)")
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
    t = plt.matplotlib.transforms.Affine2D() \
        .rotate_deg_around(0, 0, angle_deg) + ax2.transData
    bar.set_transform(t)

# ---------------- FAKE DATA GENERATOR ----------------
step = 0
def generate_fake_data():
    global step
    step += 1
    # Pitch oscillates like a sine wave (-45 to +45 degrees)
    pitch = 45 * math.sin(step * 0.1)
    # Roll oscillates like a cosine wave (-30 to +30 degrees)
    roll = 30 * math.cos(step * 0.1)
    return pitch, roll

# ---------------- INIT ----------------
def init():
    line_pitch.set_data([], [])
    line_roll.set_data([], [])
    update_bar_angle(0)
    return (line_pitch, line_roll, bar)

# ---------------- UPDATE ----------------
def update(frame):
    pitch, roll = generate_fake_data()
    pitch_buf.append(pitch)
    roll_buf.append(roll)
    x_idx.append(len(x_idx) + 1 if x_idx else 1)

    # Update time-series
    xs = list(range(len(x_idx)))
    line_pitch.set_data(xs, list(pitch_buf))
    line_roll.set_data(xs, list(roll_buf))

    ax1.set_xlim(max(0, len(xs)-WINDOW), max(WINDOW, len(xs)))

    # Update tilt bar from Pitch
    update_bar_angle(pitch_buf[-1])

    return (line_pitch, line_roll, bar)

ani = animation.FuncAnimation(fig, update, init_func=init, interval=30, blit=True)
plt.tight_layout()
plt.show()
