import sys
import math
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from collections import deque
import numpy as np

# ----- CONFIG -----
PORT = 'COM9'  # Change if needed
BAUD = 115200
WINDOW = 200   # Number of samples shown

ser = serial.Serial(PORT, BAUD, timeout=1)

pitch_buf = deque(maxlen=WINDOW)
roll_buf  = deque(maxlen=WINDOW)
yaw_buf   = deque(maxlen=WINDOW)
x_idx     = deque(maxlen=WINDOW)

fig = plt.figure(figsize=(12, 8))

# Top: 3D Airplane Visualization
ax1 = fig.add_subplot(2, 1, 1, projection='3d')
ax1.set_xlim(-2, 2)
ax1.set_ylim(-2, 2)
ax1.set_zlim(-1, 2)  # Adjusted z-limit for airplane model
ax1.set_xlabel('X (Roll)')
ax1.set_ylabel('Y (Pitch)')
ax1.set_zlabel('Z (Yaw)')
ax1.set_title("3D Tilting Airplane Visualization (Pitch/Roll/Yaw)")

# Airplane model: Simple 3D representation with fuselage, wings, tail, nose, and tail identifiable
# Vertices define an airplane:
# - Fuselage: Long cylinder-like body
# - Wings: Extending from sides
# - Tail: Vertical and horizontal stabilizers
# - Nose: Pointed front (top/forward), tail rear (bottom/backward)
bed_vertices = np.array([
    # Fuselage vertices (body: 8 points for a prism)
    [-0.2, -2, 0], [0.2, -2, 0], [0.2, -1.5, 0], [-0.2, -1.5, 0],  # Rear base
    [-0.2, -2, 0.2], [0.2, -2, 0.2], [0.2, -1.5, 0.2], [-0.2, -1.5, 0.2],  # Rear top
    [-0.2, 1, 0], [0.2, 1, 0], [0.2, 1.5, 0], [-0.2, 1.5, 0],  # Front base (nose area, pointed)
    [-0.2, 1, 0.2], [0.2, 1, 0.2], [0.2, 1.5, 0.2], [-0.2, 1.5, 0.2],  # Front top
    # Wings (left and right)
    [-1.5, -0.5, 0], [1.5, -0.5, 0], [-1.5, -1, 0], [1.5, -1, 0],  # Wing base
    # Tail stabilizer (horizontal)
    [-0.5, -2, 0], [0.5, -2, 0], [-0.5, -1.8, 0], [0.5, -1.8, 0],
    # Vertical tail fin
    [0, -2, 0], [0, -2, 0.5], [0, -1.8, 0], [0, -1.8, 0.5],
    # Nose point (for identification of top/front)
    [0, 2, 0.1]  # Pointed nose
])

# Edges for airplane
bed_edges = [
    # Fuselage rear
    (0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7),
    # Fuselage front
    (8, 9), (9, 10), (10, 11), (11, 8), (12, 13), (13, 14), (14, 15), (15, 12), (8, 12), (9, 13), (10, 14), (11, 15),
    # Connect rear to front
    (2, 10), (3, 11), (6, 14), (7, 15),
    # Wings
    (16, 17), (18, 19), (16, 18), (17, 19), (16, 2), (17, 1),  # Connect wings to fuselage
    # Horizontal tail
    (20, 21), (22, 23), (20, 22), (21, 23), (20, 0), (21, 1),  # Connect to fuselage
    # Vertical tail fin
    (24, 25), (26, 27), (24, 26), (25, 27), (24, 0), (26, 2),  # Connect to fuselage
    # Nose point
    (28, 9), (28, 13), (28, 8), (28, 12)  # Connect nose to front
]

# Initial plot lines for airplane edges, colored to distinguish nose (top/front) and tail (bottom/rear)
bed_lines = []
for i, (v1, v2) in enumerate(bed_edges):
    color = 'r' if i >= 24 else 'b' if i < 12 else 'g'  # Red for nose, blue for fuselage, green for wings/tail
    line = ax1.plot([], [], [], color + '-')[0]
    bed_lines.append(line)

# Bottom subplot: Time-series for pitch, roll, yaw
ax2 = fig.add_subplot(2, 1, 2)
(line_pitch,) = ax2.plot([], [], label="Pitch (°)")
(line_roll,)  = ax2.plot([], [], label="Roll (°)")
(line_yaw,)   = ax2.plot([], [], label="Yaw (°)")
ax2.set_xlim(0, WINDOW)
ax2.set_ylim(-180, 180)
ax2.set_xlabel("Samples")
ax2.set_ylabel("Angle (°)")
ax2.set_title("Pitch (Y), Roll (X), Yaw (Z) Time Series")
ax2.legend(loc="upper right")

def rotation_matrix_x(angle_deg):
    angle_rad = math.radians(angle_deg)
    return np.array([
        [1, 0, 0],
        [0, math.cos(angle_rad), -math.sin(angle_rad)],
        [0, math.sin(angle_rad), math.cos(angle_rad)]
    ])

def rotation_matrix_y(angle_deg):
    angle_rad = math.radians(angle_deg)
    return np.array([
        [math.cos(angle_rad), 0, math.sin(angle_rad)],
        [0, 1, 0],
        [-math.sin(angle_rad), 0, math.cos(angle_rad)]
    ])

def rotation_matrix_z(angle_deg):
    angle_rad = math.radians(angle_deg)
    return np.array([
        [math.cos(angle_rad), -math.sin(angle_rad), 0],
        [math.sin(angle_rad), math.cos(angle_rad), 0],
        [0, 0, 1]
    ])

def apply_rotations(vertices, pitch, roll, yaw):
    # Apply rotations: Yaw (Z), then Pitch (Y), then Roll (X) - standard Euler order
    rot_z = rotation_matrix_z(yaw)
    rot_y = rotation_matrix_y(pitch)
    rot_x = rotation_matrix_x(roll)
    rot_combined = np.dot(rot_z, np.dot(rot_y, rot_x))
    return np.dot(vertices, rot_combined.T)

def parse_line(line):
    try:
        parts = line.strip().split(',')
        if len(parts) != 3:
            return None, None, None
        pitch = float(parts[0])
        roll = float(parts[1])
        yaw = float(parts[2])
        return pitch, roll, yaw
    except:
        return None, None, None

def init():
    for line in bed_lines:
        line.set_data([], [])
        line.set_3d_properties([])
    line_pitch.set_data([], [])
    line_roll.set_data([], [])
    line_yaw.set_data([], [])
    return bed_lines + [line_pitch, line_roll, line_yaw]

def update(frame):
    # Read serial data quickly
    for _ in range(5):
        raw = ser.readline().decode(errors='ignore')
        if not raw:
            break
        pitch, roll, yaw = parse_line(raw)
        if pitch is None:
            continue
        pitch_buf.append(pitch)
        roll_buf.append(roll)
        yaw_buf.append(yaw)
        x_idx.append(len(x_idx) + 1 if x_idx else 1)

    # Update 3D airplane
    if pitch_buf:
        current_pitch = pitch_buf[-1]
        current_roll = roll_buf[-1]
        current_yaw = yaw_buf[-1]
        rotated_vertices = apply_rotations(bed_vertices, current_pitch, current_roll, current_yaw)
        
        # Update airplane edges
        for idx, (v1, v2) in enumerate(bed_edges):
            x = [rotated_vertices[v1][0], rotated_vertices[v2][0]]
            y = [rotated_vertices[v1][1], rotated_vertices[v2][1]]
            z = [rotated_vertices[v1][2], rotated_vertices[v2][2]]
            bed_lines[idx].set_data(x, y)
            bed_lines[idx].set_3d_properties(z)
    
    # Update time-series
    xs = list(range(len(x_idx)))
    line_pitch.set_data(xs, list(pitch_buf))
    line_roll.set_data(xs, list(roll_buf))
    line_yaw.set_data(xs, list(yaw_buf))

    # Lock x-limits to WINDOW
    ax2.set_xlim(max(0, len(xs)-WINDOW), max(WINDOW, len(xs)))
    ax2.set_ylim(-180, 180)

    return bed_lines + [line_pitch, line_roll, line_yaw]

ani = animation.FuncAnimation(fig, update, init_func=init, interval=30, blit=False)  # blit=False for 3D
plt.tight_layout()
plt.show()
