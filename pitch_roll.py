import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Configuration
SERIAL_PORT = 'COM3'  # Change to your serial port, e.g., '/dev/ttyUSB0' on Linux/Mac
BAUD_RATE = 115200    # Common baud rates: 9600, 115200, etc.
DATA_FORMAT = 'csv'   # Assume data is "pitch,roll,yaw\n" in degrees; adjust if needed

# Function to compute rotation matrix for pitch and roll (ignoring yaw)
def get_rotation_matrix(pitch_deg, roll_deg):
    pitch = np.deg2rad(pitch_deg)
    roll = np.deg2rad(roll_deg)
    
    # Rotation matrix for roll (around x-axis)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation matrix for pitch (around y-axis)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Combined rotation: pitch then roll (order can be adjusted based on convention)
    R = np.dot(Ry, Rx)
    return R

# Initial axis vectors (x: red, y: green, z: blue)
axes = np.eye(3)  # [[1,0,0], [0,1,0], [0,0,1]]

# Setup serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Setup 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Initial quivers (arrows for axes)
quivers = [
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=1.0, normalize=True),
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=1.0, normalize=True),
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=1.0, normalize=True)
]

def update(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            # Parse data (assume "pitch,roll,yaw" in degrees)
            pitch, roll, yaw = map(float, line.split(','))
            R = get_rotation_matrix(pitch, roll)
            
            # Rotate the axes
            rotated_axes = np.dot(R, axes.T).T
            
            # Update quivers
            for i, q in enumerate(quivers):
                u, v, w = rotated_axes[i]
                q.set_segments([[ [0,0,0], [u,v,w] ]])
            
            ax.set_title(f'Pitch: {pitch:.2f}°, Roll: {roll:.2f}°')
    except Exception as e:
        print(f"Error reading serial: {e}")
    
    return quivers

# Animate
ani = FuncAnimation(fig, update, interval=50)  # Update every 50ms

plt.show()

# Close serial when done (though plt.show() blocks)
ser.close()
