import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import time

# ----- CONFIG -----
PORT = 'COM6'  # Change this to your Arduino's COM port
BAUD = 115200  # Must match Arduino Serial.begin(115200)
WINDOW = 50
UPDATE_INTERVAL = 50  # 20 FPS

# Serial connection with fallback
ser = None
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Connected to {PORT} at {BAUD} baud")
except serial.SerialException as e:
    print(f"Error opening serial port {PORT}: {e}")
    print("Running in simulation mode...")
    ser = None

# Data arrays
pitch_data = np.zeros(WINDOW)
roll_data = np.zeros(WINDOW)
yaw_data = np.zeros(WINDOW)
idx = 0

# Simulation data for when serial is not available
sim_time = 0
simulation_active = ser is None

# ----- FIGURE -----
fig = plt.figure(figsize=(12, 8))
fig.suptitle('Dynamic Orientation Visualization', fontsize=16, fontweight='bold')

# 2D graph
ax1 = fig.add_subplot(2,1,1)
line_pitch, = ax1.plot(pitch_data, label="Pitch", color='blue', linewidth=2.5)
line_roll, = ax1.plot(roll_data, label="Roll", color='red', linewidth=2.5)
line_yaw, = ax1.plot(yaw_data, label="Yaw", color='green', linewidth=2.5)
ax1.set_ylim(-180, 180)
ax1.set_xlim(0, WINDOW)
ax1.set_xlabel("Samples", fontsize=12)
ax1.set_ylabel("Angle (°)", fontsize=12)
ax1.set_title("MPU6050 Orientation Data", fontsize=14, fontweight='bold')
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.legend(fontsize=11, loc='upper right')
ax1.set_facecolor('#f8f9fa')

# 3D car
ax2 = fig.add_subplot(2,1,2, projection='3d')
ax2.set_xlim(-2.5, 2.5)
ax2.set_ylim(-2.5, 2.5)
ax2.set_zlim(-1.5, 1.5)
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_zticks([])
ax2.set_title("3D Car Orientation", fontsize=14, fontweight='bold')
ax2.view_init(elev=25, azim=-45)
ax2.set_facecolor('#f0f0f0')

# Add axis labels for better understanding
ax2.text(2.2, 0, 0, 'X', fontsize=12, color='red', fontweight='bold')
ax2.text(0, 2.2, 0, 'Y', fontsize=12, color='green', fontweight='bold')
ax2.text(0, 0, 1.3, 'Z', fontsize=12, color='blue', fontweight='bold')

# ----- SIMPLE BUT VISIBLE CAR MODEL -----
# Create a clear, visible car that rotates properly
car_length, car_width, car_height = 1.5, 0.8, 0.4

# Simple car body vertices (8 vertices for a box with roof)
verts = np.array([
    # Bottom face
    [-car_length/2, -car_width/2, 0],  # 0
    [ car_length/2, -car_width/2, 0],  # 1
    [ car_length/2,  car_width/2, 0],  # 2
    [-car_length/2,  car_width/2, 0],  # 3
    # Top face
    [-car_length/2, -car_width/2, car_height],  # 4
    [ car_length/2, -car_width/2, car_height],  # 5
    [ car_length/2,  car_width/2, car_height],  # 6
    [-car_length/2,  car_width/2, car_height],  # 7
])

# Car faces (6 faces for a box)
faces = [
    [0,1,2,3],  # bottom
    [4,5,6,7],  # top
    [0,1,5,4],  # front
    [2,3,7,6],  # back
    [1,2,6,5],  # right side
    [0,3,7,4],  # left side
]

# Create the main car body
car_poly = Poly3DCollection([verts[f] for f in faces], 
                           facecolors='#FF4444',  # Bright red
                           edgecolors='#000000',  # Black edges
                           linewidths=2, 
                           alpha=0.9)
ax2.add_collection3d(car_poly)

# Add simple wheels as small spheres (much simpler)
wheel_positions = [
    [-car_length/2 + 0.2, -car_width/2 - 0.1, 0.1],
    [ car_length/2 - 0.2, -car_width/2 - 0.1, 0.1],
    [-car_length/2 + 0.2,  car_width/2 + 0.1, 0.1],
    [ car_length/2 - 0.2,  car_width/2 + 0.1, 0.1]
]

# Create simple wheel representations
wheel_polys = []
for i, (x, y, z) in enumerate(wheel_positions):
    # Simple wheel as a small box
    wheel_verts = np.array([
        [x-0.05, y-0.05, z-0.05],  # 0
        [x+0.05, y-0.05, z-0.05],  # 1
        [x+0.05, y+0.05, z-0.05],  # 2
        [x-0.05, y+0.05, z-0.05],  # 3
        [x-0.05, y-0.05, z+0.05],  # 4
        [x+0.05, y-0.05, z+0.05],  # 5
        [x+0.05, y+0.05, z+0.05],  # 6
        [x-0.05, y+0.05, z+0.05],  # 7
    ])
    
    wheel_faces = [
        [0,1,2,3],  # bottom
        [4,5,6,7],  # top
        [0,1,5,4],  # front
        [2,3,7,6],  # back
        [1,2,6,5],  # right
        [0,3,7,4],  # left
    ]
    
    wheel_poly = Poly3DCollection([wheel_verts[f] for f in wheel_faces], 
                                 facecolors='#333333',  # Dark gray
                                 edgecolors='#000000', 
                                 alpha=0.8)
    ax2.add_collection3d(wheel_poly)
    wheel_polys.append(wheel_poly)

# ----- ROTATION -----
def rotation_matrix(pitch, roll, yaw):
    """Create rotation matrix using proper Euler angle sequence (ZYX)"""
    pitch, roll, yaw = np.radians([pitch, roll, yaw])
    
    # Rotation around X-axis (pitch)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ])
    
    # Rotation around Y-axis (roll) 
    Ry = np.array([
        [np.cos(roll), 0, np.sin(roll)],
        [0, 1, 0],
        [-np.sin(roll), 0, np.cos(roll)]
    ])
    
    # Rotation around Z-axis (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: R = Rz * Ry * Rx (ZYX order)
    return Rz @ Ry @ Rx

def update_car(pitch, roll, yaw):
    """Update car orientation with proper 3D rotation"""
    R = rotation_matrix(pitch, roll, yaw)
    
    # Update main car body
    rotated_verts = verts @ R.T
    car_poly.set_verts([rotated_verts[f] for f in faces])
    
    # Update wheels
    for i, wheel_poly in enumerate(wheel_polys):
        # Get wheel vertices (simple box)
        wheel_verts = np.array([
            [wheel_positions[i][0]-0.05, wheel_positions[i][1]-0.05, wheel_positions[i][2]-0.05],
            [wheel_positions[i][0]+0.05, wheel_positions[i][1]-0.05, wheel_positions[i][2]-0.05],
            [wheel_positions[i][0]+0.05, wheel_positions[i][1]+0.05, wheel_positions[i][2]-0.05],
            [wheel_positions[i][0]-0.05, wheel_positions[i][1]+0.05, wheel_positions[i][2]-0.05],
            [wheel_positions[i][0]-0.05, wheel_positions[i][1]-0.05, wheel_positions[i][2]+0.05],
            [wheel_positions[i][0]+0.05, wheel_positions[i][1]-0.05, wheel_positions[i][2]+0.05],
            [wheel_positions[i][0]+0.05, wheel_positions[i][1]+0.05, wheel_positions[i][2]+0.05],
            [wheel_positions[i][0]-0.05, wheel_positions[i][1]+0.05, wheel_positions[i][2]+0.05],
        ])
        
        # Rotate wheel vertices
        rotated_wheel_verts = wheel_verts @ R.T
        
        # Update wheel faces
        wheel_faces = [
            [0,1,2,3], [4,5,6,7], [0,1,5,4], [2,3,7,6], [1,2,6,5], [0,3,7,4]
        ]
        wheel_poly.set_verts([rotated_wheel_verts[f] for f in wheel_faces])

def generate_simulation_data():
    """Generate realistic simulation data for testing"""
    global sim_time
    sim_time += 0.1
    
    # Create realistic motion patterns
    pitch = 15 * np.sin(sim_time * 0.5) + 5 * np.sin(sim_time * 2.1)
    roll = 20 * np.cos(sim_time * 0.3) + 8 * np.sin(sim_time * 1.7)
    yaw = 10 * np.sin(sim_time * 0.2) + 3 * np.cos(sim_time * 3.2)
    
    return pitch, roll, yaw

def parse_line(line):
    """Parse Arduino CSV data: pitch,roll,yaw"""
    try:
        # Arduino sends: pitch,roll,yaw (degrees)
        parts = line.strip().split(",")
        if len(parts) == 3:
            pitch = float(parts[0])
            roll = float(parts[1]) 
            yaw = float(parts[2])
            return pitch, roll, yaw
        else:
            return None, None, None
    except (ValueError, IndexError):
        return None, None, None

# ----- ANIMATION -----
def init():
    line_pitch.set_ydata(np.zeros(WINDOW))
    line_roll.set_ydata(np.zeros(WINDOW))
    line_yaw.set_ydata(np.zeros(WINDOW))
    update_car(0, 0, 0)
    return line_pitch, line_roll, line_yaw, car_poly

def update(frame):
    global pitch_data, roll_data, yaw_data, idx, simulation_active
    
    if simulation_active:
        # Use simulation data
        p, r, y = generate_simulation_data()
        pitch_data = np.roll(pitch_data, -1); pitch_data[-1] = p
        roll_data = np.roll(roll_data, -1); roll_data[-1] = r
        yaw_data = np.roll(yaw_data, -1); yaw_data[-1] = y
        idx += 1
    else:
        # Use serial data
        for _ in range(5):  # prevent serial buffer lag
            try:
                raw = ser.readline().decode(errors='ignore')
                if raw:
                    p, r, y = parse_line(raw)
                    if p is not None:
                        pitch_data = np.roll(pitch_data, -1); pitch_data[-1] = p
                        roll_data = np.roll(roll_data, -1); roll_data[-1] = r
                        yaw_data = np.roll(yaw_data, -1); yaw_data[-1] = y
                        idx += 1
                        break
            except:
                # If serial fails, switch to simulation
                simulation_active = True
                print("Serial connection lost, switching to simulation mode")
                break
    
    # Update graph
    line_pitch.set_ydata(pitch_data)
    line_roll.set_ydata(roll_data)
    line_yaw.set_ydata(yaw_data)
    ax1.set_xlim(max(0, idx-WINDOW), idx)
    
    # Update 3D car
    update_car(pitch_data[-1], roll_data[-1], yaw_data[-1])
    
    return line_pitch, line_roll, line_yaw, car_poly

ani = FuncAnimation(fig, update, init_func=init, interval=UPDATE_INTERVAL, blit=True, cache_frame_data=False)
plt.tight_layout()

print("Starting Dynamic Orientation Visualization...")
print("=" * 50)
if simulation_active:
    print("🔧 RUNNING IN SIMULATION MODE")
    print("   (No Arduino connected - using simulated data)")
    print("   To use real Arduino data:")
    print("   1. Upload Dynamic_orientation.ino to your Arduino")
    print("   2. Connect Arduino to COM6 (or change PORT in code)")
    print("   3. Run this script again")
else:
    print("🔌 ARDUINO CONNECTED")
    print("   Reading MPU6050 data from Arduino...")
print("=" * 50)
print("Controls:")
print("- Close the window to exit")
print("- The 3D car will rotate based on orientation data")
print("- Graph shows real-time pitch, roll, and yaw values")
print("- Move your Arduino to see the car rotate!")

try:
    plt.show()
finally:
    if ser is not None:
        ser.close()
        print("Serial connection closed")
