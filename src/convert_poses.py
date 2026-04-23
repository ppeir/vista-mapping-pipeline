import json
import math

INPUT_FILE = "data/outputs/outdoor/rtabmap_poses.txt"
OUTPUT_FILE = "data/outputs/outdoor/positions.json"

# Align the trajectory with the 2D plane
GLOBAL_YAW_DEG = 90.0

# Conversion Optical Frame -> Base Frame (Standard ROS)
LOCAL_ROLL_DEG = 0.0
LOCAL_PITCH_DEG = -90.0
LOCAL_YAW_DEG = 0.0

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def rotate_2d(x, y, angle_deg):
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a

def process_poses():
    poses = []
    first_timestamp = None

    q_global = euler_to_quaternion(0.0, 0.0, GLOBAL_YAW_DEG)
    q_local = euler_to_quaternion(LOCAL_ROLL_DEG, LOCAL_PITCH_DEG, LOCAL_YAW_DEG)

    with open(INPUT_FILE, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue

            parts = line.strip().split()
            if len(parts) < 8:
                continue

            ts = float(parts[0])

            if first_timestamp is None:
                first_timestamp = ts

            x_orig = float(parts[1])
            y_orig = float(parts[2])
            z = float(parts[3])

            q_orig = [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]

            # Spatial rotation (XY)
            x_new, y_new = rotate_2d(x_orig, y_orig, GLOBAL_YAW_DEG)

            # Orientation rotation
            q_rotated_global = quaternion_multiply(q_global, q_orig)
            q_final = quaternion_multiply(q_rotated_global, q_local)

            poses.append({
                "timestamp": ts - first_timestamp,
                "x": x_new,
                "y": y_new,
                "z": z,
                "qx": q_final[0],
                "qy": q_final[1],
                "qz": q_final[2],
                "qw": q_final[3]
            })

    with open(OUTPUT_FILE, 'w') as f:
        json.dump(poses, f, indent=2)

if __name__ == "__main__":
    process_poses()
import json
import math

INPUT_FILE = "data/outputs/outdoor/rtabmap_poses.txt"
OUTPUT_FILE = "data/outputs/outdoor/positions.json"

# Align the trajectory with the 2D plane
GLOBAL_YAW_DEG = 90.0

# Conversion Optical Frame -> Base Frame (Standard ROS)
LOCAL_ROLL_DEG = 0.0
LOCAL_PITCH_DEG = -90.0  # ou 90.0 si la flèche pointe vers l'arrière
LOCAL_YAW_DEG = 0.0

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def rotate_2d(x, y, angle_deg):
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a

def process_poses():
    poses = []
    first_timestamp = None

    q_global = euler_to_quaternion(0.0, 0.0, GLOBAL_YAW_DEG)
    q_local = euler_to_quaternion(LOCAL_ROLL_DEG, LOCAL_PITCH_DEG, LOCAL_YAW_DEG)

    with open(INPUT_FILE, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
                
            parts = line.strip().split()
            if len(parts) < 8:
                continue
                
            ts = float(parts[0])
            
            if first_timestamp is None:
                first_timestamp = ts
                
            x_orig = float(parts[1])
            y_orig = float(parts[2])
            z = float(parts[3])
            
            q_orig = [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]

            # Spatial rotation (XY)
            x_new, y_new = rotate_2d(x_orig, y_orig, GLOBAL_YAW_DEG)

            # Orientation rotation
            q_rotated_global = quaternion_multiply(q_global, q_orig)
            q_final = quaternion_multiply(q_rotated_global, q_local)

            poses.append({
                "timestamp": ts - first_timestamp,
                "x": x_new,
                "y": y_new,
                "z": z,
                "qx": q_final[0],
                "qy": q_final[1],
                "qz": q_final[2],
                "qw": q_final[3]
            })

    with open(OUTPUT_FILE, 'w') as f:
        json.dump(poses, f, indent=2)

if __name__ == "__main__":
    process_poses()