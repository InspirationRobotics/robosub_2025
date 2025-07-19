import numpy as np
from transforms3d.euler import euler2quat

def get_true_heading(q):
    """Extracts yaw (heading) directly from quaternion, avoiding gimbal lock."""
    w, x, y, z = q
    # Project quaternion to XY plane (discard roll/pitch influence)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)  # Range: [-π, π]
    return np.rad2deg(yaw) % 360  # Convert to [0°, 360°)

# Your input (converted to radians)
roll_rad = np.deg2rad(1.7)
pitch_rad = np.deg2rad(179.96)
yaw_rad = np.deg2rad(64)

# Convert to quaternion (ZYX order)
q = euler2quat(yaw_rad, pitch_rad, roll_rad, axes='szyx')

# Get true heading
true_heading = get_true_heading(q)
print(f"True heading (gimbal-lock safe): {true_heading:.2f}°")