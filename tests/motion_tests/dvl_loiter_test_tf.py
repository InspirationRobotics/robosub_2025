#!/usr/bin/env python3
"""
Combined Depth Hold + Station Keeping Script
- Uses velocity integration from DVL to compute position
- Starts depth hold at current Z
- Runs station keeping on lateral + forward axes
- Shuts down cleanly after 5 minutes
"""

import rospy
import time
import threading
import math
from auv.motion.robot_control import RobotControl
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped


def euler_from_quaternion(quat):
    """
    Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
    Pure Python implementation (replaces tf.transformations).
    """
    x, y, z, w = quat

    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class DVLLoiter:
    """DVL-based station-keeping by integrating velocity into position."""

    ERROR_THRESHOLD = 0.05
    POSE_TIMEOUT = 2.0
    CONTROL_RATE_HZ = 10

    def __init__(self, rc: RobotControl):
        self.rc = rc
        self.running = True
        self.pose_received = False
        self.last_pose_time = time.time()

        # Initialize integrated position
        self.position_est = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.prev_time = None
        self.dvl_error = 0.0
        self.error_integral = [0.0, 0.0, 0.0]

        self._setup_pids()

        # Subscribe to DVL velocity & orientation
        rospy.Subscriber("/auv/devices/dvl/velocity", Vector3Stamped, self._velocity_callback)
        # Uncomment if orientation needed:
        # rospy.Subscriber("/auv/devices/dvl/orientation", QuaternionStamped, self._orientation_callback)

        self.thread = threading.Thread(target=self._station_keep_loop, daemon=True)
        self.thread.start()

    def _setup_pids(self):
        self.pid_x = self.rc.PIDs['lateral']
        self.pid_y = self.rc.PIDs['surge']
        # Setpoints based on initial estimated position (0,0)
        self.pid_x.setpoint = 0.0
        self.pid_y.setpoint = 0.0

    def _velocity_callback(self, msg: Vector3Stamped):
        current_time = time.time()

        vx = msg.vector.x
        vy = msg.vector.y
        vz = msg.vector.z

        # Default DVL error estimate
        dvl_error = 0.0
        self.pose_received = True

        if self.prev_time is None:
            self.prev_time = current_time
            rospy.loginfo("[DVLLoiter] Waiting for velocity stability before integrating.")
            return

        dt = current_time - self.prev_time
        if dt <= 0:
            rospy.logwarn("[DVLLoiter] Invalid dt, skipping integration.")
            return

        # Apply error correction if needed
        vx_err = vx + dvl_error
        vy_err = vy + dvl_error
        vz_err = vz + dvl_error

        self.velocity.update({'vx': vx, 'vy': vy, 'vz': vz})

        # Integrate position using dead reckoning
        self.position_est['x'] += vx * dt
        self.position_est['y'] += vy * dt
        self.position_est['z'] += vz * dt

        # Optional: accumulate integration error
        self.error_integral[0] += abs(vx - vx_err) * dt
        self.error_integral[1] += abs(vy - vy_err) * dt
        self.error_integral[2] += abs(vz - vz_err) * dt

        # Update RobotControl-used position
        self.rc.position.update(self.position_est)

        self.last_pose_time = current_time
        self.prev_time = current_time

    def _orientation_callback(self, msg: QuaternionStamped):
        quat = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.rc.orientation.update({'roll': roll, 'pitch': pitch, 'yaw': yaw})

    def _station_keep_loop(self):
        rospy.loginfo("[DVLLoiter] Waiting for DVL velocity input...")
        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("[DVLLoiter] Starting station keeping loop...")
        rate = rospy.Rate(self.CONTROL_RATE_HZ)

        while self.running and not rospy.is_shutdown():
            if time.time() - self.last_pose_time > self.POSE_TIMEOUT:
                rospy.logwarn("[DVLLoiter] Velocity input timeout. Stopping.")
                self.rc.movement(lateral=0.0, forward=0.0)
                rate.sleep()
                continue

            x = self.rc.position['x']
            y = self.rc.position['y']

            delta_x = self.pid_x(x)
            delta_y = self.pid_y(y)

            if abs(delta_x) > self.ERROR_THRESHOLD or abs(delta_y) > self.ERROR_THRESHOLD:
                self.rc.movement(lateral=delta_x, forward=delta_y)
            else:
                self.rc.movement(lateral=0.0, forward=0.0)

            rate.sleep()

    def stop(self):
        self.running = False
        self.rc.movement(lateral=0.0, forward=0.0)
        rospy.loginfo("[DVLLoiter] Stopped station keeping loop.")


def main():
    rospy.init_node("dvl_loiter_velocity_based")

    rc = RobotControl(debug=True)
    rospy.loginfo("[Main] RobotControl initialized.")
    
    rc.set_control_mode("depth_hold")
    target_depth = 0.6  # Set desired depth to 0.6m
    rc.set_absolute_z(target_depth)  # Set initial depth hold
    # Depth hold

    time.sleep(5)  # Allow time for depth hold to stabilize
    rospy.loginfo(f"[Main] Holding depth at Z={target_depth:.2f} m")

    # Start station keeping (with velocity integration)
    loiter = DVLLoiter(rc)

    # Run for 5 minutes
    duration_sec = 300
    rospy.loginfo(f"[Main] Running station keeping for {duration_sec} seconds.")
    start_time = time.time()

    try:
        while time.time() - start_time < duration_sec and not rospy.is_shutdown():
            time.sleep(1)
    except KeyboardInterrupt:
        rospy.loginfo("[Main] Stopping early (keyboard interrupt)")

    loiter.stop()
    rc.exit()

    rospy.loginfo("[Main] Full system shutdown complete.")


if __name__ == "__main__":
    main()
