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
from geometry_msgs.msg import PoseStamped


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


class EKFLoiter:
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
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.desired_point = {'x': 0.0, 'y': 0.0, 'z': 0.6, 'yaw' : 0, 'pitch' : 0, 'roll' : 0}  # Desired depth hold at 0.6m
        self.prev_time = None
        self.dvl_error = 0.0
        self.error_integral = [0.0, 0.0, 0.0]

        rc.set_control_mode("depth_hold")
        
        self._setup_pids()
        # Subscribe to DVL velocity & orientation
        rospy.Subscriber("/auv/state/pose", PoseStamped, self._pose_callback, queue_size=10)

        self.thread = threading.Thread(target=self._station_keep_loop, daemon=True)
        self.thread.start()

    def _setup_pids(self):
        self.pid_x = self.rc.PIDs['lateral']
        self.pid_y = self.rc.PIDs['surge']
        self.pid_yaw = self.rc.PIDs['yaw']
        self.pid_pitch = self.rc.PIDs['pitch']
        self.pid_roll = self.rc.PIDs['roll']
        # Setpoints based on initial estimated position (0,0)
        self.pid_x.setpoint = self.desired_point['x']
        self.pid_y.setpoint = self.desired_point['y']
        self.pid_yaw.setpoint = self.desired_point['yaw']
        self.pid_pitch.setpoint = self.desired_point['pitch']
        self.pid_roll.setpoint = self.desired_point['roll']

    def _pose_callback(self, msg: PoseStamped):
        # Update latest fused position
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y

        # Extract quaternion
        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(quat)

        self.orientation['roll'] = roll
        self.orientation['pitch'] = pitch
        self.orientation['yaw'] = yaw
        
        self.pose_received = True
        self.last_pose_time = time.time()

    def _station_keep_loop(self):
        rospy.loginfo("[LOITER] Waiting for EKF pose input...")
        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("[LOITER] Starting station keeping loop with EKF pose...")
        rate = rospy.Rate(self.CONTROL_RATE_HZ)

        while self.running and not rospy.is_shutdown():
            if time.time() - self.last_pose_time > self.POSE_TIMEOUT:
                rospy.logwarn("[LOITER] Pose input timeout. Stopping.")
                self.rc.movement(lateral=0.0, forward=0.0)
                rate.sleep()
                continue

            # Use EKF fused position estimate
            x = self.position['x']
            y = self.position['y']
            
            yaw = self.orientation['yaw']
            pitch = self.orientation['pitch']
            roll = self.orientation['roll']

            # Calculate control efforts from PID controllers
            delta_x = self.pid_x(x)  # lateral
            delta_y = self.pid_y(y)  # forward
            delta_yaw = self.pid_yaw(yaw)  # yaw
            delta_pitch = self.pid_pitch(pitch)  # pitch
            delta_roll = self.pid_roll(roll)

            rospy.loginfo(f"[LOITER] EKF Position x={x:.3f}, y={y:.3f} | PID Cmd dx={delta_x:.2f}, dy={delta_y:.2f}")

            if abs(delta_x) > self.ERROR_THRESHOLD or abs(delta_y) > self.ERROR_THRESHOLD:
                self.rc.movement(lateral=delta_x, forward=delta_y, yaw = delta_yaw, pitch=delta_pitch, roll=delta_roll)
            else:
                self.rc.movement()

            rate.sleep()



    def stop(self):
        self.running = False
        self.rc.movement() # all motors to 1500 PWM, stop movement
        rospy.loginfo("[LOITER] Stopped station keeping loop.")


def main():
    rc = RobotControl()
    rospy.init_node("ekf_loiter_pos_and_orientation")
    rospy.loginfo("[Main] RobotControl initialized.")
    
    rc.desired_point['z'] = 0.6
    depth = rc.desired_point['z']
    rc.set_absolute_z(depth)
    
    # Depth hold
    time.sleep(10)  # Allow time for depth hold to stabilize
    rospy.loginfo(f"[Main] Holding depth at Z={depth:.2f} m")

    # Start station keeping (with velocity integration)
    loiter = EKFLoiter(rc)

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
