#!/usr/bin/env python3
"""
DVL Loiter Control Node
Maintains AUV position using DVL-based feedback and PID controllers through RobotControl interface.
"""

import time
import rospy
from threading import Thread
from typing import Dict
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from tf.transformations import euler_from_quaternion

from auv.motion.robot_control import RobotControl


class DVLLoiter:
    """AUV station-keeping using DVL data and PID control."""

    ERROR_THRESHOLD: float = 0.05     # meters/radians
    POSE_TIMEOUT: float = 2.0         # seconds before considering pose data stale
    CONTROL_RATE_HZ: int = 10         # PID loop frequency

    def __init__(self) -> None:
        """Initialize DVL Loiter node, subscribers, and PID controllers."""
        rospy.init_node("dvl_loiter_node")

        # Initialize RobotControl
        self.rc = RobotControl(debug=True)

        # Target Position for station keeping
        self.target: Dict[str, float] = {'x': 0.0, 'y': 0.0}

        # State
        self.pose_received: bool = False
        self.last_pose_time: float = time.time()
        self.running: bool = True

        # Setup PID controller targets
        self._setup_pid_controllers()

        # Subscribers (still needed to trigger pose_updated flag even though rc handles pose)
        rospy.Subscriber("/auv/devices/dvl/position", Vector3Stamped, self._position_callback)
        rospy.Subscriber("/auv/devices/dvl/orientation", QuaternionStamped, self._orientation_callback)

        # Start control thread
        Thread(target=self._station_keep_loop, daemon=True).start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("DVL Loiter initialized.")

    def _setup_pid_controllers(self) -> None:
        """Assign initial setpoints to PID controllers from RobotControl."""
        self.pid_x = self.rc.PIDs['lateral']
        self.pid_y = self.rc.PIDs['surge']
        self.pid_z = self.rc.PIDs['depth']
        self.pid_yaw = self.rc.PIDs['yaw']
        self.pid_pitch = self.rc.PIDs['pitch']
        self.pid_roll = self.rc.PIDs['roll']

        self.pid_x.setpoint = self.target['x']
        self.pid_y.setpoint = self.target['y']
        self.pid_z.setpoint = 0.0         # Not used here
        self.pid_yaw.setpoint = 0.0       # Not used here

    def _position_callback(self, msg: Vector3Stamped) -> None:
        """Update internal position state via callback."""
        self.rc.position['x'] = msg.vector.x
        self.rc.position['y'] = msg.vector.y
        self.rc.position['z'] = msg.vector.z
        self.pose_received = True
        self.last_pose_time = time.time()

    def _orientation_callback(self, msg: QuaternionStamped) -> None:
        """Convert quaternion to Euler RPY and store."""
        quat = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.rc.orientation.update({'roll': roll, 'pitch': pitch, 'yaw': yaw})

    def _station_keep_loop(self) -> None:
        """Control loop for station keeping."""
        rospy.loginfo("Waiting for initial DVL pose...")
        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Starting DVL Station Keep...")
        rate = rospy.Rate(self.CONTROL_RATE_HZ)

        while self.running and not rospy.is_shutdown():
            # Watchdog timeout
            if time.time() - self.last_pose_time > self.POSE_TIMEOUT:
                rospy.logwarn("Pose timeout! Stopping all motion.")
                self.rc.movement(lateral=0.0, forward=0.0)
                rate.sleep()
                continue

            # Compute PID outputs
            delta_x: float = self.pid_x(self.rc.position['x'])
            delta_y: float = self.pid_y(self.rc.position['y'])
            # delta_z: float = self.pid_z(self.rc.position['z'])  # Future: depth hold
            # delta_yaw: float = self.pid_yaw(self.rc.orientation['yaw'])  # Future: yaw hold

            # Act only if error above control threshold
            if any(abs(delta) > self.ERROR_THRESHOLD for delta in [delta_x, delta_y]):
                self.rc.movement(lateral=delta_x, forward=delta_y)
            else:
                self.rc.movement(lateral=0.0, forward=0.0)

            rate.sleep()

    def shutdown(self) -> None:
        """Safely stop station keeping and motors."""
        rospy.loginfo("Shutting down DVL Station Keep...")
        self.running = False
        self.rc.movement(lateral=0.0, forward=0.0)
        rospy.loginfo("DVL Station Keep shutdown complete.")


if __name__ == "__main__":
    DVLLoiter()
    rospy.spin()
