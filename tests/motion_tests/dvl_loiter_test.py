#!/usr/bin/env python3
"""
Combined Depth Hold + Station Keeping Script
- Starts depth hold at current depth (for Onyx, at line 241 logic)
- Launches DVL-based station keeping loop (x and y axes)
- Clean shutdown after 5 minutes
"""

import rospy
import time
import threading
from auv.motion.robot_control import RobotControl
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from tf.transformations import euler_from_quaternion


class DVLLoiter:
    """Minimal DVL station-keeping reimported and reusable."""

    ERROR_THRESHOLD = 0.05
    POSE_TIMEOUT = 2.0
    CONTROL_RATE_HZ = 10

    def __init__(self, rc: RobotControl):
        self.rc = rc
        self.running = True
        self.pose_received = False
        self.last_pose_time = time.time()

        self.target = {
            'x': self.rc.position['x'],
            'y': self.rc.position['y']
        }

        self._setup_pids()

        rospy.Subscriber("/auv/devices/dvl/position", Vector3Stamped, self._position_callback)
        rospy.Subscriber("/auv/devices/dvl/orientation", QuaternionStamped, self._orientation_callback)

        self.thread = threading.Thread(target=self._station_keep_loop, daemon=True)
        self.thread.start()

    def _setup_pids(self):
        self.pid_x = self.rc.PIDs['lateral']
        self.pid_y = self.rc.PIDs['surge']
        self.pid_x.setpoint = self.target['x']
        self.pid_y.setpoint = self.target['y']

    def _position_callback(self, msg: Vector3Stamped):
        self.rc.position['x'] = msg.vector.x
        self.rc.position['y'] = msg.vector.y
        self.rc.position['z'] = msg.vector.z
        self.pose_received = True
        self.last_pose_time = time.time()

    def _orientation_callback(self, msg: QuaternionStamped):
        quat = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.rc.orientation.update({'roll': roll, 'pitch': pitch, 'yaw': yaw})

    def _station_keep_loop(self):
        rospy.loginfo("[StationKeep] Waiting for initial DVL pose...")
        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("[StationKeep] Loop active.")
        rate = rospy.Rate(self.CONTROL_RATE_HZ)

        while self.running and not rospy.is_shutdown():
            # Timeout check
            if time.time() - self.last_pose_time > self.POSE_TIMEOUT:
                rospy.logwarn("[StationKeep] Pose timeout. Halting lateral/forward...")
                self.rc.movement(lateral=0.0, forward=0.0)
                rate.sleep()
                continue

            delta_x = self.pid_x(self.rc.position['x'])
            delta_y = self.pid_y(self.rc.position['y'])

            if abs(delta_x) > self.ERROR_THRESHOLD or abs(delta_y) > self.ERROR_THRESHOLD:
                self.rc.movement(lateral=delta_x, forward=delta_y)
            else:
                self.rc.movement(lateral=0.0, forward=0.0)

            rate.sleep()

    def stop(self):
        rospy.loginfo("[StationKeep] Stopping...")
        self.running = False
        self.rc.movement(lateral=0.0, forward=0.0)


def main():
    rospy.init_node("depth_hold_then_station_keeper")

    rc = RobotControl(debug=True)
    rospy.loginfo("[Main] RobotControl initialized for Onyx.")

    # === Perform Depth Hold (line 241 logic) ===
    current_depth = rc.position['z']
    rc.set_absolute_z(current_depth)
    rc.set_control_mode("depth_hold")
    rospy.loginfo(f"[Main] Holding depth at z = {current_depth:.2f} m")

    # === Run Station Keeping Thread ===
    loiter = DVLLoiter(rc)

    # === Operate for 5 minutes ===
    runtime_sec = 300
    rospy.loginfo("[Main] Running for 5 minutes...")
    start_time = time.time()

    try:
        while time.time() - start_time < runtime_sec and not rospy.is_shutdown():
            time.sleep(1)
    except KeyboardInterrupt:
        rospy.loginfo("[Main] Interrupted early.")

    # === Shutdown All Safely ===
    loiter.stop()
    rc.exit()
    rospy.loginfo("[Main] Full shutdown complete.")


if __name__ == "__main__":
    main()
