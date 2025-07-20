#!/usr/bin/env python2
"""
DVL Loitering with Integrated Depth Hold and PID Control (Python 2 Compatible)

This script integrates:
- DVL velocity-based position estimation (X, Y)
- Setpoint-maintaining PID control
- Direct PWM publishing to /mavros/rc/override

Standalone and ROS-native under Melodic's Python 2 runtime.
"""

import rospy
import time
import math
import threading

from simple_pid import PID
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from mavros_msgs.msg import OverrideRCIn
from tf.transformations import euler_from_quaternion

# ROS rate
CONTROL_RATE_HZ = 10

# DVLLoiter with integrated robot control functionality
class DVLLoiter(object):
    def __init__(self):
        rospy.init_node("dvl_loiter_node")

        # General settings
        self.running = True
        self.pose_received = False
        self.prev_time = None
        self.last_pose_time = time.time()
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.position_est = self.position.copy()

        # Orientation for future use (roll/pitch/yaw)
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Publishers
        self.pwm_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

        # PID Controllers
        self.pid_x = PID(4.0, 0.01, 0.1, setpoint=0.0, output_limits=(-2, 2))
        self.pid_y = PID(4.0, 0.01, 0.1, setpoint=0.0, output_limits=(-2, 2))
        self.pid_z = PID(100, 10, 0.75, setpoint=0.0)  # For depth hold

        # Subscribe to DVL velocity
        rospy.Subscriber("/auv/devices/dvl/velocity", Vector3Stamped, self._velocity_callback)

        self.thread = threading.Thread(target=self._station_keep_loop)
        self.thread.setDaemon(True)
        self.thread.start()

        # Set initial depth
        self.set_depth_hold(0.6)

    def _velocity_callback(self, msg):
        now = time.time()
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        if dt <= 0:
            return

        self.pose_received = True

        # Update velocities
        self.velocity['vx'] = msg.vector.x
        self.velocity['vy'] = msg.vector.y
        self.velocity['vz'] = msg.vector.z

        # Integrate position
        self.position['x'] += self.velocity['vx'] * dt
        self.position['y'] += self.velocity['vy'] * dt
        self.position['z'] += self.velocity['vz'] * dt

        self.last_pose_time = now
        self.prev_time = now

    def set_depth_hold(self, target_depth):
        self.pid_z.setpoint = target_depth
        rospy.loginfo("[DVLLoiter] Depth hold set to %.2f m" % target_depth)

    def _station_keep_loop(self):
        rospy.loginfo("[DVLLoiter] Waiting for DVL data...")
        rate = rospy.Rate(CONTROL_RATE_HZ)

        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("[DVLLoiter] Starting station keep loop...")

        while self.running and not rospy.is_shutdown():
            if time.time() - self.last_pose_time > 2.0:
                rospy.logwarn("DVL data timeout. Stopping robot.")
                self._publish_pwm()
                rate.sleep()
                continue

            dx = self.pid_x(self.position['x'])
            dy = self.pid_y(self.position['y'])
            dz = self.pid_z(self.position['z']) / 80.0  # Convert to scaled PWM delta

            self._publish_pwm(forward=dy, lateral=dx, vertical=dz)
            rate.sleep()

    def _publish_pwm(self, yaw=0.0, pitch=0.0, roll=0.0, forward=0.0, lateral=0.0, vertical=0.0):
        """Convert velocity command to PWM and publish to /mavros/rc/override"""
        pwm = OverrideRCIn()
        pwm.channels = [1500] * 18

        pwm.channels[0] = int((pitch * 80) + 1500)
        pwm.channels[1] = int((roll * 80) + 1500)
        pwm.channels[2] = int((vertical * 80) + 1500)
        pwm.channels[3] = int((yaw * 80) + 1500)
        pwm.channels[4] = int((forward * 80) + 1500)
        pwm.channels[5] = int((lateral * 80) + 1500)

        # Clip
        for i in range(6):
            if pwm.channels[i] > 1900:
                pwm.channels[i] = 1900
            elif pwm.channels[i] < 1100:
                pwm.channels[i] = 1100

        self.pwm_pub.publish(pwm)

    def stop(self):
        self.running = False
        self._publish_pwm()
        rospy.loginfo("[DVLLoiter] Loitering stopped.")


if __name__ == "__main__":
    try:
        loiter = DVLLoiter()
        start = time.time()
        while not rospy.is_shutdown():
            if time.time() - start > 300:
                rospy.loginfo("[DVLLoiter] 5 minutes complete. Shutting down.")
                break
            time.sleep(1)

        loiter.stop()

    except rospy.ROSInterruptException:
        pass
