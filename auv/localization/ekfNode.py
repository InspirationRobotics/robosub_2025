#!/usr/bin/env python3
import numpy as np
import rospy
import time
from statistics import mean
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Mavlink
from transforms3d.euler import euler2mat

class EKF6State:
    def __init__(self, dt):
        # State vector: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1))

        # State covariance
        self.P = np.eye(6) * 0.1

        # Process noise
        self.Q = np.diag([0.01]*3 + [0.3]*3)

        # Measurement noise (DVL)
        self.R_dvl = np.eye(3) * 0.05

        # Measurement noise (barometer)
        self.R_baro = np.array([[0.05]])

        self.dt = dt

    def predict(self):
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update_dvl(self, z):
        H = np.zeros((3, 6))
        H[0, 3] = 1
        H[1, 4] = 1
        H[2, 5] = 1

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_dvl
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_depth(self, depth):
        H = np.zeros((1, 6))
        H[0, 2] = 1  # z position

        z = np.array([[depth]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

class EKFNode:
    def __init__(self):
        rospy.init_node("ekf_6d_node")
        self.dt = 1.0 / 50.0  # 50Hz

        self.ekf = EKF6State(self.dt)

        self.dvl_velocity = np.zeros((3, 1))
        self.imu_acc_data = {"ax": 0, "ay": 0, "az": 0}
        self.imu_ori_data = {"yaw": 0, "pitch": 0, "roll": 0}

        self.depth = None
        self.depth_calib = 0
        self.calibrated = False

        self.pub = rospy.Publisher("/auv/state/pose", PoseStamped, queue_size=10)

        self.imu_sub = rospy.Subscriber("/auv/devices/vectornav", Imu, self.imu_callback)
        self.dvl_sub = rospy.Subscriber("/auv/devices/dvl/velocity", TwistStamped, self.dvl_callback)
        self.baro_sub = rospy.Subscriber("/mavlink/from", Mavlink, self.barometer_callback)

        self.calibrate_depth()
        rospy.Timer(rospy.Duration(self.dt), self.ekf_step)

    def imu_callback(self, msg):
        self.imu_acc_data["ax"] = msg.linear_acceleration.x
        self.imu_acc_data["ay"] = msg.linear_acceleration.y
        self.imu_acc_data["az"] = msg.linear_acceleration.z

        """
        Since our IMU outputs orientation as Euler angles (yaw, pitch, roll), and the ROS sensor_msgs/Imu message only supports orientation in quaternion format, I’ve been passing the yaw, pitch, and roll directly into the ZYX fields of the message, and leaving the quaternion w field empty.
        This obviously isn't correct, but I was doing it as a temporary workaround to get a precise rotation matrix — just plugging in the angles without properly converting them to a valid quaternion.
        """
        self.imu_ori_data['roll'] = msg.orientation.x
        self.imu_ori_data['pitch'] = (msg.orientation.y + 180) % 360
        self.imu_ori_data['yaw'] = msg.orientation.z

    def dvl_callback(self, msg):
        yaw = np.deg2rad(self.imu_ori_data['yaw'])
        pitch = np.deg2rad(self.imu_ori_data['pitch'])
        roll = np.deg2rad(self.imu_ori_data['roll'])

        rot_matrix = euler2mat(ai=yaw, aj=pitch, ak=roll, axes='szyx')  # Body-to-world rotation
        self.dvl_velocity = rot_matrix @ np.array([
            [msg.twist.linear.x],
            [msg.twist.linear.y],
            [msg.twist.linear.z]
        ])

    def barometer_callback(self, msg):
        try:
            if msg.msgid == 143:
                from struct import pack, unpack
                p = pack("QQ", *msg.payload64)
                _, press_abs, _, _ = unpack("Iffhxx", p)
                self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib
        except:
            pass

    def ekf_step(self, event):
        self.ekf.predict()
        self.ekf.update_dvl(self.dvl_velocity)
        if self.depth is not None and self.calibrated:
            self.ekf.update_depth(self.depth)
        self.publish_pose()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = self.ekf.x[0, 0]
        pose_msg.pose.position.y = self.ekf.x[1, 0]
        pose_msg.pose.position.z = self.ekf.x[2, 0]

        pose_msg.pose.orientation.x = self.imu_ori_data['roll']
        pose_msg.pose.orientation.y = self.imu_ori_data['pitch']
        pose_msg.pose.orientation.z = self.imu_ori_data['yaw']
        pose_msg.pose.orientation.w = 1.0
        self.pub.publish(pose_msg)

    def calibrate_depth(self, sample_time=3):
        """
        To calibrate the depth data

        Args:
            sample_time (int): The number of seconds taken to calibrate the data        
        """
        rospy.loginfo("Starting Depth Calibration...")
        samples = []

        # Wait for depth data
        while self.depth is None:
            rospy.sleep(0.1)

        prev_depth = self.depth
        start_time = time.time()

        # Collect data for sample_time seconds, then calculate the mean
        while time.time() - start_time < sample_time:
            if self.depth == prev_depth:
                continue
            samples.append(self.depth)
            prev_depth = self.depth

        self.depth_calib = mean(samples)
        self.calibrated = True
        rospy.loginfo(f"depth calibration Finished. Surface is: {self.depth_calib}")

if __name__ == "__main__":
    node = EKFNode()
    rospy.sleep(2)
    rospy.loginfo("Running the simple ekf node")
    rospy.spin()