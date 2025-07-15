import time
import threading
import rospy
import sensor_msgs.msg
import math
from transforms3d.euler import quat2euler
from auv.device.imu.vn100_serial import VN100

rospy.init_node("vectornav_ros_api", anonymous=True)

class ImuNode:
    def __init__(self):
        self.sensor = VN100()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vectornav_subscriber = rospy.Subscriber("/vectornav/IMU", sensor_msgs.msg.Imu, self.get_orientation)
        self.vectornav_publisher = rospy.Publisher("/auv/device/VN100", sensor_msgs.msg.Imu, queue_size=10)
        self.rate = rospy.Rate(40)

        # Start publishing in a background thread
        self.thread = threading.Thread(target=self.publishThread)
        self.thread.daemon = True
        self.thread.start()

    def get_orientation(self, msg):
        """Parses quaternion orientation to Euler angles (roll, pitch, yaw)"""
        self.quat_orient = msg.orientation
        orientation_list = [self.quat_orient.x, self.quat_orient.y, self.quat_orient.z, self.quat_orient.w]
        (roll, pitch, yaw) = quat2euler(orientation_list)

        # Convert to degrees and normalize
        self.roll = math.degrees(roll) % 360
        self.pitch = math.degrees(pitch) % 360
        self.yaw = math.degrees(yaw) % 360

        print(f"Roll: {self.roll:.2f}°\nPitch: {self.pitch:.2f}°\nYaw: {self.yaw:.2f}°")

    def publishThread(self):
        """Publishes orientation (quaternion), angular velocity, and linear acceleration
        to vectornav custom rostopic"""
        while not rospy.is_shutdown():
            imu_msg = sensor_msgs.msg.Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "vectornav"

            imu_msg.orientation.x = self.quat_orient.x
            imu_msg.orientation.y = self.quat_orient.y
            imu_msg.orientation.z = self.quat_orient.z
            imu_msg.orientation.w = self.quat_orient.w

            imu_msg.angular_velocity.x = self.sensor.gyroX
            imu_msg.angular_velocity.y = self.sensor.gyroY
            imu_msg.angular_velocity.z = self.sensor.gyroZ

            imu_msg.linear_acceleration.x = self.sensor.accX
            imu_msg.linear_acceleration.y = self.sensor.accY
            imu_msg.linear_acceleration.z = self.sensor.accZ

            self.vectornav_publisher.publish(imu_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        imu_node = ImuNode()
        rospy.spin()  # Keeps the node running until Ctrl+C

    except KeyboardInterrupt:
        print("\nShutting down gracefully...")

    finally:
        # Clean up if needed
        print("Node stopped.")
