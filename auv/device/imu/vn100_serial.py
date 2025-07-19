import rospy
import time
import threading
import numpy as np

from serial import Serial

from auv.utils import deviceHelper
from sensor_msgs.msg import Imu

class VN100:
    def __init__(self, port: str = deviceHelper.dataFromConfig("vectornav")):
        """Makes a serial connection to the VN100 IMU utilizing deviceHelper and starts reading"""
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port, baudrate=self.__bps, timeout=1)
        rospy.init_node("imu_node", anonymous=True)  
        self.rate = rospy.Rate(40) # 40 Hz

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accX = 0.0
        self.accY = 0.0
        self.accZ = 0.0
        self.gyroX = 0.0
        self.gyroY = 0.0
        self.gyroZ = 0.0
        self.heading_offset = 0
        self.vectornav_pub = rospy.Publisher('/auv/devices/vectornav', Imu, queue_size=10)

        self.calibrated = False
        self.running = True  # Added for Ctrl+C protection

        time.sleep(2)
        self.lock = threading.Lock()
        self.read_thread = threading.Thread(target=self.read, daemon=True)
        self.read_thread.start()
        time.sleep(2)

        self.calibrate_heading()

    def read(self):
        """Parses roll, pitch, and yaw from the serial line"""
        while self.running and not rospy.is_shutdown():
            data_line = self.__ser.readline().decode()
            data_list = data_line.split(',')

            try :
                self.yaw, self.pitch, self.roll = (float(data_list[1]) + 90 + self.heading_offset) % 360, (float(data_list[3])+180)%360, float(data_list[2])

                self.accX, self.accY, self.accZ = -float(data_list[4]), -float(data_list[5]), -float(data_list[6].split('*')[0])  # remove check sum
                self.gyroX, self.gyroY, self.gyroZ = float(data_list[7]), float(data_list[8]), float(data_list[9].split('*')[0])

                if self.calibrated:
                    self.publish_data()
                self.rate.sleep()

            except IndexError as e:
                print(f"raw data: {data_list}")
                rospy.logdebug("first data received, causing index out of range")
                rospy.logerr(e)
            except Exception as e:
                print(f"raw data: {data_list}")
                rospy.logdebug(data_list)
                rospy.logerr(e)

    def calibrate_heading(self):
        rospy.loginfo("Starting heading calibration... Keep the IMU stationary!")
        samples = []
        start_time = time.time()
        self.calibrated = False
        
        while not rospy.is_shutdown() and time.time() - start_time < 3:
            try:
                with self.lock:
                    if self.yaw:
                        samples.append(self.yaw)
            except AttributeError as e:
                rospy.logwarn("attribute not assigned with value yet")
                rospy.logwarn(e)
                self.rate.sleep()
                continue
            
        self.heading_offset = 0 - np.mean(samples)
        self.calibrated = True
        rospy.loginfo("Calibration finished, current heading set to 0")
        
        
        

    def publish_data(self):
        """Published the IMU data to /auv/devices/vectornav"""
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"

        # print(f"quat orient is {self.quat_orient}")
        imu_msg.orientation.w = 0.0
        imu_msg.orientation.x = self.roll
        imu_msg.orientation.y = self.pitch
        imu_msg.orientation.z = self.yaw

        imu_msg.angular_velocity.x = self.gyroX
        imu_msg.angular_velocity.y = self.gyroY
        imu_msg.angular_velocity.z = self.gyroZ

        imu_msg.linear_acceleration.x = self.accX
        imu_msg.linear_acceleration.y = self.accY
        imu_msg.linear_acceleration.z = self.accZ

        self.vectornav_pub.publish(imu_msg)
        

    def shutdown(self):
        """Clean shutdown of threads and serial connection"""
        self.running = False
        if self.__ser.is_open:
            self.__ser.close()


if __name__ == "__main__":
    try:
        sensor = VN100()
        rospy.loginfo("VN100 node start running...")
        rospy.spin()

    except KeyboardInterrupt:
        sensor.shutdown()
        rospy.loginfo("Shutting down vn100 node")

    except AttributeError as e:
        rospy.logerr("Attribute error occur, most likely due to None type data")
        rospy.logerr(e)
    except ValueError:
        print("Bad data")
    except Exception as e:
        print(f"Generic exception caught: {e}")
