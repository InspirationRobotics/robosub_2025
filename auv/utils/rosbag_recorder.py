import rospy
import rosbag

class RosbagRecorder:
    # Adds logging functionality
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    self.bag_path = f"/home/inspiration/bags/buoy_mission_{timestamp}.bag"
    self.bag = rosbag.Bag(self.bag_path, 'w')
    
    # Create subscribers for sensor topics with correct message types
    self.subscribers = [
        rospy.Subscriber("/auv/devices/compass", std_msgs.msg.Float64, 
                       self.bag_callback, callback_args="/auv/devices/compass"),
        rospy.Subscriber("/auv/devices/imu", Imu, 
                       self.bag_callback, callback_args="/auv/devices/imu"),
        rospy.Subscriber("/auv/devices/baro", std_msgs.msg.Float32MultiArray, 
                       self.bag_callback, callback_args="/auv/devices/baro")
    ]