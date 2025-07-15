"""
Handles the MAVROS link between Pixhawk and software
"""

# To obtain information about the Linux distribution
import lsb_release 

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")

"""
Importing necessary modules for platform information, signal handling, threading,
time, statistical analysis, converting between python values and C structs (struct)
"""
import platform
import signal
import threading
import time
from statistics import mean
from struct import pack, unpack

# Importing various message types for ROS
import geographic_msgs.msg
import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg
import std_msgs.msg

# Importing ROS, Numpy, PID controller
import numpy as np
import rospy
from simple_pid import PID

# For turning a status LED (based on a state) on and getting the configuration of a specified device, respectively
from auv.utils import statusLed, deviceHelper

# For handling ROS topics
from auv.utils.rospyHandler import RosHandler
from auv.utils.topicService import TopicService

# Import IMU
from auv.device.imu.vn100_serial import VN100

# Different modes/states of travel (predefined modes used by the Pixhawk)
MODE_MANUAL = "MANUAL"
MODE_STABILIZE = "STABILIZE"
MODE_ALTHOLD = "ALT_HOLD"
MODE_LOITER = "LOITER"
MODE_AUTO = "AUTO"
MODE_GUIDED = "GUIDED"
MODE_ACRO = "ACRO"

# Configuration of devices from either Graey or Onyx (depending)
config = deviceHelper.variables


class AUV(RosHandler):
    """
    Creates a class for the sub, inheriting from ROSHandler (all of the functions from RosHandler are now in AUV)
    """
    def __init__(self):
        # Accessing the device configurations
        self.config = config

        # Initiating construction of the RosHandler superclass
        super().__init__()

        # Attributes relating to status
        self.do_publish_thrusters = True
        self.do_get_sensors = True
        self.armed = False
        self.guided = False
        self.mode = ""

        # Hold the depth

        # Create neutral channels
        self.channels = [1500] * 18

        self.thrustTime = time.time()  # Reference timestamp for timeout for no new thruster commands
        
        # Depth status/values
        self.depth = None
        self.do_hold_depth = False
        self.depth_pwm = 0
        self.depth_calib = 0
        self.depth_pid_params = config.get("depth_pid_params", [0.5, 0.1, 0.1]) # Get PID controller parameters from config, if not found use default values (second arg)
        self.depth_pid_offset = config.get("depth_pid_offset", 1500) # Get PID offset from key, if not found set PWM value to default neutral (1500)
        
        # Initialize the depth PID controller
        self.depth_pid = PID(*self.depth_pid_params, setpoint=0.5) # Go to depth at 0.5 m
        self.depth_pid.output_limits = (-self.depth_pid_params[0], self.depth_pid_params[0]) # Output limits of PID controller

        # Add VN100 IMU
        self.vectornav = VN100()

        # Initialize topics from Pixhawk (through MAVROS)
        self.TOPIC_STATE        = TopicService("/mavros/state"      , mavros_msgs.msg.State)
        self.SERVICE_ARM        = TopicService("/mavros/cmd/arming" , mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE   = TopicService("/mavros/set_mode"   , mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM  = TopicService("/mavros/param/set"  , mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM  = TopicService("/mavros/param/get"  , mavros_msgs.srv.ParamGet)

        # Movement topics, only works/is applicable in autonomous mode
        self.TOPIC_SET_VELOCITY = TopicService("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist)
        self.TOPIC_SET_RC_OVR   = TopicService("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn)

        # Sensory data (IMU, compass, remote control input, input from flight controller, battery state)
        self.TOPIC_GET_IMU_DATA = TopicService("/mavros/imu/data", sensor_msgs.msg.Imu)
        self.TOPIC_GET_CMP_HDG  = TopicService("/mavros/global_position/compass_hdg", std_msgs.msg.Float64)
        self.TOPIC_GET_RC       = TopicService("/mavros/rc/in"  , mavros_msgs.msg.RCIn)
        self.TOPIC_GET_MAVBARO  = TopicService("/mavlink/from"  , mavros_msgs.msg.Mavlink)
        # https://discuss.bluerobotics.com/t/ros-support-for-bluerov2/1550/24
        self.TOPIC_GET_BATTERY  = TopicService("/mavros/battery"    , sensor_msgs.msg.BatteryState)

        # Custom ROS topics
        self.AUV_COMPASS = TopicService("/auv/devices/compass", std_msgs.msg.Float64)
        self.AUV_IMU = TopicService("/auv/devices/imu", sensor_msgs.msg.Imu)
        # ROS Topics for DVL
        self.AUV_DVL_Velocity = TopicService("/auv/devices/dvl/velocity", geometry_msgs.msg.Vector3Stamped)
        self.AUV_DVL_Position = TopicService("/auv/devices/dvl/position", geometry_msgs.msg.PointStamped)
        # ROS Topic for Barometer
        self.AUV_BARO = TopicService("/auv/devices/baro", std_msgs.msg.Float32MultiArray)
        self.AUV_GET_THRUSTERS = TopicService("/auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn)
        self.AUV_GET_DEPTH = TopicService("/auv/devices/setDepth", std_msgs.msg.Float64)
        self.AUV_GET_REL_DEPTH = TopicService("/auv/devices/setRelativeDepth", std_msgs.msg.Float64)
        self.AUV_GET_ARM = TopicService("/auv/status/arm", std_msgs.msg.Bool)
        self.AUV_GET_MODE = TopicService("/auv/status/mode", std_msgs.msg.String)

    def arm(self, status: bool):
        """
        Arms the sub

        Args:
            status (bool): Whether the sub will be armed or not

        Returns:
            result.success (bool): Whether the sub was able to be armed
            result.result (str): Containing extra information about the result of the arming operation
        """
        # Turning the red status LED on if sub is armed
        if status:
            statusLed.red(True)
        else:
            statusLed.red(False)
        # Creating a CommandBoolRequest message to arm/disarm the sub
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        # Set the data of the arming topic
        self.SERVICE_ARM.set_data(data)
        # Call the ROS service to arm the sub
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result

    def get_param(self, param: str):
        """
        To get a parameter value from the AUV

        Args:
            param (str): The parameter to get the value from

        Returns:
            result.success(bool) : Whether the parameter was able to be retrieved
            result.value.integer (int): The integer value of the parameter value
            result.value.real (float): The floating-point(real) value of the parameter value
        """
        # Create a ParamGetRequest message to retrieve the parameter value
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        # Setting the data for parameter retrieval service
        self.SERVICE_GET_PARAM.set_data(data)
        # Call the ROS service to get the value of the parameter
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param: str, value_integer: int, value_real: float):
        """
        To set a given parameter

        Args:
            param (str): The parameter to be changed/set
            value_integer (int): The value to set the parameter to
            value_real (float): The floating-point value to set the parameter to
        """
        # Create a ParamSetRequest message to set the parameter
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        # Setting the data for setting the parameter value
        self.SERVICE_SET_PARAM.set_data(data)
        # Call the ROS service to set the parameter value
        result = self.service_caller(self.SERVICE_SET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def change_mode(self, mode: str):
        """
        To change the mode of the sub

        Args:
            mode (str): The mode to change to
        
        Returns:
            result.mode_sent (str): The mode that was sent to autopilot to set the new mode
        """
        if not isinstance(mode, str):
            mode = str(mode)
            mode = mode[7:-1]
        print(f"[DEBUG] Set mode to {mode}")
        # Handle althold specially, setting mode to hold depth and to stabalize to be the new modes
        if mode == MODE_ALTHOLD:
            self.do_hold_depth = True
            mode = MODE_STABILIZE
        # Create a SetModeRequest message to change the mode
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        # Setting the data for the ROS service
        self.SERVICE_SET_MODE.set_data(data)
        # Call the ROS service to set the mode
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    # TODO :consider adding a lock to self.depth
    def calibrate_depth(self, sample_time=3):
        """
        To calibrate the depth data

        Args:
            sample_time (int): The number of seconds taken to calibrate the data        
        """
        print("\n[depth_calib] Starting Depth Calibration...")
        samples = []

        # Wait for depth data
        while self.depth == None:
            pass

        prevDepth = self.depth
        start_time = time.time()

        # Collect data for sample_time seconds, then calculate the mean
        while time.time() - start_time < sample_time:
            if self.depth == prevDepth:
                continue

            samples.append(self.depth)
            prevDepth = self.depth

        self.depth_calib = mean(samples)
        print(f"[depth_calib] Finished. Surface is: {self.depth_calib}")

    def depth_hold(self, depth):
        """
        Calculate the PWM values necessary to maintain depth

        Args:
            depth (float): The depth of the sub
        """
        try:
            # If the depth of the sub is out of the water or over a 100 meters, then exit
            if depth < -9 or depth > 100:
                return
            # Calculate PWM value to maintain the depth

            # Debug statements:
            # print(f"Passed in depth: {depth}")
            # print(f"PID Output from depth PID: {self.depth_pid(depth)}")
            # print(f"PID Offset: {self.depth_pid_offset}")
            
            self.depth_pwm = int(self.depth_pid(depth) * -1 + self.depth_pid_offset)
            
            # Print debug information (depth to 4 decimal places, depth_pwm, depth value to be at)
            print(f"[depth_hold] depth: {depth:.4f} depthMotorPower: {self.depth_pwm} Target: {self.depth_pid.setpoint}")
            # Assume motor range is 1200-1800 so +-300
        except Exception as e:
            print("DepthHold error")
            print(e)

    def get_baro(self, baro):
        """
        Handles barometric data by unpacking, calculating depth from raw data, then publishes raw data

        Args:
            baro: Barometric data from corresponding publisher
        """
        try:
            # If the barometric data message has the right ID
            if baro.msgid == 143:
                # Unpack the data
                p = pack("QQ", *baro.payload64)
                time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p) # Pressure is in mBar

                # Calculate the depth based on the pressure
                press_diff = round(press_diff, 2)
                press_abs = round(press_abs, 2)
                self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib

                # Publish the barometric data
                baro_data = std_msgs.msg.Float32MultiArray()
                baro_data.data = [self.depth, press_diff]
                self.AUV_BARO.set_data(baro_data)
                self.topic_publisher(topic=self.AUV_BARO)

                # Hold depth of the sub
                if self.do_hold_depth and self.armed:
                    self.depth_hold(self.depth)
        # Handle exceptions
        except Exception as e:
            print("Baro Failed")
            print(e)

    def set_depth(self, depth): 
        """Sets the absolute depth of the sub, as long as the depth is not negative (sub out of water)"""
        if depth.data < 0:
            return
        self.depth_pid.setpoint = depth.data

    def set_rel_depth(self, relative_depth):
        """To adjust depth relative to the current depth"""
        new_depth = self.depth_pid.setpoint + relative_depth.data
        if new_depth < 0:
            return
        self.depth_pid.setpoint = new_depth

    def batteryIndicator(self, msg):
        """To indicate low battery voltage by flashing the red status LED"""
        if self.config.get("battery_indicator", False):
            self.voltage = msg.voltage
            if self.voltage < 13.5:
                statusLed.flashRed()

    def thrusterCallback(self, msg):
        """Gets the current state of the thrusters (values of all PWM channels)"""
        self.thrustTime = time.time()
        self.channels = list(msg.channels)
        # print(self.channels)

    def enable_topics_for_read(self):
        """To subscribe to ROS topics"""
        self.topic_subscriber(self.TOPIC_STATE, self.update_parameters_from_topic)
        self.topic_subscriber(self.TOPIC_GET_IMU_DATA)
        self.topic_subscriber(self.TOPIC_GET_CMP_HDG)
        self.topic_subscriber(self.TOPIC_GET_RC)
        self.topic_subscriber(self.AUV_GET_THRUSTERS, self.thrusterCallback)
        self.topic_subscriber(self.AUV_GET_ARM)
        self.topic_subscriber(self.AUV_GET_MODE, self.change_mode)
        self.topic_subscriber(self.TOPIC_GET_MAVBARO, self.get_baro)
        self.topic_subscriber(self.AUV_GET_DEPTH, self.set_depth)
        self.topic_subscriber(self.AUV_GET_REL_DEPTH, self.set_rel_depth)
        self.topic_subscriber(self.TOPIC_GET_BATTERY, self.batteryIndicator)

    def start_threads(self):
        """Start threads for reading sensor data and thruster data"""
        sensor_thread   = threading.Thread(target=self.get_sensors, daemon=True)
        thruster_thread = threading.Thread(target=self.publish_thrusters, daemon=True)
        sensor_thread.start()
        thruster_thread.start()

    def publish_sensors(self):
        """Publish IMU and compass data to their corresponding publishers"""
        try:
            if self.imu != None:
                imu_data = self.imu
                # Set the data to be published
                self.AUV_IMU.set_data(imu_data)
                # Publish the data
                self.topic_publisher(topic=self.AUV_IMU)
            if self.hdg != None:
                comp_data = self.hdg
                # Set the data to be published
                self.AUV_COMPASS.set_data(comp_data)
                # Publish the data
                self.topic_publisher(topic=self.AUV_COMPASS)
            
        # Handle exceptions
        except Exception as e:
            print("publish sensors failed")
            print(e)

    def publish_thrusters(self):
        """To publish thruster data"""
        # If ROS is not shut down and do_publish_thrusters is True
        while not rospy.is_shutdown() and self.do_publish_thrusters:
            time.sleep(0.1)
            if self.connected:
                try:
                    channels = self.channels
                    # If the time that the thrusters should have been thrusting for is exceeded by 1 sec, set to neutral values
                    if time.time() - self.thrustTime > 1:
                        channels = [1500] * 18
                    # Hold depth at the calculated PWM value necessary to hold the depth
                    if self.do_hold_depth:
                        channels[2] = self.depth_pwm
                    thruster_data = mavros_msgs.msg.OverrideRCIn()
                    thruster_data.channels = channels
                    self.TOPIC_SET_RC_OVR.set_data(thruster_data)
                    self.topic_publisher(topic=self.TOPIC_SET_RC_OVR)
                # Handle exceptions
                except Exception as e:
                    print("Thrusters publish failed")
                    print(e)
            
    def get_sensors(self):
        """Get sensor data (IMU, Compass, Arming, Mode), and publish sensor data (IMU, compass)"""
        # If ROS is not shut down and do_get_sensors is True
        while not rospy.is_shutdown() and self.do_get_sensors:
            time.sleep(0.1)
            if self.connected:
                try:
                    # Get the data from compass, IMU, arm status, mode
                    self.hdg    = self.TOPIC_GET_CMP_HDG.get_data_last()
                    self.imu    = self.TOPIC_GET_IMU_DATA.get_data_last()
                    armRequest  = self.AUV_GET_ARM.get_data()
                    modeRequest = self.AUV_GET_MODE.get_data()
                    # Change the arming and mode of the sub based on request (from other files which publish to these ROS topics)
                    if armRequest != None:
                        self.armRequest = armRequest.data
                        while auv.armed != self.armRequest:
                            self.arm(self.armRequest)
                            time.sleep(0.1)
                    if modeRequest != None:
                        self.modeRequest = modeRequest.data
                        while self.mode != self.modeRequest:
                            self.change_mode(self.modeRequest)
                            time.sleep(0.1)
                    # Publish data from sensors(IMU, compass)
                    self.publish_sensors()
                # Handle exceptions
                except Exception as e:
                    print("sensor failed")
                    print(e)
            

    def update_parameters_from_topic(self, data):
        """To update parameters (status of arming, mode) based on received data from ROS topics"""
        if self.connected:
            try:
                self.armed = data.armed
                if not self.armed:
                    self.depth_pid.reset()
                self.mode = data.mode
                self.guided = data.guided
            except Exception as e:
                print("state failed")
                print(e)


def main():
    """The initialization of the interface to the Pixhawk, begins running threads"""
    try:
        # Wait for connection
        while not auv.connected:
            print("Waiting to connect...")
            time.sleep(0.5)
        print("Connected!")

        # Calibrate the depth first
        auv.change_mode("ALT_HOLD")
        auv.calibrate_depth()
        time.sleep(2)
        # arming
        # while not auv.armed:
        #     print("Attempting to arm...")
        #     auv.arm(True)
        #     time.sleep(3)
        # print("Armed!")

        # Begin running the threads (sensor retrieval and thruster control)
        print("\nNow beginning loops...")
        auv.start_threads()

    except KeyboardInterrupt:
        # Stopping sub on event of keyboard interruption (Ctrl + C)
        auv.arm(False)


def onExit(signum, frame):
    """
    To handle exiting the function (gracefully)

    Args:
        signum: An integer representing the signal number
        frame: An object representing the current excution frame
    """
    try:
        # Disarm the sub, shut down ROS
        print("\nDisarming and exiting...")
        auv.arm(False)
        rospy.signal_shutdown("Rospy Exited")
        time.sleep(1)
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass

# When Ctrl-C is pressed, call the OnExit() to exit the code
signal.signal(signal.SIGINT, onExit)

if __name__ == "__main__":
    """For running the script directly"""
    auv = AUV()
    auv.enable_topics_for_read()
    # Create a thread that will execute main(); daemon = True indicates that the thread is daemon, so it will stop when the main program exits
    mainTh = threading.Thread(target=main, daemon=True)
    mainTh.start() # Start the thread
    auv.connect("pix_standalone", rate=20)  # Change rate to 10 if issues arise
    