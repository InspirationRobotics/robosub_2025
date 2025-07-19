"""
To control the robot by setting PWM values to auv/devices/thrusters. The class RobotControl publishes PWM values to the MAVROS topics.
These MAVROS topics are predefined topics that the pixhawk subscribes to. RobotControl does not handle the interface between the 
pixhawk flight controller and the software -- that is the job that pixstandalone.py does. 
"""


# Import the MAVROS message types that are needed
import rospy
import std_msgs
from std_msgs.msg import Float64, Float32MultiArray, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg



# Get the mathematical functions that handle various navigation tasks from utils.py
from auv.motion.utils import get_distance, get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector, get_norm
from auv.utils import deviceHelper # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
from auv.utils import arm, disarm
from auv.device.dvl import dvl # DVL class that enables position estimation
from auv.device.fog import fog_interface as fog

from simple_pid import PID
from transforms3d.euler import euler2quat
from transforms3d.euler import quat2euler
import threading
import numpy as np
import time
import math


def clip(pwmMin:int, pwmMax:int, value:int):
    """
    A clip function to ensure the pwm is in the acceptable range

    Args: 
        pwmMin (int) : min pwm
        pwmMax (int) : max pwm
        value  (int) : calculated pwm
    """
    value = min(pwmMax,value)
    value = max(pwmMin,value)
    return value

class RobotControl:
    """
    Class to control the robot
    """

    def __init__(self, debug=False):
        """
        Initialize the RobotControl class

        Args:
            enable_dvl (bool): Flag to enable or disable DVL
        """
        self.rate = rospy.Rate(20) # 10 Hz
        # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
        self.config     = deviceHelper.variables
        self.debug      = debug   
        self.lock       = threading.Lock()         

        # Store informaiton
        self.sub            = deviceHelper.variables.get("sub")
        self.mode           = "pid"
        self.heading_control = False
        self.position       = {'x':0,'y':0,'z':0}
        self.orientation    = {'yaw':0,'pitch':0,'roll':0}   # in degrees, see self.pose_callback

        # Establish thruster and depth publishers
        self.sub_pose       = rospy.Subscriber("/auv/state/pose", PoseStamped, self.pose_callback)  
        self.pub_thrusters  = rospy.Publisher("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_button     = rospy.Publisher("/mavros/manual_control/send", mavros_msgs.msg.ManualControl, queue_size=10)

        # Create variable to store pwm when direct control
        self.direct_input = [0] * 6
        # store desire point
        self.desired_point  = {"x":None,"y":None,"z":None,"yaw":None,"pitch":None,"roll":None}
        # A set of PIDs (Proportional - Integral - Derivative) to handle the movement of the sub
        """
        PIDs work by continously computing the error between the desired setpoint (desired yaw angle, forward velocity, etc.) and the 
        actual value. Based on this error, PIDs generate control signals to adjust the robot's actuators (in this case thrusters) to 
        minimize the desired setpoint. 

        Video: https://www.youtube.com/watch?v=wkfEZmsQqiA

        These definitions "tune" the PID controller for the necessities of the sub -- Proportional is tuned up high, which means greater 
        response to the current error but possible overshooting and oscillation
        """

        self.PIDs = {
            "yaw": PID(
                self.config.get("YAW_PID_P", 12),
                self.config.get("YAW_PID_I", 0.01),
                self.config.get("YAW_PID_D", 0.0),
                setpoint=0,
                output_limits=(-1, 1),
            ),
            "pitch": PID(
                self.config.get("YAW_PID_P", 0.5),
                self.config.get("YAW_PID_I", 0.1),
                self.config.get("YAW_PID_D", 0.1),
                setpoint=0,
                output_limits=(-5, 5),   
            ),
            "roll": PID(
                self.config.get("YAW_PID_P", 0.5),
                self.config.get("YAW_PID_I", 0.1),
                self.config.get("YAW_PID_D", 0.1),
                setpoint=0,
                output_limits=(-5, 5),   
            ),
            "surge": PID(
                self.config.get("FORWARD_PID_P", 4.0),
                self.config.get("FORWARD_PID_I", 0.01),
                self.config.get("FORWARD_PID_D", 0.1),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "lateral": PID(
                self.config.get("LATERAL_PID_P", 4.0),
                self.config.get("LATERAL_PID_I", 0.01),
                self.config.get("LATERAL_PID_D", 0.1),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "depth": PID(
                self.config.get("DEPTH_PID_P", 100),
                self.config.get("DEPTH_PID_I", 10),
                self.config.get("DEPTH_PID_D", 0.75),
                setpoint=0,
            ),  
        }


        # Wait for the topics to run
        time.sleep(1)

        # Arm the robot
        arm.arm()

        # Run thread
        self.running = True
        self.thread = threading.Thread(target=self.publisherThread)
        self.thread.daemon = True
        self.thread.start()

    def pose_callback(self, msg):
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        self.position['z'] = msg.pose.position.z

        self.orientation['yaw']     = (msg.pose.orientation.z)
        self.orientation['pitch']   = (msg.pose.orientation.y)
        self.orientation['roll']    = (msg.pose.orientation.x)

    def publisherThread(self):
        """
        Publisher to publish the thruster values
        """
        # TODO add np clip protection to the pwms
        while self.running and not rospy.is_shutdown():
            if self.mode == "pid":
                # Update desire pose
                self.desired = {
                    # Get desired X, Y, Z
                    'x': self.desired_point["x"] if self.desired_point["x"] is not None else self.position['x'],
                    'y': self.desired_point["y"] if self.desired_point["y"] is not None else self.position['y'],
                    'z': self.desired_point["z"] if self.desired_point["z"] is not None else self.position['z'],

                    
                    'yaw': self.desired_point["yaw"] if self.desired_point.get("yaw") is not None else self.orientation['yaw'],
                    'pitch': self.desired_point["pitch"] if self.desired_point.get("pitch") is not None else self.orientation['pitch'],
                    'roll': self.desired_point["roll"] if self.desired_point.get("roll") is not None else self.orientation['roll'],
                }

                # Calculate error
                errors = {
                    "x": self.desired["x"] - self.position['x'],
                    "y": self.desired["y"] - self.position['y'],
                    "z": self.desired["z"] - self.position['z'],
                    "yaw": self.desired["yaw"] - self.orientation['yaw'] if self.desired['yaw'] > self.orientation['yaw'] else self.orientation['yaw'] - self.desired['yaw'],
                    "pitch": self.desired["pitch"] - self.orientation['pitch'],
                    "roll": self.desired["roll"] - self.orientation['roll'],
                }

                # Set the PWM values
                # Original PID outputs in world frame
                
                lateral_pwm_world = self.PIDs["lateral"](errors["x"])
                surge_pwm_world   = self.PIDs["surge"](errors["y"])
                depth_pwm_world   = self.PIDs["depth"](errors["z"])


                yaw = self.orientation["yaw"]
                pitch = self.orientation["pitch"]
                roll = self.orientation["roll"]

                # Rotation matrix from world to body frame
                R = np.array([
                    [
                        math.cos(yaw)*math.cos(pitch),
                        math.sin(yaw)*math.cos(pitch),
                        -math.sin(pitch)
                    ],
                    [
                        math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll),
                        math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll),
                        math.cos(pitch)*math.sin(roll)
                    ],
                    [
                        math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll),
                        math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll),
                        math.cos(pitch)*math.cos(roll)
                    ]
                ])

                pwm_world = np.array([surge_pwm_world, lateral_pwm_world, depth_pwm_world])
                pwm_body = R.T @ pwm_world

                surge_pwm_body, lateral_pwm_body, depth_pwm_body = pwm_body

                yaw_pwm   = self.PIDs["yaw"](errors["yaw"])
                pitch_pwm = self.PIDs["pitch"](errors["pitch"]) if "pitch" in self.PIDs else 0
                roll_pwm  = self.PIDs["roll"](errors["roll"]) if "roll" in self.PIDs else 0

                self.__movement(
                    lateral=lateral_pwm_body,
                    forward=surge_pwm_body,
                    vertical=depth_pwm_body,
                    yaw=yaw_pwm,
                    pitch=pitch_pwm,
                    roll=roll_pwm
                )
            
            elif self.mode=="depth_hold":
                
                # Set depth PWM value
                if self.sub=="graey":
                    depth_pwm = (self.PIDs['depth'](self.position['z']) * -1) /80.0
                elif self.sub=="onyx":
                    depth_pwm = (self.PIDs['depth'](self.position['z']))/80.0
                else:
                    depth_pwm = (self.PIDs['depth'](self.position['z']) * -1) /80.0

                # Calculate heading error
                if self.desired_point['yaw'] is not None:
                    error = heading_error(heading=self.orientation['yaw'], target=self.desired_point['yaw'])
                else:
                    error =0


                with self.lock:
                    pitch_pwm   = self.direct_input[0]
                    roll_pwm    = self.direct_input[1]
                    yaw_pwm     = self.direct_input[3] if not self.heading_control else self.PIDs["yaw"](-error / 180) 
                    surge_pwm   = self.direct_input[4]
                    lateral_pwm = self.direct_input[5]

                self.__movement(
                    lateral=lateral_pwm,
                    forward=surge_pwm,
                    vertical=depth_pwm,
                    yaw=yaw_pwm,
                    pitch=pitch_pwm,
                    roll=roll_pwm
                )            
            else:
                rospy.logerr("Invalid control mode")
            self.rate.sleep()

    def movement(        
        self,
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=None,
        **kwargs,
    ):
        """
        A function that sets the pwms and not sending pwm directly to mavros topic, it's for easier interface
        Args:
            yaw (float): Power for the yaw maneuver
            forward (float): Power to move forward
            lateral (float): Power for moving laterally (negative one way (less than 1500), positive the other way (more than 1500))
            pitch (float): Power for the pitch maneuver
            roll (float): Power for the roll maneuver
            vertical (float): Distance to change the depth by
        
        """
        channels = [0] * 6
        channels[0] = pitch if pitch else 0
        channels[1] = roll if roll else 0
        channels[2] = vertical if vertical else 0
        channels[3] = yaw  if yaw else 0
        channels[4] = forward if forward else 0
        channels[5] = lateral if lateral else 0
        with self.lock:
            self.direct_input = channels

    def __movement(
        self,
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=None,
        **kwargs,
    ):
        """
        Move the robot in a given direction, by directly changing the PWM value of each thruster. This does not take input from the DVL.
        This is a non-blocking function.
        Inputs are between -5 and 5

        Args:
            yaw (float): Power for the yaw maneuver
            forward (float): Power to move forward
            lateral (float): Power for moving laterally (negative one way (less than 1500), positive the other way (more than 1500))
            pitch (float): Power for the pitch maneuver
            roll (float): Power for the roll maneuver
            vertical (float): Distance to change the depth by
        """

        
        # Create a message to send to the thrusters
        pwm = mavros_msgs.msg.OverrideRCIn()

        # Calculate PWM values
        channels = [1500] * 18
        channels[0] = int((pitch * 80) + 1500) if pitch else 1500
        channels[1] = int((roll * 80) + 1500) if roll else 1500
        channels[2] = int((vertical * 80) + 1500) if vertical else 1500  
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500

        # Clip the numbers to ensure in range
        for i,num in enumerate(channels):
            channels[i] = clip(1100,1900,num)
        
        pwm.channels = channels

        # Publish PWMs to /auv/devices/thrusters
        if self.debug:
            rospy.loginfo(f"pwms : {channels[0:6]} | input: {[pitch,roll,vertical,yaw,forward,lateral]}")
        else:
            self.pub_thrusters.publish(pwm)

    def activate_heading_control(self, activate:bool):
        self.heading_control = activate

    def set_control_mode(self, msg:String):
        """
        Callback function to handle the control mode of the robot.

        Args:
            msg (String): The control mode command. Expected values are:
                        - "pid" for PID control
                        - "direct" for direct thruster control
        """
        self.reset()
        
        if msg=="pid":
            self.mode = msg
            rospy.loginfo("Set to pid control mode")
        elif msg=="depth_hold":
            self.mode = msg
            rospy.loginfo("Set to depth hold mode")
        else:
            self.mode = "direct"
            rospy.logewarn("Control mode not found")
        
    def set_absolute_z(self, depth):
        """
        Set the depth of the robot

        Args:
            depth (float): Depth to set the robot to
        """
        # Clear the PID error
        self.PIDs["depth"].reset()
        self.PIDs["depth"].setpoint = depth
    
    def set_absolute_x(self, x):
        """
        Set the x position of the robot

        Args:
            x (float): X position to set the robot to
        """
        # Clear the PID error
        self.PIDs["lateral"].reset()
        self.desired_point["x"] = -x

    def set_absolute_y(self, y):
        """
        Set the y position of the robot

        Args:
            y (float): Y position to set the robot to
        """
        # Clear the PID error
        self.PIDs["surge"].reset()
        self.desired_point["y"] = -y

    def go_to_heading(self, target):
        target = (target) % 360
        print(f"[INFO] Setting heading to {target}")
        self.prev_error = None
        while not rospy.is_shutdown():

            error = heading_error(self.orientation['yaw'], target)

            output = self.PIDs["yaw"](-error / 180)

            if abs(error) <= 5:
                print("[INFO] Heading reached")
                break

            self.movement(yaw=output)
            time.sleep(0.1)

        print(f"[INFO] Finished setting heading to {target}")

    def set_absolute_yaw(self, yaw):
        """
        Set the heading of the robot

        Args:
            yaw (float): robot desired yaw angle, unit: degrees
        """
        self.desired_point['yaw'] = yaw % 360
        rospy.loginfo(f"Set desire heading to {yaw%360}")
            
    def set_absolute_pitch(self,pitch):
        """
        Set the pitch angle of the robot

        Args:
            pitch (float): robot desired pitch angle, unit: degrees
        """
        self.PIDs["pitch"].reset()
        self.desired_point["pitch"] = np.deg2rad(pitch)

    def set_absolute_roll(self,roll):
        """
        Set the roll angle of the robot

        Args:
            roll (float): desired roll angle, unit: degrees
        """
        self.PIDs["roll"].reset()
        self.desired_point["roll"] = np.deg2rad(roll)

    def set_relative_z(self, depth):
        """
        Set the depth of the robot relative to the current depth

        Args:
            depth (float): Relative depth to set the robot to (-2 means up, 2 means down)
        """
        # Clear the PID error
        self.PIDs["depth"].reset()
        self.desired_point["z"] = depth + self.pose.pose.position.z

    def set_relative_x(self, x):
        """
        Set the x position of the robot relative to the current position

        Args:
            x (float): Relative x position to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["lateral"].reset()
        self.desired_point["x"] = x + self.pose.pose.position.x

    def set_relative_y(self, y):
        """
        Set the y position of the robot relative to the current position

        Args:
            y (float): Relative y position to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["surge"].reset()
        self.desired_point["y"] = y + self.pose.pose.position.y

    def set_relative_yaw(self, yaw):
        """
        Set the heading of the robot relative to the current heading

        Args:
            yaw (float): Relative heading to set the robot to (-2 means left, 2 means right), unit: degrees
        """
        # Clear the PID error
        self.PIDs["yaw"].reset()
        self.desired_point["yaw"] = yaw + self.orientation['yaw']
    
    def set_relative_pitch(self, pitch):
        """
        Set the heading of the robot relative to the current heading

        Args:
            pitch (float): Relative pitch to set the robot to, unit: degrees
        """
        # Clear the PID error
        self.PIDs["pitch"].reset()
        self.desired_point["pitch"] = np.deg2rad(pitch) + self.orientation['pitch']

    def set_relative_roll(self, roll):
        """
        Set the heading of the robot relative to the current heading

        Args:
            roll (float): Relative heading to set the robot to, unit: degrees
        """
        # Clear the PID error
        self.PIDs["roll"].reset()
        self.desired_point["roll"] = np.deg2rad(roll) + self.orientation['roll']

    def waypointNav(self,x,y):
        if self.mode=="depth_hold":
            self.heading_control = False
            reached = False
            
            try:
                rospy.loginfo("Waypoint loop starting")
                while not reached and not rospy.is_shutdown():
                    with self.lock:
                        dx = x - self.position['x']
                        dy = y - self.position['y']
                    D = get_norm(dx,dy)
                    if D < 1:
                        reached = True
                        rospy.loginfo("Reach waypoint or got interupted")
                    current_heading = self.orientation['yaw'] % 360
                    target_heading  = get_heading_from_coords(dx,dy)
                    yaw_error = heading_error(current_heading, target_heading)
                    yaw_pwm = self.PIDs["yaw"]( - yaw_error / 180)
                    surge_pwm = max(min(D/5.0,1) * 1.5,0.5)

                    rospy.loginfo(f"distance away: {D}")
                    rospy.loginfo(f"yaw pwm: {yaw_pwm}, forward pwm: {surge_pwm}")
                    self.movement(yaw=yaw_pwm,forward=surge_pwm)
                    time.sleep(0.1)
            except KeyboardInterrupt as e:
                reached = True

    def reset(self):
        for key, pid in self.PIDs.items():
                pid.reset()
        
        self.desired_point  = {"x":None,"y":None,"z":None,"yaw":None,"pitch":None,"roll":None}
        self.direct_input = [0] * 6

    def exit(self):
        """
        Exit the robot control
        """
        # Stop the robot
        self.__movement()
        # Stop the thread
        self.running = False
        # Wait for thread to stop
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        # Stop the node
        disarm.disarm()
        rospy.loginfo("[RobotControl] Shutdown complete.")


    
    




    