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
from geometry_msgs.msg import TwistStamped
import mavros_msgs.msg
import mavros_msgs.srv
from std_srvs.srv import Trigger
import geometry_msgs.msg



# Get the mathematical functions that handle various navigation tasks from utils.py
from auv.motion.utils import get_distance, get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector, get_norm
from auv.utils import deviceHelper # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
from auv.utils import arm, disarm
from auv.utils import fly
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
            debug (bool): Flag to enable or disable DVL
        """
        self.rate = rospy.Rate(50) # 50 Hz
        # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
        self.config     = deviceHelper.variables
        self.debug      = debug   
        self.lock       = threading.Lock()         

        # Store informaiton
        self.sub            = deviceHelper.variables.get("sub")
        self.mode           = "depth_hold"
        self.heading_control = False
        self.position       = {'x':0,'y':0,'z':0}
        self.orientation    = {'yaw':0,'pitch':0,'roll':0}   # in degrees, see self.pose_callback
        self.service        = None   # ros service name to move servos
        
        # Establish thruster and depth publishers
        self.sub_pose       = rospy.Subscriber("/auv/state/pose", PoseStamped, self.pose_callback)  
        self.sub_dvl        = rospy.Subscriber("/auv/devices/dvl/velocity", TwistStamped, self.dvl_callback)
        self.pub_thrusters  = rospy.Publisher("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_modem      = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)

        # Create variable to store pwm when direct control
        self.direct_input = [0] * 6
        # store desire point
        self.desired_point  = {"x":None,"y":None,"z":None,"yaw":None,"pitch":None,"roll":None}

        # Store dvl data and initialize dvl integration sum
        self.dvl_velocity = {'x': None, 'y': None, 'z': None}
        self.dvl_sum      = None
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
                self.config.get("FORWARD_PID_P", 2),
                self.config.get("FORWARD_PID_I", 0.05),
                self.config.get("FORWARD_PID_D", 0.01),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "lateral": PID(
                self.config.get("LATERAL_PID_P", 2),
                self.config.get("LATERAL_PID_I", 0.05),
                self.config.get("LATERAL_PID_D", 0.01),
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
        """Callback function for ekf output"""
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        self.position['z'] = msg.pose.position.z

        self.orientation['yaw']     = (msg.pose.orientation.z)
        self.orientation['pitch']   = (msg.pose.orientation.y)
        self.orientation['roll']    = (msg.pose.orientation.x)

    def dvl_callback(self, msg):
        self.dvl_velocity['x'] = msg.twist.linear.x
        self.dvl_velocity['y'] = msg.twist.linear.y
        self.dvl_velocity['z'] = msg.twist.linear.z   
    
    def publisherThread(self):
        """
        Publisher to publish the thruster values
        """
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
                    "z": None,
                    "yaw": 0,
                    "pitch": self.desired["pitch"] - self.orientation['pitch'],
                    "roll": (self.desired["roll"] - self.orientation['roll'])/180,
                }
                # Set the PWM values
                # Original PID outputs in world frame
                if self.desired_point['yaw'] is not None:
                    errors['yaw'] =  (heading_error(heading=self.orientation['yaw'], target=self.desired_point['yaw']) * -1) /180
                lateral_pwm_world = self.PIDs["lateral"](errors["x"])
                surge_pwm_world   = self.PIDs["surge"](errors["y"])
                # Set depth PWM value
                if self.sub=="graey":
                    depth_pwm_world = (self.PIDs['depth'](self.position['z']) * -1) /80.0
                elif self.sub=="onyx":
                    depth_pwm_world = (self.PIDs['depth'](self.position['z']))/80.0
                else:
                    depth_pwm_world = (self.PIDs['depth'](self.position['z']) * -1) /80.0

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
            elif self.mode=="direct":
                pitch_pwm   = self.direct_input[0]
                roll_pwm    = self.direct_input[1]
                depth_pwm   = self.direct_input[2]
                yaw_pwm     = self.direct_input[3]
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


                # minimum of 0.5 pwm for yaw
                min_pwm = 0.6
                if abs(yaw_pwm) < min_pwm:
                    if yaw_pwm < 0:
                        yaw_pwm = -min_pwm
                    elif yaw_pwm >= 0:
                        yaw_pwm = min_pwm

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
            pass
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
        with self.lock:
            if msg=="pid":
                self.reset()
                self.mode = msg
                rospy.loginfo("Set to pid control mode")
            elif msg=="depth_hold":
                self.mode = msg
                rospy.loginfo("Set to depth hold mode")
            elif msg=="direct":
                self.mode = msg
                rospy.loginfo("Set to direct control mode")
            else:
                self.mode = "depth_hold"
                rospy.logewarn("Control mode not found")
        
    def set_flight_mode(self, mode:String):
        fly.set_flight_mode(mode)
    
    def get_heading(self) -> float:
        return self.orientation['yaw']

    def go_to_heading(self, target):
        self.activate_heading_control(False)
        target = (target) % 360
        print(f"[INFO] Setting heading to {target}")
        self.prev_error = None
        start_time = time.time()
        start_heading = self.get_heading()
        while not rospy.is_shutdown():

            error = heading_error(self.orientation['yaw'], target)

            # make sure the output is greater than 0.5 for the thrusters to even move
            output = self.PIDs["yaw"](-error / 180)

            if abs(error) <= 1.0:
                print("[INFO] Heading reached")
                self.movement()
                break

            # New timeout mechanism: Timeout if moved less than 3 degrees in 3 seconds
            self.curr_time = time.time()
            if self.curr_time - start_time >= 3:
                if abs(self.get_heading() - start_heading) < 3:
                    print(f"[WARN] RobotControl.go_to_heading timed out")
                    break
                else:
                    start_time = time.time()
                    start_heading = self.get_heading()

            self.movement(yaw=output)
            time.sleep(0.1)

        print(f"[INFO] Finished setting heading to {target}")
            
    def go_to_depth(self, target):
        self.set_absolute_z(target)
        while abs(target - self.position['z']) > 0.1:
            rospy.loginfo(f"Going to depth: {target} | current depth: {self.position['z']}")
            time.sleep(1)
  
    def go_forward_distance(self, target:float):
        """
        Go forward by a certain distance base on dvl
        Args:
            target (float): desire distance
        """
        self.dvl_sum = 0
        dt = 0.05 # 20 Hz
        start_time = time.time()
        rospy.loginfo(f"Go forward {target}")
        while(abs(target-self.dvl_sum)>0.3) and time.time() - start_time < 30:
            delta = target - self.dvl_sum
            if delta>0:
                if abs(delta) < 4:
                    self.movement(forward=3*(delta/2))
                else:
                    self.movement(forward=5)
                
            else:
                if abs(delta) < 4:
                    self.movement(forward=3*(delta/2))
                else:
                    self.movement(forward=-5)
            
            # update distane traveled in body frame:
            with self.lock:
                self.dvl_sum += self.dvl_velocity['y'] * dt
            
            # rospy.loginfo(f"target: {target} | moved distance: {self.dvl_sum}")
            time.sleep(dt)

        # stop motors after reaching distance
        self.movement()

        # Push back funciton
        if target>0:
            self.movement(forward=-2)
        else:
            self.movement(forward=2)
        time.sleep(1)
        self.movement()

    def go_lateral_distance(self, target:float):
        """
        Go latera by a certain distance base on dvl
        Args:
            target (float): desire distance
        """
        self.dvl_sum = 0
        dt = 0.05 # 20 Hz
        start_time = time.time()
        rospy.loginfo(f"Go lateral {target}")
        while(abs(target-self.dvl_sum)>0.3) and time.time() - start_time < 30:
            delta = target - self.dvl_sum
            if delta>0:
                if abs(delta) < 4:
                    self.movement(lateral=3*(delta/2))
                else:
                    self.movement(lateral=5)
            else:
                if abs(delta) < 4:
                    self.movement(lateral=3*(delta/2))
                else:
                    self.movement(lateral=-5)
            # update distane traveled in body frame:
            with self.lock:
                self.dvl_sum += self.dvl_velocity['x'] * dt
            
            # rospy.loginfo(f"target: {target} | moved distance: {self.dvl_sum}")
            time.sleep(dt)
            
        # stop motors after reaching distance
        self.movement() 

        # Push back funciton
        if target>0:
            self.movement(lateral=-2)
        else:
            self.movement(lateral= 2)
        time.sleep(1)
        self.movement()
        
    def go_by_time(self, f=None, l=None, t=0):
        """Args: f - forward l - lateral t - time to sleep"""
        self.movement(forward=f,lateral=l)
        time.sleep(t)
        self.movement()


    def move_servo(self, service: str):
        """Operate a servo via the maestro_server file
        Args:
            - service (str): The servo to operate. Accepted values:
                - /auv/devices/dropper
                - /auv/devices/torpedo
                - /auv/device/gripper
        """
        if service in ["/auv/devices/dropper", "/auv/devices/torpedo", "/auv/devices/gripper"]:
            self.service = service
        else:
            raise ValueError("Unknown servo service called")
        
        rospy.wait_for_service(self.service)

        try:
            servo_client = rospy.ServiceProxy(self.service, Trigger)
            resp1 = servo_client()
            return resp1.message
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def send_modem(self, addr:str, movement:str):
        """
        Send message to the other sub. Example of expected message: destination Address-Movement-Acknowledgement-Priority. "020-ROLL-1-0"
        Args:
            addr (String): destination address
            movment (String): what movement to perform
            Ack (int) : Acknowledgement
            Priority (int) : priority
        """
        message_to_send = String()
        message_to_send.data = f"{addr}-{movement}"
        self.pub_modem.publish(message_to_send)
        rospy.loginfo(f"Send {addr}-{movement} to modem node !")

    def set_absolute_z(self, depth):
        """
        Set the depth of the robot

        Args:
            depth (float): Depth to set the robot to
        """
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

    def set_absolute_yaw(self, yaw:float):
        """
        Set the heading of the robot, this method only works when you activate heading control
        Args:
            yaw (float): robot desired yaw angle, unit: degrees
        """
        self.activate_heading_control(True)
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

    def waypointNav(self,x,y):
        """Navigate to a global waypoint in x, y, deactivate heading control"""
        if self.mode=="depth_hold":
            self.activate_heading_control(False)            
            try:
                rospy.loginfo(f"Moving to {(x,y)}")
                with self.lock:
                    dx = x - self.position['x']
                    dy = y - self.position['y']
                D = get_norm(dx,dy)
                target_heading  = get_heading_from_coords(dx,dy)
                rospy.loginfo(f"Going to heading {target_heading} degrees")
                self.go_to_heading(target_heading)
                self.set_absolute_yaw(target_heading)
                self.activate_heading_control(True)
                time.sleep(3)
                rospy.loginfo(f"Heading reached, going forward {D} m")
                self.go_forward_distance(D)
                self.activate_heading_control(False)
                rospy.loginfo(f"Reached {(x,y)}")
                time.sleep(0.1)
            except Exception as e:
                rospy.loginfo("Waypoint navigation interupted")

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


    
    
if __name__=="__main__":
    """Testing robot control funcitons"""
    import rospy
    import time
    from auv.motion import robot_control
    from auv.utils import arm, disarm


    rospy.init_node("control_test", anonymous=True)
    rc = robot_control.RobotControl()
    rc.set_control_mode("depth_hold")
    rospy.loginfo("Diving down")
    rc.set_absolute_z(0.4)
    current_heading = rc.orientation['yaw']
    rc.set_absolute_yaw(current_heading)
    rc.activate_heading_control(True)
    time.sleep(10)

    rospy.loginfo("Moving right")
    rc.movement(lateral=2)
    time.sleep(5)
    rc.movement()

    rospy.loginfo("Moving left")
    rc.movement(lateral=-2)
    time.sleep(4)
    rc.movement()


    rospy.loginfo("Reached the end")
    disarm.disarm()
    rc.exit()