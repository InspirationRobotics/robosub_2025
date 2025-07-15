"""
To control the robot by setting PWM values for each thruster. The class RobotControl publishes PWM values to the MAVROS topics.
These MAVROS topics are predefined topics that the pixhawk subscribes to. RobotControl does not handle the interface between the 
pixhawk flight controller and the software -- that is the job that pixstandalone.py does. 
"""

import time

# Import the MAVROS message types that are needed
from geometry_msgs.msg import Twist
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
from std_msgs.msg import Float64, Float32MultiArray, String
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped, TwistStamped, Vector3Stamped, PoseStamped 


# Import the PID controller
from simple_pid import PID

# Get the mathematical functions that handle various navigation tasks from utils.py
from .utils import get_distance, get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector
from ..utils import deviceHelper # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
from ..device.dvl import dvl # DVL class that enables position estimation
from ..device.fog import fog_interface as fog
import math
import numpy as np

config = deviceHelper.variables # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)

class RobotControl:
    """
    Class to control the robot
    """

    def __init__(self, enable_dvl=True, enable_fog = False):
        """
        Initialize the RobotControl class

        Args:
            enable_dvl (bool): Flag to enable or disable DVL
        """

        # Initialize the configuration of the devices, depth of the sub, compass of the sub, DVL
        self.config = config
        self.depth = self.config.get("INIT_DEPTH", 0.0)
        self.compass = None

        # Initialize vectornav variables
        self.vectornav_pitch = None
        self.vectornav_roll = None
        self.vectornav_yaw = None

        self.position = {'x':0,'y':0,'z':0}
        self.orientation = {'x':0,'y':0,'z':0,'w':0}

        # dvl sensor setup (both subs)
        if enable_dvl:
            self.dvl = dvl.DVL()
            self.dvl.start()
        else:
            self.dvl = None

        fog_enable = enable_fog

        self.fog = None

        # Establish thruster and depth publishers
        self.sub_compass = rospy.Subscriber("/auv/devices/compass", Float64, self.get_callback_compass)
        self.vectornav = rospy.Subscriber("/auv/devices/vectornav", geometry_msgs.msg.Vector3, self.callback_vectornav)
        self.sub_fog = rospy.Subscriber("/auv/devices/fog", Float64, self.get_callback_fog)
        self.sub_depth = rospy.Subscriber("/auv/devices/baro", Float32MultiArray, self.callback_depth)
        self.sub_pose = rospy.Subscriber("/auv/state/position", PoseStamped, self.callback_pose)
        self.pub_thrusters = rospy.Publisher("auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_depth = rospy.Publisher("auv/devices/setDepth", Float64, queue_size=10)
        self.pub_rel_depth = rospy.Publisher("auv/devices/setRelativeDepth", Float64, queue_size=10)
        self.pub_mode = rospy.Publisher("auv/status/mode", String, queue_size=10)
        self.pub_button = rospy.Publisher("/mavros/manual_control/send", mavros_msgs.msg.ManualControl, queue_size=10)
        self.pub_ang_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, queue_size=10)
        
        # TODO: reset pix standalone depth Integration param 

        # A set of PIDs (Proportional - Integral - Derivative) to handle the movement of the sub
        """
        PIDs work by continously computing the error between the desired setpoint (desired yaw angle, forward velocity, etc.) and the 
        actual value. Based on this error, PIDs generate control signals to adjust the robot's actuators (in this case thrusters) to 
        minimize the desired setpoint. 

        Video:  

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
            "forward": PID(
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
        }

        # Wait for the topics to run
        time.sleep(1)

    def get_callback_compass(self, msg):
        def _callback_compass(msg):
            """Get the compass heading from /auv/devices/compass topic"""
            self.compass = msg.data
            print(self.compass)

        def _callback_compass_dvl(msg):
            """Get the compass heading from dvl"""
            self.compass = msg.data
            self.dvl.compass_rad = math.radians(msg.data)

        # If DVL return compass data from the DVL, else from the compass
        if self.dvl:
            return _callback_compass_dvl
        else:
            return _callback_compass
    
    def callback_vectornav(self, msg):
        self.vectornav_pitch = msg.x
        self.vectornav_roll = msg.y
        self.vectornav_yaw = msg.z
    
    def get_callback_fog(self, msg):
        """Get the compass heading from /auv/devices/fog topic"""
        self.fog = msg.data

    def callback_depth(self, msg):
        """Get depth data from barometer /auv/devices/baro topic"""
        self.depth = msg.data[0]

    def set_depth(self, d):
        """Set depth to a given absolute value"""
        depth = Float64()
        depth.data = d
        self.pub_depth.publish(depth)
        print(f"[INFO] Depth set to {d}")

    def set_relative_depth(self, delta_depth):
        """Change the depth of the sub by a relative value (up 3 meters, down 3 meters, etc.)"""
        rel_depth = Float64()
        rel_depth.data = delta_depth
        self.pub_rel_depth.publish(rel_depth)
        print(f"[INFO] Changing Depth relatively by {delta_depth}, current {self.depth}")
    
    def set_mode(self, mode_input):
        """Change the mode of the sub to specified mode"""
        mode = String()
        mode.data = mode_input
        self.pub_mode.publish(mode)
        print(f"[INFO] Changing mode to {mode}")
    
    def button_press(self, button=4):
        """DO NOT USE. This simulates a button press on QGroundControl, primarily used to toggle roll/pitch
        using unsigned 16 bit integer. Lowest bit is button 0, second lowest bit is button 1, etc.
        BE WARNED that the sub disarms 3 seconds after any use of this method"""
        press = mavros_msgs.msg.ManualControl()
        press.buttons = button
        self.pub_button.publish(press)

    def callback_pose(self, msg):
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract orientation (quaternion)
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.position['x'] = x
        self.position['y'] = y
        self.position['z'] = z

        self.orientation['x'] = qx
        self.orientation['y'] = qy
        self.orientation['z'] = qz
        self.orientation['w'] = qw

    def movement(
        self,
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=0,
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

        # TODO Handle timeout of the pixhawk
        """

        pwm = mavros_msgs.msg.OverrideRCIn()

        # Calculate PWM values
        channels = [1500] * 18
        # channels[2] = int((vertical * 80) + 1500) if vertical else 1500
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500

        # TODO: Implement correct pitch/roll channels w/ QGroundControl
        channels[0] = int((pitch * 80) + 1500) if pitch else 1500
        channels[1] = int((roll * 80) + 1500) if roll else 1500
        pwm.channels = channels

        # Publish PWMs to /auv/devices/thrusters
        # print(f"[INFO] Channels sent to pixhawk = {pwm}")
        if vertical!=0: self.set_relative_depth(vertical)
        self.pub_thrusters.publish(pwm)

    def set_heading(self, target: int, heading_sensor="pix_compass"):
        """
        Yaw to the target heading; target heading is absolute (not relative)
        This is a blocking function
        
        Args:
            target (int): Absolute desired heading 
            fog (boolean): Whether to use FOG (True) or compass (False)
        """
        # Mod the target to make sure it is between 0 - 359 degrees
        target = (target) % 360
        print(f"[INFO] Setting heading to {target}")
        time_check = time.time()
        self.prev_error = None
        while not rospy.is_shutdown():
            if heading_sensor == "fog":
                if self.fog is None:
                    print("[WARN] FOG not ready")
                    time.sleep(0.5)
                    continue
                error = heading_error(self.fog, target)
            elif heading_sensor == "pix_compass":
                if self.compass is None:
                    print("[WARN] Compass not ready")
                    time.sleep(0.5)
                    continue
                error = heading_error(self.compass, target)
            elif heading_sensor == "vectornav_imu":
                if self.vectornav_yaw is None:
                    print("[WARN] Vectornav IMU Not ready")
                    time.sleep(0.5)
                    continue
                error = heading_error(self.vectornav_yaw, target)
            # else:
            #     print("[WARN] Unknown sensor specified")

                

            # Break the function if the error hasn't changed 
            # by 3 degrees over 3 secs - prevents the AUV from getting
            # stuck at an "incorrect" heading
            if time.time() - time_check > 3:
                time_check = time.time()
                if self.prev_error is None:
                    self.prev_error = error
                elif abs(error - self.prev_error) < 3:
                    break
                else:
                    self.prev_error = error

            # Normalize error to the range -1 to 1 for the PID controller
            output = self.PIDs["yaw"](-error / 180)

            # print(f"[DEBUG] Heading error: {error}, output: {output} {self.compass} {target}")

            if abs(error) <= 5:
                print("[INFO] Heading reached")
                break

            self.movement(yaw=output)
            time.sleep(0.1)

        print(f"[INFO] Finished setting heading to {target}")
    
    def get_heading(self, sensor="pix_compass") -> int:
        """Returns a compass heading. This may be helpful if you need
        to save a heading to re-orient the sub later"""
        if sensor == "pix_compass":
            return self.compass
        elif sensor == "vectornav_imu":
            return self.vectornav_yaw
        elif sensor == "fog":
            return self.fog
        else:
            print("[WARN] Unknown sensor specified")

    def setHeadingOld(self, target: int):
        """
        Yaws until the sub reaches desired heading
        This function is deprecated

        Args:
            target (int): Absolute (not relative) desired target heading
        """
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        target = (target) % 360

        # dir variable is the direction -- 1 is clockwise, -1 is counterclockwise
        while not rospy.is_shutdown():
            current = int(self.compass)
            dir = 1  # cw
            diff = abs(target - current)
            # Switch the direction of the yaw since going the other way will be faster (since there are 360 degrees in a circle)
            if diff >= 180:
                dir *= -1
            if current > target:
                dir *= -1
            if diff >= 180:
                diff = 360 - diff
            # If farther from desired heading, speed will increase, if close to desired heading, speed will decrease
            # Note how this has to be handled manually (there is no PID controller implemented in this function)
            if diff <= 10:
                speed = 55
            else:
                speed = 70
            # Once compass is within 2 degrees of desired heading, publish PWMs to stop yawing
            if diff <= 1:
                pwm.channels[3] = 1500
                self.pub_thrusters.publish(pwm)  # Publishing pwms to stop yawing
                break
            else:
                pwm.channels[3] = 1500 + (dir * speed)
                self.pub_thrusters.publish(pwm)  # Publishing pwms to continue yawing

    def navigate_dvl(
        self, 
        x, 
        y, 
        z, 
        end_heading=None, 
        relative_coord=True, 
        relative_heading=True, 
        update_freq=10, 
        heading_sensor="vectornav_imu"
        ):
        """
        Navigate using DVL to a 3D point (x, y, z). This is a blocking function using a compass and DVL feedback.

        Args:
            x (float): lateral distance to move (in meters).
            y (float): forward distance to move (in meters).
            z (float): target absolute depth (in meters).
            end_heading (float, optional): target heading in degrees. Defaults to heading toward target.
            relative_coord (bool): whether x, y are relative to current position.
            relative_heading (bool): whether x, y should be rotated by current heading.
            update_freq (int): control update frequency in Hz.
            heading_sensor (str): which sensor to use for heading. Options: "vectornav_imu", "pix_compass", "fog".
        """

        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        def get_heading_value():
            """Return current heading based on selected heading sensor."""
            if heading_sensor == "vectornav_imu":
                return self.vectornav_yaw
            elif heading_sensor == "pix_compass":
                return self.compass
            elif heading_sensor == "fog":
                return self.fog
            else:
                raise ValueError(f"[ERROR] Unknown heading sensor '{heading_sensor}'")

        # Reset all PID controllers
        for pid in self.PIDs.values():
            pid.reset()

        if not relative_coord:
            x -= self.dvl.position[0]
            y -= self.dvl.position[1]

        if relative_heading:
            current_heading = get_heading_value()
            x, y = rotate_vector(x, y, current_heading)

        target_heading = get_heading_from_coords(x, y)
        current_heading = get_heading_value()
        print(f"[INFO] Navigating to {x:.2f}, {y:.2f}, {z:.2f}, target_heading={target_heading:.2f}°, current_heading={current_heading:.2f}°")

        self.set_heading(target_heading)
        self.set_depth(z)

        with self.dvl:
            self.set_depth(z)

            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                current_heading = get_heading_value()
                err_x, err_y = inv_rotate_vector(
                    x - self.dvl.position[0],
                    y - self.dvl.position[1],
                    current_heading
                )

                # Position tolerance threshold
                x_err_th = 0.1 + self.dvl.error[0]
                y_err_th = 0.1 + self.dvl.error[1]

                if abs(err_x) <= x_err_th and abs(err_y) <= y_err_th:
                    print("[INFO] Target reached")
                    break

                output_x = self.PIDs["lateral"](-err_x)
                output_y = self.PIDs["forward"](-err_y)
                print(f"[DEBUG] err_x={err_x:.3f}, err_y={err_y:.3f}, output_x={output_x:.3f}, output_y={output_y:.3f}")

                self.movement(lateral=output_x, forward=output_y)


    def forward_dvl(self, distance, pid=True, throttle=None):
        """
        Move forward using the DVL.
        This is a blocking function.

        Args:
            throttle (float): power at which to move forward at
            distance (float): distance in meters to move forward by
            pid (boolean): Whether to use PID (True) or numpy.clip() (False)
        """
        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        print(f"[INFO] Moving forward {distance}m at throttle {throttle}")

        # Enter a local scope to handle coordinates cleanly
        with self.dvl:
            curr_time = time.time()
            prev_time = None
            # Navigate to the target point
            while not rospy.is_shutdown():
                time.sleep(0.25)
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.1)
                    continue
                # Ensure position data is updated/avaliable
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                # Find the y-axis error (recall that the y-axis is the forward-backwards dimension)
                y = self.dvl.position[1]
                error = distance - y

                # Check if the target has been reached
                if abs(error) <= 0.1:
                    print("[INFO] Target reached")
                    break
                # Apply gain to the error and clip by the maximum throttle value(s)
                if pid:
                    forward_output = self.PIDs["forward"](-error)
                else:
                    forward_output = np.clip(error * 4, -throttle, throttle)

                # Move forward using the PWM calculations in the movement function
                self.movement(forward=forward_output)

    def lateral_dvl(self, distance, pid=True, throttle=None):
        """
        Move laterally using the DVL -- this contains the exact same method as forward_dvl except the x, not y-axis is used.
        This is a blocking function.

        Args:
            throttle (float): power to move at
            distance (float): distance to move laterally (in meters)
        """
        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        print(f"[INFO] Moving laterally {distance}m at throttle {throttle}")

        # Enter a local scope to handle coordinates nicely
        with self.dvl:
            # Navigate to the target point
            while not rospy.is_shutdown():
                time.sleep(0.1)
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.1)
                    continue

                # Ensure position data is updated/avaliable
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                # Calculate the error between the current and target x-coordinates (recall that the x-axis is the lateral dimension)
                x = self.dvl.position[0]
                error = distance - x

                # Check if we reached the target
                if abs(error) <= 0.1:
                    print("[INFO] Target reached")
                    break
                
                # Apply gain and clip at the maximum throttle value(s)
                if pid:
                    lateral_output = self.PIDs["lateral"](-error)
                else:
                    lateral_output = np.clip(error * 4, -throttle, throttle)
                print(f"[DEBUG] error={error}, lateral_output={lateral_output}")

                # Move laterally using PWM values
                self.movement(lateral=lateral_output)

    def forwardHeading(self, power, t):
        """
        Moves forward at a specific power for a duration of time, with a backstop method (method for gradually decreasing the speed of a vehicle to allow precise movements).

        Args:
            power (float): the power to move at
            t (float): duration of movement (in seconds)

        Equations for power levels:
            Power 1: 7.8t + 3.4 (in inches)
            Power 2: 21t + 0.00952
            Power 3: 32.1t - 18.7
        """
        
        forwardPower = (power * 80) + 1500  # Calculating PWM values for forward thrusters
        if t > 3:  # Designating time for the backstop so that inertia does not increase the distance travelled by the sub
            timeStop = t / 6
        else:
            timeStop = 0.5

        # t = t + timeStop # this doesn't make sense because this increases time over long distance

        # Power to stop the sub's forward momentum (less than 1500 is reverse)
        powerStop = 1500 - (power * 40)

        # Create and publish PWM values for moving forward
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = forwardPower

        # For the amount of time specified, move forward by publishing PWMs continuously to the thrusters
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)  # Publishing PWMs for forward commands to thrusters
            time.sleep(0.1)

        print("[INFO] finished forward")
        # Calculating gradual decrease of thruster power
        gradDec = int((forwardPower - powerStop) / (timeStop * 10))
        startTime = time.time()
        # Gradually decreasing forward thruster power (PWMs) for the designated time calculated above
        while time.time() - startTime < timeStop:
            current_p = pwm.channels[4]
            pwm.channels[4] = current_p - gradDec
            self.pub_thrusters.publish(pwm)  # Publishing reduced PWMs for continued reduced forward thrust
            time.sleep(0.1)

        t2 = 0
        print("[INFO] finished backstopping")
        # Create neutral channels (1500 is the neutral value)
        pwm.channels = [1500] * 18
        startTime = time.time()
        # Publishing idle PWMs for 0.5 secs to stop the sub
        while time.time() - startTime <= 0.5:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        print("[INFO] finished forward heading")

    def lateralUni(self, power, t):
        """
        Calls the universal backstop function for lateral movement

        Args:
            power (float): power for the movement
            t (float): duration of the movement (in s)
        """
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[5] = int(forwardPower)
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t) # Universal backstop

    def forwardUni(self, power, t):
        """
        Calls the universal backstop function for forward movement

        Args:
            power (float): power for the movement
            t (float): duration of the movement (in s)
        """
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = int(forwardPower)

        # If the sub is Graey calculate the PWM differently (go to channel 5 instead of 4 and subtract by power)
        if config.get("sub", "onyx") == "graey":
            pwm.channels[5] = 1500-int(power*7)
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t) # Universal backstop

    def mapping(self, x, in_min, in_max, out_min, out_max):
        """
        Map a value from one range onto another

        Args:
            x (float): value to be mapped
            in_min (float): minimum value of the input range
            in_max (float): maximum value of the input range
            out_min (float): minimum value of the output range
            out_max (float): maximum value of the output range
        
        Returns:   
            float: mapped value
        """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    

    def backStop(self, pwm, t):
        """
        Universal backstop mechanism that is given a PWM value to reduce and overall time of movement which calculates
        inertia and sends negative PWM commands to the thrusters for reverse thrust to stop the sub.

        Args:
            pwm (int): the PWM value
            t (float): the duration of the movement (not for the backstop, but for the total movement)
        """
        # Designate the time for backstopping
        if t > 3:
            timeStop = t / 6
        else:
            timeStop = 0.5
        
        # Initialize neutral channels
        maxPower = [1500] * 18
        powerStop = [1500] * 18

        # Calculate maximum power and power to stop for each channel
        for i in range(len(pwm.channels)):
            if pwm.channels[i] != 1500:
                maxPower[i] = pwm.channels[i] # Don't go higher than the current value
                powerStop[i] = (1500 - pwm.channels[i]) / 2 + 1500 # To stop, cut the gain (current movement value - neutral) from neutral in half and reverse it (for reverse thrust)
        
        # Gradually decrease PWMs to stop the sub
        startTime = time.time()
        while time.time() - startTime <= timeStop:
            for i in range(len(pwm.channels)):
                # Map the time left from the range of the time calculated for backstopping onto the current PWM movement value to PWM value to reverse thrust
                # As the time duration for the backstop goes up, the amount of reverse thrust will increase (PWM will go down)
                if pwm.channels[i] != 1500:
                    pwm.channels[i] = int(
                        self.mapping(
                            time.time() - startTime,
                            0,
                            timeStop,
                            maxPower[i],
                            powerStop[i],
                        )
                    )
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        
        # Set at neutral PWMs to stop the sub completely
        pwm.channels = [1500] * 18
        startTime = time.time()
        while time.time() - startTime <= 0.5:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        print("[INFO] finished backstopping")

    # uses functions from testing to correlate pwms and time to distance and then utilizes forwardHeadingUni commmand to send pwms to thrusters for calculated pwms and time
    def forwardDist(self, dist, power):
        """
        Move the sub forward by a certain distance at a certain power -- uses functions created through testing to correlate the PWM value
        and time necessary to move a certain distance, then calls ForwardUni() to move that distance.

        Args:
            dist (float): distance to move forward (in meters)
            power (float): power level for forward movement 
        """
        # Convert distance from meters to inches
        inches = 39.37 * dist
        print(power, inches)
        eqPower = abs(round(power))
        time = 0
        # Calculate the amount of time needed based on the power level
        if eqPower >= 3:
            inches = inches - 9.843
            time = (inches + 18.7) / 32.1
        elif eqPower == 2:
            time = (inches - 0.01) / 21
        elif eqPower == 1:
            time = (inches - 3.4) / 7.8
        # Move forward for the specified time and at the specified power
        self.forwardUni(power, time)

    
    def roll(self, power=3, set_time=5):
        """DO NOT USE - THIS DOES NOT WORK. Meant to roll with specified 
        time and power, with toggle continuously being pressed"""
        start_time = time.time()
        button_number = 256 # Most likely?
        print("[INFO] Starting roll.")
        while time.time() - start_time < set_time:
            self.button_press(button_number)
            self.movement(lateral=power)
        print("[INFO] Roll terminated.")

    def roll2(self, roll_vel=1):
        """DO NOT USE - THIS DOES NOT WORK. Meant to 
        publish a roll velocity in rad/sec"""
        roll_cmd = geometry_msgs.msg.Twist()
        roll_cmd.angular.x = roll_vel
        self.pub_ang_vel.publish(roll_cmd)

    def waypointNav(self,x:float,y:float, error:float=1.0):
        err_x = x - self.position['x']
        err_y = y - self.position['y']
        #err_z = z - self.position['z']

        # 1 m of error
        while(err_x > error or err_y > error):
            forwardPWM = self.PIDs['forward'](-err_y)
            lateralPWM = self.PIDs['lateral'](-err_x)
            #self.set_depth(z)
            self.movement(forward=forwardPWM, lateral=lateralPWM)

            time.sleep(0.1) # 10 Hz

        