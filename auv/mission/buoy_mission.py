"""
Finds the buoy, navigates to the correct position (optimal distance to circumnavigate), and circumnavigates around the buoy.
"""

import json
import rospy
import time

from std_msgs.msg import String
from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For running the motors on the sub
from ..utils import arm, disarm

class BuoyMission:
    
    def __init__(self, target="Red", **config):
        """
        Initialize the mission class; here should be all of the things needed in the run function. 

        Args:
            config: Mission-specific parameters to run the mission.
        """
        self.cv_files = ["buoy_cv"] # CV file to run
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler
        self.next_data = {}  # Dictionary to store the newest data from the CV handler; this data will be merged with self.data.
        self.received = False
        self.target = target

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        self.positioned = False

        # Initialize the CV handlers; dummys are used to input a video file instead of the camera stream as data for the CV script to run on
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("buoy_cv", target)
        print("[INFO] Buoy Mission Init")

    def callback(self, msg):
        """
        Calls back the cv_handler output -- you can have multiple callbacks for multiple CV handlers. Converts the output into JSON format.

        Args:
            msg: cv_handler output -- this will be a dictionary of motion commands and potentially the visualized frame as well as servo commands (like the torpedo launcher)
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the topic name
        data = json.loads(msg.data) # Convert the data to JSON
        self.next_data[file_name] = data 
        self.received = True

    def run(self):
        """
        Here should be all the code required to approach the buoy.
        This could be a loop, a finite state machine, etc.
        """
        print("[INFO] before entering the loop")
        while not rospy.is_shutdown():
            time.sleep(0.01)
            if not self.received:
                print("[INFO] skip one iteration")
                continue

            # Merge self.next_data, which contains the updated CV handler output, with self.data, which contains the previous CV handler output.
            # self.next_data will be wiped so that it can be updated with the new CV handler output.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key]) # Merge the data
                else:
                    self.data[key] = self.next_data[key] # Update the keys if necessary
            self.received = False
            self.next_data = {}

            # Do something with the data.
            lateral = self.data["buoy_cv"].get("lateral", None)
            forward = self.data["buoy_cv"].get("forward", None)
            yaw = self.data["buoy_cv"].get("yaw", None)
            vertical = self.data["buoy_cv"].get("vertical", None)
            end = self.data["buoy_cv"].get("end", None)

            if end:
                print("[INFO] AUV has aligned with the buoy. Beginning circumnavigation.")
                self.positioned = True
                break
            else:
                self.robot_control.movement(lateral = lateral, forward = forward, yaw = yaw, vertical = vertical)
                print(forward, lateral, yaw) 
                
            
        print("[INFO] Buoy mission finished running")
        
        if self.positioned == True:
            print("[INFO] Beginning Buoy circumnavigation")
            self.circumnavigate()
            pass
    

    def circumnavigate(self):
        """Circumnavigates the buoy based on the gate mission choice. 
        Aims to make a square around the buoy"""
        print("Starting circumnavigation")
        
        if self.target == "Red":
            lateral_mag = -2
        elif self.target == "Blue":
            lateral_mag = 2

        compass_heading = self.robot_control.get_heading()

        # Circumnavigate using dead reckoning only. This works at a
        # significant distance from the buoy and doesn't get stuck to walls
        # (though wall collisions may affect the course)
        
        first_time = time.time()
        while time.time() - first_time < 4:
            self.robot_control.movement(lateral=lateral_mag)
        first_time = time.time()
        while time.time() - first_time < 6:
            self.robot_control.movement(forward=2.5)
        first_time = time.time()
        while time.time() - first_time < 8:
            self.robot_control.movement(lateral=-lateral_mag)
        first_time = time.time()
        while time.time() - first_time < 4:
            self.robot_control.movement(forward=-2.5)
        first_time = time.time()
        while time.time() - first_time < 8:
            self.robot_control.movement(lateral=lateral_mag)
        time.sleep(1)

        # DVL is unavailable for Onyx, this worked very well on Graey
        # self.robot_control.lateral_dvl(throttle=1, distance = lateral_dist)
        # self.robot_control.forward_dvl(throttle=1, distance=2)
        # self.robot_control.lateral_dvl(throttle=1, distance=(-2*lateral_dist))
        # self.robot_control.forward_dvl(throttle=1, distance = -2)
        # self.robot_control.lateral_dvl(throttle=1, distance=lateral_dist)
        # self.robot_control.set_heading(compass_heading + 180)



    def cleanup(self):
        """
        Here should be all the code required after the run function.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # Idle the robot
        self.robot_control.movement(lateral = 0, forward = 0, yaw = 0)
        print("[INFO] Buoy mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.buoy_mission"
    # You can also import it in a mission file outside of the package
    import time
    from auv.utils import deviceHelper
    from auv.motion import robot_control

    rospy.init_node("buoy_mission", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = BuoyMission(**config)
    rc = robot_control.RobotControl()

    # Run the mission
    arm.arm()
    rc.set_absolute_z(0.5)
    time.sleep(5)
    mission.run()
    mission.cleanup()
    disarm.disarm()
