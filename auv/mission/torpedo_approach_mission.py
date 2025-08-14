"""
Torpedo Approach Mission. The goal is to find the torpedo, and then switch to bottom facing camera to align with the center 
of the platform before surfacing.

NOTE: We could just use DVL to get inside the torpedo range if that works consistently enough.
"""

import json

import rospy
import time
from std_msgs.msg import String

from auv.device import cv_handler # For running mission-specific CV scripts
from auv.motion import robot_control # For running the motors on the sub
from auv.utils import arm, disarm

class torpedoApproachMission:

    def __init__(self, rc= None,target=None, **config):
        """
        Initialize the mission class; here should be all of the things needed in the run function. 

        Args:
            config: Mission-specific parameters to run the mission.
        """
        self.config = config
        self.cv_files = ["torpedo_approach_cv"] # CV file to run

        self.data = {}  # Dictionary to store the data from the CV handler
        self.next_data = {}  # Dictionary to store the newest data from the CV handler; this data will be merged with self.data.
        self.received = False

        self.rc = rc
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV handlers; dummys are used to input a video file instead of the camera stream as data for the CV script to run on
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        # Work around for not able to subscribe to the topic
        self.return_sub = rospy.Subscriber("/auv/cv_handler/torpedo_approach_cv", String ,self.callback)
        
        if target is not None:
            rospy.loginfo("Setting target...")
            self.cv_handler.set_target("torpedo_approach_cv", target)

        rospy.loginfo("torpedo Approach Mission Init")

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
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

        while not rospy.is_shutdown():
            time.sleep(0.05)
            if not self.received:
                rospy.logwarn("Did not receive frame")
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
            cv_data = self.data.get("torpedo_approach_cv", {})
            state = cv_data.get("state", "search")
            prev_offset = cv_data.get("prev_offset", None)
            lateral = cv_data.get("lateral", 0)
            forward = cv_data.get("forward", 0)
            yaw = cv_data.get("yaw", 0)
            vertical = cv_data.get("vertical", 0)
            end = cv_data.get("end", False)

            if end:
                rospy.loginfo("Ending torpedo Approach CV")
                self.rc.movement()
                rospy.loginfo("Launching torpedos")
                self.rc.move_servo("/auv/devices/torpedo")
                time.sleep(0.2)
                self.rc.move_servo("/auv/devices/torpedo")
                time.sleep(0.2)
                self.rc.move_servo("/auv/devices/torpedo")
                break
            else:
                self.rc.movement(lateral = lateral, forward = forward, yaw = yaw, vertical = vertical)


        rospy.loginfo("torpedo approach mission terminated")

    def cleanup(self):
        """
        Here should be all the code required after the run function.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # Idle the robot
        self.rc.movement(lateral = 0, forward = 0, yaw = 0)
        rospy.loginfo("torpedo approach mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    from auv.utils import deviceHelper
    from auv.motion import robot_control
    rospy.init_node("torpdeo_approach_mission",anonymous = True)
    robotControl = robot_control.RobotControl()
    config = deviceHelper.variables
    robotControl.set_absolute_z(0.8)
    # while abs(robotControl.position['z'] - 0.8)>0.1:
    #     time.sleep(1)
    # rospy.loginfo("Reached depth 0.8")
    rospy.loginfo("Running mission")
    mission = torpedoApproachMission(rc=robotControl, **config)
    mission.run()
    mission.cleanup()

    robotControl.exit()
    rospy.loginfo("Exit mission")
