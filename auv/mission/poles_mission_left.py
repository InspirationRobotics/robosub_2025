"""
Mission file for red pole slalom
"""

import json
import rospy
from std_msgs.msg import String
import time

from auv.device import cv_handler  # For running mission-specific CV scripts
from auv.motion import robot_control  # For running the motors on the sub
from auv.utils import disarm


class PoleSlalomMission:
    # Name of your red pole CV script file (no .py extension)
    def __init__(self, rc, target="left",**config):
        """
        Initialize the mission class; configure everything needed in the run function.
        """
        self.cv_files = ["poles_cv"]
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False
        self.target = target
        self.rc = rc
        self.cv_handler = cv_handler.CVHandler(**self.config)
        self.row_count = 0
        self.end = False

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("poles_cv", target)
        rospy.loginfo("[INFO] Pole Slalom Mission Init")

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        """
        Run the pole slalom mission loop.
        """
        rospy.loginfo("Pole Slalom mission running")

        while not rospy.is_shutdown():
            if not self.received:
                time.sleep(0.01)
                continue

            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]

            self.received = False
            self.next_data = {}

            cv_data = self.data.get("poles_cv", {})
            lateral = cv_data.get("lateral", 0)
            forward = cv_data.get("forward",1.0)
            # yaw = cv_data.get("yaw", 0)
            end = cv_data.get("end", False)
            reached = cv_data.get("reached",False)

            # rospy.loginfo(cv_data)
            if end: # time out
                rospy.loginfo("Pole slalom mission time out.")
                self.rc.movement()
                break
            elif self.row_count==3:
                rospy.loginfo("Pole Slalom Mission completed")
                self.rc.movement()
                break
            elif reached:
                # When we reached certain distance away from the red pole, perform preset maneuver
                self.rc.movement()

                # move lateral right for ###
                self.rc.movement(lateral=-2)
                time.sleep(2.5)
                self.rc.movement()

                # move forward for ###
                self.rc.movement(forward=3)
                time.sleep(3.0)
                self.rc.movement()

                # move lateral left for ###
                self.rc.movement(lateral=2)
                time.sleep(2.0)
                self.rc.movement()

                # Increase row count by one
                self.row_count += 1 

                # print status
                rospy.loginfo(f"Current row: {self.row_count}")
                reached = False
            else:
                self.rc.movement(lateral=lateral, forward=forward)

            time.sleep(0.01)

        # END OF WHILE LOOP
        self.rc.movement(forward=3)
        time.sleep(2.5)
        rospy.loginfo("Pole Slalom mission run complete")

    def cleanup(self):
        """
        Clean up after the mission.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        self.rc.movement(lateral=0, forward=0, yaw=0)
        print("[INFO] Pole Slalom mission terminated")

if __name__ == "__main__":
    from auv.utils import deviceHelper
    import time

    rospy.init_node("pole_slalom_mission", anonymous=True)
    config = deviceHelper.variables
    mission = PoleSlalomMission(target="right",rc=robot_control.RobotControl(),**config)
    mission.run()
    mission.cleanup()