"""
Mission file for red pole slalom
"""

import json
import rospy
from std_msgs.msg import String

from auv.device import cv_handler  # For running mission-specific CV scripts
from auv.motion import robot_control  # For running the motors on the sub
from auv.utils import disarm


class PoleSlalomMission:
    # Name of your red pole CV script file (no .py extension)
    def __init__(self, rc, target="right"):
        """
        Initialize the mission class; configure everything needed in the run function.
        """
        self.cv_files = ["poles_cv"]
        self.data = {}
        self.next_data = {}
        self.received = False
        self.target = target
        self.rc = rc
        self.cv_handler = cv_handler.CVHandler()

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("poles_cv", target)
        print("[INFO] Pole Slalom Mission Init")

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True
        print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Run the pole slalom mission loop.
        """
        print("[INFO] Pole Slalom mission running")
        
        # configuring robot control modes and headings
        self.rc.set_control_mode('depth_hold')
        self.rc.set_absolute_z(0.5)
        self.rc.go_to_heading(0)
        time.sleep(1)
        self.rc.set_absolute_yaw(0)
        self.rc.activate_heading_control(activate=True)
        time.sleep(2)
        
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
            forward = cv_data.get("forward", 0)
            yaw = cv_data.get("yaw", 0)
            end = cv_data.get("end", False)

            print("[MOTION] Fwd: {forward}, Lat: {lateral}")

            if end:
                print("[INFO] Pole slalom mission complete.")
                self.rc.movement(lateral=0, forward=0)
                break
            else:
                self.rc.movement(lateral=lateral, forward=forward, yaw=yaw)

            time.sleep(0.01)

        print("[INFO] Pole Slalom mission run complete")

    def cleanup(self):
        """
        Clean up after the mission.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        self.rc.movement(lateral=0, forward=0, yaw=0)
        print("[INFO] Pole Slalom mission terminated")


if __name__ == "__main__":
    
    import time

    rospy.init_node("pole_slalom_mission", anonymous=True)

    mission = PoleSlalomMission(rc=robot_control.RobotControl())
    mission.run()
    mission.cleanup()