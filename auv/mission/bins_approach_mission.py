"""
Bins Approach Mission.
The goal is to find the bin, then switch to bottom facing camera to align with the center.
"""

import json
import time
import rospy
from std_msgs.msg import String

from auv.device import cv_handler
from auv.motion import robot_control
from auv.utils import arm, disarm


class BinsApproachMission:
    cv_files = ["bin_approach_cv"]

    def __init__(self, rc=None, target=None, **config):
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        self.rc = rc
        self.cv_handler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        if target is not None:
            self.cv_handler.set_target("bin_approach_cv", target)

        print("[INFO] Bin Approach Mission Init")


        time.sleep(1)

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        print("[INFO] Bin Approach mission running")

        while not rospy.is_shutdown():
            time.sleep(0.05)
            if not self.received:
                continue

            for key in self.next_data.keys():
                if key in self.data:
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            cv_data = self.data.get("bin_approach_cv", {})
            lateral = cv_data.get("lateral", 0)
            forward = cv_data.get("forward", 0)
            yaw = cv_data.get("yaw", 0)
            vertical = cv_data.get("vertical", 0)
            end = cv_data.get("end", False)

            print(f"[MOTION] Fwd: {forward}, Lat: {lateral}, Yaw: {yaw}, Vert: {vertical}")

            if end:
                print("[INFO] Ending Bin Approach CV")
                self.rc.movement()
                self.rc.movement(lateral=2)
                time.sleep(1)
                self.rc.movement()
                self.rc.movement(forward=-2)
                time.sleep(0.7)
                self.rc.movement(0)
                break
            else:
                self.rc.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=vertical)

        print("[INFO] Bin approach mission terminated")

    def cleanup(self):
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        self.rc.movement()
        print("[INFO] Bin approach mission terminated")


if __name__ == "__main__":
    from auv.utils import deviceHelper
    from auv.motion import robot_control
    rospy.init_node("bins_approach_mission")
    robotControl = robot_control.RobotControl()
    config = deviceHelper.variables
    robotControl.set_absolute_z(0.6)
    while abs(robotControl.position['z'] - 0.6)>0.1:
        time.sleep(1)
    rospy.loginfo("Reached depth 0.6")

    mission = BinsApproachMission(rc=robotControl**config)
    mission.run()
    mission.cleanup()

    robotControl.exit()
    rospy.loginfo("Exit mission")