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


class BinsDropMission:
    cv_files = ["bin_drop_cv"]

    def __init__(self, rc = None, target="sawfish", **config):
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False

        self.rc = rc
        self.cv_handler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        time.sleep(5) # wait for the cv script to initialize
        if target is not None:
            self.cv_handler.set_target("bin_drop_cv", target)

        print("[INFO] Bin Approach Mission Init")

        time.sleep(1)

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        print("[INFO] Bin drop mission running")

        drop = False
        while not rospy.is_shutdown():
            time.sleep(0.01)
            if not self.received:
                continue

            for key in self.next_data.keys():
                if key in self.data:
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            cv_data = self.data.get("bin_drop_cv", {})
            lateral = cv_data.get("lateral", 0)
            forward = cv_data.get("forward", 0)
            yaw = cv_data.get("yaw", 0)
            vertical = cv_data.get("vertical", 0)
            end = cv_data.get("end", False)
            drop = cv_data.get("drop", False)

            rospy.loginfo(f"cv_data | {cv_data}")
            if end:
                print("[INFO] Ending Bins CV")
                self.rc.movement(lateral=0, forward=0, yaw=0, vertical=0)
                if drop:
                    self.rc.move_servo("/auv/device/dropper")
                    time.sleep(0.2)
                    self.rc.move_servo("/auv/device/dropper")
                break
            else:
                self.rc.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=vertical)

        print("[INFO] Bins mission terminated")

    def cleanup(self):
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        self.rc.movement(lateral=0, forward=0, yaw=0, vertical=0)
        print("[INFO] Bins mission terminated")


if __name__ == "__main__":
    from auv.utils import deviceHelper

    rospy.init_node("bins_mission", anonymous=True)
    RC = robot_control.RobotControl()
    RC.set_flight_mode("STABILIZE")
    config = deviceHelper.variables

    mission = BinsDropMission(rc=RC, **config)

    mission.run()
    mission.cleanup()

    RC.exit()