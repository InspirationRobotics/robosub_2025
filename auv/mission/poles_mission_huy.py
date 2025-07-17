"""
Poles Slalom Mission Handler.
Runs the mission loop, hands off to CV for visual approach, triggers dead reckoning, counts passed poles.
"""

import rospy
import time
from std_msgs.msg import String

from ..device import cv_handler    # Import your CV handler script
from ..motion import robot_control # For motion commands
from ..utils import disarm, arm

class PolesSlalomMission:
    def __init__(self, side="left", **config):
        self.cv_files = ["poles_cv_slalom"]
        self.config = config
        self.data = {}
        self.next_data = {}
        self.received = False
        self.side = side

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)
        self.cv_handler.set_target("poles_cv_slalom", side)
        print(f"[INFO] Poles Slalom Mission Init (side: {side})")

    def callback(self, msg):
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        while not rospy.is_shutdown():
            if not self.received:
                rospy.sleep(0.01)
                continue

            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            cv_key = "poles_cv_slalom"
            forward = self.data[cv_key].get("forward", 0)
            lateral = self.data[cv_key].get("lateral", 0)
            yaw = self.data[cv_key].get("yaw", 0)
            end = self.data[cv_key].get("end", False)
            pole_idx = self.data[cv_key].get("pole_idx", 0)
            state = self.data[cv_key].get("state", "")

            if end:
                print("[INFO] Poles Slalom mission finished! All red poles passed.")
                self.robot_control.movement(lateral=0, forward=0, yaw=0)
                break
            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                print(f"[CMD] F:{forward} L:{lateral} Y:{yaw} | Targeting red pole {pole_idx+1}/3 | State: {state}")

        print("[INFO] Poles Slalom mission run complete.")

    def cleanup(self):
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)
        self.robot_control.movement(lateral=0, forward=0, yaw=0)
        print("[INFO] Poles Slalom mission terminated.")

if __name__ == "__main__":
    import time
    from auv.utils import deviceHelper
    from auv.motion import robot_control

    rospy.init_node("poles_slalom_mission", anonymous=True)
    config = deviceHelper.variables
    config.update({})

    mission = PolesSlalomMission(side="left", **config)
    rc = robot_control.RobotControl()

    arm.arm()
    rc.set_depth(0.7)
    time.sleep(5)
    mission.run()
    mission.cleanup()
    disarm.disarm()
