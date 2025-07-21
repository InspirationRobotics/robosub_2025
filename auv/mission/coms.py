"""
Establishes a modem between Graey and Onyx
"""

import time

from auv.mission import gate_mission, cointoss_mission, surfacing_mission, dhd_approach_mission, gate2_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_node import Modem
import rospy

rospy.init_node("coms_mission", anonymous=True)
time.sleep(30)

# Load the sub configuration
config = deviceHelper.variables
rc = robot_control.RobotControl()

fail_modem = False

try:
    modem = Modem()
    modem.send_msg("graey handshake") # Send a modem message
except:
    fail_modem = True
    print("Failed to start modem, starting directly")

arm.arm()

rc.set_depth(0.5) # Set the depth to 0.75 m

time.sleep(90)

modem.stop()

disarm.disarm() # Disarm the sub (just in case)
