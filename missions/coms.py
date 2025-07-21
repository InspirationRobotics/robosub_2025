import time

from auv.mission import gate_mission, cointoss_mission, surfacing_mission, dhd_approach_mission, gate2_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_node import Modem
import rospy

rospy.init_node("coms_mission", anonymous=True)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()

fail_modem = False

try:
    modem = Modem()
    modem.send_msg("graey handshake")
except:
    fail_modem = True
    print("Failed to start modem, starting directly")

time.sleep(30)
arm.arm()

rc.set_depth(0.75)

time.sleep(60)

disarm.disarm()