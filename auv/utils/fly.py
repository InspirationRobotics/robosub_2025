#!/usr/bin/env python3

import rospy
import sys
from mavros_msgs.srv import SetMode, SetModeRequest

def set_flight_mode(mode="MANUAL"):
    """
    Sets the flight mode using MAVROS set_mode service.

    Args:
        mode (str): The desired flight mode (default is "MANUAL").

    Returns:
        success (bool): True if the mode was successfully set.
        result (str): A message with the result.
    """
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        req = SetModeRequest()
        req.custom_mode = mode

        resp = set_mode_srv(req)
        return resp.mode_sent, f"Mode set to '{mode}'" if resp.mode_sent else f"Failed to set mode '{mode}'"
    except rospy.ServiceException as e:
        return False, f"Service call failed: {e}"

if __name__ == "__main__":
    rospy.init_node("set_mode_client", anonymous=True)

    # Get mode from terminal argument, default to MANUAL
    mode = sys.argv[1] if len(sys.argv) > 1 else "MANUAL"

    success, message = set_flight_mode(mode)
    rospy.loginfo(f"[SET_MODE] {message}")
