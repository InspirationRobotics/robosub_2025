import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest

def disarm():
    """
    Disarms the sub.

    Returns:
        success (bool): Whether the sub was successfully disarmed.
        result (str): Message about the result of the disarming operation.
    """
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        disarm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        req = CommandBoolRequest()
        req.value = False  # Disarm

        resp = disarm_srv(req)

        # Optional: turn off LED indicator if you use one
        # statusLed.red(False)

        return resp.success, "Vehicle disarmed" if resp.success else "Failed to disarm vehicle"
    except rospy.ServiceException as e:
        return False, f"Service call failed: {e}"
    

if __name__=="__main__":
    disarm()
