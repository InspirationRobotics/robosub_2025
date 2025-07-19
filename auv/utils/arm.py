import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from std_msgs.msg import Bool

def arm():
    """
    Arms the sub.

    Returns:
        success (bool): Whether the sub was successfully armed.
        result (str): Message about the result of the arming operation.
    """
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        req = CommandBoolRequest()
        req.value = True  # Arm

        resp = arm_srv(req)

        # Optional: turn on LED indicator if you have it defined elsewhere
        # statusLed.red(True)

        return resp.success, "Vehicle armed" if resp.success else "Failed to arm vehicle"
    except rospy.ServiceException as e:
        return False, f"Service call failed: {e}"
    
if __name__=="__main__":
    arm()
