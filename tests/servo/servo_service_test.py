import rospy
from auv.motion.robot_control import RobotControl

def test_servo(service_name):
    rc = RobotControl(enable_dvl=False)
    try:
        print(f"Testing servo: {service_name}")
        result = rc.move_servo(service_name)
        print(f"Result from {service_name}: {result}")
    except Exception as e:
        print(f"Error testing {service_name}: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    rospy.init_node("test_servo_functions")
    test_servo("/auv/device/gripper")
    rospy.sleep(2)
    test_servo("/auv/device/dropper")
