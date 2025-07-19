import signal
import sys
import rospy
from auv.motion.robot_control import RobotControl
from traceback import print_exc

def test_servo(service_name):
    rc = RobotControl(enable_dvl=False)
    try:
        print(f"Testing servo: {service_name}")
        result = rc.move_servo(service_name)
        #print("Result from {service_name}: {result}".format(service_name=service_name, result=result))
        print(f"Result from {service_name}: {result}")
    except Exception as e:
        print(f"Error testing {service_name}: {e}")
        print_exc()

def shutdown_handler(signum, frame):
    print("[INFO] Shutdown signal received. Cleaning up...")
    rospy.signal_shutdown("Shutdown requested")

if __name__ == "__main__":
    # Register signal handlers for safe shutdown
    signal.signal(signal.SIGINT, shutdown_handler)   # Handles Ctrl+C
    signal.signal(signal.SIGTERM, shutdown_handler)  # Handles kill/termination

    try:
        test_servo("/auv/device/gripper")
        rospy.sleep(2)
        test_servo("/auv/device/dropper")
        rospy.sleep(2)
        test_servo("/auv/device/torpedo")
        rospy.sleep(2)
        # Keep node alive until shutdown
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    except Exception as e:
        print_exc()
    finally:
        print("[INFO] Performing safe shutdown tasks...")
        # Place any additional cleanup here (e.g., closing files, resetting hardware)
        print("[INFO] Shutdown complete.")
        sys.exit(0)
