"""
This is the servo node that controls all servos connected to our pololo mini maestro
Service names:
    - dropper : /auv/device/dropper
    - gripper : /auv/device/gripper
    - torpedo:  /auv/device/torpedo
"""
import rospy
import time
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from auv.device.maestro.mini_maestro_api import MiniMaestro
from auv.utils import deviceHelper


class MaestroServer:
    def __init__(self, port=deviceHelper.dataFromConfig("polulu")):
        rospy.init_node('maestroServer')
        self.maestro = MiniMaestro(port=port)

        self.torpedo_state = {"firing_first": (2, 1800), "firing_second": (2, 1656), "reload_required": (2, 2300)}
        self.dropper_state = {"beginning_position": (1,1765), "dropping_first": (1, 1136), "dropping_second": (1,750), "beginning_position": (1,1765)}
        self.gripper_state = {"static": (0, 1500), "opening": (0, 1550), "closing": (0, 1450)}

        self.has_launched_torpedo = False
        self.has_reloaded_torpedo = False

        self.has_dropped1_marker = False
        self.has_dropped2_marker = False

        self.dropperService = rospy.Service('/auv/device/dropper', Trigger, self.dropperCallback)
        self.gripperService = rospy.Service('/auv/device/gripper', Trigger, self.gripperCallback)
        self.torpedoService = rospy.Service('/auv/device/torpedo', Trigger, self.torpedoCallback)

        # Set all servos to default state
        self.maestro.set_pwm(*self.torpedo_state["reload_required"])
        self.maestro.set_pwm(*self.dropper_state["beginning_position"])
        self.maestro.set_pwm(*self.gripper_state["static"])
        
        rospy.loginfo("Ready to take servo requests.")
        rospy.spin()
        
    def dropperCallback(self, request):
        rospy.loginfo("dropping a marker")
       
        if not self.has_dropped1_marker:
            self.maestro.set_pwm(*self.dropper_state["dropping_first"])
            self.has_dropped1_marker = True

        elif not self.has_dropped2_marker:
            self.maestro.set_pwm(*self.dropper_state["dropping_second"])
            self.has_dropped2_marker = True

        else:
            # reset for the next cycle
            self.maestro.set_pwm(*self.dropper_state["beginning_position"])
            self.has_dropped1_marker = False
            self.has_dropped2_marker = False

        """self.maestro.set_pwm(0,1765)  # Move servo on channel 0
        time.sleep(1.0)
        self.maestro.set_pwm(0,1136)  # Move servo on channel 0
        print("marker 1 dropped")
        time.sleep(2.0) # adjust this number for the amount of time
        self.maestro.set_pwm(0,750) # Move servo on channel 0
        print("marker 2 dropped")
        time.sleep(1.0)
        self.maestro.set_pwm(0,1765)  # Move servo on channel 0
        time.sleep(1)
        """

        return TriggerResponse(
            success=True,
            message="The markers are dropped!"
        )


    def gripperCallback(self, request):
        rospy.loginfo("Gripper has been triggered")

        def gripper_state(state: str):
            """Helper function for gripper state
            Args:
                - state: State (opening, static, closing)"""
            if state not in ["opening", "static", "closing"]:
                return ValueError("Incorrect state")
            start_time = time.time()
            while time.time() - start_time < 0.5:
                self.maestro.set_pwm(*self.gripper_state[state])
                time.sleep(0.05)

        # Open gripper
        gripper_state("opening")
        
        # Wait for 0.5 secs
        gripper_state("static")

        # Close gripper
        gripper_state("closing")

        return TriggerResponse(
            success=True,
            message="Triggered successfully!"
        )


    def torpedoCallback(self, request):
        rospy.loginfo("launching torpedo")

        if not self.has_launched_torpedo:
            self.maestro.set_pwm(*self.torpedo_state["firing_first"])
            self.has_launched_torpedo = True

        elif not self.has_reloaded_torpedo:
            self.maestro.set_pwm(*self.torpedo_state["firing_second"])
            self.has_reloaded_torpedo = True

        else:
            # reset for the next cycle
            self.maestro.set_pwm(*self.torpedo_state["reload_required"])
            self.has_launched_torpedo = False
            self.has_reloaded_torpedo = False


        return TriggerResponse(
            success=True,
            message="Torpedo launched!"
        )


if __name__ == "__main__":
    rospy.init_node('maestroServer')
    server = MaestroServer()