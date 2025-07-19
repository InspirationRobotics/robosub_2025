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
from auv.device.MiniMaestro.mini_maestro_api import MiniMaestro
from auv.utils import deviceHelper


class MaestroServer:
    def __init__(self, port=deviceHelper.dataFromConfig("polulu")):
        rospy.init_node('maestroServer')
        self.maestro = MiniMaestro()

        self.torpedo_state = {"firing_first": (2, 2400), "firing_second": (2, 1700), "reload_required": (2, 1300)}
        self.dropper_state = {"dropping_first": (1, 1600), "dropping_second": (1, 1200), "reload_required": (1, 700)}
        self.gripper_state = {"static": (0, 1500), "opening": (0, 1550), "closing": (0, 1450)}

        self.has_launched_torpedo = False

        self.dropperService = rospy.Service('/auv/device/dropper', Trigger, self.dropperCallback)
        self.gripperService = rospy.Service('/auv/device/gripper', Trigger, self.gripperCallback)
        self.torpedoService = rospy.Service('/auv/device/torpedo', Trigger, self.torpedoCallback)
        rospy.loginfo("Ready to take servo requests.")
        rospy.spin()

        # TODO setting all servos to default
        
        
    def dropperCallback(self):
        rospy.loginfo("dropping a marker")

        # Logic for dropping one marker
        self.maestro.set_pwm(1,1800)
        time.sleep(0.5) # TODO find time for dropping only one marker
        self.maestro.set_pwm(1,1450)

        return TriggerResponse(
            success=True,
            message="Marker dropped!"
        )


    def gripperCallback(self):
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


    def torpedoCallback(self):
        rospy.loginfo("launching torpedo")

        if self.has_launched_torpedo:
            self.maestro.set_pwm(*self.torpedo_state["firing_second"])
        else:
            self.maestro.set_pwm(*self.torpedo_state["firing_first"])
            self.has_launched_torpedo = True

        return TriggerResponse(
            success=True,
            message="Torpedo launched!"
        )


if __name__ == "__main__":
    server = MaestroServer()