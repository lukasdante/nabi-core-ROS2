import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor

class PredefinedActor(Node):
    def __init__(self):
        super().__init__('predefined_actions')
        try:
            self.create_subscription(String, 'conversation/transcript', self.predefined_actor_check, 10)

            self.get_logger().info(f'Predefined Actor initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Predefined Actor node: {e}')

    class PredefinedAction:
        def reset_to_zero(self):
            """ A predefined action that resets a motor to home or zero position. """
            pass

    def predefined_actor_check(self, transcript: String):
        """ Checks whether the parameters include a call for a predefined action. """
        if 'reset' in transcript.data:
            self.PredefinedAction.reset_to_zero()
