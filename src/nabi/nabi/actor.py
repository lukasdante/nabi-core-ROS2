from dotenv import load_dotenv
from typing import List

from .joint import Joint

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor, Parameter

class JointActor(Node):
    def __init__(self, joint_configs):
        super().__init__('joint_actor')

        try:
            self.joints: List[Joint] = []
            for config in joint_configs:
                joint = Joint(**config)
                self.joints.append(joint)

            self.timer = self.create_timer(0.1, self.spin_joints)

            self.get_logger().info(f'JointActor node initialized.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize JointActor node: {e}')

    def spin_joints(self):
        for joint in self.joints:
            rclpy.spin_once(joint, timeout_sec=0.01)

    def move_sync(self, joint_angles, joint_velocities, joint_accelerations):
        try:
            for joint, angle, velocity, acceleration in zip(self.joints, joint_angles, joint_velocities, joint_accelerations):
                joint.move(angle, velocity, acceleration)
        except Exception as e:
            self.get_logger().error(f'Failed to move the motors synchronously: {e}')

    def reset_position(self):
        try:
            for joint in self.joints:
                joint.move(joint.zero_angle)

        except Exception as e:
            self.get_logger().error(f'Failed to move the motors synchronously: {e}')

    def terminate_process(self):
        pass

    def preset_callback(self):
        pass

    def move_preset(self):
        pass

def main(args=None):
    load_dotenv()

    joint_configs = [
        {'axis': 'X', 'can_id': 0x01, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
        {'axis': 'Y', 'can_id': 0x02, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
        {'axis': 'Z', 'can_id': 0x03, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
        {'axis': 'A', 'can_id': 0x04, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
        {'axis': 'B', 'can_id': 0x05, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
        {'axis': 'C', 'can_id': 0x06, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
        {'axis': 'G', 'can_id': 0x07, 'max_left': -90, 'max_right': 90, 'gear_ratio': 1, 'zero_angle': 0},
    ]

    try:
        rclpy.init(args=args)

        joint_actor = JointActor(joint_configs=joint_configs)

        rclpy.spin(joint_actor)
    except KeyboardInterrupt:
        pass

    joint_actor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()