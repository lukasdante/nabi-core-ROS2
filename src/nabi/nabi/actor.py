from dotenv import load_dotenv
from typing import List
import subprocess
import serial.tools.list_ports
import time
import can
import os

from .joint import Joint

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor, Parameter

class JointActor(Node):
    def __init__(self, joint_configs):
        super().__init__('joint_actor')

        try:

            # Find the CAN port
            self.can_port = self.find_can_port()
            if self.can_port is None:
                self.get_logger().error("No CAN interface found. Please check your connection.")
                return
            
            # Initialize the CAN communication
            self.change_permissions(self.can_port, "777", os.getenv('PASSWORD'))

            # Set the CAN bus
            self.bus = can.interface.Bus(interface='slcan', channel=self.can_port, bitrate=500000)
            
            self.joints: List[Joint] = []
            for config in joint_configs:
                joint = Joint(**config, bus=self.bus)
                self.joints.append(joint)

            self.timer = self.create_timer(0.1, self.spin_joints)
            self.stop_loop = False

            self.jumpstart_conversation()

            self.get_logger().info(f'JointActor node initialized.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize JointActor node: {e}')

    def jumpstart_conversation(self):
        jumpstart = Bool()
        jumpstart.data = True
        self.create_publisher(Bool, 'conversation/jumpstart', 10).publish(jumpstart)

    def spin_joints(self):
        for joint in self.joints:
            rclpy.spin_once(joint, timeout_sec=0.01)

    def move_sync(self, joints: List[Joint], joint_angles, joint_velocities, joint_accelerations):
        try:
            for joint, angle, velocity, acceleration in zip(joints, joint_angles, joint_velocities, joint_accelerations):
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

    def preset_callback(self, msg):
        pass

    def move_preset(self, preset_angles: List[List]):
        """ Moves to a preset position. """
        self.reset_position()

        while not self.stop_loop:
            for preset_angle in preset_angles:
                self.move_sync(self.joints[:6], preset_angle[:6])
                # TODO: Wait until all the joints are done moving
                time.sleep(0.5)
                self.joints[6].move(preset_angle[6])
                time.sleep(0.5)

        self.reset_position()

    def change_permissions(self, file_path, permissions, password):
        """ Changes permission of the USB ports to allow automated CAN bus connection. """
        try:
            # Construct the chmod command
            command = ["sudo", "-S", "chmod", permissions, file_path]
            
            # Pass the password via stdin
            subprocess.run(
                command,
                input=f"{password}\n",
                text=True,
                capture_output=True,
                check=True
            )
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Subprocess error: {e}')

    # Locate the CAN port for USB2CAN or similar adapter
    def find_can_port(self):
        """ Finds a CAN connection from USB ports. """
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "CAN" in port.description or "USB2CAN" in port.description:
                return port.device
        return None

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

if __name__ == '__main__':
    main()