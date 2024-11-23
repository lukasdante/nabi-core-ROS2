import os
import can
import subprocess
import serial.tools.list_ports
from dotenv import load_dotenv
from collections import namedtuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Int8
from rcl_interfaces.msg import ParameterDescriptor
# from nabi_interfaces.msg import MoveData

class Joint(Node):
    def __init__(self, axis, can_id, max_left, max_right, gear_ratio, zero_angle, bus=None):
        super().__init__(axis)

        try:
            # Set the CAN bus
            self.bus = bus

            # If no CAN bus provided
            if bus is None:
                # Find the CAN port
                self.can_port = self.find_can_port()
                if self.can_port is None:
                    self.get_logger().error("No CAN interface found. Please check your connection.")
                    return
                
                # Initialize the CAN communication
                self.change_permissions(self.can_port, "777", os.getenv('PASSWORD'))
                self.bus = can.interface.Bus(interface='slcan', channel=self.can_port, bitrate=500000)

            self.notifier = can.Notifier(self.bus, [self.notifier_callback]) ## add callback



            # Declare parameters for the node
            self.declare_parameter('axis', axis, ParameterDescriptor(description='Name of the joint.'))
            self.declare_parameter('can_id', can_id, ParameterDescriptor(description='Arbitration ID for CAN bus message priority.'))
            self.declare_parameter('max_left', max_left, ParameterDescriptor(description='Maximum clockwise joint angle.'))
            self.declare_parameter('max_right', max_right, ParameterDescriptor(description='Maximum counter-clockwise joint angle.'))
            self.declare_parameter('gear_ratio', gear_ratio, ParameterDescriptor(description='Gear ratio of the joint.'))
            self.declare_parameter('zero_angle', zero_angle, ParameterDescriptor(description='Home or zero angle of the joint.'))
            self.declare_parameter('default_speed', 0x0258, ParameterDescriptor(description='Default speed (rpm) of the joint.'))
            self.declare_parameter('default_acceleration', 0x02, ParameterDescriptor(description='Default acceleration (rpm2) of the joint.'))


            # Set node parameters as class parameters
            self.axis = self.get_parameter('axis').get_parameter_value().string_value
            self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
            self.max_left = self.get_parameter('max_left').get_parameter_value().integer_value
            self.max_right = self.get_parameter('max_right').get_parameter_value().integer_value
            self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().integer_value
            self.zero_angle = self.get_parameter('zero_angle').get_parameter_value().integer_value
            self.default_speed = self.get_parameter('default_speed').get_parameter_value().integer_value
            self.default_acceleration = self.get_parameter('default_acceleration').get_parameter_value().integer_value

            # ROS components
            self.joint_data_publisher = self.create_publisher(String, 'joint/joint_data', 10)
            # self.move_srv = self.create_subscription(MoveData, 'move_joint', self.move_callback)

            # Set joint defaults
            self.current_angle = 0
            self.commands = {'absolute': 0xF5,
                             'relative': 0xF6,
                             'carry': 0x30,
                             'speed': 0x32,
                            }

            self.get_logger().info(f'Joint {self.axis} initialized.')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize joint: {e}')

    def crc(self, data):
        """ Calculate the CRC code for a data bytearray. """
        return sum(data) & 0xFF
    
    def send_message(self, msg):
        """ Sends a message to the CAN bus. """
        try:
            self.bus.send(msg)
            data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data])
            self.get_logger().debug(f"Sent message: can.Message(arbitration_id=0x{msg.arbitration_id:02X}, data=[{data_bytes}], is_extended_id=False)")
        except Exception as e:
            self.get_logger().error(f'Failed to send message: {e}')

    def notifier_callback(self, msg):
        """ Publishes the notifier received messages to a topic. """
        try:
            # TODO: process msg
            self.joint_data_publisher.publish(msg)
            self.get_logger().debug(f'Published notifier message for Joint {self.axis}.')
        except Exception as e:
            self.get_logger().warn(f'Failed to publish Joint {self.axis} notifier: {e}')

    def move(self, position, speed=None, acceleration=None, type='absolute'):
        """ Moves the joint to a specified position, speed, and acceleration. """
        if speed is None:
            speed = self.default_speed
        if acceleration is None:
            acceleration = self.default_acceleration

        data = [
            self.commands[type],
            (speed >> 8) & 0xFF,           # Speed high byte
            speed & 0xFF,                  # Speed low byte
            acceleration,                  # Acceleration byte
            (position >> 16) & 0xFF,       # Position high byte
            (position >> 8) & 0xFF,        # Position middle byte
            position & 0xFF                # Position low byte
        ]
        
        crc = self.crc(data + [self.can_id])
        data.append(crc)  # Add CRC as the last byte

        # Create CAN message
        try:
            msg = can.Message(arbitration_id=self.can_id, data=data,ded_id=False)
            self.send_message(msg)
            return True
        except Exception as e:
            self.get_logger().fatal(f'Failed to move the joint in {type} position mode: {e}')
            return False
        
    def move_callback(self, request, response):
        """ Callback for services. """
        try:
            self.move(request.speed, request.acceleration, request.position, request.type)
            response.result = True
            self.get_logger().info('Successfully moved the joint with the required parameters.')
        except Exception as e:
            response.result = False
            self.get_logger().warn('Failed to move the joint with the required parameters.')

    def read(self, property):
        """ Read joint position or velocity. """
        data = [
            self.commands[property],
        ]

        crc = self.crc(data + [self.can_id])
        data.append(crc)

        try:
            msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
            self.send_message(msg)
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to read the joint's {type}: {e}")
            return False

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

    try:
        rclpy.init(args=args)

        joint = Joint()

        rclpy.spin(joint)
    except KeyboardInterrupt:
        pass

    joint.destroy_node()
    joint.bus.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()