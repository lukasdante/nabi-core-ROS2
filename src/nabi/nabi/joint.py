import os
import can
import subprocess
import serial.tools.list_ports
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor

# class CanBus(Node):
#     _instance = None  # Singleton instance

#     def __new__(cls, *args, **kwargs):
#         # Check if an instance already exists
#         if cls._instance is None:
#             cls._instance = super(CanBus, cls).__new__(cls)
#         return cls._instance
    
#     def __init__(self):
#         if hasattr(self, 'initialized') and self.initialized:
#             return

#         super().__init__('canbus')
#         try:
#             self.can_port = self.find_can_port()
#             if self.can_port is None:
#                 self.get_logger().error("No CAN interface found. Please check your connection.")
#                 return
            
#             self.password = os.getenv('PASSWORD')
            
#             # Initialize the CAN Bus
#             self.change_permissions(self.can_port, "777", self.password)
#             self.declare_parameter('bitrate', 500000,
#                                 ParameterDescriptor(description='Bitrate of CAN bus.'))
#             self.bitrate = self.get_parameter('bitrate').get_parameter_value().string_value
#             self.bus = can.interface.Bus(interface='slcan', channel=self.can_port)

#             self.get_logger().info(f'CAN bus initialized, proceeding to initializing joints.')
        
#         except Exception as e:
#             self.get_logger().error(f'Failed to initialize CAN bus: {e}')
        
#         self.initialized = True

#     def change_permissions(self, file_path, permissions, password):
#         try:
#             # Construct the chmod command
#             command = ["sudo", "-S", "chmod", permissions, file_path]
            
#             # Pass the password via stdin
#             result = subprocess.run(
#                 command,
#                 input=f"{password}\n",
#                 text=True,
#                 capture_output=True,
#                 check=True
#             )
            
#         except subprocess.CalledProcessError as e:
#             self.get_logger().error(f'Subprocess error: {e}')

#     # Locate the CAN port for USB2CAN or similar adapter
#     def find_can_port(self):
#         ports = list(serial.tools.list_ports.comports())
#         for port in ports:
#             if "CAN" in port.description or "USB2CAN" in port.description:
#                 return port.device
#         return None


class Joint(Node):
    def __init__(self):
        super().__init__('joint')

        try:
            # Find the CAN port
            self.can_port = self.find_can_port()
            if self.can_port is None:
                self.get_logger().error("No CAN interface found. Please check your connection.")
                return
            
            # Initialize the CAN communication
            self.change_permissions(self.can_port, "777", os.getenv('PASSWORD'))

            # Declare parameters for the node
            self.declare_parameter('can_id', 0x01,
                                   ParameterDescriptor(description='Arbitration ID for CAN bus message priority.'))
            self.declare_parameter('bitrate', 500000,
                                ParameterDescriptor(description='Bitrate of CAN bus.'))
            
            # Set node parameters as class parameters
            self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
            self.bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value

            # Set the CAN bus
            self.bus = can.interface.Bus(interface='slcan', channel=self.can_port, bitrate=self.bitrate)

            self.get_logger().info('Joint initialized.')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize joint: {e}')

    def change_permissions(self, file_path, permissions, password):
        try:
            # Construct the chmod command
            command = ["sudo", "-S", "chmod", permissions, file_path]
            
            # Pass the password via stdin
            result = subprocess.run(
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
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "CAN" in port.description or "USB2CAN" in port.description:
                return port.device
        return None

    def crc(self, data):
        return sum(data) & 0xFF
    
    def send_can_message(self, msg):
        self.bus.send(msg)
        data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data])
        self.get_logger().info(f"Sent message: can.Message(arbitration_id=0x{msg.arbitration_id:02X}, data=[{data_bytes}], is_extended_id=False)")

    def generate_movement_message(arbitration_id, command, speed, acceleration, position):
        data = [
            command,                       # Command byte
            (speed >> 8) & 0xFF,           # Speed high byte
            speed & 0xFF,                  # Speed low byte
            acceleration,                  # Acceleration byte
            (position >> 16) & 0xFF,       # Position high byte
            (position >> 8) & 0xFF,        # Position middle byte
            position & 0xFF                # Position low byte
        ]
        
        # Compute CRC as the sum of all relevant bytes modulo 256
        crc = (arbitration_id + command + (speed >> 8) + (speed & 0xFF) + acceleration + 
                (position >> 16) + ((position >> 8) & 0xFF) + (position & 0xFF)) & 0xFF
        
        data.append(crc)  # Add CRC as the last byte

        # Create CAN message
        return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    
    def can_receive_message(self, bus):
        while True:
            message = bus.recv()  # Wait for a message
            if message:
                data_bytes = ', '.join([f'0x{byte:02X}' for byte in message.data])
                print(f"Received message: can.Message(arbitration_id=0x{message.arbitration_id:X}, data=[{data_bytes}], is_extended_id=False)")
                break

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