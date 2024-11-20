import os
import can
import subprocess
import serial.tools.list_ports
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterDescriptor

class CanBus(Node):
    def __init__(self):
        super().__init__('canbus')
        try:
            self.can_port = self.find_can_port()
            if self.can_port is None:
                self.get_logger().error("No CAN interface found. Please check your connection.")
                return
            
            self.password = os.getenv('PASSWORD')
            
            # Initialize the CAN Bus
            self.change_permissions(self.can_port, "777", self.password)

            # self.declare_parameter('bitrate', 500000,
            #                     ParameterDescriptor(description='Bitrate of CAN bus.'))
            # self.bitrate = self.get_parameter('bitrate').get_parameter_value().string_value
            # self.bus = can.interface.Bus(interface='slcan', channel=self.can_port)
            self.publisher = self.create_publisher(String, 'can/port', 10)
            
            msg = String()
            msg.data = self.can_port
            self.publisher.publish(msg)

            self.get_logger().info(f'CAN bus initialized, proceeding to joint initialization.')
        
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')

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

def main(args=None):
    load_dotenv()

    rclpy.init(args=args)

    canbus = CanBus()

    rclpy.spin_once(canbus)

if __name__ == '__main__':
    main()