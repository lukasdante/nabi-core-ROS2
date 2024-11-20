import can
import time
import os
import serial.tools.list_ports
from dotenv import load_dotenv

from mks_servo_can import can_commands, can_motor, can_set, mks_servo, mks_enums # type: ignore

import subprocess

def change_permissions(file_path, permissions, password):
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
        
        print("Command Output:", result.stdout)
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr)

# Send a CAN message on the bus
def can_send_message(bus, msg):
    bus.send(msg)
    data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data])
    print(f"Sent message: can.Message(arbitration_id=0x{msg.arbitration_id:02X}, data=[{data_bytes}], is_extended_id=False)")
    time.sleep(0.5)  # 500ms delay

# Locate the CAN port for USB2CAN or similar adapter
def find_can_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "CAN" in port.description or "USB2CAN" in port.description:
            return port.device
    return None

# Generate a CAN message with the specified data format
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

# Receive CAN messages from the bus
def can_receive_message(bus):
    while True:
        message = bus.recv()  # Wait for a message
        if message:
            data_bytes = ', '.join([f'0x{byte:02X}' for byte in message.data])
            print(f"Received message: can.Message(arbitration_id=0x{message.arbitration_id:X}, data=[{data_bytes}], is_extended_id=False)")

# Main function for initializing CAN interface and sending messages
def main():
    load_dotenv()
    # Set CAN port
    can_port = find_can_port()
    if can_port is None:
        print("No CAN interface found. Please check your connection.")
        return

    print(f"Using CAN interface on port: {can_port}")
    change_permissions(can_port, "777", password=os.getenv('PASSWORD'))
    bus = can.interface.Bus(bustype='slcan', channel=can_port, bitrate=500000)

    # Prompt user for each input in hexadecimal
    try:
        arbitration_id = int(input("Enter Arbitration ID (in hex, e.g., 0x01): "), 16)
        command = int(input("Enter Command (in hex, e.g., 0xF5): "), 16)
        speed = int(input("Enter Speed (in hex, e.g., 0x0258): "), 16)
        acceleration = int(input("Enter Acceleration (in hex, e.g., 0x02): "), 16)
        position = int(input("Enter Position (in hex, e.g., 0x4000): "), 16)

        # Generate and send the movement message
        msg = generate_movement_message(arbitration_id, command, speed, acceleration, position)
        can_send_message(bus, msg)

        # Start receiving messages (this will run indefinitely)
        print("Listening for incoming CAN messages...")
        can_receive_message(bus)
    except ValueError:
        print("Invalid input. Please enter the values correctly in hexadecimal format.")
    finally:
        # Close the bus after sending all messages
        bus.shutdown()

if __name__ == "__main__":
    main()