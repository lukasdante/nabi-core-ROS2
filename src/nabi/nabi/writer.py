import os
import requests
import base64
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class Writer(Node):
    def __init__(self):
        super().__init__('writer')

        try:
            self.api_key = os.getenv('STT_API_KEY')
            self.api_endpoint = f'https://speech.googleapis.com/v1/speech:recognize?key={self.api_key}'
            
            self.declare_parameter('language', 'en-US')
            self.declare_parameter('sample_rate', 16000)
            self.declare_parameter('encoding_format', 'LINEAR16')
            self.declare_parameter('input_file', 'input.wav')

            self.language = self.get_parameter('language').get_parameter_value().string_value
            self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
            self.encoding_format = self.get_parameter('encoding_format').get_parameter_value().string_value
            self.input_file = self.get_parameter('input_file').get_parameter_value().string_value

            self.publisher = self.create_publisher(String, 'conversation/transcript', 10)
            self.reset_publisher = self.create_publisher(Bool, 'conversation/reset', 10)
            self.subscriber = self.create_subscription(String, 'conversation/request_audio', self.write, 10)

            self.get_logger().info("Writer initialized.")
        except Exception as e:
            self.get_logger().error(f"Unable to initialize writer: {e}")

    def write(self, msg):
        """ Encodes audio to text. """

        # Configure the request
        headers = {'Content-Type': 'application/json'}
        body = {
            'config': {
                'encoding': self.encoding_format,
                'sampleRateHertz': self.sample_rate,
                'languageCode': self.language
            },
            'audio': {
                'content': msg.data
            }
        }

        # Send the request to the API endpoint
        try:
            response = requests.post(self.api_endpoint, headers=headers, json=body)
        except Exception as e:
            self.get_logger().error(f"Writer request error: {e}")

        # Check if the request is successful
        if response.status_code == 200:
            result = response.json()

            # Get the transcription from results
            if 'results' in result:
                for res in result['results']:
                    transcription = res['alternatives'][0]['transcript']
                    self.get_logger().info(f"Transcript: {transcription}")

                    # Prepare message
                    msg = String()
                    msg.data = transcription
                    
                    # Publish the message
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Transcription published.")

                    return transcription
            else:
                self.get_logger().info("No transcription found, resetting conversation.")

                # Prepare message
                msg = Bool()
                msg.data = True
                
                # Publish message
                self.reset_publisher.publish(msg)
                return None
        else:
            self.get_logger().error(f"Writer request error {response.status_code}.")
            return None


def main(args=None):
    load_dotenv()

    rclpy.init(args=args)

    lone_writer = Writer()

    rclpy.spin(lone_writer)

    lone_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()