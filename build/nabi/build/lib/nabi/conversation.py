#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
import argparse
from pathlib import Path
from recorder import Recorder
from writer import Writer
from parser import Parser
from talker import Talker
from dotenv import load_dotenv
from ctypes import *

class ConversationalNode(Node):
    def __init__(self, recorder: Recorder, writer: Writer, parser: Parser, talker: Talker):
        super().__init__('conversational_node')  # Initialize ROS 2 Node
        self.recorder = recorder
        self.writer = writer
        self.parser = parser
        self.talker = talker

        # Define ROS 2 Publisher
        self.parameters_publisher = self.create_publisher(String, 'conversation/parameters', 10)

    def publish(self, msg):
        msg = json.dumps(msg)
        message = String()
        message.data = msg
        self.parameters_publisher.publish(message)
        self.get_logger().info("Parameters have been published.")

    def start_conversation(self):
        try:
            while rclpy.ok():
                # Record audio stream until silence is detected
                self.recorder.record()

                if self.recorder.has_speech:
                    # Transcribe audio to text
                    transcription = self.writer.write()

                    if transcription:

                        if 'terminate' in transcription:
                            self.talker.talk('Terminating conversation.')
                            break
                        
                        # Obtain AI agent response and parameters extracted
                        intent = self.parser.detect_intent(transcription)

                        # Publish the parameters to the conversation/parameters topic
                        self.publish(intent['parameters'])
                        
                        # Convert response to audio and play the audio
                        self.talker.talk(intent['response'])

        except Exception as e:
            self.get_logger().error(f"Failed to start conversation: {e}")

def main(args=None):
    # Handle ASLA errors for cleaner output.
    ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
    def py_error_handler(filename, line, function, err, fmt):
        pass
    c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    
    # Load environment variables
    load_dotenv()

    # Argument passer for scripting
    parser = argparse.ArgumentParser()

    # Argument for configuration
    parser.add_argument(
        "--config",
        type=str,
        required=False,
        default='config.yaml',
        help="Configuration file to instantiate the classes, should be found in nabi package, defaults to config.yaml."
    )
    args = parser.parse_args()

    # Get config.yaml directory
    cwd = Path.cwd()
    config_path = (cwd.parent / args.config).resolve()

    # Initialize ROS 2 Node
    rclpy.init(args=args)

    # Using config.yaml, initialize ConversationalNode()
    try:
        with open(config_path) as file:
            config = yaml.safe_load(file)

            recorder = Recorder(config)
            writer = Writer(config)
            parser = Parser(config)
            talker = Talker(config)
            node = ConversationalNode(recorder, writer, parser, talker)

            node.get_logger().info('Successfully initialized conversational node.')
            
    except Exception as e:
        rclpy.get_logger().error(f'Failed to initialize conversational node: {e}.')

    node.start_conversation()

if __name__ == '__main__':
    main()
