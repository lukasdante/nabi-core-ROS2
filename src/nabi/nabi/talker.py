import os
import json
import time
import requests
import wave
import pyaudio
import base64
import io
from pathlib import Path
from dotenv import load_dotenv
from ctypes import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor

from google.auth import jwt as google_jwt
from google.auth import crypt

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        try:
            self.service_account = str(Path(__file__).parent.parent.parent.parent) + '/json/' + os.getenv("TTS_SERVICE_ACCOUNT")

            with open(self.service_account) as f:
                self.service_account_info = json.load(f)

            self.declare_parameter('language', 'en-US',
                                   ParameterDescriptor(description='Language of the transcription output.'))
            self.declare_parameter('gender', 'FEMALE',
                                   ParameterDescriptor(description='Gender of the text-to-speech voice agent.'))
            self.declare_parameter('accent', 'en-US-Neural2-C',
                                   ParameterDescriptor(description='Accent or voice type of the text-to-speech voice agent.'))
            self.declare_parameter('encoding_format', 'LINEAR16',
                                   ParameterDescriptor(description='Encoding format of the audio recording.'))
            self.declare_parameter('token_life', 3600,
                                   ParameterDescriptor(description='Validity of the text-to-speech API token in seconds.'))

            self.language = self.get_parameter('language').get_parameter_value().string_value
            self.gender = self.get_parameter('gender').get_parameter_value().string_value
            self.accent = self.get_parameter('accent').get_parameter_value().string_value
            self.encoding_format = self.get_parameter('encoding_format').get_parameter_value().string_value
            self.token_validity = self.get_parameter('token_life').get_parameter_value().integer_value

            self.token_url = "https://oauth2.googleapis.com/token"
            self.scopes = "https://www.googleapis.com/auth/cloud-platform"
            self.api_endpoint = "https://texttospeech.googleapis.com/v1/text:synthesize"

            
            self.publisher = self.create_publisher(Bool, 'conversation/reset', 10)
            self.subscriber = self.create_subscription(String, 'conversation/response', self.talk, 10)

            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)

            self.get_logger().info("Talker initialized.")
        except Exception as e:
            self.get_logger().error(f"Unable to initialize talker: {e}")

    def write_json(self, text):
        """ Writes JSON payload for Text-to-Speech Inference """
        
        data = {
            "input": {
                "text": f"{text}"
            },
            "voice": {
                "languageCode": f"{self.language}",
                "name": f"{self.accent}",
                "ssmlGender": f"{self.gender}"
            },
            "audioConfig": {
                "audioEncoding": f"{self.encoding_format}"
            }
        }

        return json.dumps(data)

    def vocalize(self, data):
        """ Returns speech of a given API response. """

        now = int(time.time())
        payload = {
            "iss": self.service_account_info["client_email"],
            "scope": self.scopes,
            "aud": self.token_url,
            "iat": now,
            "exp": now + self.token_validity
        }

        # Sign the JWT using the private key from the service account
        signed_jwt = google_jwt.encode(crypt.RSASigner.from_service_account_info(self.service_account_info), payload)

        # Request access token
        response = requests.post(self.token_url, data={
            "grant_type": "urn:ietf:params:oauth:grant-type:jwt-bearer",
            "assertion": signed_jwt
        })

        # Check response
        if response.status_code == 200:
            access_token = response.json()["access_token"]

            # Use the access token to call the Text-to-Speech API
            api_headers = {
                "Authorization": f"Bearer {access_token}",
                "Content-Type": "application/json; charset=utf-8"
            }

            # Make the POST request
            api_response = requests.post(self.api_endpoint, headers=api_headers, data=data)

            api_response_text = api_response.text

            # Print a message indicating completion
            if api_response.status_code == 200:
                self.get_logger().info(f"Response saved.")
            else:
                self.get_logger().error(f"API Error: {api_response.status_code} {api_response.text}")
        else:
            self.get_logger().error(f"Error: {response.status_code} {response.text}")

        

        return api_response_text

    def talk(self, msg):
        """ Talks given a response in text. """

        response = self.write_json(msg.data)
        speech = self.vocalize(response)

        # Parse the API response and decode base64 audio content
        response_json = json.loads(speech)
        audio_data = base64.b64decode(response_json['audioContent'])

        # Use io.BytesIO to handle audio data in memory
        audio_stream = io.BytesIO(audio_data)
        with wave.open(audio_stream, 'rb') as wav_file:
            # Set up the PyAudio stream
            audio = pyaudio.PyAudio()
            stream = audio.open(
                format=audio.get_format_from_width(wav_file.getsampwidth()),
                channels=wav_file.getnchannels(),
                rate=wav_file.getframerate(),
                output=True
            )

            # Read and play audio data
            data = wav_file.readframes(1024)
            while data:
                stream.write(data)
                data = wav_file.readframes(1024)

            # Stop and close the stream
            stream.stop_stream()
            stream.close()
            audio.terminate()

        # Prepare the message
        msg = Bool()
        msg.data = True

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info("Playback finished, resetting conversation.")

def main(args=None):
    # Handle ASLA errors for cleaner output.
    ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
    def py_error_handler(filename, line, function, err, fmt):
        pass
    c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)

    load_dotenv()

    rclpy.init(args=args)

    lone_talker = Talker()

    rclpy.spin(lone_talker)

    lone_talker.destroy_node()
    rclpy.shutdown()