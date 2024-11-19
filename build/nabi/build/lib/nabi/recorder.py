import os
import rclpy
import pyaudio
import wave
import yaml
import requests
import base64
import json
import time
import uuid
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Bool
from google.cloud.dialogflowcx_v3beta1.services.agents import AgentsClient
from google.cloud.dialogflowcx_v3beta1.services.sessions import SessionsClient
from google.cloud.dialogflowcx_v3beta1.types import session
from google.api_core.client_options import ClientOptions
from google.protobuf.json_format import MessageToDict
from google.auth import jwt as google_jwt
from google.auth import crypt

class Recorder(Node):
    def __init__(self, config):
        try:
            super().__init__()
            self.chunk_size = config.get('CHUNK_SIZE', 1024)
            self.channels = config.get('CHANNELS', 1)
            self.sample_rate = config.get('SAMPLE_RATE', 16000)
            self.threshold = config.get('THRESHOLD', 1000)
            self.silence_limit = config.get('SILENCE_LIMIT', 1.2)
            self.input_file = config.get('INPUT_AUDIO_FILE', 'input.wav')
            self.audio_format = pyaudio.paInt16
            self.has_speech = True
            self.get_logger().info("Recorder initialized.")
        except Exception as e:
            self.get_logger().error(f"Unable to initialize recorder: {e}")

    def record(self):
        """ Record audio until silence is detected. """

        audio = pyaudio.PyAudio()

        # Open audio stream for recording
        try:
            stream = audio.open(format=self.audio_format,
                            channels=self.channels,
                            rate=self.sample_rate,
                            input=True,
                            frames_per_buffer=self.chunk_size)
            
            self.get_logger().info("Recording started. Speak into the microphone...")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}.")
            return

        # Read and store the audio stream until silence is detected
        frames = []
        silent_chunks = 0
        last_log_time = self.get_clock().now()
        initial_time = self.get_clock().now()
        while True:
            try:
                data = stream.read(self.chunk_size)
            except Exception as e:
                self.get_logger().error(f"Error reading audio data: {e}")
                break

            frames.append(data)
            
            # Calculate the volume
            volume = self.get_volume(data)

            # For every 200ms display the volume
            if self.get_clock().now().seconds_nanoseconds - last_log_time >= 0.2:
                self.get_logger().info(f"Current volume: {volume}")
                last_log_time = self.get_clock().now().seconds_nanoseconds
            
            # Record silent chunks and stop recording once limit is reached
            if volume < self.threshold:
                silent_chunks += 1
            else:
                silent_chunks = 0
            
            if silent_chunks >= int(self.silence_limit * self.sample_rate / self.chunk_size):
                self.get_logger().info("Silence detected. Stopping recording...")

                # If audio is entirely silent, set status to silent
                if self.get_clock().now().seconds_nanoseconds - initial_time > (self.silence_limit + 0.1):
                    self.has_speech = True
                else:
                    self.has_speech = False

                break

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save the recorded audio to a file
        wf = wave.open(self.input_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(audio.get_sample_size(self.audio_format))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()
    
        self.get_logger().info(f"Recording has been saved as {self.input_file}.")

    def get_volume(self, data):
        """ Obtains the mean absolute value of the current audio. """

        if not data:
            return 0
        audio_data = np.frombuffer(data, dtype=np.int16)
        return np.abs(audio_data).mean()

