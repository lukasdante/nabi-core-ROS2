from rclpy.node import Node
import os
import requests
import base64

class Writer(Node):
    def __init__(self, config):
        try:
            super().__init__()
            self.api_key = os.getenv('STT_API_KEY')
            self.api_endpoint = f'https://speech.googleapis.com/v1/speech:recognize?key={self.api_key}'
            self.language = config.get('LANG', 'en-US')
            self.sample_rate = config.get('SAMPLE_RATE', 16000)
            self.encoding_format = config.get('ENCODING', 'LINEAR16')
            self.input_file = config.get('INPUT_AUDIO_FILE', 'input.wav')

            self.get_logger().info("Writer initialized.")
        except Exception as e:
            self.get_logger().error(f"Unable to initialize writer: {e}")

    def write(self):
        """ Encodes audio to text. """

        # Load and encode the audio file
        encoded_audio = self.encode_audio()

        # Configure the request
        headers = {'Content-Type': 'application/json'}
        body = {
            'config': {
                'encoding': self.encoding_format,
                'sampleRateHertz': self.sample_rate,
                'languageCode': self.language
            },
            'audio': {
                'content': encoded_audio
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

                    # Remove the audio file
                    if os.path.exists(self.input_file):
                        os.remove(self.input_file)

                    return transcription
            else:
                self.get_logger().info("No transcription found.")
                return None
        else:
            self.get_logger().error(f"Writer request error {response.status_code}.")
            return None
    
    def encode_audio(self):
        """ Encode audio to base64 and decode to utf-8. """

        with open(self.input_file, 'rb') as f:
            audio_content = f.read()
        return base64.b64encode(audio_content).decode('utf-8')