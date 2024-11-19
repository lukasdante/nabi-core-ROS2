class Talker(Node):
    def __init__(self, config):
        try:
            self.service_account = os.getenv("TTS_SERVICE_ACCOUNT")

            with open(self.service_account) as f:
                self.service_account_info = json.load(f)

            self.language = config.get('LANG', 'en-US')
            self.gender = config.get('GENDER', 'FEMALE')
            self.accent = config.get('ACCENT', 'en-US-Neural2-C')
            self.encoding_format = config.get('ENCODING', 'LINEAR16')
            self.output_file = config.get('OUTPUT_AUDIO_FILE', 'output.wav')
            self.token_validity = config.get('TOKEN_LIFE', 3600)
            self.token_url = "https://oauth2.googleapis.com/token"
            self.scopes = "https://www.googleapis.com/auth/cloud-platform"
            self.api_endpoint = "https://texttospeech.googleapis.com/v1/text:synthesize"

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

    def save_audio(self, speech):
        response = json.loads(speech)
        audio_data = response['audioContent']

        # Decode base64 to binary data and save as .wav
        with open(self.output_file, "wb") as file:
            file.write(base64.b64decode(audio_data))

    def talk(self, response):
        """ Talks given a response in text. """

        response = self.write_json(response)
        speech = self.vocalize(response)
        self.save_audio(speech)

        try:
            # Open the wav file
            with wave.open(self.output_file, 'rb') as wav_file:
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

            self.get_logger().info("Playback finished.")
            os.remove(self.output_file)
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
