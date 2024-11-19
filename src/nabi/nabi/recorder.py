import pyaudio
import wave
import numpy

import rclpy
from rclpy.node import Node
import rclpy.parameter
from std_msgs.msg import String, Bool


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        
        try:
            self.declare_parameter('chunk_size', 1024)
            self.declare_parameter('channels', 1)
            self.declare_parameter('sample_rate', 16000)
            self.declare_parameter('threshold', 1000)
            self.declare_parameter('silence_limit', 1.2)
            self.declare_parameter('input_file', 'input.wav')

            self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
            self.channels = self.get_parameter('channels').get_parameter_value().integer_value
            self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
            self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
            self.silence_limit = self.get_parameter('silence_limit').get2_parameter_value().double_value
            self.input_file = self.get_parameter('input_file').get_parameter_value().string_value

            self.publisher = self.create_publisher(Bool, 'conversational/input', 10)

            self.get_logger().info("Recorder initialized.")
        except Exception as e:
            self.get_logger().error(f"Unable to initialize recorder: {e}")

    def record(self):
        """ Record audio until silence is detected. """

        

        audio = pyaudio.PyAudio()

        # Open audio stream for recording
        try:
            stream = audio.open(format=pyaudio.paInt16,
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
        last_log_time = self.get_clock().now().nanoseconds
        initial_time = self.get_clock().now().nanoseconds
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
            if (self.get_clock().now().nanoseconds - last_log_time) >= (0.2 * 1e9):
                self.get_logger().info(f"Current volume: {volume}")
                last_log_time = self.get_clock().now().nanoseconds
            
            # Record silent chunks and stop recording once limit is reached
            if volume < self.threshold:
                silent_chunks += 1
            else:
                silent_chunks = 0
            
            if silent_chunks >= int(self.silence_limit * self.sample_rate / self.chunk_size):
                self.get_logger().info("Silence detected. Stopping recording...")

                # If audio is not entirely silent throw it
                if (self.get_clock().now().nanoseconds - initial_time) > ((self.silence_limit + 0.1) * 1e9):
                    msg = Bool()
                    msg.data = True
                    self.publisher.publish(msg)
                
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save the recorded audio to a file
        wf = wave.open(self.input_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()
    
        self.get_logger().info(f"Recording has been saved as {self.input_file}.")

    def get_volume(self, data):
        """ Obtains the mean absolute value of the current audio. """

        if not data:
            return 0
        audio_data = numpy.frombuffer(data, dtype=numpy.int16)
        return numpy.abs(audio_data).mean()

def main(args=None):
    rclpy.init(args=args)

    lone_recorder = Recorder()

    rclpy.spin(lone_recorder)

    lone_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()