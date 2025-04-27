#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool
import pyaudio
import numpy as np

import time
from faster_whisper import WhisperModel
import math
import struct
from scipy.signal import resample
import os, wave

import openwakeword
from openwakeword.model import Model

from rclpy.qos import qos_profile_sensor_data
import rclpy.time

SHORT_NORMALIZE = (1.0/32768.0)

TIMEOUT_LENGTH = 3
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024  # Number of audio frames per buffer
WAKE_WORD_THRESHOLD = 0.5
RMS_THRESHOLD = 30

openwakeword.utils.download_models()

class ProcessAudio(Node):

    def __init__(self):
        super().__init__("processAudio")
        self.nodename = "processAudio"
        
        self.model = WhisperModel("large-v3", device="cuda", compute_type="float16")

        model_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "scripts/audioProcess/spem_v2.tflite"
        )

        self.model_wake_word = Model(wakeword_models=["jarvis"],inference_framework="tflite")

        self.start_record = False
        self.audio = pyaudio.PyAudio()
        self.data_audio = None
        self.current = 0
        self.rec = []
        self.frames = []
        self.end = 0
        self.running = True

        self.declare_parameters("", [
            ("channels", CHANNELS),
            ("rate", RATE),
            ("device", -1),
            ("format", FORMAT)
        ])

        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.device = self.get_parameter(
            "device").get_parameter_value().integer_value
        self.format = self.get_parameter(
            "format").get_parameter_value().integer_value

        if self.device < 0:
            self.device = None

        # Initialize PyAudio microphone stream
        self.mic_stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=CHUNK,
        )

        # for publish question of the user to ros
        self.pub_tts = self.create_publisher(String, "/user_input", 10)

        # for app animation
        self.pub_startAnswer = self.create_publisher(Bool, "/start_listening", 10)

        # Create a timer for processing audio
        self.timer = self.create_timer(0.01, self.process_audio)
        
        self.get_logger().info(f"-I- {self.nodename} started with direct microphone input")
        


    def rms(self, frame):
        """Calculate rms of audio data"""
        count = len(frame) / 2
        format = "%dh" % (count)
        shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in shorts:
            n = sample * SHORT_NORMALIZE
            sum_squares += n * n
        rms = math.pow(sum_squares / count, 0.5)

        return rms * 1000

    def start_recording(self):
        """Start record audio"""
        self.start_record = True
        self.pub_startAnswer.publish(Bool(data=True))
        self.get_logger().info("Recording...")
        
        # Initialize recording variables
        self.rec = []
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.current_time = current_time
        self.end = current_time + TIMEOUT_LENGTH

    def process_recording(self, audio_data):
        """Function to check if sound is active and store audio data"""
        # Check if sound is still active
        if self.rms(audio_data) >= RMS_THRESHOLD:
            self.end = self.get_clock().now().seconds_nanoseconds()[0] + TIMEOUT_LENGTH
        
        # Update current time
        self.current = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Add audio data to recording
        self.rec.append(audio_data)

    def finish_recording(self):
        """Function to finish recording and process the audio data"""
        if self.rec:  # Check if there is any recorded audio
            audio_data = b''.join(self.rec)
            # transcribe the audio
            self.transcribe_from_memory(audio_data)
            self.rec = []  # Clear the buffer
        
        self.start_record = False
        self.pub_startAnswer.publish(Bool(data=False))
        self.get_logger().info("Recording finished")

    def transcribe_from_memory(self, audio_data):
        """Transcribe audio data from memory and publish the result over ROS topic"""
        try:
            # Convert audio bytes to numpy array
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            
            # Check if we have enough audio data
            if len(audio_np) < 1000:  # Minimum required length
                self.get_logger().warning("Audio recording too short, ignoring")
                return False
                
            # Audio needs to be normalized to float32 in range [-1, 1]
            audio_float = audio_np.astype(np.float32) / 32768.0
            
            self.get_logger().info("Transcribing audio...")
            
            # Transcribe directly from the numpy array
            segments, _ = self.model.transcribe(audio_float, beam_size=5)
            
            transcribed_text = ""
            for segment in segments:
                transcript = segment.text
                transcribed_text += transcript + " "
                self.get_logger().info(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {transcript}")
            
            if transcribed_text.strip():
                # Publish transcript
                msg_text = String()
                msg_text.data = transcribed_text.strip()
                self.pub_tts.publish(msg_text)
                return True
            else:
                self.get_logger().warning("No transcription produced")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Error transcribing: {str(e)}")
            return False

    def process_audio(self):
        """Process audio data from the microphone"""
        
        try:
            
            self.data_audio = self.mic_stream.read(CHUNK, exception_on_overflow=False)


            # Convert audio data to numpy array for processing
            audio_np = np.frombuffer(self.data_audio, dtype=np.int16)
            
            # Calculate prediction for wake word
            prediction = self.model_wake_word.predict(audio_np)
            
                
            # Calculate current score for wake word
            scores = list(self.model_wake_word.prediction_buffer["jarvis"])
                
            curr_score = float(format(scores[-1], '.6f').replace("-", ""))
            
            # Debug output but not on every frame (too verbose)
            if curr_score > 0.3:  # Only show scores that are somewhat significant
                self.get_logger().info(f"Wake word confidence: {curr_score:.6f}")
            
            # Check if score for wake word is high and flag is ok and start recording
            if curr_score > WAKE_WORD_THRESHOLD and not self.start_record:
                self.get_logger().info(f"Wake word detected with confidence {curr_score:.6f}")
                self.start_recording()
            
            # Process recording if active
            if self.start_record:
                if self.current <= self.end:
                    self.process_recording(self.data_audio)
                else:
                    self.finish_recording()
                    
        except Exception as e:
            self.get_logger().error(f"Error in process_audio: {str(e)}")

    def destroy_node(self):
        """Clean up when node is destroyed"""
        if hasattr(self, 'mic_stream') and self.mic_stream.is_active():
            self.mic_stream.stop_stream()
            self.mic_stream.close()
        
        if hasattr(self, 'audio'):
            self.audio.terminate()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ProcessAudio()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


