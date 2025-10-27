#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool
import pyaudio
import numpy as np

import time
import math
import struct
from scipy.signal import resample
import os, wave

import openwakeword
from openwakeword.model import Model

from rclpy.qos import qos_profile_sensor_data
import rclpy.time

import socket
import pickle
import struct

#import Jetson.GPIO as GPIO

SHORT_NORMALIZE = (1.0/32768.0)

TIMEOUT_LENGTH = 3
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024  # Number of audio frames per buffer
WAKE_WORD_THRESHOLD = 0.3
RMS_THRESHOLD = 30

openwakeword.utils.download_models()

class ProcessAudio(Node):

    def __init__(self):
        super().__init__("processAudio")
        self.nodename = "processAudio"
        
        time.sleep(5) #wait for other nodes to be ready

        #self.model = WhisperModel("large-v3", device="cuda", compute_type="float16")
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('localhost', 8765)  # Replace SERVER_IP with your server's IP
        try:
            self.sock.connect(self.server_address)
            self.get_logger().info("Socket client connected to server.")
        except Exception as e:
            self.get_logger().error(f"Socket connection failed: {e}")

        # model_path = os.path.join(
        #     os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        #     "scripts/audioProcess/spem_v2.tflite"
        # )

        self.model_wake_word = Model(wakeword_models=["jarvis"],inference_framework="tflite")

        self.start_record = False
        self.audio = pyaudio.PyAudio()
        self.data_audio = None
        self.current = 0
        self.rec = []
        self.frames = []
        self.end = 0
        self.running = True

        # List all available audio input devices
        # self.get_logger().info("Available audio input devices:")
        # for i in range(self.audio.get_device_count()):
        #     info = self.audio.get_device_info_by_index(i)
        #     self.get_logger().info(f"Device {i}: {info['name']} (Input Channels: {info['maxInputChannels']})")

        self.declare_parameters("", [
            ("channels", CHANNELS),
            ("rate", RATE),
            ("device", 0),
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

        # Initialize PyAudio microphone stream PROBLEM HERE : with both microphone and speaker connected,
        # the microphone device is 5 and the speaker device is 0 --> how to change the input_device_index
        self.mic_stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            input_device_index=0,
            frames_per_buffer=CHUNK,
        )

        # for publish question of the user to ros
        self.pub_tts = self.create_publisher(String, "/user_input", 10)

        # for app animation
        self.pub_startAnswer = self.create_publisher(Bool, "/start_listening", 10)

        # Create a timer for processing audio
        self.timer = self.create_timer(0.01, self.process_audio)

        # create gpio mode for pin LED
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(22, GPIO.OUT)
        #GPIO.output(22, GPIO.LOW)
        
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
        # set LED ON
        #GPIO.output(22, GPIO.HIGH)

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

        # set LED OFF
        #GPIO.output(22, GPIO.LOW)

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
            
            # Transcribe with websocket server
            data = pickle.dumps(audio_float)
            # Send length of data first
            self.sock.sendall(len(data).to_bytes(4, byteorder='big'))
            self.sock.sendall(data)
            self.get_logger().info("Sent audio_float to server.")
            
            #segments, _ = self.model.transcribe(audio_float, beam_size=5)
            
            # --- Receive response from server ---
            # First, receive the length of the response (4 bytes)
            response_len_bytes = self.sock.recv(4)
            if len(response_len_bytes) < 4:
                self.get_logger().error("Failed to receive response length from server.")
                return False
            response_len = int.from_bytes(response_len_bytes, byteorder='big')

            # Now receive the actual response data
            response_data = b''
            while len(response_data) < response_len:
                packet = self.sock.recv(response_len - len(response_data))
                if not packet:
                    break
                response_data += packet

            if len(response_data) != response_len:
                self.get_logger().error("Incomplete response received from server.")
                return False

            # Unpickle the response (assuming it's a string or object)
            transcribed_text = pickle.loads(response_data)
            self.get_logger().info(f"Received transcription: {transcribed_text}")

            
            # if transcribed_text.strip():
            #     # Publish transcript
            #     msg_text = String()
            #     msg_text.data = transcribed_text.strip()
            #     self.pub_tts.publish(msg_text)
            #     return True
            # else:
            #     self.get_logger().warning("No transcription produced")
            #     return False
        
        except Exception as e:
            self.get_logger().error(f"Error transcribing: {str(e)}")
            return False

    def process_audio(self):
        """Process audio data from the microphone"""
        
        try:
            #self.get_logger().info("Listening for wake word...")
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
            # set LED OFF
            #GPIO.output(22, GPIO.LOW)
            
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


