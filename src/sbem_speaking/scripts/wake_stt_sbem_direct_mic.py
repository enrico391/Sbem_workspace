#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool

import numpy as np
import pyaudio
import time
import math
import struct
import os
from datetime import datetime


import socket
import pickle
import struct
from pvrecorder import PvRecorder
import pvporcupine

#import Jetson.GPIO as GPIO

SHORT_NORMALIZE = (1.0/32768.0)
TIMEOUT_LENGTH = 3
WAKE_WORD_THRESHOLD = 0
RMS_THRESHOLD = 30

class ProcessAudio(Node):

    def __init__(self):
        super().__init__("processAudio")
        self.nodename = "processAudio"
        
        time.sleep(4) #wait for other nodes to be ready
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('localhost', 8765)  # Replace SERVER_IP with your server's IP
        try:
            self.sock.connect(self.server_address)
            self.get_logger().info("Socket client connected to server.")
        except Exception as e:
            self.get_logger().error(f"Socket connection failed: {e}")

        # Initialize Porcupine wake word detection
        wake_words = ["porcupine", "bumblebee"]
        keyword_paths = [pvporcupine.KEYWORD_PATHS[x] for x in wake_words]

        self.keywords = list()
        for x in keyword_paths:
            keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
            if len(keyword_phrase_part) > 6:
                self.keywords.append(' '.join(keyword_phrase_part[0:-6]))
            else:
                self.keywords.append(keyword_phrase_part[0])

        try:
            self.porcupine = pvporcupine.create(
                access_key='YOURKEY==',
                library_path=None,
                model_path=None,
                keyword_paths= keyword_paths,
                sensitivities=[0.5] * len(keyword_paths))
        except pvporcupine.PorcupineInvalidArgumentError as e:
            print("One or more arguments provided to Porcupine is invalid: ", args)
            print(e)
            raise e
        except pvporcupine.PorcupineActivationError as e:
            print("AccessKey activation error")
            raise e
        except pvporcupine.PorcupineActivationLimitError as e:
            print("AccessKey '%s' has reached it's temporary device limit" % args.access_key)
            raise e
        except pvporcupine.PorcupineActivationRefusedError as e:
            print("AccessKey '%s' refused" % args.access_key)
            raise e
        except pvporcupine.PorcupineActivationThrottledError as e:
            print("AccessKey '%s' has been throttled" % args.access_key)
            raise e
        except pvporcupine.PorcupineError as e:
            print("Failed to initialize Porcupine")
            raise e


        self.start_record = False
        self.audio = pyaudio.PyAudio()
        self.data_audio = None
        self.current = 0
        self.rec = []
        self.frames = []
        self.end = 0

        # recorder for porcupine
        self.recorder = PvRecorder(
        frame_length= self.porcupine.frame_length,
        device_index=2)
        self.recorder.start()


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
        #shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in frame:
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
        self.rec.extend(audio_data)

    def finish_recording(self):
        """Function to finish recording and process the audio data"""
        if self.rec:  # Check if there is any recorded audio
            # convert list to bytes
            audio_bytes = struct.pack('<' + ('h' * len(self.rec)), *self.rec)
            
            # transcribe the audio
            self.transcribe_from_memory(audio_bytes)
            self.rec = []  # Clear the buffer
        
        # restore flags
        self.start_record = False

        # set LED OFF
        #GPIO.output(22, GPIO.LOW)

        self.pub_startAnswer.publish(Bool(data=False))
        self.get_logger().info("Recording finished")

    def transcribe_from_memory(self, audio_data):
        """Transcribe audio data from memory and publish the result over socket"""
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
            #print("Listening for wake word...")
            audio_np = self.recorder.read()
            curr_score = self.porcupine.process(audio_np)

            #volume = np.mean(np.abs(audio_np))

            if curr_score >= 0:
                print('[%s] Detected %s' % (str(datetime.now()), self.keywords[curr_score]))
        
            # Check if score for wake word is high and flag is ok and start recording
            if curr_score >= WAKE_WORD_THRESHOLD and not self.start_record:
                self.get_logger().info(f"Wake word detected with confidence {curr_score:.6f}")
                self.start_recording()
            
            # Process recording if active
            if self.start_record:
                if self.current <= self.end:
                    self.process_recording(audio_np)
                else:
                    self.finish_recording()
                    
        except Exception as e:
            # set LED OFF
            #GPIO.output(22, GPIO.LOW)
            self.start_record = False
            self.get_logger().error(f"Error in process_audio: {str(e)}")
            

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.get_logger().info("Shutting down ProcessAudio node...")
        self.porcupine.delete()
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


