#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool


import queue
import re
import threading


import sounddevice as sd
import numpy as np
import time
import math
import struct
import os
from datetime import datetime
import struct
from faster_whisper import WhisperModel


SHORT_NORMALIZE = (1.0/32768.0)
TIMEOUT_LENGTH = 3
WAKE_WORD_THRESHOLD = 0
RMS_THRESHOLD = 1000  # Adjust this threshold based on your environment

class ProcessAudio(Node):

    def __init__(self):
        super().__init__("PorcupineCheetahMicrophoneNode")
        self.nodename = "PorcupineCheetahMicrophoneNode"

        self.declare_parameter("index_mic", -1)
        self.declare_parameter("use_wake_word", False)
        self.declare_parameter("whisper_model_size", "base.en")
        self.declare_parameter("device_index", 0)
        self.declare_parameter("rate", 41000)
        self.declare_parameter("chunk", 2048)
        self.declare_parameter("nb_channels", 1)

        print(sd.query_devices())

        # initialize wake word only if parameter is set to true
        if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
            pass
            
        # initialize recorder and whisper STT
        self.get_logger().info("Initialing")
        try:
            if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
                pass

            self.whisper = WhisperModel("tiny", device="cpu", compute_type="int8", cpu_threads=4)

        except Exception as e:
            self.get_logger().error(f"Error initializing whisper model: {str(e)}")
            return


        self.start_record = False
        self.current = 0
        self.end = 0

        self.current_transcript = ""
        self.volume = 0

        # for publish question of the user to ros
        self.pub_tts = self.create_publisher(String, "/user_input", 1)
        #queue for audio data
        self.audio_queue = queue.Queue()

        threading.Thread(target=self.listen_audio, daemon=True).start()
        threading.Thread(target=self.process_recording, daemon=True).start()
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
    
        self.get_logger().info("Recording...")
        
        # Initialize recording variables
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.current_time = current_time
        self.end = current_time + TIMEOUT_LENGTH


    def process_recording(self, audio_data):
        """Function to check if user is still talking"""
        # Check if sound is still active
        if self.volume >= RMS_THRESHOLD:
            self.end = self.get_clock().now().seconds_nanoseconds()[0] + TIMEOUT_LENGTH
        
        # Update current time
        self.current = self.get_clock().now().seconds_nanoseconds()[0]

        # transcribe with cheetah in real-time
        partial_transcript, is_endpoint = self.cheetah.process(audio_data)
        self.current_transcript += partial_transcript
        self.get_logger().info(f"Partial transcript: {self.current_transcript}")
        
        # if cheetah detect the end of the query, send final message
        if is_endpoint:
            self.current_transcript += self.cheetah.flush()
            self.start_record = False

            self.get_logger().info(f"Final transcript: {self.current_transcript}")
            self.send_message(self.current_transcript)
            

    def send_message(self, query: str):
        """Send data over ros""" 
        msg = String()
        msg.data = self.current_transcript
        # reset current transcript
        self.current_transcript = ""

        self.pub_tts.publish(msg)


    def finish_recording(self):
        """Function to finish recording and process the audio data"""
        self.send_message(self.current_transcript)
    

    def audio_callback(self, indata, frames, time, status):
        """Callback function to process audio data from microphone"""
        if status:
            self.get_logger().warning(f"Audio callback status: {status}")
        
        #get volume
        volume = np.linalg.norm(indata) * 10
        self.get_logger().info(f"Volume: {volume}")

        if volume >= RMS_THRESHOLD:
            self.audio_queue.put(audio_data)
            audio_data += pcm
            audio_queue.put(audio_data)


    def listen_audio(self):
        """Listen audio data from the microphone"""
        with sd.InputStream(channels=self.get_parameter("nb_channels").get_parameter_value().integer_value,
                            samplerate=self.get_parameter("rate").get_parameter_value().integer_value,
                            device=self.get_parameter("device_index").get_parameter_value().integer_value,
                            callback=self.audio_callback):
            while True:
                pass
        
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.get_logger().info("Shutting down ProcessAudio node...")
        
        if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
            pass


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


