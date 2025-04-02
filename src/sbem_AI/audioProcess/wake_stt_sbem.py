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


from audio_common_msgs.msg import AudioStamped
from audio_common.utils import msg_to_array, array_to_data, pyaudio_to_np

from rclpy.qos import qos_profile_sensor_data
import rclpy.time

SHORT_NORMALIZE = (1.0/32768.0)

TIMEOUT_LENGTH = 3
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
swidth = 2
CHUNK = 1280
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
            "audioProcess/spem_v2.tflite"
        )

        self.model_wake_word = Model(wakeword_models=[model_path],inference_framework="tflite")

        self.start_record = False
        self.audio = pyaudio.PyAudio()
        self.stream_dict = {}
        self.data_audio = None
        self.current = 0
        self.rec = []
        self.end = 0

        self.declare_parameters("", [
            ("channels", 1),
            ("device", -1),
        ])

        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.device = self.get_parameter(
            "device").get_parameter_value().integer_value

        if self.device < 0:
            self.device = None

        # subscriptions
        self.create_subscription(AudioStamped, "/sbem_audio", self.get_audio, qos_profile_sensor_data)

        # for publish question of the user to ros
        self.pub_tts = self.create_publisher(String, "/user_input", 10)

        # for app animation
        self.pub_startAnswer = self.create_publisher(Bool, "/start_listening", 10)

        self.get_logger().info(f"-I- {self.nodename} started")


    # function to handle stream setup
    def handle_stream_setup(self, msg):
        
        stream_key = f"{msg.audio.info.format}_{msg.audio.info.rate}_{self.channels}"

        if stream_key not in self.stream_dict:
            self.stream_dict[stream_key] = self.audio.open(
                format=msg.audio.info.format,
                channels=self.channels,
                rate=msg.audio.info.rate,
                output=True,
                output_device_index=self.device
            )
        
        return stream_key


    # function to process audio data
    def process_audio_data(self, msg):
        
        array_data = msg_to_array(msg.audio)

        if array_data is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return None

        # Channel conversion if needed
        if msg.audio.info.channels != self.channels:
            if msg.audio.info.channels == 1 and self.channels == 2:
                # mono to stereo
                array_data = np.repeat(array_data, 2)
            elif msg.audio.info.channels == 2 and self.channels == 1:
                # stereo to mono
                array_data = np.mean(array_data.reshape(-1, 2), axis=1)
                array_data = array_data.astype(pyaudio_to_np[msg.audio.info.format])

        return array_to_data(array_data)


    #function to detect sound power
    def rms(self, frame):
        count = len(frame) / 2
        format = "%dh" % (count)
        shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in shorts:
            n = sample * SHORT_NORMALIZE
            sum_squares += n * n
        rms = math.pow(sum_squares / count, 0.5)

        return rms * 1000

    # function to start recording
    def start_recording(self):
        self.start_record = True
        self.pub_startAnswer.publish(Bool(data=True))
        self.get_logger().info("Recording started...")
        
        # Initialize recording variables
        self.rec = []
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.current_time = current_time
        self.end = current_time + TIMEOUT_LENGTH


    # function to view if sound is active
    def process_recording(self, audio_data):
       
        # Check if sound is still active
        if self.rms(audio_data) >= RMS_THRESHOLD:
            self.end = self.get_clock().now().seconds_nanoseconds()[0] + TIMEOUT_LENGTH
        
        # Update current time
        self.current = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Add audio data to recording
        self.rec.append(audio_data)


    # function that store the audio in a file and clear the variables
    def finish_recording(self):
        
        if self.rec:  # Check if there is any recorded audio
            audio_data = b''.join(self.rec)
            # transcribe the audio
            self.transcribe_from_memory(audio_data)
            self.rec = []  # Clear the buffer
        
        self.start_record = False
        self.pub_startAnswer.publish(Bool(data=False))
        self.get_logger().info("Recording finished")


    # function to transcribe the audio from the buffer
    def transcribe_from_memory(self, audio_data):
        try:
            # Convert audio bytes to numpy array
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            
            # Audio needs to be normalized to float32 in range [-1, 1]
            audio_float = audio_np.astype(np.float32) / 32768.0
            
            # Transcribe directly from the numpy array
            segments, _ = self.model.transcribe(audio_float, beam_size=5)
            
            for segment in segments:
                transcript = segment.text
                self.get_logger().info(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {transcript}")
                
                # Publish transcript
                msg_text = String()
                msg_text.data = transcript
                self.pub_tts.publish(msg_text)
                
            return True
        except Exception as e:
            self.get_logger().error(f"Error transcribing: {e}")
            return False


    #main function 
    def get_audio(self, msg : AudioStamped) -> None:
        self.handle_stream_setup(msg)


        #convert audio from msg
        self.data_audio = self.process_audio_data(msg)
        if self.data_audio is None:
            return

        
        #calculate prediction wake word
        prediction = self.model_wake_word.predict(np.frombuffer(self.data_audio, dtype=np.int16))

        #calculate current score for wake word
        scores = list(self.model_wake_word.prediction_buffer["spem_v2"])
        curr_score = format(scores[-1], '.20f').replace("-", "")

        
        #check is score for wake word is high and flag is ok and start recording
        if float(curr_score) > WAKE_WORD_THRESHOLD and self.start_record == False:
            self.start_recording()
        

        if self.start_record:
            if self.current <= self.end :
                self.process_recording(self.data_audio)
            else : #if command voice is finished store and create file audio and reset variables
                self.finish_recording()
        

