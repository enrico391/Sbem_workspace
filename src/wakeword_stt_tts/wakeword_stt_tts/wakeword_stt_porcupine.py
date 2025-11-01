#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String

import numpy as np
import math
import os

from pvrecorder import PvRecorder
import pvporcupine
import pvcheetah

#import Jetson.GPIO as GPIO

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

        # initialize wake word only if parameter is set to true
        if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
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
            
        # initialize wake_word, cheetah STT and recorder
        self.get_logger().info("Initialing")
        try:
            if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
                self.porcupine = pvporcupine.create(
                    access_key=os.environ.get("PORCUPINE_KEY"),
                    library_path=None,
                    model_path=None,
                    keyword_paths= keyword_paths,
                    sensitivities=[0.5] * len(keyword_paths))

            self.cheetah = pvcheetah.create(
                access_key=os.environ.get("PORCUPINE_KEY"),
                enable_automatic_punctuation=True,
                endpoint_duration_sec = 3.0)

            # recorder for porcupine
            self.recorder = PvRecorder(
                frame_length= self.cheetah.frame_length,
                device_index=self.get_parameter("index_mic").get_parameter_value().integer_value)

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
        self.current = 0
        self.end = 0

        self.current_transcript = ""
        self.volume = 0

        # start recording from microphone
        self.recorder.start()

        # for publish question of the user to ros
        self.pub_tts = self.create_publisher(String, "/user_input", 1)

        # Create a timer for processing audio
        self.timer = self.create_timer(0.01, self.process_audio)
        
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
        # restore flags
        self.start_record = False

        # get final transcript from cheetah
        self.current_transcript += self.cheetah.flush()
        self.send_message(self.current_transcript)
        

    def process_audio(self):
        """Process audio data from the microphone"""
        try:
            pcm = self.recorder.read()
            self.volume = np.mean(np.abs(pcm))
            self.get_logger().info(f"Volume: {self.volume}")

            if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
                curr_score = self.porcupine.process(pcm)

            # if curr_score >= 0:
            #     print('[%s] Detected %s' % (str(datetime.now()), self.keywords[curr_score]))
        
            # Check if use wake word or sound threshold to start recording
            if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
                if curr_score != -1 and not self.start_record:
                    self.get_logger().info(f"Wake word detected")
                    self.start_recording()
            
            else:
                if self.volume >= RMS_THRESHOLD and not self.start_record:
                    self.get_logger().info(f"Sound detected with volume {self.volume}")
                    self.start_recording()
            
            # Process recording if active
            if self.start_record:
                if self.current <= self.end:
                    self.process_recording(pcm)
                else:
                    self.finish_recording()
                    
        except Exception as e:
            self.start_record = False
            self.get_logger().error(f"Error in process_audio: {str(e)}")
            

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.get_logger().info("Shutting down ProcessAudio node...")
        
        if self.get_parameter("use_wake_word").get_parameter_value().bool_value:
            self.porcupine.delete()

        self.cheetah.delete()
        self.recorder.stop()
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


