#!/usr/bin/env python3


#PART of the script come from audio_player_node.py of :
# authors:
#   - family-names: "González-Santamarta"
#     given-names: "Miguel Á."
# title: "audio_common"
# date-released: 2023-05-06
# url: "https://github.com/mgonzs13/audio_common"

import rclpy
from rclpy.node import Node

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

openwakeword.utils.download_models()

class ProcessAudio(Node):

    def __init__(self):
        super().__init__("processAudio")
        self.nodename = "processAudio"
        

        
        self.model = WhisperModel("large-v3", device="cuda", compute_type="float16")

        self.model_wake_word = Model(wakeword_models=["/home/morolinux/Projects/Sbem/sbem_project_ws/src/sbem_AI/audioProcess/spem_v2.tflite"],inference_framework="tflite")

        

        self.start_record = False
        self.audio = pyaudio.PyAudio()
        self.stream_dict = {}
        self.data_audio = None
        self.f_name_directory = r"/home/morolinux/Projects/Sbem/sbem_project_ws/src/sbem_AI/audioProcess/samples"

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
        self.pub_startAnswer = self.create_publisher(Bool, "/start_answer", 10)

        self.get_logger().info(f"-I- {self.nodename} started")




    # create wav sound and store in file 
    def write(self, recording):
        #n_files = len(os.listdir(self.f_name_directory))

        #filename = os.path.join(self.f_name_directory, '{}.wav'.format(n_files))
        filename = os.path.join(self.f_name_directory, 'audio.wav')

        wf = wave.open(filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(self.audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(recording)
        wf.close()
        print('Written to file: {}'.format(filename))
        print('Returning to listening')


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


    #main function 
    def get_audio(self, msg : AudioStamped) -> None:
        #get stream key from msg
        stream_key = f"{msg.audio.info.format}_{msg.audio.info.rate}_{self.channels}"

        if stream_key not in self.stream_dict:
            self.stream_dict[stream_key] = self.audio.open(
                format=msg.audio.info.format,
                channels=self.channels,
                rate=msg.audio.info.rate,
                output=True,
                output_device_index=self.device
            )

        #convert audio from msg
        array_data = msg_to_array(msg.audio)

        if array_data is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return

        if msg.audio.info.channels != self.channels:
            if msg.audio.info.channels == 1 and self.channels == 2:
                # mono to stereo
                array_data = np.repeat(array_data, 2)

            elif msg.audio.info.channels == 2 and self.channels == 1:
                # stereo to mono
                array_data = np.mean(array_data.reshape(-1, 2), axis=1)
                array_data = array_data.astype(
                    pyaudio_to_np[msg.audio.info.format])


        #convert audio in chunk
        self.data_audio = array_to_data(array_data)
        
        #calculate prediction wake word
        prediction = self.model_wake_word.predict(np.frombuffer(self.data_audio, dtype=np.int16))

        #calculate current score for wake word
        scores = list(self.model_wake_word.prediction_buffer["spem_v2"])
        curr_score = format(scores[-1], '.20f').replace("-", "")


        #print("Score wake word : ", curr_score)
        #print("Rms value : ", self.rms(self.data_audio))

        
        #check is score for wake word is high and flag is ok and start recording
        if float(curr_score) > 0.5 and self.start_record == False:
            self.start_record = True

            #publish over topic that publishes start answer message for sbem APP 
            self.pub_startAnswer.publish(Bool(data=True))

            self.get_logger().info(f"Recording....")

            #initialize variables 
            self.rec = []
            self.current = self.get_clock().now().seconds_nanoseconds()[0]
            self.end = self.get_clock().now().seconds_nanoseconds()[0] + TIMEOUT_LENGTH
        

        if self.start_record and self.current <= self.end :
            #check if command is finished or end of timer
            if self.rms(self.data_audio) >= 30: self.end = self.get_clock().now().seconds_nanoseconds()[0] + TIMEOUT_LENGTH

            #update current variable
            self.current = self.get_clock().now().seconds_nanoseconds()[0]


            #append dato to file audio
            self.rec.append(self.data_audio)

        #if command voice is finished store and create file audio and reset variables
        else :
            if self.start_record:
                self.write(b''.join(self.rec))
                self.start_record = False
                self.pub_startAnswer.publish(Bool(data=False))
        

        #if there is the file wav perform transcribe with faster whisper 
        if os.path.isfile(os.path.join(self.f_name_directory, 'audio.wav')):
            self.get_logger().info("Audio file available")
            segments, info = self.model.transcribe(os.path.join(self.f_name_directory, 'audio.wav'), beam_size=5)

            
            for segment in segments:
                print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
  
                #publish over topic that publishes transcribe message
                msg_text = String()
                msg_text.data = segment.text
                self.pub_tts.publish(msg_text)

            os.remove(os.path.join(self.f_name_directory, 'audio.wav'))


    


    # def destroy_node(self) -> bool:
    #     for key in self.stream_dict:
    #         self.stream_dict[key].close()
    #     self.audio.terminate()
    #     os.remove(os.path.join(self.f_name_directory, 'audio.wav'))
    #     return super().destroy_node()
        


# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         process_audio_sbem = ProcessAudio()
#         rclpy.spin(process_audio_sbem)
#     except rclpy.exceptions.ROSInterruptException:
#         pass

#     process_audio_sbem.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
