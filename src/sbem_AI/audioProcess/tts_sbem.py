#!/usr/bin/env python3

import pyaudio
import wave
import sys
import numpy as np
from gtts import gTTS 
from io import BytesIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydub import AudioSegment
from audioProcess.send_toTTS import SenderTTS

from rclpy.qos import qos_profile_sensor_data



class AudioPlayerNode(Node):

    def __init__(self, useLocalTTS= False) -> None:
        super().__init__("tts_sbem")


        #check if use local TTS and initialize the class
        self.useLocalTTS = useLocalTTS
        if(self.useLocalTTS):
            self.requestTTS = SenderTTS("coqui")


        self.declare_parameters("", [
            ("channels", 2),
            ("device", -1),
        ])
        
        self.length_speed = 1


        self.audio = pyaudio.PyAudio()

        qos_profile = qos_profile_sensor_data

        self.sub = self.create_subscription(
            String, "/response_to_user", self.audio_callback, qos_profile)

        self.get_logger().info("AudioPlayer sbem node started")

    def destroy_node(self) -> bool:
        self.audio.terminate()
        return super().destroy_node()

    def audio_callback(self, msg: String) -> None:
        if(msg.data != ""):
            
             #send request to piper server for transcribe or publish over topic
            if self.useLocalTTS :
                self.requestTTS.sendRequest(msg.data)

                chunk = 1024

                wf = wave.open("output.wav", 'rb')

                # create an audio object
                p = pyaudio.PyAudio()

                # open stream based on the wave object which has been input.
                stream = p.open(format =
                                p.get_format_from_width(wf.getsampwidth()),
                                channels = wf.getnchannels(),
                                rate = wf.getframerate(),
                                output = True)

                # read data (based on the chunk size)
                data = wf.readframes(chunk)

                # play stream (looping from beginning of file to the end)
                while data:
                    # writing to the stream is what *actually* plays the sound.
                    stream.write(data)
                    data = wf.readframes(chunk)

                # cleanup stuff.
                wf.close()
                stream.close()    
                p.terminate()


            # or use gTTS
            else:
                tts = gTTS(text=msg.data, lang='it', slow=False)
                with BytesIO() as fp:
                    tts.write_to_fp(fp)
                    fp.seek(0)

                    audio = AudioSegment.from_file(fp, format="mp3")

                    raw_data = audio.raw_data
                    sample_width = audio.sample_width
                    channels = audio.channels
                    frame_rate = audio.frame_rate

                    stream = self.audio.open(
                        format=self.audio.get_format_from_width(sample_width),
                        channels=channels,
                        rate=frame_rate // self.length_speed,
                        output=True
                    )

                    stream.write(raw_data)
        
                stream.stop_stream()
                stream.close()



# def main(args=None):
#     rclpy.init(args=args)
#     node = AudioPlayerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
