#!/usr/bin/env python3

import pyaudio
import numpy as np
from gtts import gTTS 
from io import BytesIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydub import AudioSegment

from rclpy.qos import qos_profile_sensor_data



class AudioPlayerNode(Node):

    def __init__(self) -> None:
        super().__init__("tts_sbem")


        self.declare_parameters("", [
            ("channels", 2),
            ("device", -1),
        ])
        
        self.length_talk = 5


        self.audio = pyaudio.PyAudio()

        self.sub = self.create_subscription(
            String, "/response_to_user", self.audio_callback, qos_profile_sensor_data)

        self.get_logger().info("AudioPlayer sbem node started")

    def destroy_node(self) -> bool:
        self.audio.terminate()
        return super().destroy_node()

    def audio_callback(self, msg: String) -> None:
        if(msg.data != ""):
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
                    rate=frame_rate // 1,
                    output=True
                )

                stream.write(raw_data)


        # words = msg.data.split()
        # chunks = [' '.join(words[i:i + self.length_talk]) for i in range(0, len(words), self.length_talk)] 
        # for chunk in chunks:
        #     tts = gTTS(text=chunk, lang='it', slow=False)
        #     with BytesIO() as fp:
        #         tts.write_to_fp(fp)
        #         fp.seek(0)

        #         audio = AudioSegment.from_file(fp, format="mp3")
        #         raw_data = audio.raw_data
        #         sample_width = audio.sample_width
        #         channels = audio.channels
        #         frame_rate = audio.frame_rate

        #         stream = self.audio.open(
        #             format=self.audio.get_format_from_width(sample_width),
        #             channels=self.channels,
        #             rate=frame_rate,
        #             output=True
        #         )

        #         stream.write(raw_data)
        
        stream.stop_stream()
        stream.close()



def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
