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
        try:
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
                        rate=frame_rate // self.length_speed,
                        output=True,
                        output_device_index=self.get_parameter("device").value,
                        frames_per_buffer=2048
                    )


            #chunk_size = 1024
            #for i in range(0, len(raw_data), chunk_size):
                    stream.write(raw_data)	

                    self.get_logger().info("Playback completed")
                    
            
                stream.stop_stream()
                stream.close()
        except Exception as e:
            self.get_logger().error(f"Error during audio playback: {e}")
            self.get_logger().info("AudioPlayer sbem node stopped")



def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
	
