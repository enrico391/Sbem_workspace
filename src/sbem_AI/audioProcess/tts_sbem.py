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
    """Node to play audio response from agent"""
    def __init__(self, useLocalTTS= False, typeLocalTTS="coqui") -> None:
        super().__init__("tts_sbem")

        #check if use local TTS and initialize the class
        self.useLocalTTS = useLocalTTS
        if(self.useLocalTTS):
            self.requestTTS = SenderTTS(typeLocalTTS)
        
        self.audio = pyaudio.PyAudio()

        self.declare_parameters("", [
            ("channels", 1),
            ("device", -1),
        ])

        self.sub = self.create_subscription(
            String, "/response_to_user", self.audio_callback, qos_profile_sensor_data)

        self.get_logger().info("AudioPlayer sbem node started")

    def destroy_node(self) -> bool:
        self.audio.terminate()
        return super().destroy_node()
    

    def play_audio(self, content, format):
        """Function to play audio data"""
        # Convert the audio data to an AudioSegment
        audio = AudioSegment.from_file(BytesIO(content), format=format)
        
        # Extract audio properties
        raw_data = audio.raw_data
        sample_width = audio.sample_width
        channels = audio.channels
        frame_rate = audio.frame_rate
        
        # Create and configure audio stream
        stream = self.audio.open(
            format=self.audio.get_format_from_width(sample_width),
            channels=channels,
            rate=frame_rate,
            output=True
        )
        
        # Play audio in chunks
        chunk_size = 1024
        for i in range(0, len(raw_data), chunk_size):
            stream.write(raw_data[i:i+chunk_size])
        
        # Clean stream
        stream.stop_stream()
        stream.close()
        #self.audio.terminate()


    def audio_callback(self, msg: String) -> None:
        """callback for subscriber to string response from agent"""
        if(msg.data != ""):
            #send request to piper server for transcribe or publish over topic
            if self.useLocalTTS:
                # send local request and get content directly
                content = self.requestTTS.sendRequest(msg.data)
                
                try:
                    self.play_audio(content,"wav")
                    
                except Exception as e:
                    self.get_logger().error(f"Error playing local TTS content: {str(e)}")

            # or use gTTS
            else:
                try:
                    # Create TTS object
                    tts = gTTS(text=msg.data, lang='it', slow=False)
                    
                    # Transorm tts to right format
                    mp3_fp = BytesIO()
                    tts.write_to_fp(mp3_fp)
                    mp3_fp.seek(0)
                    
                    self.play_audio(mp3_fp.read(), "mp3")
                    
                except Exception as e:
                    self.get_logger().error(f"Error in gTTS playback: {str(e)}")
