#!/usr/bin/env python3

import pyaudio
import wave
import sys
import numpy as np
from gtts import gTTS 
from io import BytesIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from pydub import AudioSegment
from audioProcess.send_toTTS import SenderTTS
import threading

from rclpy.qos import qos_profile_sensor_data



class AudioPlayerNode(Node):
    """Node to play audio response from agent"""
    def __init__(self, useLocalTTS= False, typeLocalTTS="coqui") -> None:
        super().__init__("tts_sbem")

        #check if use local TTS and initialize the class
        self.useLocalTTS = useLocalTTS
        if(self.useLocalTTS):
            self.requestTTS = SenderTTS(typeLocalTTS)
        
        # variable for audio playback
        self.audio = pyaudio.PyAudio()

        # variable for interrupting the playback
        self.interrupt_playback = False

        # declare parameters for the audio player
        self.declare_parameters("", [
            ("channels", 1),
            ("device", -1),
        ])

        # subscribe to the response from the agent
        self.sub = self.create_subscription(
            String, "/response_to_user", self.audio_callback, qos_profile_sensor_data)
        
        # subscribe to the wake word detected for interrupt the audio streaming
        self.interrupt_sub = self.create_subscription(
            Bool, "/start_listening", self.interrupt_callback, 10)

        self.get_logger().info("TTS Sbem node started")

    def destroy_node(self) -> bool:
        self.audio.terminate()
        return super().destroy_node()
    

    def interrupt_callback(self, msg: Bool) -> None:
        """Callback triggered when a wake word is detected to interrupt playback"""
        if msg.data:
            self.interrupt_playback = True


    def play_audio(self, content, format):
        """Function to play audio data"""
        
        try:
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
            
            
            
            # Reset interrupt flag at start of playback
            self.interrupt_playback = False

            chunk_size = 1024
            for i in range(0, len(raw_data), chunk_size):
                if self.interrupt_playback:
                    self.get_logger().info("Playback interrupted")
                    break
                stream.write(raw_data[i:i+chunk_size])
            
            # reset interrupt flag after playback
            self.interrupt_playback = False
            # Clean stream
            stream.stop_stream()
            stream.close()

        except Exception as e:
            self.get_logger().error(f"Error playing audio: {str(e)}")
        


    def audio_callback(self, msg: String) -> None:
        """callback for subscriber to string response from agent"""
        # for interrupting the playback when a new message is received
        self.interrupt_playback = True
        
        if(msg.data != ""):
            #send request to piper server for transcribe or publish over topic
            if self.useLocalTTS:
                # send local request and get content directly
                content = self.requestTTS.sendRequest(msg.data)
                
                try:
                    # Start playback in a new thread
                    self.playback_thread = threading.Thread(
                        target=self.play_audio,
                        args=(content, "wav")
                    )
                    self.playback_thread.daemon = True  # Thread will exit when main program exits
                    self.playback_thread.start()
                    
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
                    
                    # Start playback in a new thread
                    self.playback_thread = threading.Thread(
                        target=self.play_audio,
                        args=(mp3_fp.read(), "mp3")
                    )
                    self.playback_thread.daemon = True  # Thread will exit when main program exits
                    self.playback_thread.start()
                    
                except Exception as e:
                    self.get_logger().error(f"Error in gTTS playback: {str(e)}")
