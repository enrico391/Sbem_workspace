#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import socket
import pickle
import numpy as np
import threading
from faster_whisper import WhisperModel

SERVER_IP = '0.0.0.0'  # Listen on all interfaces
SERVER_PORT = 8765

class AudioTranscriptionServer(Node):
    def __init__(self):
        super().__init__('audio_transcription_server')
        self.pub = self.create_publisher(String, '/user_input', 10)
        self.get_logger().info("AudioTranscriptionServer node started.")

        self.model = WhisperModel("large-v3", device="cuda", compute_type="float16")

        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

    def run_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((SERVER_IP, SERVER_PORT))
            s.listen(1)
            self.get_logger().info(f"Socket server listening on {SERVER_IP}:{SERVER_PORT}")
            while rclpy.ok():
                conn, addr = s.accept()
                self.get_logger().info(f"Connection from {addr}")
                threading.Thread(target=self.handle_client, args=(conn,), daemon=True).start()

    def handle_client(self, conn):
        with conn:
            while True:
                try:
                    # Receive length of incoming data (4 bytes)
                    length_bytes = conn.recv(4)
                    if not length_bytes or len(length_bytes) < 4:
                        break
                    data_len = int.from_bytes(length_bytes, byteorder='big')
                    # Receive the actual data
                    data = b''
                    while len(data) < data_len:
                        packet = conn.recv(data_len - len(data))
                        if not packet:
                            break
                        data += packet
                    if len(data) != data_len:
                        break
                    # Unpickle the numpy array
                    audio_float = pickle.loads(data)
                    self.get_logger().info(f"Received audio_float array of shape {audio_float.shape}")

                    # Dummy transcription logic (replace with real model)
                    # Transcribe directly from the numpy array
                    segments, _ = self.model.transcribe(audio_float, beam_size=5)

                    # Publish to ROS topic
                    for segment in segments:
                        transcript = segment.text
                        
                        # Publish transcript
                        msg = String()
                        msg.data = transcript
                        self.pub.publish(msg)
            

                    # Pickle and send back the transcription
                    response = pickle.dumps(transcript)
                    conn.sendall(len(response).to_bytes(4, byteorder='big'))
                    conn.sendall(response)
                    self.get_logger().info("Sent transcription response to client.")
                except Exception as e:
                    self.get_logger().error(f"Error handling client: {e}")
                    break

def main(args=None):
    rclpy.init(args=args)
    node = AudioTranscriptionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()