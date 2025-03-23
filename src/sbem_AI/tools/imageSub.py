import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import base64
import google.generativeai as genai


#imports for agent
from langchain_core.tools import tool
from langchain_core.tools import BaseTool
from pydantic import BaseModel, Field
from typing import Optional, Type
from langchain_core.callbacks import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)



class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        
        self.cv_image = None

        # Create a subscription to the image topic
        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 1)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            #self.get_logger().info("Get image")

            # Convert ROS2 Image message to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            
            
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


    #get image for tool langraph
    def getImageBytes(self):
        # Save the image as a JPEG file
        #rclpy.spin_once(self)

        # get the base64 format img
        retval, buffer = cv2.imencode('.jpg', self.cv_image)
        # jpg_as_text = base64.b64encode(buffer)
        # contents=["What is this image?",
        #       types.Part.from_bytes(data=buffer.tobytes(), mime_type="image/jpeg")]
        
        return buffer.tobytes()

        
        

# def main(args=None):
#     rclpy.init(args=args)
#     image_sub = ImageSubscriber()
#     rclpy.spin(image_sub)
#     image_sub.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()