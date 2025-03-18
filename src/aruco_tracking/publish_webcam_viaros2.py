import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.cv_bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)  # Change the argument to switch between different cameras

        # Set the width and height of the capture frame
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Create a timer to periodically publish the images
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        ret, frame = self.capture.read()
        if ret:
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()