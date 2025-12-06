#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

# GStreamer pipeline function
def gstreamer_pipeline(
        capture_width=1280, # camera captured width
        capture_height=720, # height
        display_width=1280, # displayed window witdh
        display_height=720, # height
        framerate=60,       # captured fps
        flip_method=0,      # whether rotate image
    ):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters (change as needed)
        self.capture_width = 1280
        self.capture_height = 720
        self.display_width = 640
        self.display_height = 480
        self.framerate = 10
        self.flip_method = 0

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Bridge for OpenCV -> ROS
        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(
            gstreamer_pipeline(
                self.capture_width,
                self.capture_height,
                self.display_width,
                self.display_height,
                self.framerate,
                self.flip_method
            ),
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            return

        # Timer to publish at ~framerate
        timer_period = 1.0 / self.framerate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # CameraInfo setup (example values)
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.display_width
        self.camera_info_msg.height = self.display_height
        self.camera_info_msg.k = [1000.0, 0.0, self.display_width/2, 0.0, 1000.0, self.display_height/2, 0.0, 0.0, 1.0]  # fx,0,cx,0,fy,cy,0,0,1
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # distortion coefficients
        self.camera_info_msg.distortion_model = 'plumb_bob'

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # Convert OpenCV image to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        # CameraInfo header must match Image header
        self.camera_info_msg.header = msg.header

        # Publish messages
        self.image_pub.publish(msg)
        self.camera_info_pub.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
