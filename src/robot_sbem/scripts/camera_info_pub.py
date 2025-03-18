import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, '/camera_info/rasp', 1)
        timer_period = 1.0  # Publish every second
        self.timer = self.create_timer(timer_period, self.publish_camera_info)

    def publish_camera_info(self):
        camera_info_msg = CameraInfo()

        # Set header (with current time)
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.header.frame_id = 'camera_frame'

        # Camera resolution
        camera_info_msg.width = 640
        camera_info_msg.height = 480

        # Camera intrinsic matrix (K)
        camera_info_msg.k = [
            694.20522301, 0.0, 298.70012202,  # fx, 0, cx
            0.0, 696.74964052, 244.74441977,  # 0, fy, cy
            0.0, 0.0, 1.0       # 0, 0, 1
        ]

        # Camera distortion model and coefficients
        camera_info_msg.distortion_model = 'plumb_bob'  # Or 'rational_polynomial'
        camera_info_msg.d = [-3.88181994e-01, -2.24390023e-01,  2.94297314e-04,  2.20218685e-03, 9.44258171e-01]  # Distortion coefficients

        # Rectification matrix (R)
        camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (P)
        camera_info_msg.p = [
            694.20522301, 0.0, 298.70012202, 0.0,  # fx, 0, cx, Tx
            0.0, 696.74964052, 244.74441977, 0.0,  # 0, fy, cy, Ty
            0.0, 0.0, 1.0, 0.0       # 0, 0, 1, 0
        ]

        self.publisher_.publish(camera_info_msg)
        self.get_logger().info('Published camera info')

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()