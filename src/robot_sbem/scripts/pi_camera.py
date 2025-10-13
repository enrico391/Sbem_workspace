import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
  
class ImagePublisher(Node):
  
  def __init__(self):
    super().__init__('image_publisher')
    self.publisher_ = self.create_publisher(Image, '/camera/image', 10)
    
    self.pub_camera_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

    # Load YAML
    yaml_file = '/home/moro-jetson/Projects/Sbem_workspace/src/robot_sbem/config/ost.yaml'
    with open(yaml_file, 'r') as f:
        cam_data = yaml.safe_load(f)

    # Fill CameraInfo
    self.cam_info = CameraInfo()
    self.cam_info.width = cam_data['image_width']
    self.cam_info.height = cam_data['image_height']
    self.cam_info.distortion_model = cam_data['distortion_model']
    self.cam_info.k = cam_data['camera_matrix']['data']
    self.cam_info.d = cam_data['distortion_coefficients']['data']
    self.cam_info.r = cam_data['rectification_matrix']['data']
    self.cam_info.p = cam_data['projection_matrix']['data']
    self.cam_info.header.frame_id = 'camera_link'
    

    timer_period = 0.01
    print(cv2.getBuildInformation())
    self.cvb = CvBridge()
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)480, height=(int)360,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    
    
  def timer_callback(self):
    time = self.get_clock().now().to_msg()
    self.cam_info.header.stamp = time
    
    ret, frame = self.cap.read()
    if ret == True:
      msg = self.cvb.cv2_to_imgmsg(frame, encoding='bgr8')
      msg.header.stamp = time
      msg.header.frame_id = 'camera_link'
      
      self.publisher_.publish(msg)
      self.pub_camera_info.publish(self.cam_info)
      self.get_logger().info('Publishing video frame')
   
def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
