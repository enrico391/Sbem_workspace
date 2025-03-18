
import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import cv2


class DetectBall(Node):

    def __init__(self):
        super().__init__('calibration_getImages')

        self.get_logger().info('Start calibration')
        self.image_sub = self.create_subscription(Image,"/image_raw",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

        self.bridge = CvBridge()
        self.num = 0


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            k = cv2.waitKey(5)

            if k == ord('s'): # wait for 's' key to save and exit
                cv2.imwrite('/home/morolinux/Documents/sbem_project_ws/src/aruco_tracking/aruco_calib_realCamera/images/img' + str(self.num) + '.png', cv_image)
                print("image saved!")
                self.num += 1

            cv2.imshow('Img',cv_image)

        except CvBridgeError as e:
            print(e)



def main(args=None):

    rclpy.init(args=args)

    detect_ball = DetectBall()
    while rclpy.ok():
        rclpy.spin_once(detect_ball)

    detect_ball.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()