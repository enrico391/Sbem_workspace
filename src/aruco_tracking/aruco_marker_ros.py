#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import math

from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.node import Node
from geometry_msgs.msg      import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg        import Image
import math
from geometry_msgs.msg import Quaternion

from arucoPoseEstimation import ArucoPoseEstimation


class DetectAruco(Node):

    def __init__(self):
        super().__init__('detect_aruco')
        self.nodename = "detect_aruco"
        self.get_logger().info(f"-I- {self.nodename} started")

        self.get_logger().info('Detecting in 3D')
        # self.camera_msg  = self.create_subscription(Image,"/camera/image_raw/uncompressed",self.aruco_callback, 10)
        self.camera_msg  = self.create_subscription(Image,"/camera/image_raw",self.aruco_callback, 10)
        self.camera_msg  = self.create_subscription(Odometry,"/odom",self.getdata_callBack, 10)
        self.aruco_marker_pub  = self.create_publisher(Marker,"/aruco/marker",10)
        self.aruco_check_marker_pub  = self.create_publisher(Bool,"/aruco/check_marker",10)

        #for docking nav2
        self.aruco_pose_pub  = self.create_publisher(PoseStamped,"detected_dock_pose",1)

        self.aruco_marker_trans_pub  = self.create_publisher(TransformStamped,"/aruco/marker_transform",10)

        self.camera_pos_pub  = self.create_publisher(TransformStamped,"/camera/poseEstimation",10)

        self.rate_hz = self.declare_parameter("rate_hz", 5.0).value # the rate at which to publish the transform
        self.create_timer(1.0/self.rate_hz, self.publish_transform)
        

        self.declare_parameter("h_fov",1.089)
        self.declare_parameter("aspect_ratio",4.0/3.0)
        self.declare_parameter("camera_frame",'camera_link')
        
        self.h_fov = self.get_parameter('h_fov').get_parameter_value().double_value
        self.v_fov = self.h_fov/self.get_parameter('aspect_ratio').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.camera_frame_id = "camera_link"
        self.aruco_marker_id = "fiducial_"

        self.timeOdom = None

        self.processImage = ArucoPoseEstimation()

        self.aruco_pose_broadcaster = TransformBroadcaster(self)

        self.marker_check = Bool()

        self.R_flip  = np.zeros((3,3), dtype=np.float32)
        self.R_flip[0,0] = 1.0
        self.R_flip[1,1] =-1.0
        self.R_flip[2,2] =-1.0


    def getdata_callBack(self, data: Odometry):
        self.timeOdom = data._header.stamp


    def publish_transform(self):
            #rotation_matrix = cv2.Rodrigues(rot)[0]
            #quat = self.quaternion_from_matrix(rotation_matrix)
            
            if(self.processImage.flag_is_detected()):

                #publish if marker is detected or not
                
                self.marker_check.data = True
                self.aruco_check_marker_pub.publish(self.marker_check)


                for i in range(0, len(self.ids)):
                    transform_stamped_msg = TransformStamped()
                    if(self.timeOdom):
                        transform_stamped_msg.header.stamp =   self.timeOdom   #self.get_clock().now().to_msg()
                    else:
                        transform_stamped_msg.header.stamp =  self.get_clock().now().to_msg()

                    transform_stamped_msg.header.frame_id = self.camera_frame_id
                    
                    current_id = (self.ids[i][0])
                    transform_stamped_msg.child_frame_id = f"{self.aruco_marker_id}{current_id}"

                    rot_matrix = np.eye(4)
                    #rodrig = cv2.Rodrigues(np.array(self.rot_array[i]))
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(self.rot_array[i]))[0]
                    
                    quaternion_trans = Quaternion()
                    quaternion_trans = self.get_quaternion_from_euler(self.rot_array[i][0][0])


                    
                    #right configuration for mockmini charger
                    transform_stamped_msg.transform.translation.x =   self.trans_array[i][0][0][2] #self.trans[i][0][2]
                    transform_stamped_msg.transform.translation.y = - self.trans_array[i][0][0][0] #self.trans[i][0][1]
                    transform_stamped_msg.transform.translation.z = self.trans_array[i][0][0][1] #self.trans[i][0][0]


                    transform_stamped_msg.transform.rotation.x = quaternion_trans.y 
                    transform_stamped_msg.transform.rotation.y = - quaternion_trans.x  #- quaternion_trans.z 
                    transform_stamped_msg.transform.rotation.z = - quaternion_trans.z  #- quaternion_trans.y 
                    transform_stamped_msg.transform.rotation.w = quaternion_trans.w 


                    self.get_logger().info(f"Translation id {i} : {self.trans_array[i][0][0][2]} , {self.trans_array[i][0][0][0]} , {self.trans_array[i][0][0][1]}" )
                    self.get_logger().info(f"Rotation id {i} : {quaternion_trans.x} , {quaternion_trans.y} , {quaternion_trans.z} , {quaternion_trans.w}" )


                    message_stamped = PoseStamped()
                    message_stamped.pose.position.x = self.trans_array[i][0][0][2]
                    message_stamped.pose.position.y = - self.trans_array[i][0][0][0]
                    message_stamped.pose.position.z =  self.trans_array[i][0][0][1]
                    message_stamped.pose.orientation.w = quaternion_trans.w 
                    message_stamped.pose.orientation.x = quaternion_trans.y
                    message_stamped.pose.orientation.y = - quaternion_trans.x
                    message_stamped.pose.orientation.z = - quaternion_trans.z

                    self.aruco_pose_pub.publish(message_stamped)

                    #get camera position and orientation from marker
                    # camera_position_transform_stamped_msg = TransformStamped()
                    
                    
                    # rot_ct = np.matrix(cv2.Rodrigues(self.rot_array[i])[0])
                    # rot_tc = rot_ct.T
                    
                    # pos_camera = np.matrix(self.trans_array[i]) * -rot_tc
                    
                    
                    # pitch_camera, yaw_camera, roll_camera = self.rotationMatrixToEulerAngles(self.R_flip * rot_ct)
                    
                    # if(self.timeOdom):
                    #     camera_position_transform_stamped_msg.header.stamp =   self.timeOdom   #self.get_clock().now().to_msg()
                    # else:
                    #     camera_position_transform_stamped_msg.header.stamp =  self.get_clock().now().to_msg()
                    
                    # camera_position_transform_stamped_msg.header.frame_id = transform_stamped_msg.child_frame_id
                    # camera_position_transform_stamped_msg.child_frame_id = "camera_pose"

                    # camera_position_transform_stamped_msg.transform.translation.x = pos_camera.item(0,2)
                    # camera_position_transform_stamped_msg.transform.translation.y = pos_camera.item(0,0)
                    # camera_position_transform_stamped_msg.transform.translation.z = pos_camera.item(0,1)

                    # camera_position_transform_stamped_msg.transform.rotation.x = roll_camera
                    # camera_position_transform_stamped_msg.transform.rotation.y = pitch_camera
                    # camera_position_transform_stamped_msg.transform.rotation.z = yaw_camera

                    # self.camera_pos_pub.publish(camera_position_transform_stamped_msg)


                    #publish transform for auto dock
                    
                    self.aruco_marker_trans_pub.publish(transform_stamped_msg)

                    self.aruco_pose_broadcaster.sendTransform(transform_stamped_msg)

                    # self.aruco_pose_broadcaster.sendTransform(camera_position_transform_stamped_msg)


               
            else:
                self.marker_check.data = False
                self.aruco_check_marker_pub.publish(self.marker_check)


    def aruco_callback(self, data : Image):
        self.processImage.aruco_process_image(data)

        if(self.processImage.flag_is_detected()):
            self.trans = self.processImage.get_translation()
            self.rot = self.processImage.get_rotation()
            self.ids = self.processImage.get_ids()

            self.trans_array = self.processImage.get_translation_array()
            self.rot_array = self.processImage.get_rotation_array()
            

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-2
    

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self,R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-2

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def get_quaternion_from_euler(self, rot_values):
        roll = rot_values[0] 
        pitch = rot_values[1] 
        yaw = rot_values[2] #- np.deg2rad(180)
        """
        Convert an Euler angle to a quaternion.

        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw

        return q


    # def rot2eul(self, R):
    #     beta = -np.arcsin(R[2,0])
    #     alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    #     gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    #     return np.array((alpha, beta, gamma))
    
    # def quaternion_from_euler(self, ai, aj, ak):
    #     ai /= 2.0
    #     aj /= 2.0
    #     ak /= 2.0
    #     ci = math.cos(ai)
    #     si = math.sin(ai)
    #     cj = math.cos(aj)
    #     sj = math.sin(aj)
    #     ck = math.cos(ak)
    #     sk = math.sin(ak)
    #     cc = ci*ck
    #     cs = ci*sk
    #     sc = si*ck
    #     ss = si*sk

    #     q = np.empty((4, ))
    #     q[0] = cj*sc - sj*cs
    #     q[1] = cj*ss + sj*cc
    #     q[2] = cj*cs - sj*sc
    #     q[3] = cj*cc + sj*ss

    #     return q

    # def quaternion_from_matrix(self, rotation_matrix):
    #     """Calculates the quaternion that corresponds to the given rotation matrix.

    #     Args:
    #         rotation_matrix: A 3x3 rotation matrix.

    #     Returns:
    #         A quaternion that corresponds to the given rotation matrix.
    #     """

    #     q = np.empty(4)
    #     m00 = rotation_matrix[0, 0]
    #     m01 = rotation_matrix[0, 1]
    #     m02 = rotation_matrix[0, 2]
    #     m10 = rotation_matrix[1, 0]
    #     m11 = rotation_matrix[1, 1]
    #     m12 = rotation_matrix[1, 2]
    #     m20 = rotation_matrix[2, 0]
    #     m21 = rotation_matrix[2, 1]
    #     m22 = rotation_matrix[2, 2]
    #     t = m00 + m11 + m22
    #     if t > 0:
    #         s = np.sqrt(t + 1.0) * 2
    #         qw = 0.25 * s
    #         qx = (m21 - m12) / s
    #         qy = (m02 - m20) / s
    #         qz = (m10 - m01) / s
    #     elif (m00 > m11) and (m00 > m22):
    #         s = np.sqrt(1.0 + m00 - m11 - m22) * 2
    #         qw = (m21 - m12) / s
    #         qx = 0.25 * s
    #         qy = (m01 + m10) / s
    #         qz = (m02 + m20) / s
    #     elif m11 > m22:
    #         s = np.sqrt(1.0 + m11 - m00 - m22) * 2
    #         qw = (m02 - m20) / s
    #         qx = (m01 + m10) / s
    #         qy = 0.25 * s
    #         qz = (m12 + m21) / s
    #     else:
    #         s = np.sqrt(1.0 + m22 - m00 - m11) * 2
    #         qw = (m10 - m01) / s
    #         qx = (m02 + m20) / s
    #         qy = (m12 + m21) / s
    #         qz = 0.25 * s
    #     q[0] = qw
    #     q[1] = qx
    #     q[2] = qy
    #     q[3] = qz
    #     return q
    

def main(args=None):
    rclpy.init(args=args)
    detect_aruco = DetectAruco()
    rclpy.spin(detect_aruco)
    detect_aruco.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()