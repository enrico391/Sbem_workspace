#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy import time
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
#from sensor_msgs.msg import Imu

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster




class ImuTf(Node):
   
    def __init__(self):
        super().__init__("imu_tf")
        self.nodename = "imu_tf"
        self.get_logger().info(f"-I- {self.nodename} started")

        #### parameters #######
        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value # the rate at which to publish the transform
        self.create_timer(1.0/self.rate_hz, self.update)  

        self.base_frame_id = "base_link"
        self.imu_frame_id = self.declare_parameter('imu_frame_id',
                                                    'imu').value  # the name of the imu reference frame

        # internal data
        self.imu_message = Vector3
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.quat_w = 0.0

        self.timeOdom = self.get_clock().now().to_msg()
        
        # subscriptions
        self.create_subscription(Vector3, "imu_publish", self.imu_callback, 10)
        self.camera_msg  = self.create_subscription(Odometry,"/odom",self.getdata_callBack, 10)
        self.imu_broadcaster = TransformBroadcaster(self)


    #get timestamp from odometry
    def getdata_callBack(self, data: Odometry):
        self.timeOdom = data._header.stamp


    def update(self):
        #now = self.get_clock().now()

        # publish the imu transform
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.timeOdom
        transform_stamped_msg.header.frame_id = self.imu_frame_id #base_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id #imu_frame_id
        transform_stamped_msg.transform.translation.x = 0.0
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = self.quat_z #self.imu_message.orientation.x
        transform_stamped_msg.transform.rotation.y = self.quat_w #self.imu_message.orientation.y
        transform_stamped_msg.transform.rotation.z = self.quat_x #self.imu_message.orientation.z
        transform_stamped_msg.transform.rotation.w = self.quat_y #self.imu_message.orientation.w

        self.imu_broadcaster.sendTransform(transform_stamped_msg)


    def imu_callback(self, msg):
        self.imu_message = msg
        self.euler_to_quat(self.imu_message.x, self.imu_message.y, self.imu_message.z)


    def euler_to_quat(self, x, y, z):
        c1 = cos((y*3.14/180.0)/2)
        c2 = cos((z*3.14/180.0)/2)
        c3 = cos((x*3.14/180.0)/2)      
                
        s1 = sin((y*3.14/180.0)/2)
        s2 = sin((z*3.14/180.0)/2)
        s3 = sin((x*3.14/180.0)/2)

        self.quat_x = c1 * c2 * c3 - s1 * s2 * s3
        self.quat_y = s1 * s2 * c3 + c1 * c2 * s3
        self.quat_z = s1 * c2 * c3 + c1 * s2 * s3
        self.quat_w = c1 * s2 * c3 - s1 * c2 * s3




def main(args=None):
    rclpy.init(args=args)
    try:
        imu_tf = ImuTf()
        rclpy.spin(imu_tf)
    except rclpy.exceptions.ROSInterruptException:
        pass

    imu_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
