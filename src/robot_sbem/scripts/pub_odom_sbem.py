#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy

NS_TO_SEC= 1000000000

RADIUS_WHEEL = 0.08
CPR = 90

class DiffTf(Node):

    def __init__(self):
        super().__init__("odometry_publisher")
        self.nodename = "odometry_publisher"
        self.get_logger().info(f"-I- {self.nodename} started")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # or RELIABLE
            depth=10
            )  

        #### parameters #######
        self.rate_hz = self.declare_parameter("rate_hz", 30.0).value # the rate at which to publish the transform
        self.create_timer(1.0/self.rate_hz, self.update)  

        self.ticks_meter = float(
            self.declare_parameter('ticks_meter', 1.79).value)  # The number of wheel encoder ticks per meter of travel  179 == CPR /(2 * pi * RADIUS_WHEEL)
        self.base_width = float(self.declare_parameter('base_width', 0.360).value)  # The wheel base width in meters

        self.base_frame_id = self.declare_parameter('base_frame_id',
                                                    'base_link').value  # the name of the base frame of the robot
        self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                    'odom').value  # the name of the odometry reference frame

        self.encoder_min = self.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.declare_parameter('encoder_max', 32768).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        # internal data
        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.left = 0.0  # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()

        self.flag_offsetL = True
        self.flag_offsetR = True
        self.offsetL = 0.0
        self.offsetR = 0.0

        # subscriptions
        self.create_subscription(Float32, "lwheel", self.lwheel_callback, qos_profile=qos_profile)
        self.create_subscription(Float32, "rwheel", self.rwheel_callback, qos_profile=qos_profile)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)


    def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right

        # distance traveled is the average of the two wheels 
        d = (d_left + d_right) / 2
        # this approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            # calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0      
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        #get time 
        current_time = self.get_clock().now().to_msg()

        #publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id

        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        
        self.odom_pub.publish(odom)

    def lwheel_callback(self, msg):
        #store first value as initial value TODO
        if self.flag_offsetL:
            self.offsetL = msg.data
            self.flag_offsetL = False
            
        enc = msg.data - self.offsetL 

        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheel_callback(self, msg):
        #store first value as initial value TODO
        if self.flag_offsetR:
            self.offsetR = msg.data
            self.flag_offsetR = False

        enc = msg.data - self.flag_offsetR

        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc


def main(args=None):
    rclpy.init(args=args)
    try:
        diff_tf = DiffTf()
        rclpy.spin(diff_tf)
    except rclpy.exceptions.ROSInterruptException:
        pass

    diff_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
