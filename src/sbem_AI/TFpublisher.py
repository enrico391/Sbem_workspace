#!/usr/bin/env python3

# Class for agent tool savePosition


import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped


current_pose = TransformStamped()


class TfEcho(Node):
    def __init__(self):
        super().__init__('tf_echo')
        self.target_frame = "map"
        self.source_frame = "base_link"

        # Create a TF2 buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.tf_pub = self.create_publisher(TransformStamped,"/current_pose",10)

        # Create a timer to check for the transform every second.
        self.timer = self.create_timer(1.0, self.timer_callback)

        
    
    def timer_callback(self):
        try:
            # Use the current time to lookup the latest transform.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.target_frame,
                                                    self.source_frame,
                                                    now)
        
            
            current_pose.child_frame_id = self.source_frame
            current_pose.header.frame_id = self.target_frame
            current_pose.header.stamp.sec = rclpy.time.Time().seconds_nanoseconds()[0]
            current_pose.transform.translation.x = trans.transform.translation.x
            current_pose.transform.translation.y = trans.transform.translation.y
            current_pose.transform.rotation.w = trans.transform.rotation.w
            current_pose.transform.rotation.z = trans.transform.rotation.z


        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")


    def return_current_pose(self):
        return current_pose
