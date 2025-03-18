#!/usr/bin/env python3
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
            # self.get_logger().info(f"Transform from {self.source_frame} to {self.target_frame}:")
            # self.get_logger().info(f"Translation: x={trans.transform.translation.x:.2f}, "
            #                        f"y={trans.transform.translation.y:.2f}, "
            #                        f"z={trans.transform.translation.z:.2f}")
            # self.get_logger().info(f"Rotation (quaternion): x={trans.transform.rotation.x:.2f}, "
            #                        f"y={trans.transform.rotation.y:.2f}, "
            #                        f"z={trans.transform.rotation.z:.2f}, "
            #                        f"w={trans.transform.rotation.w:.2f}")
            
    
            
            current_pose.child_frame_id = self.source_frame
            current_pose.header.frame_id = self.target_frame
            current_pose.header.stamp.sec = rclpy.time.Time().seconds_nanoseconds()[0]
            current_pose.transform.translation.x = trans.transform.translation.x
            current_pose.transform.translation.y = trans.transform.translation.y
            current_pose.transform.rotation.w = trans.transform.rotation.w
            current_pose.transform.rotation.z = trans.transform.rotation.z

            #self.tf_pub.publish(current_pose)

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")


    def return_current_pose(self):
        return current_pose
# def main(args=None):
#     rclpy.init(args=args)
#     # Change these frame names as necessary.
#     target_frame = "map"
#     source_frame = "base_link"
#     node = TfEcho()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("TF echo node shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()