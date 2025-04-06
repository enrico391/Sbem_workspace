#!/usr/bin/env python3

# Class for agent tool savePosition


import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Vector3
import json
import os


class PositionsManager(Node):
    def __init__(self):
        super().__init__('tf_echo')
        self.target_frame = "map"
        self.source_frame = "base_link"

        # Create a TF2 buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        #self.tf_pub = self.create_publisher(Vector3,"/current_poses",10)

        # variable to store current position get from robot tf
        self.current_pose = TransformStamped()
        # variable to store all saved positions
        self.positions = dict()

        # Create a timer to check for the transform every second.
        self.timer = self.create_timer(1.0, self.timer_callback)

        # get the path of the json file
        self.position_json_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "sbem_AI/saved_param/positions.json"
        )

        # Load saved positions if file exists
        self.positions = self.load_positions()


    def load_positions(self):
        """Load positions from file json"""
        try:
            if os.path.exists(self.position_json_path):
                with open(self.position_json_path, 'r') as f:
                    return json.load(f)
            else:
                return {}
        except Exception as e:
            self.get_logger().error(f"Error loading positions: {e}")
            return {}


    def save_positions(self):
        """Save positions to file json"""
        try:
            # check if the file exists
            os.makedirs(os.path.dirname(self.position_json_path), exist_ok=True)
            
            with open(self.position_json_path, 'w') as f:
                json.dump(self.positions, f, indent=2)
            
            self.get_logger().debug("Positions saved to file")
        except Exception as e:
            self.get_logger().error(f"Error saving positions: {e}")


    def timer_callback(self):
        try:
            # Use the current time to lookup the latest transform.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.target_frame,
                                                    self.source_frame,
                                                    now)
        
            self.current_pose.child_frame_id = self.source_frame
            self.current_pose.header.frame_id = self.target_frame
            self.current_pose.header.stamp.sec = rclpy.time.Time().seconds_nanoseconds()[0]
            self.current_pose.transform.translation.x = trans.transform.translation.x
            self.current_pose.transform.translation.y = trans.transform.translation.y
            self.current_pose.transform.rotation.w = trans.transform.rotation.w
            self.current_pose.transform.rotation.z = trans.transform.rotation.z

        except TransformException as ex:
            #self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")
            pass

    def return_current_pose(self):
        """Return the current pose of the robot"""
        return self.current_pose
    
    def return_position(self):
        """Return the saved positions with coordinates"""
        return self.positions
    
    def return_name_position(self):
        """Return the name of saved positions"""
        return list(self.positions.keys())
    
    def update_position(self, namePos, pose):
        """Add a new position to the list of saved positions"""
        self.positions.update({namePos: {"x":  pose.transform.translation.x, "y":  pose.transform.translation.y, "w": pose.transform.rotation.w, "z": pose.transform.rotation.z}})
        self.save_positions()

    def modify_position(self, namePos, newNamePos):
        """Modify the name of a saved position"""

        if newNamePos in self.positions:
            return "New position name already exists"
        if namePos not in self.positions:
            return "Position not found"
        #else
        position_data = self.positions[namePos]
        self.positions[newNamePos] = position_data
        # Remove the old entry
        del self.positions[namePos]
        self.save_positions()
        return f"Position renamed from '{namePos}' to '{newNamePos}' successfully"
    
    def delete_position(self, namePos):
        """Delete a position from the list of saved positions"""
        if namePos in self.positions:
            del self.positions[namePos]
            self.save_positions()
            return f"Position '{namePos}' deleted successfully"
        else:
            return "Position not found"
        