from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np 
import math

class MoveRobot(Node):

    def __init__(self):
        super().__init__(node_name='move_robot')

        self.publisherSpeed = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_diameter = 0.4
        self.wheel_radius = 0.08
        self.cir = self.robot_diameter * math.pi


    def rotate(self, degree, speed):
        time = (np.deg2rad(abs(degree))*self.cir) / (2 * math.pi * speed * self.wheel_radius)
        time = Duration(seconds=time)
        
        twist = Twist()
        
        self.get_logger().info(f"Rotating for {time}")

        now = self.get_clock().now()

        self.get_logger().info(f"Now is {now}")

        while (self.get_clock().now() < (time + now)):
            self.get_logger().info("Rotating")
            
            #check for clockwise or counterclockwise
            if degree < 0 : twist.angular.z = - speed
            if degree > 0 : twist.angular.z = speed

            self.publisherSpeed.publish(twist)
        
        twist.angular.z = 0.0
        self.publisherSpeed.publish(twist)



    def moveForward(self, distance, speed):
        time = Duration(seconds= (distance / speed))
        twist = Twist()
        
        now = self.get_clock().now()

        while self.get_clock().now() < (time + now):
            self.get_logger().info("Moving forward")
            twist.linear.x = speed
            self.publisherSpeed.publish(twist)

        twist.linear.x = 0.0
        self.publisherSpeed.publish(twist)


    def moveBackward(self, distance, speed):
        time = Duration(seconds= (distance / speed))
        twist = Twist()
        
        now = self.get_clock().now()

        while self.get_clock().now() < (time + now):
            self.get_logger().info("Moving backward")
            twist.linear.x = - speed
            self.publisherSpeed.publish(twist)

        twist.linear.x = 0.0
        self.publisherSpeed.publish(twist)





#imports for agent
from langchain_core.tools import tool
from langchain_core.tools import BaseTool
from pydantic import BaseModel, Field
from typing import Optional, Type
from langchain_core.callbacks import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from pydantic import PrivateAttr



#Tool for move robot
class MoveInput(BaseModel):
    degrees: float = Field(description="Degrees of rotation")
    meters: float = Field(description="Meters of movement")
    typeAction: str = Field(description="It must be one of the following: 'rotate', 'moveForward', 'moveBackward'")
    speed: float = Field(description="Speed for moviments, default = 0.5  and NEVER ABOVE 0.5")
    
class MoveCommander(BaseTool):
    name: str = "Move_Commander"
    description: str = "Useful to rotate, move forward and backward"
    args_schema: Type[BaseModel] = MoveInput
    #return_direct: bool = True

    def __init__(self):
        super().__init__()

    def _run(self, degrees: float, meters: float , typeAction: str, speed: float, run_manager: Optional[CallbackManagerForToolRun] = None):
        
        moveRobot = MoveRobot()
        
        if(typeAction == 'rotate'):
            moveRobot.rotate(degrees, speed)
            return "Rotation completed"
        elif(typeAction == 'moveForward'):
            moveRobot.moveForward(meters, speed)
            return "Moving forward completed"
        elif(typeAction == 'moveBackward'):
            moveRobot.moveBackward(meters, speed)
            return "Moving backward completed"
        else:
            return "Type of action not recognized"



moveCommander = MoveCommander()