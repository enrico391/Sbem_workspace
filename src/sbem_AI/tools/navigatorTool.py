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


#imports for ROS
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import TransformStamped


from positionManager import PositionsManager

#Tool for nav to pose
class NavigatorInput(BaseModel):
    x_pos: float = Field(description="This is the X coordinate for navigation, set to 0.0 if you use the namePos")
    y_pos: float = Field(description="This is the Y coordinate for navigation, set to 0.0 if you use the namePos")
    namePos: str = Field(description="the appropriate name of the position or leaves blank if you don't know")
    
class NavigatorCommander(BaseTool):
    name: str = "Navigator_Commander"
    description: str = "Useful to go in a specific position if the user wants to move in a specific position or to go in a position with his name"
    args_schema: Type[BaseModel] = NavigatorInput
    #return_direct: bool = True

    _navigator = PrivateAttr()
    _tf_manage: PositionsManager = PrivateAttr()

    def __init__(self, tf_manage):
        super().__init__()
        self._tf_manage = tf_manage

    def _run(self, x_pos: float, y_pos: float, namePos: str, run_manager: Optional[CallbackManagerForToolRun] = None) -> str:
        
        if(namePos != ""):
            coord = self._tf_manage.return_position().get(namePos)
            x_pos = coord["x"]
            y_pos = coord["y"]
            
        #initialize the navigator
        self._navigator = BasicNavigator()
            
        #wait for nav to activate
        self._navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x_pos)
        goal_pose.pose.position.y = float(y_pos)
        #goal_pose.pose.orientation.w = 1.0 #float(coord["w"])
        #goal_pose.pose.orientation.z = 1.0 #float(coord["z"])

        self._navigator.goToPose(goal_pose)
        i = 0

        while not self._navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = self._navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self._navigator.cancelTask()

        # Do something depending on the return code
        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return "Goal succeeded!"
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return "Goal was canceled!"
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            return "Goal failed!"
        else:
            return "Error"
    
    async def _arun(self, x_pos: float, y_pos: float, namePos: str, run_manager: Optional[CallbackManagerForToolRun] = None) -> str:
        
        if(namePos != ""):
            coord = self._tf_manage.return_position().get(namePos)
            x_pos = coord["x"]
            y_pos = coord["y"]
            
        #initialize the navigator
        self._navigator = BasicNavigator()
            
        #wait for nav to activate
        self._navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x_pos)
        goal_pose.pose.position.y = float(y_pos)
        #goal_pose.pose.orientation.w = 1.0 #float(coord["w"])
        #goal_pose.pose.orientation.z = 1.0 #float(coord["z"])

        self._navigator.goToPose(goal_pose)
        i = 0

        while not self._navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = self._navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self._navigator.cancelTask()

        # Do something depending on the return code
        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return "Goal succeeded!"
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return "Goal was canceled!"
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            return "Goal failed!"
        else:
            return "Error"
                
