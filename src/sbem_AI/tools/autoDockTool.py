#imports for ROS
from rclpy.action import ActionClient
from rclpy.node import Node
from tools.dock import DockingTester
from geometry_msgs.msg import PoseStamped
import rclpy

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


#imports for utilities
from math import sqrt
from enum import Enum
import time

#Tool for autodocking with agent

#class for handle stutus
class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class AutoDockInput(BaseModel):
    typeAction: bool = Field(description="True if it is a dock action, false if it is a undock action")
    # coord: dict = Field(description="dict of three number of x, y, z coordinates for the dock ")

class AutoDockCommander(BaseTool):
    name: str = "Autodock_Commander"
    description: str = "Use it for recharging the robot. It can dock or undock the robot. The output is a string with the result of the action."
    args_schema: Type[BaseModel] = AutoDockInput
    #return_direct: bool = True

    _tester: DockingTester = PrivateAttr()
    _realEnvironment : bool = PrivateAttr()
    
    def __init__(self, realEnvironment: bool = False):
        super().__init__()
        self._realEnvironment = realEnvironment
        #rclpy.init()
        # self._tester = DockingTester()
        # self._tester.startup()
    
    def _run(self, typeAction: bool, run_manager: Optional[CallbackManagerForToolRun] = None) -> str:
        """Use dock or undock."""
        self._tester = DockingTester()
        self._tester.startup()

        if(typeAction):
            dock_pose = PoseStamped()
            dock_pose.header.stamp = self._tester.get_clock().now().to_msg()
            dock_pose.header.frame_id = "map"

            
            if self._realEnvironment:
                #coordinates for the dock real world
                dock_pose.pose.position.x = 5.993222767289538 
                dock_pose.pose.position.y = -2.0058217548165835 
                dock_pose.pose.position.z = 0.0
                dock_pose.pose.orientation.x = 0.0
                dock_pose.pose.orientation.y = 0.0
                dock_pose.pose.orientation.z = 0.96093202830228586
                dock_pose.pose.orientation.w = 0.27678445943162139
            else:
                #coordinates for the dock gazebo
                dock_pose.pose.position.x = -4.28
                dock_pose.pose.position.y = -10.6
                dock_pose.pose.position.z = 0.0
                dock_pose.pose.orientation.x = 0.0
                dock_pose.pose.orientation.y = 0.0
                dock_pose.pose.orientation.z = 0.77
                dock_pose.pose.orientation.w = 0.64
            

            self._tester.dockRobot(dock_pose)

            # Test cancel action
            # time.sleep(0.5)
            # tester.cancelAction()

            i = 0
            while not self._tester.isTaskComplete():
                i = i + 1
                if i % 5 == 0:
                    print('Docking in progress...')
                time.sleep(1)

            # Do something depending on the return code
            result = self._tester.getResult()
            if result.name == "SUCCEEDED":
                print('Docking succeeded!')
                return "Docking succeded!"
            elif result.name == "CANCELED":
                print('Docking canceled!')
                return "Docking canceled!"
            elif result.name == "FAILED":
                print('Docking failed!')
                return "Docking failed!"
            else:
                print('Docking has an invalid return status!')
                return "Docking has an invalid return status!"

        
        elif(typeAction == False):

            self._tester.undockRobot("nova_carter_dock")

            i = 0
            while not self._tester.isTaskComplete():
                i = i + 1
                if i % 5 == 0:
                    print('Undocking in progress...')
                time.sleep(1)

            # Do something depending on the return code
            result = self._tester.getResult()
            if result.name == "SUCCEEDED":
                print('Undock succeeded!')
                return "Undock succeeded!"
            elif result.name == "CANCELED":
                print('Undock canceled!')
                return "Undock canceled"
            elif result.name == "FAILED":
                print('Undock failed!')
                return "Undock failed!"
            else:
                print('Undock has an invalid return status!')

