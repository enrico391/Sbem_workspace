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


from positionManager import PositionsManager


#Tool for nav to pose
class SavePosInput(BaseModel):
    namePos: str = Field(description="the appropriate name for the stored position")
    getPos: bool = Field(description="False if you want to store the current position, True if you want to get the stored position with a specific name")
    
    
class SavePosition(BaseTool):
    name: str = "Save_position"
    description: str = "Useful to store the current position. If the user wants to obtain coordinates you can use the appropriate name and get the coordinates"
    args_schema: Type[BaseModel] = SavePosInput
    #return_direct: bool = True
    _tf_manage: PositionsManager = PrivateAttr()
    

    def __init__(self, tf_manage: PositionsManager):
        super().__init__()
        self._tf_manage = tf_manage
        

    def _run(self, getPos: bool, namePos: str, run_manager: Optional[CallbackManagerForToolRun] = None)-> str:
        #store current position in the list
        if(getPos == False):
            if(namePos == ""):
                return "You must insert a name for the position"
            else:
                #get current pose from tf manager
                pose = self._tf_manage.return_current_pose()
                # add position
                self._tf_manage.update_position(namePos, pose)
                #positions.update({namePos: {"x":  pose.transform.translation.x, "y":  pose.transform.translation.y, "w": pose.transform.rotation.w, "z": pose.transform.rotation.z}})
                return "The position is stored successfully"
        #return position coordinates
        elif(getPos == True):
            #get the stored position
            if(namePos in self._tf_manage.return_position()):
                return self._tf_manage.return_position().get(namePos)
            else:
                return "The position is not stored"
