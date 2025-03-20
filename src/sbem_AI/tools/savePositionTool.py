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


from TFpublisher import current_pose

positions = dict()

#Tool for nav to pose
class SavePosInput(BaseModel):
    namePos: str = Field(description="the appropriate name for the stored position")
    getPos: bool = Field(description="False if you want to store the current position, True if you want to get the stored position with a specific name")
    
    
class SavePosition(BaseTool):
    name: str = "Save_position"
    description: str = "Useful to store the current position. If the user wants to obtain coordinates you can use the appropriate name and get the coordinates"
    args_schema: Type[BaseModel] = SavePosInput
    #return_direct: bool = True
    

    def __init__(self):
        super().__init__()
        

    def _run(self, getPos: bool, namePos: str, run_manager: Optional[CallbackManagerForToolRun] = None)-> str:
        #store current position in the list
        if(getPos == False):
            
            print(current_pose)

            print(positions)
            if(namePos == ""):
                return "You must insert a name for the position"
            else:
                positions.update({namePos: {"x":  current_pose.transform.translation.x, "y":  current_pose.transform.translation.y, "w": current_pose.transform.rotation.w, "z": current_pose.transform.rotation.z}})
                return "The position is stored successfully"
        
        elif(getPos == True):
            #get the stored position
            if(namePos in positions):
                return positions[namePos]
            else:
                return "The position is not stored"



savePosCommander = SavePosition()