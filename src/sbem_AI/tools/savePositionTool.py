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
    
    
class SavePosition(BaseTool):
    name: str = "Save_position"
    description: str = "Useful to store the current position"
    args_schema: Type[BaseModel] = SavePosInput
    #return_direct: bool = True
    _tf_manage: PositionsManager = PrivateAttr()
    

    def __init__(self, tf_manage: PositionsManager):
        super().__init__()
        self._tf_manage = tf_manage
        

    def _run(self, namePos: str, run_manager: Optional[CallbackManagerForToolRun] = None)-> str:
        #store current position in the list
        if(namePos == ""):
            return "You must insert a name for the position that you want to store"
        else:
            #get current pose from tf manager
            pose = self._tf_manage.return_current_pose()
            # add position
            self._tf_manage.update_position(namePos, pose)
            return "The position is stored successfully"
