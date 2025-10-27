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
class ModifyPosInput(BaseModel):
    namePos: str = Field(description="the appropriate name for the stored position that you want to modify")
    newNamePos: str = Field(description="the appropriate name for the new position that you want to assign to the stored position")

class ModifyPosition(BaseTool):
    name: str = "Modify_position"
    description: str = "Useful to modify a position"
    args_schema: Type[BaseModel] = ModifyPosInput
    #return_direct: bool = True
    _tf_manage: PositionsManager = PrivateAttr()
    

    def __init__(self, tf_manage: PositionsManager):
        super().__init__()
        self._tf_manage = tf_manage
        

    def _run(self, namePos: str, newNamePos: str, run_manager: Optional[CallbackManagerForToolRun] = None)-> str:
        #store current position in the list
        if(namePos == ""):
            return "You must insert a name for the position that you want to modify"
        else:
            # update position with the new pose
            return self._tf_manage.modify_position(namePos, newNamePos)
            
