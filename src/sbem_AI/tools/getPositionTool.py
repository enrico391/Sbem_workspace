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
class GetPosInput(BaseModel):
    namePos: str = Field(description="the appropriate name for the stored position")
    
    
class GetPosition(BaseTool):
    name: str = "Get_position"
    description: str = "Useful to get coordinates of a stored position. You must insert the name of the position you want to get"
    args_schema: Type[BaseModel] = GetPosInput
    #return_direct: bool = True
    _tf_manage: PositionsManager = PrivateAttr()
    

    def __init__(self, tf_manage: PositionsManager):
        super().__init__()
        self._tf_manage = tf_manage
        

    def _run(self, namePos: str, run_manager: Optional[CallbackManagerForToolRun] = None)-> str:
        if(namePos == ""):
            return "You must insert a name for the position"
        else:
            if(namePos in self._tf_manage.return_position()):
                return self._tf_manage.return_position().get(namePos)
            else:
                return "The position is not stored"
