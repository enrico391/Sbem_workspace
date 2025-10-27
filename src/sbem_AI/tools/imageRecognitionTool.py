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
from langchain_core.messages import HumanMessage

from tools.imageSub import ImageSubscriber
import base64
import ollama

class ImageRecognitionInput(BaseModel):
    #typeAction: bool = Field(description="True if you want the description of the view")
    pass

class ImageRecognition(BaseTool):
    name: str = "ImageRecognition_commander"
    description: str = "Useful to view the world with your eyes, get the description of your visual"
    args_schema: Type[BaseModel] = ImageRecognitionInput

    #return_direct: bool = True
    _img_handler: ImageSubscriber = PrivateAttr()
    
    def __init__(self, imageCommander: ImageSubscriber):
        super().__init__()
        self._img_handler = imageCommander
    
    def _run(self, run_manager: Optional[CallbackManagerForToolRun] = None)-> str:
        """Use to get description of the current view of your eyes"""
        try:
            # get description from ollama local server
            response = ollama.chat(model="gemma3:12b",
                messages=[
                    {
                        "role": "user",
                        "content": "descrivi l'immagine in modo generale e elenca gli oggetti che vedi",
                        "images": [self._img_handler.getImageBytes()],
                    }
                ],
                stream=True,
            )

            full_response = "".join(part["message"]["content"] for part in response)
            
            return full_response
        
        except :
            raise ValueError("Error getting image ")
             

                

        



