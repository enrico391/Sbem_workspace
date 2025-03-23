#imports for agent
from langchain_core.tools import tool





#imports for utilities
from math import sqrt


from PIL import Image
from io import BytesIO
import base64
from tools.imageSub import ImageSubscriber
import google.generativeai as genai
from PIL import Image





@tool
def getsqrt(a: int) ->int:
    """Get the square root of a number in the input"""
    return sqrt(a)




# @tool
# def get_image() -> dict:
#     """Return content in base64 format"""

#     return [
#         {"type": "text", "text": "The image is attached below in base64 format"},
#         {"type": "image_url" , "image_url": {"url": f"data:image/jpeg;base64,{imageHandler.getImageBase64()}"}},
#     ]






    

