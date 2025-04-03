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



    

