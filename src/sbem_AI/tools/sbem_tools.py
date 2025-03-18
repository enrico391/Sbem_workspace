#imports for agent
from langchain_core.tools import tool





#imports for utilities
from math import sqrt


from PIL import Image
from io import BytesIO
import base64
from tools.imageTool import ImageSubscriber





@tool
def getsqrt(a: int) ->int:
    """Get the square root of a number in the input"""
    return sqrt(a)




@tool
def get_image() -> dict:
    """Load an image and return it as dict """

    imageHandler = ImageSubscriber()

    imageHandler.getImage()

    with open('src/sbem_AI/ggg.jpg', 'rb') as image_file:
        image_data = base64.b64encode(image_file.read()).decode("utf-8")


    #image_url = "src/sbem_AI/tarta.jpg"

    return [
        {"type": "text", "text": "The image is attached below in base64 format"},
        {"type": "image_url" , "image_url": {"url": f"data:image/jpeg;base64,{image_data}"}},
    ]






    

