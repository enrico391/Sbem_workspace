

#langchain imports
import getpass
import os
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_ollama import ChatOllama
from langgraph.checkpoint.memory import MemorySaver
from langchain_community.tools.tavily_search import TavilySearchResults
from langchain_core.messages import HumanMessage
from langgraph.prebuilt import create_react_agent
from langchain_core.prompts import ChatPromptTemplate
from langchain.schema import AIMessage
from langchain_core.messages import SystemMessage

from datetime import date

#ROS imports
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_msgs.msg import Bool


#tools imports
import tools.sbem_tools as tools_sbem
from tools.navigatorTool import NavigatorCommander
from tools.autoDockTool import AutoDockCommander
from tools.moveTool import MoveCommander
from tools.savePositionTool import SavePosition
from tools.imageRecognitionTool import ImageRecognition
from tools.getPositionTool import GetPosition
from tools.modifyPositionTool import ModifyPosition
from tools.removePositionTool import RemovePosition

from positionManager import PositionsManager
from tools.imageSub import ImageSubscriber

#audio imports files
from audioProcess.tts_sbem import AudioPlayerNode
from audioProcess.wake_stt_sbem import ProcessAudio



if "GOOGLE_API_KEY" not in os.environ:
    os.environ["GOOGLE_API_KEY"] = "api_key"


class AgentClass(Node):

    def __init__(self, tf_manager: PositionsManager, image_manager: ImageSubscriber):
        super().__init__("agent_Sbem")
        
        # get the answer from the user
        self.inputUser = self.create_subscription(String,"/user_input",self.user_input_callback,10)
        
        # publish text response to app user and tts
        self.responseUser = self.create_publisher(String,"/response_to_user",10)

        # for app animation
        self.pub_startResponse = self.create_publisher(Bool, "/start_answer", 10)

        search = TavilySearchResults(max_results=2)
        
        # If we want, we can create other tools.
        navCommander = NavigatorCommander(tf_manager)
        savePositionTool = SavePosition(tf_manager)
        modifyPositionTool = ModifyPosition(tf_manager)
        getPositionTool = GetPosition(tf_manager)
        removePositionTool = RemovePosition(tf_manager)
        moveTool = MoveCommander()
        autoDockTool = AutoDockCommander(realEnvironment=True)
        imageRecognitionTool = ImageRecognition(image_manager)
        

        # Once we have all the tools we want, we can put them in a list that we will reference later.
        tools = [search, tools_sbem.getsqrt, moveTool, savePositionTool, getPositionTool, modifyPositionTool, removePositionTool, autoDockTool, navCommander, imageRecognitionTool ]

        memory = MemorySaver()

        print(tools)

        prompt = SystemMessage(content=(f"""
            You are SBEM, an advanced differential robot designed to assist users in their home environment.
            
            Your capabilities include:
            - Moving around the house autonomously using navigation tools
            - Saving and returning to specific locations when requested
            - Searchin3g the web for information to answer user questions
            - Visual recognition to identify objects and people through your camera
            - Self-maintenance like returning to your charging dock when needed
            
            Communication guidelines:
            - Respond with concise, helpful sentences
            - Adapt your language to match the user (respond in Italian if the user speaks Italian)
            - Always confirm when you've completed a task or if you need more information
            - Be friendly but efficient in your responses
                                        
            Your tools:
            -AutoDockCommander: use it to go to the docking station for battery charging.
            -NavigatorCommander: use it to navigate around the house, you need to provide the name of the position or the coordinates.
            -SavePosition: use it to save a position in the house, you need to provide the name of the position.
            -GetPosition: use it to get the coordinates of a position in the house, you need to provide the name of the stored position.
            -ModifyPosition: use it to modify a position in the house, you need to provide the name of the stored position.
            -RemovePosition: use it to remove a position in the house, you need to provide the name of the stored position.
            -MoveCommander: use it to move to a specific position
            -ImageRecognition: use it to recognize objects and people in the house
            -TavilySearchResults: use it to search the web for information
            -GetSqrt: use it to calculate the square root of a number
                                        
            
            Current date: {str(date.today())}.

            Your saved positions are: {tf_manager.return_name_position()}.
            
            Remember to use your tools appropriately for each request rather than simulating actions.
            WHEN YOU SEND THE LAST MESSAGE AFTER TOOL USAGE YOU HAVE TO PROVIDE A FEEDBACK OF THE COMPLETE ACTION.
            
        """))
        print(tf_manager.return_name_position())

        # model = ChatOllama(
        #     base_url="http://localhost:11434",
        #     model="qwen2.5:14b",
        #     temperature=0,    
        # )

        model = ChatGoogleGenerativeAI(
                    model="gemini-2.0-flash-lite-001",
                    temperature=0,    
                )
    
        #create agent
        self.agent_executor = create_react_agent(model, tools, prompt=prompt, checkpointer=memory)
        self.config = {"configurable": {"thread_id": "abc1333"}}


    def print_stream(self, stream):
        """Print the stream of messages from the agent"""
        for s in stream:
            message = s["messages"][-1]
            if isinstance(message, tuple):
                print(message)

            else:
                message.pretty_print()

                if(message.type == 'ai'):
                    resp = String()
                    resp.data = message.content.replace('*', '')
                    # publish the response to the user
                    self.responseUser.publish(resp)
                    
                    # start flag for animation on the app
                    #self.pub_startResponse.publish(Bool(data=True))
        

    def user_input_callback(self,msg):
        """Callback function to get the answer from the user"""
        answer = msg.data
        inputs = {"messages": [("user", answer)]}
        response = self.agent_executor.stream(inputs,config=self.config, stream_mode="values")

        self.print_stream(response)

    

def main(args=None):
    rclpy.init(args=args)

    # node to manage all positions 
    position_manager = PositionsManager()

     # image subscribe node
    img_node = ImageSubscriber()

    # object for the agent with langraph
    agent = AgentClass(position_manager, img_node)

    # audio nodes
    tts_node = AudioPlayerNode(useLocalTTS=False,typeLocalTTS="coqui")
    wake_stt_node = ProcessAudio()


    executor = MultiThreadedExecutor()
    executor.add_node(position_manager)
    executor.add_node(tts_node)
    executor.add_node(wake_stt_node)
    executor.add_node(img_node)
    executor.add_node(agent)

    executor.spin()
    executor.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
