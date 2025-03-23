

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



#ROS imports
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult





#audio imports files
import audioProcess.tts_sbem as tts_sbem
import audioProcess.wake_stt_sbem as wake_stt_sbem

# import agent class
from custom_agent import myAgent

# import TFpublisher 
from positionManager import PositionsManager


if "GOOGLE_API_KEY" not in os.environ:
    os.environ["GOOGLE_API_KEY"] = "api_key"




class AgentClass(Node):

    def __init__(self, positions_manager):
        super().__init__("agent_executor")
        
        #create a service for interact with other module
        self.inputUser = self.create_subscription(String,"/user_input",self.user_input_callback,10)

        self.responseUser = self.create_publisher(String,"/response_to_user",10)

       

        #model = ChatGoogleGenerativeAI(
        #            model="gemini-2.0-pro-exp-02-05",
        #            temperature=0,          
        #        )
    

        #create agent
        self.agent = myAgent(positions_manager)

    def print_stream(self, stream):
        for s in stream:
            message = s["messages"][-1]
            if isinstance(message, tuple):
                print(message)

            else:
                message.pretty_print()

                if(message.type == 'ai'):
                    resp = String()
                    resp.data = message.content
                    self.responseUser.publish(resp)
        
    def user_input_callback(self, msg):
        answer = msg.data
        # send question to agent
        self.agent.stream_graph_updates(answer)

        #self.print_stream(response)

    

def main(args=None):
    rclpy.init(args=args)

    
    #initiate the class for publishing the current pose
    tfpub = PositionsManager()

    #initiate the class for the agent with langchain
    agent = AgentClass(tfpub)


    #initiate the audio nodes
    tts_node = tts_sbem.AudioPlayerNode(useLocalTTS=False)
    wake_stt_node = wake_stt_sbem.ProcessAudio()

    executor = MultiThreadedExecutor()

    
    executor.add_node(tfpub)
    executor.add_node(tts_node)
    executor.add_node(wake_stt_node)
    executor.add_node(agent)


    executor.spin()
    
    
    executor.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()


# while True:
#     answer = input("Question : ")
#     for chunk in agent_executor.stream(
#         {"messages": [HumanMessage(content=answer)]}, config
#     ):
#         print(chunk)
#         print("----")

