

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


#tools imports
import tools.sbem_tools as tools_sbem
import tools.navigatorTool as navCommandTool
import tools.autoDockTool as autoDockTool
import tools.MoveTool as moveTool
import tools.savePositionTool as savePosTool
import TFpublisher


#audio imports files
import audioProcess.tts_sbem as tts_sbem
import audioProcess.wake_stt_sbem as wake_stt_sbem



if "GOOGLE_API_KEY" not in os.environ:
    os.environ["GOOGLE_API_KEY"] = "api_key"




class AgentClass(Node):

    def __init__(self):
        super().__init__("agent_executor")
        
        #create a service for interact with other module
        self.inputUser = self.create_subscription(String,"/user_input",self.user_input_callback,10)

        self.responseUser = self.create_publisher(String,"/response_to_user",10)

        search = TavilySearchResults(max_results=2)
        #search_results = search.invoke("what is the weather in SF")
        #print(search_results)
        # If we want, we can create other tools.
        # Once we have all the tools we want, we can put them in a list that we will reference later.
        tools = [search, tools_sbem.getsqrt,moveTool.moveCommander, savePosTool.savePosCommander , autoDockTool.DockCommander,
                  navCommandTool.NavCommander, tools_sbem.get_image ]

        memory = MemorySaver()

        prompt = ("You are SBEM a differential robot type with the ability to move in the house using your tools. " +
             "You can save positions if required or navigate to the web to find useful informations for the user. " +
             "You can see the word and use you tool to manage what you see. " +
             "You can also dock to recharge your battery. You MUST required all the actions that you wants to do. " +
             "You need to response with a simple sentence. You can talk in italian if the user wants to talk in italian." +
             "You MUST send a message to the user before the usage of tools and one after the usage of tools.")

        model = ChatOllama(
            base_url="http://localhost:11434",
            model="qwen2.5:14b",
            temperature=0,    
        )

        #model = ChatGoogleGenerativeAI(
        #            model="gemini-2.0-pro-exp-02-05",
        #            temperature=0,          
        #        )
    

        #create agent
        self.agent_executor = create_react_agent(model, tools, prompt=prompt, checkpointer=memory)
        self.config = {"configurable": {"thread_id": "abc1333"}}

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
        
    def user_input_callback(self,msg):
        answer = msg.data
        inputs = {"messages": [("user", answer)]}
        response = self.agent_executor.stream(inputs,config=self.config, stream_mode="values")

        self.print_stream(response)

    

def main(args=None):
    rclpy.init(args=args)

    
    #initiate the class for publishing the current pose
    tfpub = TFpublisher.TfEcho()

    #initiate the class for the agent with langchain
    agent = AgentClass()


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

