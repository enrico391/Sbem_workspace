from typing import Annotated

from typing_extensions import TypedDict

from langgraph.graph import StateGraph, START, END
from langgraph.graph.message import add_messages
from langchain_ollama import ChatOllama
from langchain_community.tools.tavily_search import TavilySearchResults
from langgraph.prebuilt import ToolNode, tools_condition
from langgraph.checkpoint.memory import MemorySaver
from langchain_core.messages import SystemMessage, RemoveMessage, HumanMessage
from typing import Literal
from langchain_google_genai import ChatGoogleGenerativeAI


from datetime import date


# tools imports
import tools.sbem_tools as tools_sbem
import tools.navigatorTool as navCommandTool
import tools.autoDockTool as autoDockTool
import tools.MoveTool as moveTool
import tools.savePositionTool as savePosTool



class State(TypedDict):
    # Messages have the type "list". The `add_messages` function
    # in the annotation defines how this state key should be updated
    # (in this case, it appends messages to the list, rather than overwriting them)
    summary: str
    messages: Annotated[list, add_messages]


class myAgent():
    def __init__(self):
        # tools 
        search = TavilySearchResults(max_results=2)

        tools = [search, tools_sbem.getsqrt,moveTool.moveCommander, savePosTool.savePosCommander , autoDockTool.DockCommander,
                        navCommandTool.NavCommander, tools_sbem.get_image ]
        
        
        


        prompt = ("You are SBEM a differential robot type with the ability to move in the house using your tools. " +
                    "You can save positions if required or navigate to the web to find useful informations for the user. " +
                    "You can see the word and use you tool to manage what you see. " +
                    "You can also dock to recharge your battery. You MUST required all the actions that you wants to do. " +
                    "You need to response with a simple sentence. You can talk in italian if the user wants to talk in italian." +
                    "You MUST send a message to the user before the usage of tools and one after the usage of tools." +
                    "Today is " + str(date.today()) + " . " +
                    "YOU MUST NEED TO REMEMBER THAT YOU ARE A ROBOT AND YOU CAN USE THE TOOLS PROVIDED TO YOU.")
             

        memory = MemorySaver()

        # create a state graph
        self.graph_builder = StateGraph(State)

        #self.model = ChatGoogleGenerativeAI(
        #            model="gemini-2.0-pro-exp-02-05",
        #            temperature=0,          
        #        )

        self.model = ChatOllama(
            base_url="http://localhost:11434",
            model="qwen2.5:14b",
            temperature=0,    
        )

        self.model.bind_tools(tools)

    
        # add summarize node to the graph
        #self.graph_builder.add_node(self.summarize_conversation)

        # add tools to the graph
        tool_node = ToolNode(tools=tools)
        self.graph_builder.add_node("tools",tool_node)

        # add chatbot node to the graph with the tools
        self.graph_builder.add_node("chatbot", lambda state: {"messages":self.model.bind_tools(tools).invoke(state['messages'])})

        # add condition edges to the graph
        self.graph_builder.add_conditional_edges("chatbot", tools_condition)
        #self.graph_builder.add_conditional_edges("chatbot", self.should_continue)

        # entry point
        self.graph_builder.add_edge(START, "chatbot")

        # add exit point
        self.graph_builder.add_edge("chatbot", END)

        # compile the graph
        self.graph = self.graph_builder.compile(checkpointer=memory)

        # define a new thread for memory
        self.config = {"configurable": {"thread_id": "1"}}

        # define what you are
        system_message = SystemMessage(content=prompt)
        self.graph.update_state(self.config, {"messages": [system_message]})


    # We now define the logic for determining whether to end or summarize the conversation
    def should_continue(self, state: State) -> Literal["summarize_conversation", END]:
        """Return the next node to execute."""
        messages = state["messages"]
        # If there are more than six messages, then we summarize the conversation
        if len(messages) > 6:
            return "summarize_conversation"
        # Otherwise we can just end
        return END


    def summarize_conversation(self, state: State):
        # First, we summarize the conversation
        summary = state.get("summary", "")
        if summary:
            # If a summary already exists, we use a different system prompt
            # to summarize it than if one didn't
            summary_message = (
                f"This is summary of the conversation to date: {summary}\n\n"
                "Extend the summary by taking into account the new messages above:"
            )
        else:
            summary_message = "Create a summary of the conversation above:"

        messages = state["messages"] + [HumanMessage(content=summary_message)]
        response = self.model.invoke(messages)
        # We now need to delete messages that we no longer want to show up
        # I will delete all but the last two messages, but you can change this
        delete_messages = [RemoveMessage(id=m.id) for m in state["messages"][:-2]]
        return {"summary": response.content, "messages": delete_messages}



    def chatbot(self, state : State):
        # If a summary exists, we add this in as a system message
        summary = state.get("summary", "")
        if summary:
            system_message = f"Summary of conversation earlier: {summary}"
            messages = [SystemMessage(content=system_message)] + state["messages"]
        else:
            messages = state["messages"]
        response = self.model.invoke(messages)
        # We return a list, because this will get added to the existing list
        return {"messages": [response]}


    def stream_graph_updates(self, user_input: str):
        events = self.graph.stream({"messages": [{"role": "user", "content": user_input}]},
                                self.config,
                                stream_mode="values")
        for event in events:
            print(event)
            event["messages"][-1].pretty_print()
            if "summary" in event:
                print(event["summary"])







# while True:
#     try:
#         my_custom_agent = myAgent()
#         user_input = input("User: ")
#         if user_input.lower() in ["quit", "exit", "q"]:
#             print("Goodbye!")
#             break

#         my_custom_agent.stream_graph_updates(user_input)
#     except:
#         # fallback if input() is not available
#         user_input = "What do you know about LangGraph?"
#         print("User: " + user_input)
#         my_custom_agent.stream_graph_updates(user_input)
#         break