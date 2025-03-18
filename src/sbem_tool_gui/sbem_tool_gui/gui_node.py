#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time


from tkinter import *
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from example_interfaces.srv import SetBool

from sbem_tool_gui.sbem_tool_gui.create_gui import SBEMGraphicTool


class NodeGui(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info("Loading...")

        #subscribtions
        self.lwheel_sub = self.create_subscription(
                Float32,
                "lwheel",
                self.lwheel_callback,
                10)
        self.lwheel_sub  # prevent unused variable warning

        self.rwheel_sub = self.create_subscription(
                Float32,
                "rwheel",
                self.rwheel_callback,
                10)
        self.rwheel_sub  # prevent unused variable warning


        self.check_service = True

        #service clients
        self.motor_start = self.create_client(Empty, 'start_motor')
        if not self.motor_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: start_motor service not available')
            self.check_service = False
        else:
            self.get_logger().info('start_motor service available')


        self.motor_stop = self.create_client(Empty, 'stop_motor')
        if not self.motor_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: stop_motor service not available')
            self.check_service = False
        else:
            self.get_logger().info('stop_motor service available')
        
        self.motor_req = Empty.Request()


        self.restart_esp = self.create_client(SetBool, '/restartESP')
        if not self.restart_esp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: restartESP service not available')
            self.check_service = False
        else:
            self.get_logger().info('restartESP service available')
        
        
    
        self.restart_esp_req = SetBool.Request()
        self.restart_esp_req.data = True

        


        #create a list of service calls
        self.client_futures = []


        #build graphic page

        self.ttk = Tk()
        self.ttk.title("SBEM UI")
        self.ttk.geometry("1024x600+0+0")


        self.createPage()
    

    def lwheel_callback(self,value_left):
        
        value_l = value_left.data
        if(value_l != -0.0):
            self.page.value_display_label_left.config(text=value_l)

    def rwheel_callback(self,value_right):
        
        value_r = value_right.data
        if(value_r != 0.0):
            self.page.value_display_label_right.config(text=value_r)

    def send_start_motor_req(self):
        if self.motor_start.service_is_ready():
            print("Starting lidar...")
            self.client_futures.append(self.motor_start.call_async(self.motor_req))
        else:
            print("start_motor not ready!")

    def send_stop_motor_req(self):
        if self.motor_stop.service_is_ready():
            print("Stopping lidar...")
            self.client_futures.append(self.motor_stop.call_async(self.motor_req))
        else:
            print("stop_motor not ready!")

    
    def send_restartESP(self):
        if self.restart_esp.service_is_ready():
        #self.requestESP = self.restart_esp.call_async(self.restart_esp_req)
            print("Restarting ESP...")
            self.client_futures.append(self.restart_esp.call_async(self.restart_esp_req))
        else:
            print("restartESP not ready!")


    def createPage(self):

        if(self.check_service):
            self.page = SBEMGraphicTool(self.ttk)

            self.page.start_lidar_btn.config(command=self.send_start_motor_req)
            self.page.restart_esp_btn.config(command=self.send_restartESP)
            self.page.stop_lidar_btn.config(command=self.send_stop_motor_req)
        else:
            self.get_logger().info('WARNING: services not availables')
        


    def update_image(self):
        self.page.update_image()


    def check_for_finished_calls(self):
        incomplete_futures = []
        for f in self.client_futures:
            if f.done():
                res = f.result()
            else:
                incomplete_futures.append(f)


def main(args=None):
    rclpy.init(args=args)
    
    app = NodeGui()
    
    while rclpy.ok():
      rclpy.spin_once(app)
      app.check_for_finished_calls()
      app.update_image()
      time.sleep(0.01)
    
    #app.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()

    