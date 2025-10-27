# Sbem - A Robotic Friend

This is my project, a robot built with ROS2 and completely 3D-printed. I created it to learn robotics and the ROS framework. I took inspiration from the **Really Useful Robot** by James Bruton, using his wheelbase but with modified motors and electronics. For ROS2, I followed the **Articulated Robotics** YouTube channel, which greatly helped me during my learning journey. I'm using also the ros2 package **audio_common** provided by **Miguel Ángel González Santamarta**. The chassis is completely designed by me and 3d printed.

The idea behind this robot is to use it for helping people in houses and use it as a friend with the power of LLM 

<div style="overflow-x: auto; white-space: nowrap;">
  <img src="img_readme/img1.jpg" alt="Image 1" style="width:400; display:inline-block;"/>
  <img src="img_readme/img2.jpg" alt="Image 2" style="width:400px; display:inline-block;"/>
  <img src="img_readme/img3.jpg" alt="Image 3" style="width:400px; display:inline-block;"/>
  <img src="img_readme/img4.jpg" alt="Image 4" style="width:400px; display:inline-block;"/>
</div>

## Main Components
- **Two motors** from a hoverboard
- **Odrive 3.6** for motor control
- **Raspberry Pi 4** for communication with the server PC and ROS2 framework
- **ESP32 with MicroROS** for controlling Odrive via UART
- **Pi Camera** for docking system and AprilTag identification
- **Lidar A1** for navigation and Nav2 usage
- **LCD Screen** for GUI interface and simple debugging on Raspberry Pi
- **Microphone** for human-machine interaction *(TODO: Implement an array mic for spatial detection)*
- **(TODO) Intel RealSense D435i** for 3D perception

---

# Starting SBEM Nodes on Jetson
To run the real SBEM robot, start the following nodes on the Raspberry Pi:
```sh
cd /sbem_ws/
source install/setup.bash
ros2 launch robot_sbem sbem_total_rpy.launch.py
```


# LLM System
```
### Scripts for microphone and play sound in raspberry
In order to enable microphone and speakers of sbem is required to start 2 scripts in raspberry environment:
```sh
ros2 run audio_common audio_capturer_node
python3 tts_sbem.py
```

### SBEM App
There is simple gui interface that can be used in order to chat with sbem llm system. The app is written in javascript and it use electron framework, 
to provide communication between ROS2 and javascript world I use rosbridge and roslib library. Start rosbridge server:
```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```



