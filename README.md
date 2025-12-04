# Sbem - A Robotic Friend

This is my project, a robot built with ROS2 and completely 3D-printed. I created it to learn robotics and the ROS framework. I took inspiration from the **Really Useful Robot** by James Bruton, using his wheelbase but with modified motors and electronics. For ROS2, I followed the **Articulated Robotics** YouTube channel, which greatly helped me during my learning journey. The chassis is completely designed by me and 3d printed.

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
- **Jetson Orin nano** for communication with the server PC and ROS2 framework
- **ESP32 with MicroROS** for controlling Odrive via UART
- **Pi Camera** for docking system and AprilTag identification
- **Lidar A1** for navigation and Nav2 usage
- **LCD Screen** for GUI interface and simple debugging on Raspberry Pi
- **Microphone** for human-machine interaction *(TODO: Implement an array mic for spatial detection)*
- **(TODO) Intel RealSense D435i** for 3D perception

---

# Simple Navigation Guide
This section explains how to start all required packages for simple navigation using the **Nav2** stack in the real world or simulation.

### Steps to Start Navigation with Isaac Sim:
1. Navigate to the project workspace:

   ```sh
   cd sbem_project_ws
   source install/setup.bash
   ```

2. Start nodes and isaac sim:
  - Start isaac sim

  - Start rviz, localization and navigation nodes:
    ```sh
    ros2 launch robot_sbem sbem_isaac_sim.launch.py # start rviz, joystick, teleop
    ros2 launch robot_sbem localization.launch.py  # start localization and nav2
    ros2 launch robot_sbem navigation_docking.launch.py use_sim_time:=true # start navigation and docking system
    ```


### Start Apriltag detection:
```sh
ros2 run apriltag_ros apriltag_node -ros-args -r image_rect:=/image -r camera_info:=/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
```

### Command to send message to docking server: *(change coordinates)*
```sh
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "
{
  use_dock_id: false,
  dock_pose: {
    pose: {
      position: {x: 5.95, y: -1.85, z: 0.0},
      orientation: {x: -0.0, y: -0.0, z: 0.963, w: 0.268}
    },
    header: {
      frame_id: 'map'
    }
  },
  dock_type: 'nova_carter_dock',
  navigate_to_staging_pose: true
}"
```

### Command for Undocking: 
```sh
ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{dock_type: 'nova_carter_dock'}"
```


