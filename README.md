<h1> Sbem - a robotics friend </h1>

<p>
This is my project, it is a robot built with ROS2 and completely 3D printed.
I made this in order to learn robotics and ROS framework. I took inspiration from Really Useful robot by James Bruton.
I used his base for the wheels but I used a different types of motors and electronics. For the ROS2 framework I took inspiration from 
articuled robotics youtube channel and it gave me a lot of help during my journey to learn ROS2.

	
These are the main components :    
	- Two motors from an hoverboard    
 	- Odrive 3.6 for the motors control    
  	- Raspberry pi 4 for the comunication with server pc with main features of ROS2    
    	- ESP32 with MicroROS for the control of Odrive with UART communication    
    	- Pi camera for docking system and AprilTag identification    
    	- Lidar A1 for navigation and Nav2 usage  
    	- LCD Screen for GUI interface and simple debugging of raspberry    
    	- A microphone for interaction human-machine (TODO array-mic for space detection)  
     	- (TODO) IntelRealsense d435i for 3D perception  

</p>

<hr>

<h1> Simple navigation guide </h1>
<p>This is the procedure to start all packages to perform simple navigation with nav2 stack in real world or in a simulated one</p>
<p>In the folder of project sbem_project_ws : </p>

<code>
	
	source install/setup.bash
 
	ros2 launch robot_sbem sbem.launch.py  //launch robot description
 
	ros2 run robot_sbem pub_odom_sbem.py  // publish odom wheel
 
	ros2 launch robot_sbem footprint_filter_laser.launch.py  // for filtering the shape of robot in the laserscan 
 
	ros2 launch robot_sbem joystick.launch.py   // for joystick
 
	ros2 launch robot_sbem robot_localization.launch.py   //for fusing wheel odom and imu data (TODO)
 
</code>
<p>After starting this initials nodes required, there is two options :</p>
<li>
	Create a new map
</li>
<li>
	Use a custom map already created
</li>

<h3>Launch this two from terminal to use SBEM with mapping and navigation </h3>
<code>
	
	ros2 launch robot_sbem online_async_launch.py params_file:=./config/mapper_params_online_async.yaml use_sim_time:=false

	ros2 launch robot_sbem navigation_launch.py
</code>

<h3>Launch this for using amcl localitation (must set the initial pose in rviz2) and navigation with param map_subscribe_transient_local for not update map </h3>
<code>

	ros2 launch robot_sbem localization_launch.py   //substitute the map that you want (ex: for gazebo ros2 launch robot_sbem localization_launch.py map:=src/robot_sbem/maps/map_gazebo_april.yaml)

	ros2 launch robot_sbem navigation_launch.py map_subscribe_transient_local:=true   // not update the map with navigation
</code>

<hr>

<h2>Send docking message for autodocking</h2>
<code>

	ros2 launch sbem_docking docking_sbem.launch.py 

	ros2 run apriltag_ros apriltag_node -ros-args -r image_rect:=/image_raw     -r camera_info:=/camera_info     --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml   //start node that publish apriltag transform
	
	// for gazebo : ros2 run apriltag_ros apriltag_node -ros-args -r image_rect:=/camera/image_raw     -r camera_info:=/camera/camera_info     --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
</code>

<h3>Message to publish in terminal to perform autodock</h3>
<code>

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
</code>

<h3>Message for undocking</h3>
<code>

	ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{dock_type: 'nova_carter_dock'}"
</code>

<hr>

<h2>Sbem AI</h2>
<p>There is a folder name sbem_AI with all scripts for interact with SBEM LLM and use voice to control the robot. I use langchain and langraph to create an agent that use tools in order to control robot position </p>

<h3>Procedure </h3>

<list>
	Create a server with Piper or coquiTTS
</list>
<list>
	Start agent_sbem.py
</list>







<hr>

<h2>Procedure to start SBEM raspberry nodes</h2>
<p>In order to use real sbem robot start this nodes in raspberry</p>
<code>

	cd Desktop/sbem_ws/

	source install/setup.bash 

	ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0  //to start communication with esp32 with microros

	ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard -p serial_baudrate:=115200
	// alternative : ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard

	ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_link_optical  //for publish camera view

</code>



