# robot_sbem

Main ROS 2 configuration and launch files for the Sbem robot, including
hardware bringup, localization, Nav2 navigation and docking.

## Start the robot

Inside the robot's Isaac ROS container, from the built workspace:

```sh
source install/setup.bash
ros2 launch robot_sbem sbem_bringup.launch.py
```

This starts the peripherals, navigation and docking, waypoint manager,
RealSense camera, AprilTag detection and rosbridge on port `9090`.
Automatic low-battery docking is enabled by default; disable it with
`enable_autonomy:=false`.

## Main files

- `launch/`: full bringup and individual subsystem launch files.
- `config/`: navigation, localization, joystick and laser filter parameters.
- `maps/`: saved maps for the robot and simulation.

See the [workspace README](../../README.md) for hardware and setup details.
