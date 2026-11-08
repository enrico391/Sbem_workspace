# sbem_description

URDF/Xacro description and STL meshes for the Sbem robot, including its
chassis, wheels, sensors and ros2_control interfaces.

## Publish the robot description

From the built workspace:

```sh
source install/setup.bash
ros2 launch sbem_description sbem.launch.py
```

This starts `robot_state_publisher`. Movable joints need `/joint_states`
from the robot controllers or a joint state publisher.

Launch arguments:

- `use_sim_time` (default: `false`): use the simulation clock and Gazebo control configuration.
- `use_ros2_control` (default: `true`): include the ros2_control interfaces in the model.

## Main files

- `description/robot_sbem.urdf.xacro`: main robot description.
- `description/robot_core.xacro`: robot links and joints.
- `description/ros2_control.xacro`: hardware and simulation control configuration.
- `meshes/`: STL models for the robot parts.

See [robot_sbem](../robot_sbem/README.md) for the full robot bringup.
