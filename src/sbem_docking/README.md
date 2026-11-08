# sbem_docking

ROS 2 launch files and configuration for Sbem's Nav2 docking server and
RealSense AprilTag detection. The `charging_dock` uses the
`opennav_docking::SimpleChargingDock` plugin, registered as `nova_carter_dock`.

## Start docking

The full [robot bringup](../robot_sbem/README.md) already starts the docking
server, camera, AprilTag detection and dock pose publisher.
To start only the docking server and its lifecycle manager, from the built workspace:

```sh
source install/setup.bash
ros2 launch sbem_docking docking_sbem.launch.py
```

Run alongside localization, Nav2, AprilTag detection and the dock pose publisher
from `robot_sbem`. The configured dock also uses battery status to detect charging.

## Dock and undock

```sh
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{use_dock_id: true, dock_id: 'charging_dock', navigate_to_staging_pose: true}"

ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{dock_type: 'nova_carter_dock'}"
```

## Configuration

- `params/sbem_docking_params.yaml`: dock location, AprilTag offsets and controller settings.
- `launch/apriltag_detection_realsense.launch.py`: AprilTag detection; by default, loads into the existing nvblox component container.
- `launch/docking_sbem_sim.launch.py` and `params/docking_simulation.yaml`: simulation setup.

Set the dock pose and detection offsets for your environment. To load another
configuration, add `params_file_dock:=/path/to/docking.yaml` to the server launch.

See the [workspace README](../../README.md) for the full startup guide.
