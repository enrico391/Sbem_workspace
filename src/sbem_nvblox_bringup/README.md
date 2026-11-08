# sbem_nvblox_bringup

ROS 2 launch files and configuration for Sbem's RealSense camera and nvblox
3D mapping, used for local obstacle avoidance with Nav2.

## Start nvblox

Inside the robot's Isaac ROS container, from the built workspace:

```sh
source install/setup.bash
ros2 launch sbem_nvblox_bringup sbem_realsense_nvblox.launch.py
```

This starts one RealSense camera and nvblox in static mapping mode.
Run alongside the robot's TF publishers so camera observations can be
transformed into the mapping frame.

If the full [robot bringup](../robot_sbem/README.md) is already running,
reuse its camera and component container:

```sh
ros2 launch sbem_nvblox_bringup sbem_realsense_nvblox.launch.py run_realsense:=False attach_to_container:=True
```

## Main files

- `launch/sbem_realsense_nvblox.launch.py`: camera and mapping bringup.
- `launch/sensors/realsense.launch.py`: RealSense driver setup.
- `launch/perception/nvblox.launch.py`: nvblox component setup.
- `config/nvblox/`: mapping parameters and camera specializations.
- `config/sensors/`: camera settings.

The RealSense configuration uses a `3.0 m` map clearing radius and a
`2.5 m` maximum depth integration distance. Adjust these in
`config/nvblox/specializations/nvblox_realsense.yaml`.

See the [workspace README](../../README.md) for the full startup guide.
