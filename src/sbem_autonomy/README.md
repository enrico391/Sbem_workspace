# SBEM automatic charging

`autonomy_node` runs an application BehaviorTree.CPP v3 tree continuously,
including while Nav2 has no active goal. It loads the official Humble
`opennav_docking_bt` docking plugin;

Below 20% for 5 seconds, it latches a recharge request, cancels both Nav2
navigation actions, waits for canceled goals to reach a terminal status, and
requests docking at `charging_dock`. The docking server handles staging navigation
and final approach. The supervisor waits for fresh battery readings showing
charging (or full) and at least 80%, then stays docked. It does not disable the
physical charger at 80%, undock, or resume the interrupted mission.

## Build and launch

Run in the ROS Humble development container:

```bash
sudo apt-get update
sudo apt-get install ros-humble-opennav-docking-bt
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select battery_state_broadcaster sbem_autonomy robot_sbem
source install/setup.bash
```

Start the complete real-robot navigation/docking bringup with supervision:

```bash
ros2 launch robot_sbem sbem_loc_nav_dock.launch.py enable_autonomy:=true
```

Or, with Nav2 and the docking server already running, start just supervision:

```bash
ros2 launch sbem_autonomy autonomy.launch.py
```

## Operation

```bash
ros2 topic echo /sbem_autonomy/state
ros2 topic echo /sbem_autonomy/charging_required
ros2 service call /sbem_autonomy/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /sbem_autonomy/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Disabling halts the BT and requests cancellation of an active docking action.

States are `waiting_for_battery`, `idle`, `recharging`, `charged`, `retry_wait`,
`failed`, and `disabled`. Invalid, out-of-range, or stale battery data halts the
charging tree; it never clears the recharge latch. A charging interruption longer
than `charging_loss_timeout`, an overall charging timeout, navigation cancellation
failure, or docking failure consumes an attempt. After `max_attempts`, the node
stays in `failed` until explicitly re-enabled. Each docking request additionally
uses the docking server's configured internal retries.

Navigation and docking must be active, with consistent velocity routing, and
the dock pose, detection, current threshold, and charging calibration must be
validated before enabling motion. `charging_threshold: 1.5` in the docking
configuration is a current threshold in amperes, not the battery percentage.