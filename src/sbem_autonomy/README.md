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

## Lidar motor supervision

`lidar_motor_node` is a **separate ROS node**, optionally launched with
`autonomy.launch.py enable_lidar_motor:=true`. It subscribes directly to
`/battery_state`; disabling the charging
BT through `set_enabled` does not disable this node. It can also run alone,
including when automatic docking supervision is not launched:

```bash
ros2 launch sbem_autonomy lidar_motor.launch.py
```

Run only one motor supervisor. Set `enable_lidar_motor:=false` on
`autonomy.launch.py` if already launching it separately. A custom motor YAML can
be supplied with `lidar_params_file:=/path/to/lidar_motor.yaml` to either launch.

The policy uses battery status, independently of battery percentage:

- Stable `CHARGING` or `FULL`: request motor stopped.
- Stable `DISCHARGING`: request motor started, including after manual removal.
- `UNKNOWN`, `NOT_CHARGING`, unsupported status, no readings, or stale readings:
  request motor started conservatively.

Defaults in `config/lidar_motor.yaml` are a 1 second check period, 2 second
debounce, and 5 second battery timeout. Timeouts use steady time, so they still
work when simulation time pauses. Charging/full share the same debounce state.
Unknown and stale data bypass debounce. After a data gap, charging must pass
debounce again before stopping the motor.

The node calls `/start_motor` and `/stop_motor` (`std_srvs/srv/Empty`)
asynchronously, on desired-state changes, starting with a desired motor-on state.
Unavailable or timed-out services are retried every 3 seconds; request timeout
is 3 seconds. Requests are tracked one at a time; pending calls are cleaned up
on timeout, and retries use the latest battery state. These names and all timing
values are configurable. Ensure the RPLIDAR driver's `auto_standby` mode is
disabled, since that mode ignores explicit motor service calls.

At startup the node observes `/scan` for 2 seconds before issuing commands. Fresh
scans confirm an already running lidar and suppress a redundant start request:
the RPLIDAR start service reconfigures scanning even when the motor is running.
`scan_topic`, `scan_timeout` (2 seconds), and `startup_observation_duration`
(2 seconds) are configurable. A start is still sent after a stop, even if recent
scan messages remain in the pipeline. Timeouts log the service name and requested
state. A client timeout cannot interrupt a driver callback blocked in serial I/O;
restart the lidar driver if parameter services also stop responding and scans
cease. Increasing the client timeout cannot repair a blocked serial wait.

Planner errors repeatedly referencing one old `base_link` timestamp indicate a
pose outside TF history. Submit navigation goals in `map` (set RViz Fixed Frame
to `map`) and cancel/resubmit an old goal after scans and localization recover.
Stopping scans also pauses scan-based localization updates; this supervisor does
not republish localization transforms or rewrite navigation goal timestamps.

An Empty response confirms receipt, not physical motor operation or fresh scans.
Manual removal restarts the motor after battery detection, debounce, the next
check, and spin-up. This node does not prevent motion or orchestrate undocking;
automatic undocking must still start the lidar and wait for fresh scans before
moving. Confirm current polarity and motor service behavior on the robot.
