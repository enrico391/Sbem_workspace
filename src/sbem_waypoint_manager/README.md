# SBEM waypoint manager

Save the current robot pose under a name and return to it later using Nav2.
Locations and their static map snapshots persist in a SQLite database.

## Build and run

In the robot's ROS 2 Humble environment, with Nav2 installed:

```bash
cd /workspaces/isaac_ros-dev/src/sbem_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sbem_waypoint_interfaces sbem_waypoint_manager
source install/setup.bash
ros2 launch sbem_waypoint_manager waypoint_manager.launch.py
```

Run alongside your existing localization/Nav2 bringup. For simulation, add
`use_sim_time:=true`. The node needs Nav2's static `/map` and a recent
`map -> base_link` TF. It does not start navigation or localization servers.

## Services

```bash
# Capture the robot's current map pose, including its orientation.
ros2 service call /waypoint_manager/save_location sbem_waypoint_interfaces/srv/SaveLocation "{name: kitchen, overwrite: false}"

# Retrieve a freshly stamped pose suitable for a Nav2 goal.
ros2 service call /waypoint_manager/get_location sbem_waypoint_interfaces/srv/GetLocation "{name: kitchen}"

ros2 service call /waypoint_manager/list_locations sbem_waypoint_interfaces/srv/ListLocations '{}'

# Dispatch a Nav2 NavigateToPose goal for the saved location.
ros2 service call /waypoint_manager/goto_location sbem_waypoint_interfaces/srv/GotoLocation "{name: kitchen}"

ros2 service call /waypoint_manager/get_map sbem_waypoint_interfaces/srv/GetMap '{}'

ros2 service call /waypoint_manager/delete_location sbem_waypoint_interfaces/srv/DeleteLocation "{name: kitchen}"
```

An empty `map_id` selects the currently received map. Supply a saved map ID to
retrieve/list/delete locations or retrieve a map from a different session.
`save_location` always uses the current map. Duplicate names require
`overwrite: true`; different maps can each have a `kitchen`.

Every response includes `success` and `message`. Save/get/list responses include
location map IDs, capture times and `map_matches_current`. Returned poses have
current header timestamps; `captured_at` preserves the original TF timestamp.

`goto_location` success means **Nav2 accepted the goal**, not that the robot
arrived. Its `goal_id` is Nav2's UUID as 32 hex characters. Results are logged;
use the Nav2 action's status/result/cancel interfaces for ongoing navigation.
The manager rejects requests while its previous goal is pending or active.
External clients must coordinate navigation ownership: Nav2 may preempt goals
when another client sends a new one.

If acceptance times out, the service reports acceptance as unknown. The manager
keeps tracking the request and cancels a late-accepted goal. If the current map
changes, it requests cancellation of its pending/active goal. Cancellation can
be rejected by Nav2; the node logs failures and retains ownership until a
terminal result. Missing replies or lost result tracking keep the node busy
rather than silently sending another goal. Restarting loses action tracking;
check Nav2's active goals before restarting. Shutdown cancellation is best effort.

## Maps and database

Default file: `~/.ros/sbem_waypoints.sqlite3`. Python's standard SQLite library
handles storage; no database server is needed. The schema has `maps` and
`locations` tables

Only maps referenced by a saved location are persisted. `get_map` also returns
the live current map before its first save. Deleting the last location retains
the stored map. Map restoration, automatic map switching, SLAM map streams,
localization initialization, and web JSON migration are outside this package.
Use the static map from Nav2, not a costmap. Correct localization is required
when capturing a pose. After switching maps, wait for localization to be initialized
and TF to reflect the new map before saving. TF freshness alone does not establish
localization quality.

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `database_path` | `~/.ros/sbem_waypoints.sqlite3` | Persistent SQLite file |
| `map_topic` | `map` | Reliable, transient-local static map topic |
| `global_frame` | `map` | Must match the map's frame |
| `robot_frame` | `base_link` | Robot TF frame |
| `nav_action_name` | `navigate_to_pose` | Existing Nav2 action server |
| `max_transform_age` | `1.0` | Maximum TF age in seconds; future TF is rejected |
| `server_timeout` | `2.0` | Nav2 discovery timeout in seconds |
| `acceptance_timeout` | `5.0` | Nav2 acceptance timeout in seconds |
| `use_sim_time` | `false` | Use the simulation clock |

All parameters are available as launch arguments. Names are relative to the
node namespace, so map/action topics can also be remapped normally.
Configuration parameters are read at startup; restart to change them.
`