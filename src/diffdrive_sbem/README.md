# diffdrive_sbem

   *DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
   The robot is basically a box moving according to differential drive kinematics.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_2/doc/userdoc.html).

## Battery broadcaster

Bringup loads `battery_state_broadcaster/BatteryStateBroadcaster` and passes
[diffbot_controllers.yaml](bringup/config/diffbot_controllers.yaml) to its spawner
before configuration. The hardware exports `battery_state/voltage` in volts and
`battery_state/current` in amperes. Positive current reports charging; negative
current reports discharging. Battery messages are published on `/battery_state`.

Set `charging_voltages` and `charging_percentages` under
`battery_state_broadcaster.ros__parameters` to calibrate the charging curve.
