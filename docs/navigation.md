# Gazebo, SLAM Toolbox, and Nav2 evaluation

## Final velocity output

The navigation launch now changes the installed Waffle SDF's differential-drive
command topic to `/cmd_vel_safe`, retaining the other model settings. It refuses
unexpected drive plugins or topic remappings instead of silently bypassing the
gate. The temporary SDF is removed on launch shutdown.

```text
Nav2 /cmd_vel -------------------\
                                  velocity_gate -> /cmd_vel_safe -> Gazebo drive
manual /turtlebot3/manual_cmd_vel /
```

Navigation is selected by default. `/turtlebot3/select_manual` (SetBool) selects
manual when true and navigation when false; switching clears buffered input.
`/turtlebot3/emergency_stop` latches the final gate; `/turtlebot3/reset_stop`
clears it and requires a fresh input. Input loss beyond 0.5 seconds latches a
stop, as does a timer discontinuity. The output timer uses a steady clock, so
pausing simulated time does not freeze the watchdog. The gate bounds output to
0.22 m/s and 1.5 rad/s. Unselected inputs cannot take over automatically.

In this combined launch the manual executor's original stop/reset services are
under `/turtlebot3/manual/`. Use the final gate services to stop the drive.
The standalone `safe_controller.launch.py` retains its original interface.

ROS CI runs `scripts/gate_smoke.py` for continuing-nav emergency stop, exclusive
source selection, and stale-input behavior, then runs guarded Gazebo navigation.
The checked-in navigation numbers predate this routing change; use the Actions
artifact for the current run. This software gate cannot stop a process that
publishes directly to `/cmd_vel_safe`, or guarantee stopping after gate/process
failure without a separate actuator-level command timeout.

Phase 2 runs the official TurtleBot3 Waffle model in Gazebo Classic, builds an
occupancy grid online with SLAM Toolbox, and sends a `NavigateToPose` goal to
Nav2. The evaluator subscribes to `/odom`, captures action feedback, and writes
the raw trajectory, action outcome, map geometry/coverage, PNG/YAML occupancy
map, summary metrics, and a trajectory figure. After Nav2 reports success, it
records two additional seconds so the endpoint speed reflects settling rather
than the success callback.

The goal is sent only after Nav2 is active, odometry is flowing, and SLAM has
expanded beyond its initial placeholder grid (at least 50 x 50 cells and 200
known cells). This prevents the target being submitted off the global costmap
on slower CI runners.

The smoke script resolves `turtlebot3_gazebo` through the ROS package index and
adds its `models/` directory to `GAZEBO_MODEL_PATH`; this ensures the
`model://turtlebot3_world` geometry and its laser returns are available on both
desktop and clean CI installations.

## Dependencies

On Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

## Reproduce

```bash
scripts/verify_ros2.sh
scripts/navigation_smoke.sh results/navigation
```

The default run starts at the pose configured by Nav2's TurtleBot3 simulation
and targets `(-1.2, -0.5)` in the `map` frame. Override `goal_x` and `goal_y`
through the launch file for another reachable pose. The test is headless and
sets software OpenGL rendering for compatibility with CI and remote hosts.

Results are odometry-based and can vary slightly with simulator scheduling.
Nav2's success status uses its configured goal tolerance; therefore the final
position does not need to equal the requested coordinate exactly. The default
evaluation records and draws the standard 0.25 m tolerance used by this launch.
