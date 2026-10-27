# Antropi TurtleBot3 navigation demo

A TurtleBot3 Waffle drives from **(0, 0)** to **(6, 0)** while SLAM Toolbox builds
its map from laser scans. The custom Gazebo world has an enclosing room, a central
divider, shelves, crates and a red moving obstacle. Green and blue floor pads mark
the start and finish. The robot receives the finish coordinates, not the world's
occupancy layout. No prebuilt map, intermediate waypoints, or teleop is used.

The red obstacle moves on a physical Gazebo sliding joint while the robot navigates.
The final recording is subsequent work. Reaching the finish does not imply mapping every
part of the room: this is navigation while mapping, not full-coverage exploration.

**Last tested configuration (local inflation 0.60 m):** 37.8 seconds, zero recoveries and zero moving-box contact
reports. Final goal error: 0.239 m (0.25 m tolerance). See
[the integration results](docs/validation.md) for measured runs and limitations.
The current local inflation is 0.55 m, reduced afterward at user request without
another simulation. The reported results apply to 0.60 m, not the new setting.
The original 27-second baseline used the earlier, simpler course.

## Documentation

- [All launch and inspection commands](docs/commands.md)
- [Packages, nodes, topics, actions, TF and custom versus reused work](docs/system_walkthrough.md)
- [Test results and known limitations](docs/validation.md)

## Fresh workspace setup

Clone the repository into `~/antropi_ws`.
The workspace root must contain `src/antropi_demo`. ROS 2 Lyrical must already be
installed and `/opt/ros/lyrical/setup.bash` available. The tested platform is
Ubuntu 26.04; older ROS/Gazebo releases may require configuration changes.

## Environment

Developed against the installed Ubuntu 26.04 / ROS 2 Lyrical packages:

| Dependency | Version |
|---|---|
| Gazebo Sim | 10.5.0 |
| Nav2 | 1.5.2 |
| SLAM Toolbox | 2.10.0 |
| nav2_minimal_tb3_sim | 1.3.0 |
| ros_gz | 3.0.10 |

The package uses Nav2's minimal TurtleBot3 simulation, rather than requiring the
separate ROBOTIS `turtlebot3_gazebo` package. Robot meshes and models are supplied
by the installed dependency; there are no remote Fuel model downloads.

With ROS 2 Lyrical installed, the main dependencies are:

```bash
sudo apt install ros-lyrical-navigation2 ros-lyrical-nav2-bringup \
  ros-lyrical-nav2-minimal-tb3-sim ros-lyrical-slam-toolbox \
  ros-lyrical-ros-gz ros-lyrical-rviz2 ros-lyrical-xacro \
  ros-lyrical-nav2-smac-planner ros-lyrical-rqt-graph ros-lyrical-tf2-tools \
  python3-colcon-common-extensions python3-yaml
```

## Build and run

```bash
cd ~/antropi_ws
source demo_env.bash
colcon build --symlink-install --packages-select antropi_demo
source install/setup.bash
ros2 launch antropi_demo demo.launch.py
```

Gazebo and RViz open with the robot stationary, allowing a view of the world before
the run. SLAM already processes scans while stationary. In a second terminal:

```bash
cd ~/antropi_ws
source demo_env.bash
ros2 run antropi_demo run_mission.py --ros-args -p use_sim_time:=true
```

Or launch and send the goal automatically once the stack is ready:

```bash
ros2 launch antropi_demo demo.launch.py autostart_mission:=true
```

Headless integration run (still uses Gazebo physics and rendered laser scans):

```bash
ros2 launch antropi_demo demo.launch.py \
  headless:=true gazebo_gui:=false rviz:=false autostart_mission:=true
```

Stop the launch with Ctrl+C. Relaunch for a fresh world, robot pose and SLAM map.
Cancel an active mission with Ctrl+C in its terminal. If using `ROS_DOMAIN_ID` or
`GZ_PARTITION`, use matching values in every terminal.

`demo_env.bash` sets `ROS_DOMAIN_ID=42` and `GZ_PARTITION=antropi_demo` consistently.
Source it in inspection terminals too. Closing only the Gazebo GUI does not stop
the simulation server; Ctrl+C in the launch terminal stops the whole launch.

For the nodes, topics, TF tree, planner/controller explanation, and debugging
commands, read [the system walkthrough](docs/system_walkthrough.md).

The robot is the upstream TurtleBot3 Waffle with cosmetic green shell, head, eyes
and flippers. The generated SDF/URDF retain its original physics and sensors.
`antropi_demo/turtle_style.py` applies the same appearance to both descriptions.
The RViz **Frames (TF)** display is enabled; toggle it to declutter the map.

Save the map observed during a run:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/expanded_run \
  --ros-args -p use_sim_time:=true
```

## Moving obstacle and endpoint markers

The red box crosses the upper passage: its center moves along x=4.7 m,
y=0.50–2.20 m with a 36-second period. It is a physical collider driven by
Gazebo's JointPositionController. Nav2 discovers it through `/scan`; its joint
state is not fed into navigation. Ten fixed interior objects make up the course.

To start with the obstacle stationary, add `dynamic_obstacle:=false` to launch.
Toggle movement while running (source `demo_env.bash` first):

```bash
ros2 service call /demo/set_obstacle_motion std_srvs/srv/SetBool "{data: false}"
ros2 service call /demo/set_obstacle_motion std_srvs/srv/SetBool "{data: true}"
```

RViz shows persistent START and GOAL labels through `/demo/markers`. The mission
publishes its requested endpoint on `/demo/goal_pose`, including command-line
goal overrides. The Gazebo floor pads remain at the default start/finish.

Live SLAM can retain moving-object traces in its map. This demo is not dynamic-object
tracking or prediction; MPPI reacts to the obstacle costs supplied by Nav2.
For a final staged recording, mapping a static scene first and introducing motion
afterward is also possible using the toggle above.

## Architecture and choices

```text
Gazebo laser ── ros_gz_bridge ── /scan ── SLAM Toolbox ── /map, map → odom
Gazebo DiffDrive ── bridge ── /odom, odom → base_footprint
URDF + robot_state_publisher ── base_footprint → robot/sensor frames

Mission: NavigateToPose(6, 0)
             ↓
Nav2 BT → Smac 2D A* global planner → MPPI controller → velocity smoother
             ↑                                        ↓
     live SLAM map + scans                     collision monitor
                                                      ↓
                                      /cmd_vel (TwistStamped)
                                                      ↓
                                          bridge → Gazebo DiffDrive
```

- **Live mapping throughout:** asynchronous SLAM Toolbox owns `map → odom`.
  AMCL and the static map server are disabled. All ROS nodes use simulation time.
- **Goal beyond known space:** the global costmap is a 20 × 20 m rolling window
  at 5 cm resolution, with `track_unknown_space` and Smac `allow_unknown` enabled.
  Its static layer consumes the live SLAM map. The window prevents the planner's
  bounds from being restricted to SLAM's initially small observed map. Unknown
  cells stay unknown; the robot learns obstacles from scans and replans.
- **Known endpoint:** start is the world origin with zero yaw, so the initially
  established SLAM frame approximately aligns with the world frame. The goal is
  expressed in `map`. Changing spawn position/yaw requires updating this relation.
- **Planner and clearance:** Smac 2D uses A*. Global inflation is 0.50 m with cost scaling 5.0; local inflation is
  0.55 m with scaling 3.0 and MPPI CostCritic weight 8.0, with the physical planning
  radius unchanged at 0.23 m. Inflation is a graded cost field, not a required
  0.50 m clearance from every obstacle.
- **Controller limits:** forward speed is capped at 0.30 m/s; acceleration limits
  match the Gazebo differential drive. Nav2's built-in replanning/recovery behavior
  tree is retained. The mission only submits one endpoint and reports the result.
- **Configuration:** small YAML overrides are merged with the installed Lyrical
  defaults at launch. This avoids maintaining a stale copy of all Nav2 parameters,
  but means package upgrades should be followed by another integration run.
- **World markers:** the start/finish pads are visual-only and sit below the laser.
  They do not add collisions or encode navigation costs.
- **Launch ordering:** xacro expansion and YAML merging finish synchronously
  before launching Gazebo. The mission waits for Nav2/SLAM lifecycle readiness,
  an observed map, and a map-to-robot transform before sending its goal.

## Files to change

| File under `src/antropi_demo` | Purpose |
|---|---|
| `worlds/base_world.sdf.xacro` | Layout, collisions, start/finish pads |
| `config/nav2_overrides.yaml` | Costmaps, planner, controller and speed limits |
| `config/slam_overrides.yaml` | Mapping frequency and scan insertion thresholds |
| `launch/demo.launch.py` | Simulation, bridges, SLAM, Nav2 and RViz bringup |
| `scripts/demo_scene.py` | Endpoint markers and obstacle joint targets; no robot control |
| `config/scene_bridge.yaml` | Moving obstacle commands, joint state and contact bridges |
| `scripts/run_mission.py` | Single finish goal, progress and success/failure reporting |
| `rviz/demo.rviz` | Live map, robot, scans, local costmap and global path |

## References

- [Nav2 navigation while mapping](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam.md)
- [Nav2 minimal TurtleBot simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation)
- [Smac 2D planner](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

Existing navigation and mapping algorithms are used as permitted by the assignment.
The custom work here is the world, integration, configuration and mission orchestration.

## License and attribution

See [LICENSE](LICENSE) and [NOTICE.md](NOTICE.md). Build, install and runtime
outputs are excluded by `.gitignore`. The screen recording is not yet included.
