# How this demo actually works

This document describes the current Lyrical configuration. The checked-in
`live_graph.json`, `rqt_graph.*` and `tf_tree.*` are observed snapshots from the
original baseline, not freshly captured graphs of the expanded course. The core
sensor/control wiring is unchanged; the scene topics added later are listed below.
Use [commands.md](commands.md) to regenerate graphs from your running instance.

[Actual topic graph](rqt_graph.svg) · [Observed TF tree](tf_tree.svg) ·
[Turtle close-up](turtle_preview.png) · [RViz frame display](rviz_frames.png)

## Start it yourself

Terminal 1 — build after source changes, then bring up the system:

```bash
cd ~/antropi_ws
source demo_env.bash
colcon build --symlink-install --packages-select antropi_demo
source install/setup.bash
ros2 launch antropi_demo demo.launch.py
```

Gazebo opens with the robot on the green START pad. RViz opens with the live map,
laser, costmap, robot model and TF display. The robot stays still. SLAM starts
immediately, so nearby visible space will already appear in the map.

Terminal 2 — start autonomous travel to the blue FINISH pad:

```bash
source ~/antropi_ws/demo_env.bash
ros2 run antropi_demo run_mission.py --ros-args -p use_sim_time:=true
```

The script waits for readiness and sends exactly one endpoint `(6, 0, yaw=0)`.
There is no teleoperation, stored route or list of hidden waypoints. Alternatively,
`autostart_mission:=true` on the launch command sends it automatically when ready.

For a different endpoint, use RViz's **Nav2 Goal** tool or override the script:

```bash
ros2 run antropi_demo run_mission.py --ros-args -p use_sim_time:=true \
  -p goal_x:=5.5 -p goal_y:=1.0 -p goal_yaw:=0.0
```

Use Ctrl+C in Terminal 2 to cancel a mission. Use Ctrl+C in Terminal 1 to stop the
whole system. Relaunch for a fresh robot pose and SLAM map. Closing the Gazebo
window alone leaves its separate server running. Avoid resetting only simulation
time while keeping the SLAM/TF nodes alive.

Every terminal must source `demo_env.bash`: it selects ROS domain **42**, Gazebo
partition **antropi_demo**, sources ROS/the workspace, and puts logs in `.runtime`.

## What is the robot?

The original white robot was the installed `nav2_minimal_tb3_sim` **TurtleBot3
Waffle**, with ROBOTIS meshes and a differential drive model. Its Gazebo visuals
used white materials; that was not a new custom robot.

The updated version adds a green segmented shell, head, eyes, flippers and tail.
The launch modifies only `<visual>` elements/materials in the upstream SDF.
The URDF also groups the colored decorations into fixed cosmetic links because
RViz otherwise applies the chassis material across multiple visuals in a link.
The physics still uses the original collisions, inertias and joints. The
laser and drive plugins are unchanged. Keeping both descriptions in agreement
makes it look the same in Gazebo and RViz. The shell stays below the laser plane.

## Topic wiring

Gazebo transport and ROS DDS are separate transports. The sensors and drive
plugin run inside Gazebo, so they appear on the ROS graph through `/ros_gz_bridge`,
not as individual ROS nodes.

| ROS topic | Type | Producer | Relevant consumers / purpose |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → bridge | Simulation time for ROS nodes |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo GPU lidar → bridge | SLAM Toolbox, global/local costmaps, collision monitor, RViz |
| `/odom` | `nav_msgs/Odometry` | Gazebo DiffDrive → bridge | Controller's motion estimate; BT navigator and velocity smoother also subscribe |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo joint-state plugin → bridge | Robot state publisher updates wheel transforms |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU → bridge | Published, but unused by this baseline; there is no EKF |
| `/map` | `nav_msgs/OccupancyGrid` | `/slam_toolbox` | Global costmap's static layer and RViz |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Global costmap | Visualization/inspection of global planning costs |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Local costmap | RViz costmap overlay; controller uses its internal costmap |
| `/plan` | `nav_msgs/Path` | `/planner_server` during this mission | RViz's cyan planned path |
| `/cmd_vel_nav` | `geometry_msgs/TwistStamped` | `/controller_server` | `/velocity_smoother` |
| `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | `/velocity_smoother` | `/collision_monitor` |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | `/collision_monitor` during this mission | Bridge → Gazebo DiffDrive |

The message namespace abbreviations above omit `/msg/` for readability.
Recovery behaviors can also publish to `/cmd_vel_nav`. Upstream bringup includes
route, docking, following and waypoint servers, whose endpoints may appear in
rqt_graph even when their actions are not being used. In particular, the route
server also advertises `/plan`, and docking/following servers advertise `/cmd_vel`.
Their presence is not evidence that those behaviors ran.

The loop is:

```text
Gazebo wheel motion ── bridge ── /odom and odom → base_footprint
Gazebo laser ───────── bridge ── /scan
                                  ├── SLAM Toolbox → /map and map → odom
                                  ├── global/local obstacle layers
                                  └── collision monitor

controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
    → collision_monitor → /cmd_vel → bridge → Gazebo DiffDrive → wheel motion
```

**SLAM Toolbox uses the odometry TF, not a direct `/odom` subscription in this
configuration.** It looks up the robot's odometric pose at the scan timestamp.
An Odometry message alone does not automatically create a TF transform: the
DiffDrive plugin explicitly publishes both, and the bridge carries both to ROS.

## Actions connect the planning and control servers

Not all connections are ordinary topics. The mission submits the
`nav2_msgs/action/NavigateToPose` action to `/bt_navigator`. Its behavior tree
coordinates the planner and controller:

```text
mission / RViz
  └─ NavigateToPose action → bt_navigator
       ├─ ComputePathToPose action → planner_server [Smac 2D A*]
       │      └─ path returned as the action result
       └─ FollowPath action → controller_server [MPPI]
              └─ velocity commands on /cmd_vel_nav
```

The behavior tree passes the returned path to FollowPath. **The controller does
not subscribe to `/plan` to obtain its route.** That topic is a visualization
output. The installed behavior tree evaluates planning at 1 Hz and can validate
and retain the remaining path near the goal. It also contains costmap-clearing,
spin, wait and backup recovery branches.

## Global A* planning and MPPI control

**Smac 2D is now the global planner plugin.** It searches the global costmap with
cost-aware A*, returning a path for the controller. It does not send robot
velocities. The plugin is `nav2_smac_planner::SmacPlanner2D`, with unknown space
allowed and cost travel multiplier 2.0. See the
[Smac 2D documentation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/).

The earlier baseline used **NavFn**, first with its default Dijkstra expansion,
then with A*. NavFn computes a potential field and extracts a path from it. In our
expanded-course tests it repeatedly reported path-extraction cycle exhaustion
after finding a legal potential. That observed failure motivated trying Smac 2D;
it is not evidence that Dijkstra is inherently bad or A* always wins.

**MPPI means Model Predictive Path Integral.** It is the controller plugin. It
starts with a candidate control sequence, perturbs it to generate alternatives,
predicts the resulting robot motion, scores the trajectories, and uses their
cost-weighted contributions to improve the control sequence. It issues the next
velocity command and repeats using the latest robot state. Critics penalize
collisions, excessive cost, departure from the path and poor goal alignment.
See the [MPPI documentation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/).

The installed defaults used here are:

- Controller update rate: **20 Hz**.
- Candidate batch size: **2,000**.
- Prediction: **56 steps × 0.05 s = 2.8 seconds**.
- Motion model: **differential drive**.
- Our forward speed override: **0.30 m/s**.

These are existing Nav2 plugins. MPPI comes from the installed Lyrical defaults;
Smac 2D is selected explicitly in our overrides.
We have not written a custom planner/controller or established that this pair is
optimal for every world. It is a working baseline for the assignment.

## Map, odom and the actual TF chain

The important TF edges, observed from `/tf` and `/tf_static`, are:

```text
map
 └─ odom                          SLAM Toolbox, dynamic /tf
     └─ base_footprint             Gazebo DiffDrive → bridge, dynamic /tf
         └─ base_link             robot_state_publisher, static /tf_static
             ├─ base_scan         robot_state_publisher, static /tf_static
             ├─ imu_link          robot_state_publisher, static /tf_static
             ├─ wheel_left_link   robot_state_publisher, dynamic /tf
             ├─ wheel_right_link  robot_state_publisher, dynamic /tf
             ├─ caster_back_*     robot_state_publisher, static /tf_static
             ├─ turtle_visual_*   cosmetic fixed links, static /tf_static
             └─ camera_link       robot_state_publisher, static /tf_static
                 ├─ camera_rgb_frame → camera_rgb_optical_frame
                 └─ camera_depth_frame → camera_depth_optical_frame
```

`/tf` and `/tf_static` are topics; `map`, `odom`, and `base_scan` are frame names.
There is also a `/map` topic, but it carries occupancy data rather than a transform.
This setup does not broadcast a Gazebo `world → map` transform.

`odom` is the locally continuous motion estimate. It can drift. `map` is the
reference frame of the globally corrected SLAM estimate. SLAM may adjust that
estimate after scan matching or loop closure; the odometry frame remains useful
for smooth local control.

SLAM does not make this relationship correct by simply declaring `map = odom`.
It obtains an odometric robot pose and independently estimates the robot pose
against its scan/map data. Conceptually, with `A_T_B` mapping coordinates from B
into A:

```text
map_T_odom = map_T_base × inverse(odom_T_base)
```

SLAM broadcasts that correction as `map → odom`. In a snapshot after our original
GUI run, its translation was `(0.0392, 0.0329, 0)` metres. It also has a rotation;
the translation alone is not the complete correction.

Starting at world `(0,0)` with zero yaw makes the initial world and SLAM axes
approximately align for our predefined endpoint. It does not bypass localization
or force the map/odom transform to remain identity.

## Why this odometry is easier than the real robot's

Unlike scan-matching odometry such as RF2O, here,
Gazebo's DiffDrive plugin reads the simulated left/right wheel joint positions
and integrates them with its configured wheel radius and wheel separation. It
does **not** simply publish the Gazebo ground-truth model pose as odometry.
The [Gazebo DiffDrive source](https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/diff_drive/DiffDrive.cc)
shows that wheel-position integration and the separate odometry/TF publications.

For an ideal differential drive, wheel increments give:

```text
delta_s     = r × (delta_phi_R + delta_phi_L) / 2
delta_theta = r × (delta_phi_R - delta_phi_L) / wheel_separation
```

The plugin handles integrating these increments into pose/velocity. The current
model uses radius **0.033 m** and separation **0.287 m**, with odometry published
at **30 Hz**. We receive synchronized simulated sensor timestamps and known
geometry. That removes much of the real-world calibration, timing, serial
transport, encoder and scan-matching difficulty; it does not demonstrate that
hardware odometry has been solved.

On hardware, common faults to distinguish are a poor motion estimate, a missing
`odom → base` TF despite an `/odom` message existing, conflicting TF publishers,
wrong lidar extrinsics, and transforms unavailable at scan timestamps. These are distinct failure modes to check rather than assuming all odometry
problems have the same cause.

## Inspect it yourself

Start another terminal while the demo runs:

```bash
source ~/antropi_ws/demo_env.bash
ros2 run rqt_graph rqt_graph
```

Select **Nodes/Topics (all)**, uncheck **Leaf topics** and **Dead sinks**, and use
the topic filter below for the main sensor/control loop:

```text
/scan,/map,/odom,/cmd_vel_nav,/cmd_vel_smoothed,/cmd_vel,/joint_states
```

Actions and TF have their own graph controls. A topic-only filtered view will not
show the complete planner/controller action flow. Inspect those explicitly:

```bash
ros2 action list -t
ros2 action info /navigate_to_pose
ros2 action info /compute_path_to_pose
ros2 action info /follow_path
ros2 topic info /scan --verbose
ros2 topic info /cmd_vel --verbose
ros2 node info /slam_toolbox
```

For live transforms, run each separately and Ctrl+C to stop it:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link base_scan
```

The **Frames (TF)** display in RViz shows the main axes and lets you select other
frames. For a full generated TF diagram:

```bash
mkdir -p ~/antropi_ws/.runtime/tf
cd ~/antropi_ws/.runtime/tf
ros2 run tf2_tools view_frames
```

For a close-up of the styled TurtleBot3 while the simulation is running:

```bash
rviz2 -d ~/antropi_ws/src/antropi_demo/rviz/turtle.rviz \
  --ros-args -p use_sim_time:=true -r __node:=turtle_preview
```

## Expanded course and moving obstacle

Current overrides use 0.50 m inflation / scaling 5.0 globally and
0.55 m inflation / scaling 3.0 locally, with MPPI CostCritic weight 8.0. The robot planning radius remains 0.23 m. This narrows and steepens
the soft cost region without pretending the robot is smaller.

```text
demo_scene → /demo/obstacle_target (Float64) → scene_bridge
           → Gazebo JointPositionController → moving box's slider joint
Gazebo box surface → simulated LiDAR → /scan → costmaps + SLAM
Gazebo slider → /demo/obstacle_joint_states (inspection only)
Gazebo contact sensor → /demo/obstacle_contacts (inspection only)
run_mission → /demo/goal_pose → demo_scene → /demo/markers → RViz
```

The scene node uses simulation time for a 36-second oscillation. It never reads
the robot pose/path or sends robot velocities. All planning, control, recovery,
costmap updating and mapping remain upstream Nav2/SLAM Toolbox. The custom code
creates the world, styles the robot, integrates launch/configuration, sends one
mission goal, animates the test obstacle and displays endpoints.

The mover has collision geometry and moves through a constrained physical joint;
it is not a Gazebo actor with visuals alone. Motion can be paused or resumed with
`/demo/set_obstacle_motion` (`std_srvs/srv/SetBool`). A false launch argument
`dynamic_obstacle:=false` keeps it at its initial position.

## Packages, nodes and ownership

A package is installed software; a node is a running ROS participant; a plugin
is code loaded into a host node. Gazebo systems are not separate ROS nodes.
Nav2 composition puts multiple nodes in one container process; it does not turn
them into one logical node. Smac and MPPI therefore do not appear as independent
nodes named `/smac` or `/mppi` in rqt_graph.

| Package/component | Runtime node/system | Responsibility in this demo | Origin |
|---|---|---|---|
| `antropi_demo` launch/config | Launch process | Expands world/model, merges parameters, starts components | Custom integration |
| `nav2_minimal_tb3_sim` | Spawn helper; upstream model | TurtleBot3 Waffle model, meshes, spawn launch and robot bridge configuration | Existing Nav2 simulation package |
| Gazebo Sim | Physics, Sensors, DiffDrive, JointStatePublisher, IMU systems | Physics, laser rays, wheel motion/odometry and sensors | Existing simulator/plugins |
| `ros_gz_sim` | Transient spawn process | Creates the robot in the simulated world | Existing |
| `ros_gz_bridge` | `/ros_gz_bridge` | Robot sensors, clock, TF and command transport | Existing, upstream config |
| `ros_gz_bridge` | `/scene_bridge` | Moving-box joint target/state/contact transport | Existing bridge, custom topic config |
| `robot_state_publisher` | `/robot_state_publisher` | URDF and joint states to body/sensor/wheel TF | Existing |
| `slam_toolbox` | `/slam_toolbox` | Scan-based mapping/localization; `/map` and `map → odom` | Existing, async online mode |
| `nav2_bringup` / `nav2_lifecycle_manager` | Navigation container and lifecycle manager | Configure/activate Nav2 nodes and supervise lifecycle | Existing launch |
| `nav2_bt_navigator` | `/bt_navigator` and action-related helper nodes | Runs the navigation behavior tree and coordinates actions | Existing |
| `nav2_planner` + `nav2_smac_planner` | `/planner_server`, plugin `GridBased` | Global cost-aware A* planning | Existing Smac 2D plugin, selected here |
| `nav2_costmap_2d` | `/global_costmap/global_costmap` | Live map + scan obstacles + inflation for planning | Existing layers |
| `nav2_controller` + `nav2_mppi_controller` | `/controller_server`, plugin `FollowPath` | Scores predicted control trajectories and outputs commands | Existing MPPI plugin, tuned here |
| `nav2_costmap_2d` | `/local_costmap/local_costmap` | Rolling local voxel/scan obstacles and inflation | Existing layers |
| `nav2_behaviors` | `/behavior_server` | Spin, wait, backup and related behavior actions | Existing |
| `nav2_velocity_smoother` | `/velocity_smoother` | Limits/smooths commanded velocity changes | Existing |
| `nav2_collision_monitor` | `/collision_monitor` | Additional scan-based velocity intervention | Existing; not a guarantee against an object moving into a stopped robot |
| `nav2_simple_commander` | `/demo_mission` while script runs | API used to submit one NavigateToPose goal and inspect results | Existing API, custom client script |
| `antropi_demo` scene script | `/demo_scene` | Marker display and box motion target; pause/resume service | Custom, no robot steering |
| `rviz2` / `nav2_rviz_plugins` | `/rviz2` | Map, scans, costmap, TF, markers and interactive goal UI | Existing, custom saved view |
| `rqt_graph`, `tf2_tools`, `tf2_ros` | Optional inspection tools | ROS graph and coordinate-frame inspection | Existing; not required for navigation |
| `tools/observe_run.py` | `/obstacle_test_observer` when invoked | Records telemetry and moving-box contacts | Custom, read-only diagnostic |

Upstream bringup also launches smoother, waypoint, route, docking and following
servers. Their existence does not mean this NavigateToPose mission invokes their
actions. In particular, `smoother_server` smooths paths when requested and is not
the same component as `velocity_smoother`. AMCL, a static-map server, RF2O and
`robot_localization` are not active in this live-mapping launch. An IMU is
published but no EKF fuses it here.

### Additional topic and service interfaces

| Interface | Type | Producer → consumer |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | SLAM, bridge, state publisher → TF consumers |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | State publisher → TF consumers; transient-local static transforms |
| `/robot_description` | `std_msgs/msg/String` | State publisher → RViz robot model |
| `/demo/goal_pose` | `geometry_msgs/msg/PoseStamped` | Mission → scene marker update; transient local |
| `/demo/markers` | `visualization_msgs/msg/MarkerArray` | Scene → RViz; transient local and republished |
| `/demo/obstacle_target` | `std_msgs/msg/Float64` | Scene → scene bridge → Gazebo joint controller |
| `/demo/obstacle_joint_states` | `sensor_msgs/msg/JointState` | Gazebo → scene bridge → optional observer |
| `/demo/obstacle_contacts` | `ros_gz_interfaces/msg/Contacts` | Gazebo contact system → scene bridge → optional observer |
| `/demo/set_obstacle_motion` | `std_srvs/srv/SetBool` service | CLI → scene; true resumes and false holds target |

Laser subscribers must use compatible QoS; a best-effort subscriber is useful
for inspection. Map/description/static TF/marker subscribers may need transient
local durability to receive retained data when joining late. A visible topic
name alone does not prove that messages, QoS, timestamps and transforms are valid.

## What was custom and what was reused?

| Custom file/work | Why it exists | What it does not replace |
|---|---|---|
| `worlds/base_world.sdf.xacro` | Designed room, fixed obstacles, physical sliding box and endpoint pads | Gazebo physics and sensor implementation |
| `launch/demo.launch.py` | Synchronous xacro/model generation and coordinated bringup | Nav2 or SLAM algorithms |
| `config/*_overrides.yaml` | Parameters for this course and installed ROS release | Upstream plugin source code |
| `antropi_demo/turtle_style.py` | Matching decorative turtle visuals in SDF/URDF | TurtleBot3 collision model, inertia, drive or laser |
| `scripts/run_mission.py` | Submit a single goal, display progress, check result/error | Planning, control or obstacle avoidance |
| `scripts/demo_scene.py` | Animate test object and publish visible endpoint markers | Robot control or dynamic-object tracking |
| `config/scene_bridge.yaml`, `rviz/*.rviz` | Scene transport wiring and presentation | Bridge or RViz implementation |
| `tools/observe_run.py` | Inspect actual movement, contacts and result evidence | Independent ground-truth clearance measurement |

The complete map is not handcrafted as an occupancy grid. The world geometry is
handcrafted; SLAM builds the occupancy grid from sensor observations at runtime.

The local inflation radius was reduced from 0.60 m to 0.55 m after validation,
at user request. No simulation was run after that change; the successful measured
run used 0.60 m. Robot radius and MPPI obstacle weight were unchanged.
