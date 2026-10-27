# Commands: build, run, inspect and record

Use Bash. Run commands from the repository root (`~/antropi_ws` in the examples).
Source `demo_env.bash` in **every terminal**: it sets ROS domain 42, Gazebo
partition `antropi_demo`, simulation log location and the workspace overlay.
Only start **one** demo launch at a time. The alternatives below are alternatives,
not additional launches to run together.

## 1. Build

```bash
cd ~/antropi_ws
source demo_env.bash
colcon build --symlink-install --packages-select antropi_demo
source install/setup.bash
ros2 launch antropi_demo demo.launch.py --show-args
```

## 2. Normal demonstration: Gazebo + RViz + SLAM + Nav2

Terminal 1:

```bash
cd ~/antropi_ws
source demo_env.bash
ros2 launch antropi_demo demo.launch.py
```

This launches the Gazebo server/client, TurtleBot3 spawn and bridge, robot state
publisher, SLAM Toolbox, composed Nav2 servers, scene node/bridge and RViz.
The robot stays at START until a goal is sent. The red box moves by default.
SLAM is already active; the launch does not wait until the mission to start mapping.

Terminal 2:

```bash
cd ~/antropi_ws
source demo_env.bash
ros2 run antropi_demo run_mission.py --ros-args -p use_sim_time:=true
```

One endpoint `(6, 0, yaw=0)` is sent. Read progress and the final SUCCEEDED/FAILED
result in this terminal. No map file, teleop or intermediate waypoint is supplied.

To start automatically, use this **instead of** the normal Terminal 1 launch:

```bash
ros2 launch antropi_demo demo.launch.py autostart_mission:=true
```

## 3. Stage the moving obstacle

Start with the red box held at its initial position:

```bash
ros2 launch antropi_demo demo.launch.py dynamic_obstacle:=false
```

This does not remove the box; it remains a physical obstacle. Start the mission
normally. From a third sourced terminal, enable motion when ready:

```bash
ros2 service call /demo/set_obstacle_motion std_srvs/srv/SetBool '{data: true}'
```

Pause it at its current target:

```bash
ros2 service call /demo/set_obstacle_motion std_srvs/srv/SetBool '{data: false}'
```

The default moving-box run was tested with 0.60 m local inflation; the current
0.55 m setting is an untested follow-up adjustment. Enabling it midway
through a mission creates a different encounter and should be rehearsed before
recording. This launch keeps SLAM running throughout: it does not switch to AMCL.

## 4. Other goals and GUI combinations

Send a different endpoint (coordinates in `map`, yaw in radians):

```bash
ros2 run antropi_demo run_mission.py --ros-args -p use_sim_time:=true \
  -p goal_x:=6.0 -p goal_y:=0.0 -p goal_yaw:=0.0 -p timeout_sec:=180.0
```

RViz's **Nav2 Goal** tool also sends navigation goals. Avoid sending another goal
while measuring the scripted mission: it can replace the active action. The
persistent goal marker is explicitly updated by the mission's `/demo/goal_pose`.

Headless physics and laser simulation:

```bash
ros2 launch antropi_demo demo.launch.py \
  headless:=true gazebo_gui:=false rviz:=false autostart_mission:=true
```

Simulation without automatic GUI startup:

```bash
ros2 launch antropi_demo demo.launch.py gazebo_gui:=false rviz:=false
```

Attach either GUI later, in separate sourced terminals:

```bash
gz sim -g
```

```bash
rviz2 -d "$(ros2 pkg prefix --share antropi_demo)/rviz/demo.rviz" \
  --ros-args -p use_sim_time:=true
```

Optional robot close-up:

```bash
rviz2 -d "$(ros2 pkg prefix --share antropi_demo)/rviz/turtle.rviz" \
  --ros-args -p use_sim_time:=true -r __node:=turtle_preview
```

## 5. rqt graph and node/topic inspection

```bash
ros2 run rqt_graph rqt_graph
```

Select **Nodes/Topics (all)**. Uncheck **Leaf topics** and **Dead sinks** if links
are missing. Start with this topic filter to avoid parameter/logging clutter:

```text
/scan,/map,/odom,/joint_states,/cmd_vel_nav,/cmd_vel_smoothed,/cmd_vel
```

Add `/demo/markers`, `/demo/goal_pose`, `/demo/obstacle_target`,
`/demo/obstacle_joint_states` and `/demo/obstacle_contacts` for scene wiring.
Export the graph using rqt's save/export controls. A filtered topic graph does not
explain action orchestration or the coordinate-frame tree by itself.

```bash
ros2 node list
ros2 node info /slam_toolbox
ros2 node info /planner_server
ros2 node info /controller_server
ros2 node info /demo_scene
ros2 topic list -t
ros2 topic info /scan --verbose
ros2 topic info /cmd_vel --verbose
ros2 action list -t
ros2 action info /navigate_to_pose
ros2 action info /compute_path_to_pose
ros2 action info /follow_path
ros2 lifecycle get /slam_toolbox
ros2 lifecycle get /bt_navigator
```

Sensor inspection (Ctrl+C ends continuous commands):

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /cmd_vel --once
ros2 topic echo /demo/obstacle_joint_states --once
ros2 topic echo /demo/markers --once --qos-durability transient_local
```

Run these one at a time; `hz` keeps running until stopped. Rates are measured by
this subscriber and may differ from configured simulation rates under load.

## 6. TF frames

Each `tf2_echo` command runs continuously; use separate terminals or stop it:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link base_scan
```

Generate a timestamped TF graph and frame timing report:

```bash
mkdir -p .runtime/tf
cd .runtime/tf
ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true
```

The output appears in that directory. RViz's **Frames (TF)** display shows axes
and frame names on the running robot.

## 7. Verify the actual loaded configuration

```bash
ros2 param get /planner_server GridBased.plugin
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.CostCritic.cost_weight
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /demo_scene obstacle_period
```

Expected: SmacPlanner2D, MPPIController, 8.0, 0.50, 0.55, and 36.0 respectively.
`expected_planner_frequency` is a timing expectation/warning setting; it does not
schedule replanning. The behavior tree controls planning requests.

## 8. Save a map and collect evidence

Save the current live occupancy grid without overwriting the old baseline map:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/expanded_run \
  --ros-args -p use_sim_time:=true
```

This writes a map image and YAML. The normal launch never loads either file.
It does not save SLAM's pose graph for continued mapping.

Start telemetry **before** sending the mission goal:

```bash
mkdir -p .runtime
python3 tools/observe_run.py .runtime/my_run.json 120
```

This observes motion, plans, scans and moving-box contacts. Its distance estimates
use odometry, not independent ground truth; the sensor only covers box contacts.
A zero-contact result is not a guarantee that every static obstacle was avoided.

Optional ROS bag for debugging, in a separate terminal:

```bash
ros2 bag record -o .runtime/demo_bag \
  /clock /scan /odom /tf /tf_static /map /plan /cmd_vel \
  /demo/markers /demo/obstacle_joint_states /demo/obstacle_contacts
```

A ROS bag is not the required screen recording. Use the desktop screen recorder
for the assignment video. A useful 1–2 minute sequence is: overview of the Gazebo
world and designated endpoints; fresh-run SLAM map growing in RViz; navigation
and the moving-box encounter; then the goal result. Show Gazebo's complete world
as the designed environment, not as a preloaded map given to the robot.

## 9. Stop and reset

Ctrl+C in the mission terminal requests cancellation. Ctrl+C in the launch
terminal stops the stack. Separately opened rqt/RViz/Gazebo clients must also be
closed. Allow launch to finish shutdown before relaunching. Known upstream bridge
and composed-container shutdown errors are recorded in [validation.md](validation.md).
Relaunch to reset robot pose and SLAM together; resetting only Gazebo time while
SLAM remains alive can leave inconsistent timestamps and transforms.
