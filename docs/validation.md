# Integration results

## Original baseline — 26 October 2026

Both runs started a fresh Gazebo world and a fresh SLAM session, and sent one
`NavigateToPose` goal at `(6, 0, yaw=0)` from a spawn at `(0, 0, yaw=0)`.
No map file was loaded and no teleoperation or intermediate goals were used.

| Measurement | Headless run | Integrated Gazebo + RViz run |
|---|---:|---:|
| Nav2 result | SUCCEEDED | SUCCEEDED |
| Wall time after goal submission | 27.5 s | 27.1 s |
| Reported recovery count | 0 | 0 |
| Final SLAM-frame position | (5.918, 0.191) | (5.874, 0.143) |
| Position error to requested goal | 0.208 m | 0.190 m |
| Configured position tolerance | 0.25 m | 0.25 m |
| Observed cells at goal submission | 1,543 | 1,573 |
| Observed cells at completion | 24,939 | 24,905 |

Observed cells count all non-unknown occupancy grid cells. Position errors are
measured from SLAM's `map → base_footprint` transform, not an independent Gazebo
ground-truth comparison. These are two baseline integration runs, not a statistical
reliability benchmark.

Additional checks during the GUI run:

- `slam_toolbox` and `bt_navigator` both reported lifecycle state `active [3]`.
- Laser messages arrived with `frame_id: base_scan` and simulation timestamps.
- `/cmd_vel` used `geometry_msgs/msg/TwistStamped`, matching the bridge.
- The live map grew during motion at 0.05 m/cell.
- Gazebo and RViz rendered the world, robot, map and costmap; their actual window
  contents were inspected. RViz emitted a startup GLSL sampler-link error, but the
  live map rendered and updated. Gazebo also emitted EGL driver warnings without
  preventing rendering. These should be kept in mind when preparing the recording.

The headless run's saved map is `maps/base_run.yaml` + `maps/base_run.pgm`
(183 × 144 cells). It is an output artifact; the demo launch never loads it.

Local detailed logs are ignored by Git in `.runtime/base_run.log` and
`.runtime/gui_run.log`. Tests used `ROS_DOMAIN_ID=42` and
`GZ_PARTITION=antropi_demo` to isolate their communications.

## Problems encountered and resolved

1. The ROS package manifest rejected a local-only maintainer email. A valid-format
   maintainer address is now present.
2. The Python mission needed its executable bit set for an ament symlink install.
3. Empty `filters: []` overrides were rejected by the installed ROS parameter parser.
   The installed typed filter lists are retained, with keepout/speed filters disabled
   through the existing Nav2 launch switches.
4. The default SLAM minimum laser range differed from the model's `0.00001 m`
   minimum. The override now matches the model.
5. The default RViz navigation panel consumed excessive space at this desktop's DPI.
   The baseline RViz layout now emphasizes the map, with mission feedback in the log.

## Remaining assignment work

The turtle-styled model also completed the same mission in **27.1 seconds**,
with **0.213 m** SLAM-frame position error. This run had **one recovery**: the
first NavFn path extraction exhausted its cycle limit; the behavior tree cleared
the global costmap and its retry succeeded. This is an observed baseline
limitation, not evidence that every run will have zero recoveries.

Before running it, a structural comparison confirmed that the styled SDF retained
the upstream collision, inertia, joint, sensor and plugin elements exactly.
Later URDF-only cosmetic link grouping corrected RViz's per-link coloring.

During the final stationary session's Ctrl+C shutdown, the bridge exited with
SIGSEGV and the composed Nav2 container needed launch's SIGKILL escalation after
its shutdown timeouts. All test processes exited. This is a remaining teardown
issue; it did not occur during the successful navigation action. Allow shutdown
to finish before starting another instance.

The expanded world and dynamic obstacle are described below. Remaining submission
work is the 1–2 minute staged recording, repository publication and form responses.

## Expanded course — 27 October 2026

Seven additional fixed blocks were added (ten interior fixed objects total),
plus a red physical sliding box. Endpoints remain (0,0) and (6,0). Initial expanded-course tests used NavFn A*; the final configuration uses Smac 2D A*.
RViz now shows persistent start/goal markers. Its saved camera values were
changed to floating-point YAML values so the map opens at the intended zoom.

Experiments, including failures:

| Run | Conditions | Result | Moving-box contacts |
|---|---|---|---|
| 1 | Narrow divider/mover gap, 18 s mover period, 0.50 m inflation | FAILED, 41.8 s | 0 reported |
| 2 | Wider passage, 18 s period | SUCCEEDED, 46.3 s, 1 recovery, 0.141 m error | 46 contact records: right wheel |
| 3 | Wider passage, 36 s period | SUCCEEDED, 60.0 s, 6 recoveries, 0.107 m error | 80 contact records: wheel and chassis |
| 4 | Stronger local avoidance, original exit gap | Canceled during repeated stalls near exit | 0 in recorded windows |
| 5 | Stronger local avoidance, relocated exit island | SUCCEEDED, 42.8 s, 4 recoveries, 0.247 m error | 0 |
| 6 | Final layout and tuning, Smac 2D A* + MPPI | SUCCEEDED, 37.8 s, 0 recoveries, 0.239 m error | 0 |

Runs 2 and 3 are **not obstacle-avoidance passes**, despite Nav2 reporting success.
Contact records are repeated physics sensor reports, not separate collision events.
Run 1's map/costmap contained a streak of previous moving-box positions; the robot
entered a narrowing gap and planning failed inside the inflated region. Run 2 also
experienced NavFn path-extraction exhaustion before recovering.

The contact sensor proved useful: checking the NavigateToPose result alone would
have missed the physical interactions. The revised tuning preserves 0.50 m global
inflation but uses 0.60 m local inflation, local scaling 3.0 and MPPI CostCritic
weight 8.0. Robot planning radius remains 0.23 m; mover period is 36 seconds.

### Reproduce telemetry

Start the observer in another sourced terminal before sending the mission goal:

```bash
source demo_env.bash
python3 tools/observe_run.py /tmp/navigation_run.json 90
```

This subscribes to `/odom`, `/scan`, `/plan`, `/cmd_vel`,
`/demo/obstacle_joint_states` and `/demo/obstacle_contacts`. It never controls the
robot. The JSON includes trajectories, plans and moving-box contact reports.
Clearance and scan-hit estimates use wheel odometry, so localization drift makes
them approximate; the contact reports come directly from Gazebo physics. The
contact sensor covers the moving box, not every static obstacle in the room.

`maps/base_run.*` and older graph/screenshots describe the original baseline.
They are not loaded by the expanded-course launch. Live SLAM can retain dynamic
object traces; no dynamic-object segmentation or prediction is implemented.

Run 5 completed in the GUI with the final layout; the four recoveries came from
NavFn's path-extraction failures before reaching the mover. The final planner
was therefore changed to Nav2's Smac 2D A* plugin. The controller remains MPPI.

The final Smac run used fresh SLAM and the visible Gazebo + RViz launch. Its map
grew from 1,523 to 22,279 observed cells. The observer received 2,453 joint
messages and 441 scans, with the box center covering y=0.50–2.20 m. Runtime
parameter reads confirmed SmacPlanner2D, MPPI CostCritic 8.0, and inflation
radii 0.60 m local / 0.50 m global. This is one successful final-configuration
run, not proof of reliability for every obstacle phase or arbitrary goal.

A compact record is in [expanded_run_summary.json](expanded_run_summary.json).
[Gazebo world](expanded_gazebo.png). RViz markers and map zoom were inspected
during the earlier live run; the final background-window screenshot had a black
render surface and was discarded.

The obstacle pause/resume service was also checked against joint telemetry:
while paused, its center stayed at y=1.320187 m throughout a three-second sample;
after resuming, the sampled center moved from y=1.395536 to 1.800180 m.

## Untested follow-up adjustment

At user request, local inflation was reduced from 0.60 m to 0.55 m after run 6.
No further simulations were run. All reported final-run metrics apply to 0.60 m.
Global inflation (0.50 m), robot radius (0.23 m) and MPPI CostCritic (8.0) remain
unchanged.
