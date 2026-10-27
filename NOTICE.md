# Attribution and reproducibility

This repository contains custom integration, configuration, world geometry,
visual styling and demonstration/diagnostic scripts for a ROS 2 navigation demo.
The package declares Apache-2.0; the license text is included in `LICENSE`.

Navigation and mapping implementations are supplied by installed dependencies:

- ROS 2 and its client libraries: https://docs.ros.org/
- Navigation2, including Smac 2D, MPPI, costmaps, behaviors and Simple Commander:
  https://github.com/ros-navigation/navigation2
- Nav2 minimal TurtleBot3 simulation and upstream robot assets:
  https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- Gazebo Sim: https://github.com/gazebosim/gz-sim
- ROS/Gazebo integration: https://github.com/gazebosim/ros_gz
- TurtleBot3 hardware/model lineage: ROBOTIS.

Upstream dependencies and robot assets retain their own licenses. They are read
from the installed ROS packages rather than bundled into this repository. The
styling code generates decorative mesh geometry at launch. Screenshots show
those dependencies running in the custom scene.

AI tools assisted integration, debugging and documentation. All result claims
refer to the recorded simulation tests in `docs/validation.md`; no original
navigation algorithm or hardware validation is claimed.
