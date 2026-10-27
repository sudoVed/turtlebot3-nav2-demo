# Source this file in EVERY terminal used for this demo.
# Fixed local domain/partition keeps the GUI, mission and inspection tools together.
source /opt/ros/lyrical/setup.bash
_antropi_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${_antropi_workspace}/install/setup.bash" ]]; then
  source "${_antropi_workspace}/install/setup.bash"
fi
export ROS_DOMAIN_ID=42
export GZ_PARTITION=antropi_demo
export ROS_LOG_DIR="${_antropi_workspace}/.runtime/ros_logs"
mkdir -p "$ROS_LOG_DIR"
unset _antropi_workspace
