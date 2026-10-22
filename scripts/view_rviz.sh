#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"

# VS Code installed as a Snap may pass libraries from its runtime to RViz.
# Start ROS with the desktop display/session variables and system libraries.
exec env -i \
  HOME="$HOME" \
  USER="${USER:-}" \
  DISPLAY="${DISPLAY:-}" \
  XAUTHORITY="${XAUTHORITY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
  DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
  LANG="${LANG:-C.UTF-8}" \
  PATH="/opt/ros/humble/bin:/usr/bin:/bin" \
  bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 launch husky_ur5e_description view_robot.launch.py'
