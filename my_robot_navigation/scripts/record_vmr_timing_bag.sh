#!/usr/bin/env bash

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WORKSPACE/nav_diagnostic_bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${1:-$OUTPUT_ROOT/vmr_timing_$STAMP}"
BAG_DIR="$SESSION_DIR/bag"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WORKSPACE/install/setup.bash"
fi

set -u

export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY

mkdir -p "$SESSION_DIR"

monitor_graph() {
  while true; do
    {
      echo "===== $(date --iso-8601=seconds) ====="
      ros2 node list 2>&1 || true
      ros2 topic info /vmr_base_bridge/pose --verbose 2>&1 || true
      ros2 topic info /vmr_base_bridge/odom --verbose 2>&1 || true
      ros2 topic info /vmr_base_bridge/timing_diagnostics --verbose 2>&1 || true
    } >>"$SESSION_DIR/ros_graph.log"
    sleep 10
  done
}

cleanup() {
  if [ -n "${MONITOR_PID:-}" ]; then
    kill "$MONITOR_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

{
  echo "recorded_at=$(date --iso-8601=seconds)"
  echo "workspace=$WORKSPACE"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
  git -C "$WORKSPACE/src" rev-parse HEAD 2>/dev/null || true
} >"$SESSION_DIR/session_info.txt"

monitor_graph &
MONITOR_PID=$!

echo "Recording VMR/Nav2 timing diagnostics to: $SESSION_DIR"
echo "Complete one full navigation run, then press Ctrl+C here."

ros2 bag record --output "$BAG_DIR" \
  /vmr_base_bridge/timing_diagnostics \
  /vmr_base_bridge/location \
  /vmr_base_bridge/pose \
  /vmr_base_bridge/odom \
  /estimated_pose \
  /estimated_odom \
  /tf \
  /tf_static \
  /rosout \
  /diagnostics \
  /cmd_vel_nav \
  /cmd_vel \
  /livox/lidar \
  /selected_lidar/points_stamped \
  /scan \
  /plan \
  /local_costmap/costmap \
  /local_costmap/published_footprint \
  /global_costmap/costmap \
  /global_costmap/published_footprint \
  /navigate_to_pose/_action/status \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/result
