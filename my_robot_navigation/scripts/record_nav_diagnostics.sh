#!/usr/bin/env bash

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
LOG_ROOT="${LOG_ROOT:-$WORKSPACE/runs/nav_diagnostics}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${1:-$LOG_ROOT/$STAMP}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
HZ_SAMPLE_SECONDS="${HZ_SAMPLE_SECONDS:-6}"
HZ_SLEEP_SECONDS="${HZ_SLEEP_SECONDS:-10}"
DELAY_SAMPLE_SECONDS="${DELAY_SAMPLE_SECONDS:-6}"
DELAY_SLEEP_SECONDS="${DELAY_SLEEP_SECONDS:-20}"
ECHO_SLEEP_SECONDS="${ECHO_SLEEP_SECONDS:-5}"

mkdir -p "$LOG_DIR"

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

PIDS=()
CLEANED_UP=0

run_bg() {
  local name="$1"
  shift
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
}

loop_topic_hz() {
  local topic="$1"
  while true; do
    echo
    echo "===== $(date --iso-8601=seconds) $topic hz ====="
    timeout "$HZ_SAMPLE_SECONDS" ros2 topic hz "$topic"
    sleep "$HZ_SLEEP_SECONDS"
  done
}

loop_topic_delay() {
  local topic="$1"
  while true; do
    echo
    echo "===== $(date --iso-8601=seconds) $topic delay ====="
    timeout "$DELAY_SAMPLE_SECONDS" ros2 topic delay "$topic"
    sleep "$DELAY_SLEEP_SECONDS"
  done
}

loop_topic_echo_once() {
  local topic="$1"
  while true; do
    echo
    echo "===== $(date --iso-8601=seconds) $topic echo once ====="
    timeout 3 ros2 topic echo --once "$topic"
    sleep "$ECHO_SLEEP_SECONDS"
  done
}

loop_tf_monitor() {
  while true; do
    echo
    echo "===== $(date --iso-8601=seconds) tf2_monitor map base_footprint ====="
    timeout 12 ros2 run tf2_ros tf2_monitor map base_footprint
    sleep 3
  done
}

loop_system_stats() {
  while true; do
    echo
    echo "===== $(date --iso-8601=seconds) system ====="
    uptime
    free -h
    ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -30
    sleep 2
  done
}

loop_ros_snapshot() {
  sleep 10
  while true; do
    echo
    echo "===== $(date --iso-8601=seconds) topic list ====="
    timeout 8 ros2 topic list -t || true
    echo
    echo "===== $(date --iso-8601=seconds) velocity topic info ====="
    timeout 8 ros2 topic info -v /cmd_vel_nav || true
    echo
    timeout 8 ros2 topic info -v /cmd_vel || true
    echo
    timeout 8 ros2 topic info -v /vmr_base_bridge/pose || true
    echo
    timeout 8 ros2 topic info -v /vmr_base_bridge/odom || true
    echo
    timeout 8 ros2 topic info -v /estimated_pose || true
    echo
    timeout 8 ros2 topic info -v /estimated_odom || true
    echo
    echo "===== $(date --iso-8601=seconds) node list ====="
    timeout 8 ros2 node list || true
    echo
    echo "===== $(date --iso-8601=seconds) controller params ====="
    timeout 8 ros2 param dump /controller_server || true
    echo
    echo "===== $(date --iso-8601=seconds) velocity smoother params ====="
    timeout 8 ros2 param dump /velocity_smoother || true
    sleep 60
  done
}

cleanup() {
  if [ "$CLEANED_UP" -eq 1 ]; then
    return
  fi
  CLEANED_UP=1
  echo "Stopping diagnostics..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "Logs saved to: $LOG_DIR"
}

terminate() {
  cleanup
  exit 0
}

trap cleanup EXIT
trap terminate INT TERM

{
  echo "stamp=$STAMP"
  echo "workspace=$WORKSPACE"
  echo "log_dir=$LOG_DIR"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -a)"
  echo
  echo "===== env ROS ====="
  env | sort | grep -E '^(ROS|RMW|CYCLONE|FAST|AMENT|COLCON)_' || true
} >"$LOG_DIR/session_info.log"

{
  echo "===== git status ====="
  git -C "$WORKSPACE/src" status --short || true
  echo
  echo "===== navigation config snapshot ====="
  sed -n '1,380p' "$WORKSPACE/src/my_robot_navigation/config/real_nav2_no_odom_mppi.yaml" || true
} >"$LOG_DIR/config_snapshot.log" 2>&1

run_bg system_stats loop_system_stats
run_bg tf_monitor loop_tf_monitor
run_bg ros_snapshot loop_ros_snapshot

for topic in /cmd_vel_nav /cmd_vel /vmr_base_bridge/pose /vmr_base_bridge/odom /estimated_pose /estimated_odom /scan /local_costmap/costmap_raw; do
  safe_name="${topic#/}"
  safe_name="${safe_name//\//_}"
  run_bg "${safe_name}_hz" loop_topic_hz "$topic"
done

for topic in /vmr_base_bridge/pose /vmr_base_bridge/odom /estimated_pose /estimated_odom /scan; do
  safe_name="${topic#/}"
  safe_name="${safe_name//\//_}"
  run_bg "${safe_name}_delay" loop_topic_delay "$topic"
done

for topic in /cmd_vel_nav /cmd_vel /vmr_base_bridge/pose /vmr_base_bridge/odom /estimated_pose /estimated_odom; do
  safe_name="${topic#/}"
  safe_name="${safe_name//\//_}"
  run_bg "${safe_name}_echo" loop_topic_echo_once "$topic"
done

echo "Recording Nav2 diagnostics."
echo "Log dir: $LOG_DIR"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "Press Ctrl+C after the test run."

while true; do
  sleep 3600
done
