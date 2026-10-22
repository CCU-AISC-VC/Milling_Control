#!/usr/bin/env bash
# M4: play a trajectory folder on the ros2_control mock hardware and record joint states.
# Usage: scripts/m4_run_mock_test.sh <trajectory_dir> <session_out_dir> [speed_scale]
# Needs ROS 2 Jazzy and a built ros2_ws (colcon build). Everything is killed at the end.
set -eo pipefail
TRAJ=$(realpath "$1"); OUT=$(realpath -m "$2"); SCALE=${3:-1.0}
REPO=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-37}

# `ros2 run` / `ros2 launch` are wrappers: signal the real processes / the launch process group.
# Patterns use the [x] trick so pkill never matches this script's own command line.
cleanup() {
  pkill -INT -f "[m]illing_bridge/joint_state_recorder" 2>/dev/null || true
  kill -INT -- -${MOCK_PGID:-0} 2>/dev/null || true
  sleep 3
  pkill -9 -f "[m]illing_bridge/joint_state_recorder" 2>/dev/null || true
  kill -9 -- -${MOCK_PGID:-0} 2>/dev/null || true
  pkill -9 -f "[r]os2_control_node" 2>/dev/null || true
  pkill -9 -f "[r]obot_state_publisher --ros-args" 2>/dev/null || true
}
trap cleanup EXIT

setsid ros2 launch milling_bridge mock_hardware.launch.py > "$OUT/mock_hardware.log" 2>&1 &
MOCK_PID=$!; MOCK_PGID=$MOCK_PID
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then break; fi
  sleep 1
done
ros2 action list | grep -q "/arm_controller/follow_joint_trajectory" || { echo "mock hardware did not come up"; exit 3; }
echo "mock hardware up after ${i}s"

ros2 run milling_bridge joint_state_recorder --ros-args -p output_dir:="$OUT" > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 2
ros2 run milling_bridge trajectory_player --ros-args -p trajectory_dir:="$TRAJ" -p dry_run:=false -p speed_scale:="$SCALE" -p allow_fast:=true -p start_min_duration:=2.0 2>&1 | tee "$OUT/player.log"
sleep 1
pkill -INT -f "[m]illing_bridge/joint_state_recorder" || true
for i in $(seq 1 10); do pgrep -f "[m]illing_bridge/joint_state_recorder" > /dev/null || break; sleep 1; done
echo "done: $(wc -l < "$OUT/joint_states.csv") joint-state rows"
