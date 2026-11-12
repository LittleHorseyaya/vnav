#!/usr/bin/env bash
# Compute dense Farneback flow on a ROS bag and save one representative pair.
set -eo pipefail

if [ "$#" -lt 4 ] || [ "$#" -gt 5 ]; then
  echo "Usage: $0 <bag-path> <playback-rate> <output-dir> <save-pair> [image-topic]" >&2
  exit 2
fi
bag_path="$1"
playback_rate="$2"
output_dir="$3"
save_pair="$4"
image_topic="${5:-/camera/color/image_raw}"
mkdir -p "$output_dir"

source /opt/ros/noetic/setup.bash
source "$HOME/vnav_ws/devel/setup.bash"
core_pid=""
flow_pid=""
cleanup() {
  if [ -n "$flow_pid" ] && kill -0 "$flow_pid" 2>/dev/null; then
    rosnode kill /dense_flow >/dev/null 2>&1 || true
    wait "$flow_pid" || true
  fi
  if [ -n "$core_pid" ]; then
    kill -TERM "$core_pid" 2>/dev/null || true
    wait "$core_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! rosparam get /rosversion >/dev/null 2>&1; then
  roscore > "$output_dir/roscore.log" 2>&1 &
  core_pid=$!
  for attempt in $(seq 1 30); do
    if rosparam get /rosversion >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
rosparam get /rosversion >/dev/null

rosrun lab_5 self_flow __name:=dense_flow _visualize:=false \
  _save_pair:="$save_pair" _output_dir:="$output_dir" \
  /images_topic:=/camera/color/image_raw > "$output_dir/flow.log" 2>&1 &
flow_pid=$!

rosbag play --clock --delay=2 --rate="$playback_rate" "$bag_path" \
  "$image_topic:=/camera/color/image_raw" --topics "$image_topic" \
  > "$output_dir/rosbag.log" 2>&1
sleep 2
rosnode kill /dense_flow >/dev/null
wait "$flow_pid"
flow_pid=""

grep -E 'Dense flow pairs:|Avg\. mean motion|Saved dense flow pair' \
  "$output_dir/flow.log"
