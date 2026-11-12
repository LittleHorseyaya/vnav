#!/usr/bin/env bash
# Run one tracker over a full ROS 1 bag and stop it after playback to print stats.

if [ "$#" -lt 3 ] || [ "$#" -gt 5 ]; then
  echo "Usage: $0 <SIFT|SURF|ORB|FAST|LK> <bag-path> <playback-rate> [log-dir] [image-topic]" >&2
  exit 2
fi

descriptor="$1"
bag_path="$2"
playback_rate="$3"
log_dir="${4:-.}"
image_topic="${5:-/camera/color/image_raw}"
mkdir -p "$log_dir"

source /opt/ros/noetic/setup.bash
source "$HOME/vnav_ws/devel/setup.bash"
set -eo pipefail

core_pid=""
tracker_pid=""
cleanup() {
  if [ -n "$tracker_pid" ] && kill -0 "$tracker_pid" 2>/dev/null; then
    rosnode kill /lab_5 >/dev/null 2>&1 || true
    wait "$tracker_pid" || true
  fi
  if [ -n "$core_pid" ]; then
    kill -TERM "$core_pid" 2>/dev/null || true
    wait "$core_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! rosparam get /rosversion >/dev/null 2>&1; then
  roscore > "$log_dir/roscore.log" 2>&1 &
  core_pid=$!
  for attempt in $(seq 1 30); do
    if rosparam get /rosversion >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
fi
rosparam get /rosversion >/dev/null

rosrun lab_5 track_features __name:=lab_5 _mode:=1 \
  _descriptor:="$descriptor" _save_outputs:=false _visualize:=false \
  /images_topic:=/camera/color/image_raw > "$log_dir/tracker.log" 2>&1 &
tracker_pid=$!

rosbag play --clock --delay=2 --rate="$playback_rate" "$bag_path" \
  "$image_topic:=/camera/color/image_raw" --topics "$image_topic" \
  > "$log_dir/rosbag.log" 2>&1

# Let the subscriber finish its remaining queued messages before shutdown.
sleep 2
rosnode kill /lab_5 >/dev/null
wait "$tracker_pid"
tracker_pid=""

grep -E '^Avg\.|^Num\. of samples:' "$log_dir/tracker.log"
