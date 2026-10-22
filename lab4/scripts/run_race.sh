#!/bin/bash
set -e
source /home/han/lab4_ws/devel/setup.bash
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export DISABLE_ROS1_EOL_WARNINGS=1
run=${1:-race01}
if [[ ! "$run" =~ ^[A-Za-z0-9_-]+$ ]]; then exit 2; fi
out=/home/han/lab4_results/$run
if [ -f "$out/race_result.json" ]; then echo 'Existing completed attempt will not be overwritten.';exit 3;fi
mkdir -p "$out"
exec > "$out/run.log" 2>&1
export LAB4_RUN_DIR="$out"
pkill -INT -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch (planner_pkg|trajectory_generation) ' || true
sleep 3
rosnode kill /controller_node /simple_traj_planner /traj_vertices_publisher /trajectory_generation_node /lab4_race_monitor || true
rosparam delete /trajectory_generation_node || true
sleep 2
python3 /home/han/lab4_results/reset_race.py
rosbag record --lz4 -O "$out/race.bag" /current_state /desired_state /desired_traj_vertices /clock /tesse/collision /tf > "$out/rosbag.log" 2>&1 &
bag_pid=$!
size=$(xdpyinfo | awk '/dimensions:/ {print $2;exit}')
win=$(xdotool search --onlyvisible --name '^tesse_vnav_scenes$' | head -1 || true)
if [ -n "$win" ];then
    xdotool windowactivate --sync "$win"
    xdotool windowsize "$win" 1280 720
    xdotool windowmove "$win" 70 40
fi
ffmpeg -y -f x11grab -video_size "$size" -framerate 15 -i :0.0 -vf 'scale=1280:-2' -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -movflags +faststart "$out/drone_race.mp4" > "$out/video.log" 2>&1 &
video_pid=$!
finish() {
    kill -INT "$bag_pid" "$video_pid" 2>/dev/null || true
    wait "$bag_pid" "$video_pid" 2>/dev/null || true
}
trap finish EXIT
python3 /home/han/lab4_results/race_monitor.py > "$out/monitor.log" 2>&1 &
monitor_pid=$!
roslaunch trajectory_generation traj_following.launch v_max:=${2:-3} a_max:=${3:-2} time_scale:=${4:-1.3} > "$out/follower.log" 2>&1 &
echo $! > "$out/follower.pid"
sleep 1
roslaunch planner_pkg traj_gen.launch > "$out/waypoints.log" 2>&1 &
echo $! > "$out/waypoints.pid"
wait "$monitor_pid"
echo RACE_RECORDING_FINISHED
