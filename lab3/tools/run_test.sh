#!/bin/bash
set -e
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export DISABLE_ROS1_EOL_WARNINGS=1
source /home/han/lab3_ws/devel/setup.bash
rosnode kill /controller_node /traj_publisher || true
pkill -TERM -f '^rviz -d /home/han/lab3_ws/src/controller_pkg/rviz/tracking.rviz$' || true
rosservice call /tesse/reposition_request '{pose: {position: {x: 0, y: 0.3, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}'
mode=${1:-hover}
out=/home/han/lab3_results/$mode
mkdir -p "$out"
exec > "$out/run.log" 2>&1
if [ "$mode" = hover ]; then
    rosparam set /static_pose true
    export LAB3_DURATION=65
else
    rosparam set /static_pose false
    export LAB3_DURATION=85
fi
export LAB3_RESULTS="$out"
gnome-session-inhibit --inhibit idle --reason 'LAB3 flight validation' sleep 600 &
roslaunch controller_pkg traj_tracking.launch > "$out/controller.log" 2>&1 &
echo $! > "$out/controller-launch.pid"
python3 /home/han/lab3_results/track_metrics.py > "$out/metrics.log" 2>&1 &
echo $! > "$out/metrics.pid"
rviz -d /home/han/lab3_ws/src/controller_pkg/rviz/tracking.rviz > "$out/rviz.log" 2>&1 &
echo $! > "$out/rviz.pid"
if [ "$mode" = circle ]; then
    (sleep 30; /bin/bash /home/han/lab3_results/record_flight.sh) > /tmp/lab3-record-schedule.log 2>&1 &
fi
echo TEST_STARTED
