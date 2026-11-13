#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
root="$PWD"
source /opt/ros/noetic/setup.bash
export PYTHONPATH="$root/runtime${PYTHONPATH:+:$PYTHONPATH}"
rosdep install --from-paths "$root/src" --ignore-src --rosdistro noetic -r -y
find "$root/src/tesse_ros_bridge" -type f -name '*.py' -exec chmod +x {} +
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build planner_pkg trajectory_generation controller_pkg tesse_ros_bridge --no-status -j2 -p2
