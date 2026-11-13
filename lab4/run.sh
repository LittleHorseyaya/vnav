#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
root="$PWD"
sim_dir="${LAB4_SIM_DIR:-$HOME/vnav-builds/lab4}"
export PYTHONPATH="$root/runtime${PYTHONPATH:+:$PYTHONPATH}"
mode="${1:-help}"
if [ "$mode" = simulator ]; then
  if [ ! -f "$sim_dir/lab4.x86_64" ]; then
    echo "Place the original Lab 4 simulator in $sim_dir or set LAB4_SIM_DIR." >&2
    exit 1
  fi
  cd "$sim_dir"
  chmod +x ./lab4.x86_64
  exec ./lab4.x86_64
fi
if [ "$mode" = help ]; then
  echo "Usage: bash run.sh simulator|bridge|timer|follower|gates|record"
  exit 0
fi
if [ ! -f "$root/devel/setup.bash" ]; then
  echo "Run bash setup.sh first." >&2
  exit 1
fi
source "$root/devel/setup.bash"
case "$mode" in
  bridge) exec roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch ;;
  timer) exec python3 "$root/src/race_timer_gui.py" ;;
  follower)
    exec roslaunch trajectory_generation traj_following.launch \
      timing_mode:=continuous \
      v_max:=21 a_max:=16 lateral_accel:=8 \
      time_scale:=1.27 first_gate_vz:=3 \
      segment_time_scales:="[]" corner_gate:=0 \
      attitude_ff_gain:=0.25 attitude_ff_start:=2.0 \
      attitude_ff_ramp:=0.5 attitude_ff_max_torque:=2 \
      retime:=true retime_ratio:=0.93 retime_passes:=12 \
      finish_extension:=0 use_identified_inertia:=false \
      path_speedup:=1.10 path_ramp:=1.0 learning_gain:=0
    ;;
  gates)
    if [ ! -d "$sim_dir/lab4_Data" ]; then
      echo "Lab 4 simulator data not found in $sim_dir." >&2; exit 1
    fi
    exec roslaunch planner_pkg traj_gen.launch "simulator_data_directory:=$sim_dir/lab4_Data"
    ;;
  record)
    mkdir -p "$root/reproduced"
    stamp="$(date +%Y%m%d_%H%M%S)"
    rosparam dump "$root/reproduced/parameters_$stamp.yaml"
    exec rosbag record -O "$root/reproduced/run_$stamp.bag" \
      /clock /current_state /desired_state /desired_attitude_ff \
      /desired_traj_vertices /rotor_speed_cmds /tesse/collision
    ;;
  *) echo "Unknown command: $mode" >&2; exit 2 ;;
esac
