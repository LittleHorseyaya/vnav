# VNAV Lab 4: trajectory optimization and drone racing

This directory contains the motor-driven racing implementation: continuous
segment timing, bounded local retiming, a smooth path clock and delayed
attitude feedforward. The team achieved a fastest complete lap of **7.44 s**,
with successful fast trials approximately **7.44-7.58 s**. The retained source
snapshot and launch configuration are associated with the representative
**7.58 s** recorded run; an exact source revision for the 7.44 s trial was not
identified. A newly recorded complete flight achieved **7.56 s**, with all twelve
gates passed. Its on-screen timer displays **7563.312 ms**. The video and the
retained 7.58 s recording are from separate flights. A new run may yield a
different time.

## Requirements

Ubuntu 20.04 x86-64, ROS Noetic, Python 3.8 and a graphical desktop with working
3D acceleration. Install ROS Noetic first. Use a separate workspace for this
experiment. The original Lab 4 simulator must be available at
`~/vnav-builds/lab4`, containing `lab4.x86_64` and `lab4_Data/`.
The simulator is distributed separately from the Git source tree. If stored
elsewhere, set `LAB4_SIM_DIR` to its absolute directory in each terminal.

## Build

Clone the repository, then run from its `lab4` directory:

```bash
sudo apt update
sudo apt install python3-catkin-tools python3-rosdep python3-tk \
  python3-numpy python3-scipy python3-pil python3-defusedxml \
  libgoogle-glog-dev libnlopt-cxx-dev libyaml-cpp-dev libeigen3-dev \
  ros-noetic-ackermann-msgs ros-noetic-cv-bridge
```

If rosdep has never been initialized, run `sudo rosdep init` once. Then:

```bash
rosdep update
cd ~/vnav/lab4
bash setup.sh
```

Replace `~/vnav/lab4` with your clone's location. The script builds this
folder as a catkin workspace, uses its `src/`, and loads the included TESSE
Python interface from `runtime/`. It does not depend on `/home/han` or another
user's workspace. Start fresh terminals for the following commands.

## Run

Each terminal first runs `cd ~/vnav/lab4` (adjust the clone location).

| Terminal | Command | Purpose |
| --- | --- | --- |
| 1 | `bash run.sh simulator` | Start the original simulator. |
| 2 | `bash run.sh bridge` | Connect ROS to the simulator. |
| 3 | `bash run.sh timer` | Start the external race timer. |
| 4 | `bash run.sh follower` | Start trajectory generation and the motor controller. |
| 5 | `bash run.sh gates` | Publish the ordered gates and trigger planning. |

Wait for bridge initialization and press R in the simulator before starting
Terminal 4. Start Terminal 5 only after Terminal 4 is ready. To repeat, stop
Terminals 4 and 5 with Ctrl+C, reset with R, then restart 4 followed by 5.
Use `bash run.sh record` in an optional sixth terminal before gate publication;
stop it with Ctrl+C after the flight. New bags and parameter dumps go to
`reproduced/`.

The follower explicitly selects this configuration:

```bash
timing_mode:=continuous
v_max:=21 a_max:=16 lateral_accel:=8
time_scale:=1.27 first_gate_vz:=3
segment_time_scales:="[]" corner_gate:=0
attitude_ff_gain:=0.25 attitude_ff_start:=2.0
attitude_ff_ramp:=0.5 attitude_ff_max_torque:=2
retime:=true retime_ratio:=0.93 retime_passes:=12
finish_extension:=0 use_identified_inertia:=false
path_speedup:=1.10 path_ramp:=1.0 learning_gain:=0
```

Controller gains are `kx=1`, `kv=2`, `kr=4`, `komega=1`, `kx_z=4`, `kv_z=4`.
Running `traj_following.launch` without these trajectory arguments selects its
legacy defaults, rather than this saved racing configuration. Ground/wall
contacts occurred in aggressive fast trials; the historical low-error race
below is a different experiment and must not be combined with these times.

## Files

- `src/controller_pkg/`: geometric controller and attitude feedforward.
- `src/trajectory_generation_pkg/`: trajectory planning, timing and sampling;
  includes `src/lab4_attitude_ff.h`.
- `src/planner_pkg/`: gate publication and course information.
- `src/tesse_ros_bridge/`: ROS interface to the simulator.
- `src/dependencies/`: source dependencies with their original notices.
- `src/race_timer_gui.py`: external timer.
- `runtime/tesse/`: TESSE Python interface.
- `setup.sh`, `run.sh`: portable build and process entry points.

## Recorded data and historical material

The current experiment record is stored in `validation/fast_run/`:

| File | Content |
| --- | --- |
| `flight.bag` | Representative flight recorded with a 7.58 s external-timer result. |
| `metrics.json` | Statistics computed from that recording. |
| `trajectory.pdf` | Reference and measured XY trajectories. |
| `flight_profile.pdf` | Altitude, speed and reference acceleration. |
| `position_error.pdf` | Position tracking error. |
| `motor_commands.pdf` | Signed commands sent to the four rotors. |

The recording was previously named `path110.bag`; it has only been renamed,
not re-recorded or modified. Existing references to that filename in the
report and historical logs identify this same recording. The external
historical download keeps its original filename:


[Flight-data download page](https://vnav-lab4-group14-flight-data.civilsycamore8.chatgpt.site)

[Direct path110.bag download](https://vnav-lab4-group14-flight-data.civilsycamore8.chatgpt.site/path110.bag)

The historical slower trial is stored in `validation/early_run/`; the initial
fixed-point test remains in `validation/static/`. Its screenshots are
`images/early_run_simulator.png` and `images/early_run_tracking.png`. The
`scripts/` directory retains the old tools. These are historical materials
from the slower configuration. Its reported 81.200 s gate-arrival time,
84.468 s planned duration and approximately 0.0723 m tracking RMSE do not
describe either the retained fast-run recording or the new video. The old
slow-flight video has been replaced in the current tree; earlier versions
remain available in Git history.
The old scripts contain machine-specific paths; use the root `run.sh` above
for the updated configuration. The stored build log also predates this update.

This source update was compared against the retained reproduction archive.
The new portable launch wrappers still require a build and flight check on
Ubuntu/ROS Noetic before claiming a new reproduction result.

## Report and flight video

[Experiment report](report.pdf) combines the individual derivations, personal
contribution and group experiment. It reports the fastest observed lap of
7.44 s separately from the retained 7.58 s recording/configuration.

[Watch the 7.56 s flight](media/fast_run.mp4) ·
[Download the MP4](https://raw.githubusercontent.com/LittleHorseyaya/vnav/main/lab4/media/fast_run.mp4)

The video records a complete flight through all twelve gates. The on-screen
timer finishes at **7563.312 ms (7.56 s rounded)**. It uses the original
motor-driven simulator and the retained racing configuration. The MP4 is a
lossless remux of the OBS recording: its full approximately 25 s recording,
encoded video stream is preserved, with no speed change or trimming.

This video's 7.56 s lap, the previously saved 7.58 s rosbag and the team's
fastest 7.44 s lap are separate trials. The tracking plots and `metrics.json`
still describe the original 7.58 s recording; they are not measurements of
this video's flight. The report remains the existing experiment report and
predates this new video. `media/links.txt` provides the video and data links.

## References

- [Official Lab 4 exercises](https://vnav.mit.edu/labs_2023/lab4/exercises.html)
- [Course source release](https://github.com/MIT-SPARK/VNAV-labs/releases/tag/VNAV2021)

Third-party source retains its original copyright and license notices.
