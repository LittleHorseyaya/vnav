# MIT VNAV LAB5：视觉特征匹配与跟踪

本目录按 [MIT LAB5 题目](https://vnav.mit.edu/labs_2023/lab5/exercises.html) 整理。必做题 D1–D7 和两项可选加分内容（D6 FAST+BRIEF、D8 稠密光流）均已实现。实测环境为 Ubuntu 20.04、ROS Noetic 和课程 VNAV2021 代码。

## 交付内容

| 内容 | 对应题目 |
| --- | --- |
| [`individual_solutions.pdf`](individual_solutions.pdf)、[`INDIVIDUAL_SOLUTIONS.md`](INDIVIDUAL_SOLUTIONS.md) | D1 球体透视投影；D2 消失点推导。PDF 是排版后的个人答案副本；课程要求个人答案另交个人仓库。 |
| [`team_report.pdf`](team_report.pdf) | D3–D8 的关键点、匹配、RANSAC、Harris+LK 和稠密光流图；示例图及两组真实数据的统计表、分析。 |
| [`feature_tracking/src/`](feature_tracking/src/) 与 [`feature_tracking/include/`](feature_tracking/include/) | SIFT、SURF、ORB、FAST+BRIEF、Harris+LK、Farneback 稠密光流算法及 ROS 节点。 |
| [`feature_tracking/launch/`](feature_tracking/launch/)、[`feature_tracking/run_bag_benchmark.sh`](feature_tracking/run_bag_benchmark.sh)、[`feature_tracking/run_dense_flow.sh`](feature_tracking/run_dense_flow.sh) | 双图与 rosbag 运行方式；脚本在数据包播放完毕后保存日志与平均统计。 |
| [`feature_tracking/results/`](feature_tracking/results/)、[`feature_tracking/EXPERIMENTS.md`](feature_tracking/EXPERIMENTS.md) | 原始匹配图和稠密光流图、`metrics.json`、参数、完整帧对计数与结果解释。 |
| [`dependencies/`](dependencies/) | `catkin_simple`、`gflags_catkin`、`glog_catkin`、`opencv3_catkin` 四个构建依赖。本目录只保留一份各依赖，避免 catkin 包名冲突。 |

`feature_tracking/images/` 是课程提供的 `box.png` 与 `box_in_scene.png`。`generate_individual_pdf.py`、`generate_team_pdf.py` 为报告生成源码；提交用 PDF 已生成。仓库不含体积较大的原始 `.bag` 数据，也不含虚拟机、编译产物或 SSH 凭据。

## 构建与运行

在 Ubuntu 20.04 的 ROS Noetic 环境安装 `catkin_tools` 及课程包所需 ROS 依赖。实测虚拟机还需要 `libtool libgtk2.0-dev pkg-config`。本目录可直接作为独立 catkin 工作区的来源：

```bash
source /opt/ros/noetic/setup.bash
LAB5_ROOT=/absolute/path/to/vnav/lab5
mkdir -p ~/vnav_ws/src
ln -s "$LAB5_ROOT/dependencies/catkin_simple" ~/vnav_ws/src/catkin_simple
ln -s "$LAB5_ROOT/dependencies/gflags_catkin" ~/vnav_ws/src/gflags_catkin
ln -s "$LAB5_ROOT/dependencies/glog_catkin" ~/vnav_ws/src/glog_catkin
ln -s "$LAB5_ROOT/dependencies/opencv3_catkin" ~/vnav_ws/src/opencv3_catkin
ln -s "$LAB5_ROOT/feature_tracking" ~/vnav_ws/src/feature_tracking
cd ~/vnav_ws
catkin build lab_5 -j4
source devel/setup.bash
roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT
```

已有同名包时不要重复创建链接。`descriptor` 可改为 `SURF`、`ORB`、`FAST`；视频模式还支持 `LK`。Farneback 稠密光流使用独立的 `dense_flow.launch` 或 `run_dense_flow.sh`。完整运行参数、图像话题、输出文件以及 OpenCV 版本注意事项见 [`feature_tracking/README.md`](feature_tracking/README.md)。

两份指定数据集分别是 `30fps_424x240_2018-10-01-18-35-06.bag` 和 `vnav-lab5-smooth-trajectory.bag`，可从 [课程数据仓库](https://github.com/MIT-SPARK/VNAV-lab-data) 获取。实测统计覆盖前者 1583 对、后者 1385 对相邻图像；播放速度根据虚拟机处理能力降低，以避免漏帧。

本目录汇集个人与团队材料供核对。课程页面要求个人数学答案推送到个人仓库，团队源码与讨论 PDF 推送到团队仓库；仅在此仓库保留个人 PDF 副本不等于完成个人仓库提交。
