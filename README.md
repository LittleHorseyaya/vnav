# MIT VNAV：LAB3、LAB4 与 LAB5

ROS Noetic / Ubuntu 20.04 下的实验源码、运行说明和实测记录。

| 实验 | 内容 | 实测结果 | 入口 |
| --- | --- | --- | --- |
| LAB3 | 四旋翼几何控制、悬停与圆轨迹跟踪 | 悬停 RMS 0.260 m；圆轨迹 RMS 0.441 m | [源码与运行说明](lab3/README.md) |
| LAB4 | 单段/多段优化题解、minimum-snap 位置与连续 yaw 轨迹、12 门竞速 | 12 门按序到达；规划 84.468 s；跟踪 RMSE 0.0723 m | [源码、题解与运行说明](lab4/README.md) |
| LAB5 | SIFT/SURF/ORB 特征匹配与 Harris+LK 跟踪 | 两份指定 rosbag 分别完成 1583、1385 对相邻图像评测 | [源码、报告与运行说明](lab5/README.md) |

指标来自保存的实际 ROS 数据；两次实验的统计窗口与口径不同，详情见各实验说明。LAB3 原始测试存在高度波动；LAB4 起飞阶段有地板接触记录，飞行期间没有撞门记录。

## 内容导航

- LAB3：[控制器](lab3/controller_pkg/src/controller_node.cpp) · [飞行视频](lab3/media/rviz_drone_flight.mp4) · [验证数据](lab3/validation)
- LAB4：[轨迹生成](lab4/src/trajectory_generation_pkg/src/trajectory_generation_node.cpp) · [个人题解 PDF](lab4/theory/LAB4_individual.pdf) · [比赛视频](lab4/media/drone_race.mp4) · [rosbag](lab4/validation/race03/race.bag) · [离线核验](lab4/validation/race03/offline_validation.json)
- LAB5：[特征跟踪源码](lab5/feature_tracking/src) · [团队报告 PDF](lab5/team_report.pdf) · [个人答案 PDF 副本](lab5/individual_solutions.pdf) · [实验统计](lab5/feature_tracking/EXPERIMENTS.md)

各实验源码、依赖、脚本和结果保存在对应目录；LAB3、LAB4 还包含视频或 rosbag，LAB5 的原始大体积 rosbag 需另行下载。不包含 Unity 仿真器、虚拟机镜像或编译产物。各实验分别构建工作空间，避免重复 ROS 包名冲突。部分附带脚本按原实验虚拟机路径配置，移到其他机器需调整路径。

## 提交状态

- LAB3：小组代码和飞行验证已整理；已补齐 [个人题1 TXT](lab3/deliverable_1.txt) 和 [个人题2 PDF](lab3/deliverable_2.pdf)。
- LAB4：个人题解、团队代码、完整视频及 rosbag 已整理。
- LAB5：必做题 D1–D7 的代码、两份 PDF、结果图和实验统计已整理；FAST+BRIEF、稠密光流为未完成的可选加分项。原始大体积 rosbag 不在仓库中。
- LAB3、LAB4 个人交付另行整理至 [yawangjiang/Robot-Group-Individual](https://github.com/yawangjiang/Robot-Group-Individual)，按 `lab3/`、`lab4/` 分目录。LAB5 个人 PDF 副本放在本仓库 `lab5/`，仍需按课程要求提交到个人仓库。
- 两份视频链接文件已提供 GitHub 公开下载地址。课程原文指定 Google Drive/Dropbox，若严格按课程提交，仍需替换为认可的云盘链接；本仓库不代表课程系统已接收作业。

## 来源

- [LAB3 要求](https://vnav.mit.edu/labs_2023/lab3/exercises.html)
- [LAB4 要求](https://vnav.mit.edu/labs_2023/lab4/exercises.html)
- [LAB5 要求](https://vnav.mit.edu/labs_2023/lab5/exercises.html)
- [MIT-SPARK/VNAV-labs VNAV2021 代码与仿真器入口](https://github.com/MIT-SPARK/VNAV-labs/releases/tag/VNAV2021)

课程模板和第三方依赖保留原有版权及许可文件；没有为第三方代码重新指定许可证。
