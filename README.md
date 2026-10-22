# MIT VNAV：LAB3 与 LAB4

ROS Noetic / Ubuntu 20.04 下的实验源码、运行说明和实测记录。

| 实验 | 内容 | 实测结果 | 入口 |
| --- | --- | --- | --- |
| LAB3 | 四旋翼几何控制、悬停与圆轨迹跟踪 | 悬停 RMS 0.260 m；圆轨迹 RMS 0.441 m | [源码与运行说明](lab3/README.md) |
| LAB4 | 单段/多段优化题解、minimum-snap 位置与连续 yaw 轨迹、12 门竞速 | 12 门按序到达；规划 84.468 s；跟踪 RMSE 0.0723 m | [源码、题解与运行说明](lab4/README.md) |

指标来自保存的实际 ROS 数据；两次实验的统计窗口与口径不同，详情见各实验说明。LAB3 原始测试存在高度波动；LAB4 起飞阶段有地板接触记录，飞行期间没有撞门记录。

## 内容导航

- LAB3：[控制器](lab3/controller_pkg/src/controller_node.cpp) · [飞行视频](lab3/media/rviz_drone_flight.mp4) · [验证数据](lab3/validation)
- LAB4：[轨迹生成](lab4/src/trajectory_generation_pkg/src/trajectory_generation_node.cpp) · [个人题解 PDF](lab4/theory/LAB4_individual.pdf) · [比赛视频](lab4/media/drone_race.mp4) · [rosbag](lab4/validation/race03/race.bag) · [离线核验](lab4/validation/race03/offline_validation.json)

源码、依赖、脚本、视频和 rosbag 均在对应实验目录中；不包含 Unity 仿真器、虚拟机镜像或编译产物。分别构建 LAB3、LAB4 工作空间，避免重复 ROS 包名冲突。附带脚本按原实验虚拟机路径配置，移到其他机器需调整路径。

## 提交状态

- LAB3：小组代码和飞行验证已整理；个人理论 Deliverable 1、2 尚未包含。
- LAB4：个人题解、团队代码、完整视频及 rosbag 已整理。
- 两份视频链接文件已提供 GitHub 公开下载地址。课程原文指定 Google Drive/Dropbox，若严格按课程提交，仍需替换为认可的云盘链接；本仓库不代表课程系统已接收作业。

## 来源

- [LAB3 要求](https://vnav.mit.edu/labs_2023/lab3/exercises.html)
- [LAB4 要求](https://vnav.mit.edu/labs_2023/lab4/exercises.html)
- [MIT-SPARK/VNAV-labs VNAV2021 代码与仿真器入口](https://github.com/MIT-SPARK/VNAV-labs/releases/tag/VNAV2021)

课程模板和第三方依赖保留原有版权及许可文件；没有为第三方代码重新指定许可证。
