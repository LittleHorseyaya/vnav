# MIT VNAV LAB3 小组控制器与飞行验收

本文件主要说明小组 Deliverable 3。个人部分已另行补齐：[Deliverable 1 TXT](deliverable_1.txt)、[Deliverable 2 PDF](deliverable_2.pdf)，以及 [可编辑源码](theory/source/)。个人交付仓库：[yawangjiang/Robot-Group-Individual](https://github.com/yawangjiang/Robot-Group-Individual)。

## 已配置的虚拟机

- VMware：E 盘的 Ubuntu 虚拟机。
- Ubuntu 20.04.6、ROS Noetic。
- 独立工作区：`/home/han/lab3_ws`，未覆盖原来的 `vnav_ws`。
- 控制器：`/home/han/lab3_ws/src/controller_pkg`。
- 官方仿真器：`/home/han/vnav-builds/lab3/lab3.x86_64`。
- 测试工具和记录：`/home/han/lab3_results`。

## 代码变更

保留正确的向上重力补偿、姿态误差、世界系角速度到机体系的转换、推力投影、X 形电机分配和有符号转速计算。补上缺失参数的默认值、期望姿态叉积退化保护和构建依赖声明。轨迹发布器增加 `static_pose` 参数，便于在悬停与圆轨迹间切换。

最终增益：`kx=1`、`kv=2`、`kr=1`、`komega=0.5`、`kx_z=4`、`kv_z=4`。控制频率为 100 Hz。保留官方允许的简化 `eomega=omega`；这些增益来自本机仿真实测，不保证换电脑后无需微调。

## 重新运行

终端 1，启动官方仿真器。启动参数用于避开本机旧版 Unity 的音频初始化和调试监听问题：

```bash
cd ~/vnav-builds/lab3
./lab3.x86_64 -noaudio -disableManagedDebugger -screen-fullscreen 0 -screen-width 960 -screen-height 540
```

仿真器启动后，在终端 2 启动桥接：

```bash
source ~/lab3_ws/devel/setup.bash
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch
```

终端 3，运行圆轨迹：

```bash
source ~/lab3_ws/devel/setup.bash
rosparam set /static_pose false
roslaunch controller_pkg traj_tracking.launch
```

如需悬停，将 `/static_pose` 设为 `true` 后重新启动轨迹节点。悬停目标为 `(0,0,2 m)`、偏航 45°。圆轨迹半径为 5 m、高度为 2 m、周期约 12.57 秒，未降低原轨迹速度。

终端 4，显示 RViz 和测量轨迹：

```bash
source ~/lab3_ws/devel/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1
LAB3_DURATION=300 python3 ~/lab3_results/track_metrics.py &
rviz -d ~/lab3_ws/src/controller_pkg/rviz/tracking.rviz
```

青色表示期望轨迹，橙色表示实际轨迹；TF 显示期望和实际机体姿态。测量程序结束后路径停止更新，TF 仍随飞行更新。

## 验证文件与限制

`validation/` 中保留真实 ROS 测量 CSV、误差统计和图，包含起飞及进入圆轨迹的过渡过程。原始 `metrics.json` 的统计窗口从测量开始后的第 25 秒墙钟时间算起；ROS 仿真时间可能比墙钟时间慢。误差是当前状态位置与最近收到的期望位置之差，包含消息传输与调度延迟的影响。

视频是 VMware 内 RViz 的实际录屏。圆轨迹闭环飞行已验证，但轨迹仍有可见误差与高度波动，不代表零误差跟踪或课程评分保证。

本目录已按仓库结构整理：`controller_pkg/` 为源码，`tools/` 为原虚拟机测试脚本，`validation/` 为编译日志和测量记录。

- [实际 RViz 飞行视频](media/rviz_drone_flight.mp4)（[直接下载](https://raw.githubusercontent.com/LittleHorseyaya/vnav/main/lab3/media/rviz_drone_flight.mp4)）
- [悬停测量](validation/hover/metrics.json) · [圆轨迹测量](validation/circle/metrics.json)
- [视频链接文件](rviz_drone_flight.txt)

原测试脚本使用 `/home/han/lab3_ws` 和 `/home/han/lab3_results` 等当前虚拟机路径；在其他环境运行前需修改。下文指标来自原始测试，LAB4 开启 VMware 3D 加速后没有重新测试 LAB3。

课程原文指定 Google Drive/Dropbox 公开视频链接。链接文件现提供 GitHub 公开下载地址；严格按课程提交时，仍需改为课程认可的云盘链接。个人题文件虽然在此提供副本，各成员仍应按课程要求分别提交到自己的个人仓库。

实验要求：https://vnav.mit.edu/labs_2023/lab3/exercises.html

## 本机最终测量结果

- 最终交付版本已通过 catkin 编译，无警告。
- 悬停记录 2252 个状态样本，统计窗口 RMS 位置误差 0.260 m。
- 圆轨迹记录 2709 个状态样本，统计窗口覆盖 38.62 秒仿真时间（约 3.07 圈）。
- 圆轨迹统计窗口 RMS 位置误差 0.441 m，最大误差 1.281 m，高度范围 1.391–1.979 m。
- 视频 50 秒、15 fps；实际原始数据与统计口径见 validation。
