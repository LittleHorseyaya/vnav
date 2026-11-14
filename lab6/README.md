# LAB6：视觉相对位姿估计

作者：[LittleHorseyaya](https://github.com/LittleHorseyaya)。基于 MIT VNAV 2023 ROS1 实验骨架完成；课程模板与第三方代码保留原有版权和署名。

本目录包含小组实验 D3–D5 的 **ROS1 C++ 源码、RViz 演示视频、误差曲线、五随机种子原始结果与复现脚本**。**实验报告暂不上传，个人 D1/D2 不在本次范围内。**

## 快速导航

- [主节点源码](src/pose_estimation.cpp) · [离线共同输入求解器](src/cache_eval.cpp)
- [Arun RViz 演示视频](media/Lab6_Arun_RViz.mp4)
- [2D 旋转曲线](figures/error_curves/2d_rotation_seed12345.png) · [2D 平移方向曲线](figures/error_curves/2d_translation_seed12345.png)
- [Arun 旋转曲线](figures/error_curves/arun_rotation_seed12345.png) · [Arun 米制平移曲线](figures/error_curves/arun_translation_seed12345.png)
- [五种子误差汇总图](figures/robustness/across_seed_mean_std.png) · [逐种子表现](figures/robustness/per_seed_errors.png) · [失败计数](figures/robustness/failure_counts.png)
- [中文统计表](results/tables/summary_zh.csv) · [每轮统计](results/tables/per_run_metrics.csv) · [20 份逐帧 CSV](results/seed_runs)

## 已实现

- 使用 Lab5 的 SIFT/FLANN 匹配；调用 `calibrateKeypoints` 去畸变并转换单位 bearing 向量。
- Nistér 五点、Longuet-Higgins 八点、已知旋转两点、Arun 三点配准，均使用 OpenGV RANSAC。
- 两点法固定真值相对旋转，以全部内点 SVD 重估平移方向，不使用真值平移选择符号。
- 缓存机体真值，按图像时间对位置线性插值、旋转 SLERP，再应用相机外参；等待两侧真值，不外推。
- RGB/深度时间差检查、深度有限值/正值/边界检查、失败帧处理、RPE CSV 和输入缓存。
- 离线工具在相同缓存帧对上运行四种方法；独立子进程提供每帧、每方法 2 秒超时保护，超时保留为失败行。

## 最终共同输入实验

相同的 **3,889 组连续帧对、1,400,564 组 SIFT 对应点**，预先指定种子 `12345, 23456, 34567, 45678, 56789`。五点/八点均采用 bearing 残差阈值 `1e-6` 并在内点上精化；两点阈值 `1e-6`；Arun 阈值 `0.05 m`；RANSAC 上限 1,000。

| 方法 | 全程平均平移误差的跨种子均值 ± 样本标准差 | 失败 / 总尝试 |
| --- | ---: | ---: |
| 五点法 | 0.495488 ± 0.007968（方向误差） | 4 / 19,445 |
| 八点法 | 0.366430 ± 0.003764（方向误差） | 0 / 19,445 |
| 两点法 | 0.063565 ± 0.001146（方向误差） | 0 / 19,445 |
| Arun | 0.004084 ± 0.000015 m | 0 / 19,445 |

**误差均值仅统计成功结果，失败数另报。** 四条失败均为五点法超时；种子23456的长时间求解经GDB定位于OpenGV `Sturm::bracketRoots`。种子12345复用已完成、数值复现一致的非隔离运行，其余采用超时保护。因此不跨这些运行比较计时。每帧重新以该轮种子初始化抽样器；固定输入条件下结果稳定，不代表所有场景的普遍排名。

此配置下八点法在五个种子上均优于五点法。两点方向误差低于0.1的比例为85.99%–86.73%；它借用了真值旋转，不能作为独立视觉旋转估计的优越性证明。Arun米制误差与2D方向误差单位不同，不能直接横向比较大小。

曲线采用预先固定的种子12345作代表，保留原始尖峰，不平滑、不删除离群值。1组真值插值区间达到35 ms，其余约5 ms；[区间记录](results/tables/long_gt_brackets.csv)和[包含/排除该组的敏感性统计](results/tables/gt_bracket_sensitivity.csv)均保留。

## 环境与构建

实际环境：Ubuntu 20.04.6 / ROS Noetic / catkin_tools / Eigen 3.3.7。Lab5工作区提供`lab_5`、`catkin_simple`、`glog_catkin`、`opencv3_catkin`。还需`opengv`、`eigen_catkin`、`cmake_modules`、`tf_conversions`等配置中声明的依赖。

新建Lab6工作区，将本仓库`lab6/`作为单一`lab_6`包放进`~/lab6_ws/src`，在同一工作区源码中准备OpenGV和eigen_catkin；不要放入两份同名包。然后：

```bash
source ~/lab5_ws/devel/setup.bash
cd ~/lab6_ws
catkin config --extend ~/lab5_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_EIGEN=ON
catkin build lab_6 -j2 -p1
source ~/lab6_ws/devel/setup.bash
```

Lab6骨架来自官方`VNAV2023`标签，提交`7d234d8b3618f0090b61fc6551f3e582db116f9c`，不是ROS2的main版本。`install/lab_6.rosinstall`保留上游地址，但其中master未固定版本；本仓库没有捆绑依赖源码，跨机器复现需自行准备。

## 数据

[教师更新的ROS2 office数据集](https://drive.google.com/file/d/1TohOnfjR_FKf_y9wYTEkmv-XCuFPFRPt/view)。使用独立Python ≥3.10环境安装`rosbags==0.11.9`后：

```bash
rosbags-convert --src ./vnav-lab6-office --dst office_ros1.bag
```

数据总时长236.995 s，时间戳从24 s开始；347,574条消息；左RGB、深度各3,890帧。分辨率720×480，`fx=fy=415.69219381653056, cx=360, cy=240`，畸变为零；深度为`32FC1`，按相机Z深度解释。

原始约19 GB的bag、140万组匹配的完整大缓存、编译产物和实验报告均不提交。`results/shared_frame_index.csv`保留了每组帧对的时间、匹配数、相对真值和真值插值区间。不能用Lab5的其他bag替代本实验数据。

## ROS节点与RViz

以下按原实验机的`/mnt/lab6_data/office_ros1.bag`示例，换机器需改路径。终端一启动`roscore`；终端二：

```bash
source ~/lab6_ws/devel/setup.bash
rosparam set /use_sim_time true
rosrun lab_6 pose_estimation --pose_estimator=2 --use_ransac=true --fixed_seed=true --twopt_threshold=1e-6 --scale_translation=true --rpe_csv=/mnt/lab6_data/results/NEW_twopt.csv /ground_truth_pose:=/tesse/odom /rgb_images_topic:=/tesse/left_cam/rgb/image_raw /depth_images_topic:=/tesse/depth_cam/mono/image_raw _visualize:=false _save_outputs:=false
```

终端三：

```bash
source ~/lab6_ws/devel/setup.bash
rosbag play /mnt/lab6_data/office_ros1.bag --clock -r 0.1 --delay=2
```

`pose_estimator=0/1/2/3`对应五点/八点/两点/Arun。Arun使用`--scale_translation=false`。**CSV同名会覆盖；每次使用新文件名。** 仅演示时省略`--rpe_csv`。另开终端：

```bash
rviz -d "$(rospack find lab_6)/rviz/office.rviz"
```

显示固定坐标`world`、RGB、深度、`/camera_pose`和`/gt_camera_pose`。视频为前期Arun ROS演示。显示借用前帧真值；2D还借真值恢复尺度，这是讲义指定的局部可视化，不是独立累计视觉里程计。

## 缓存及离线公平对照

先在节点中使用**全新前缀**采集共同输入，无需运行求解器：

```bash
rosrun lab_6 pose_estimation --fixed_seed=true --cache_only=true --match_cache=/mnt/lab6_data/results/NEW_cache /ground_truth_pose:=/tesse/odom /rgb_images_topic:=/tesse/left_cam/rgb/image_raw /depth_images_topic:=/tesse/depth_cam/mono/image_raw _visualize:=false _save_outputs:=false
```

配合另一终端的bag播放。生成`NEW_cache_frames.csv`和`NEW_cache_matches.csv`；缓存拒绝覆盖已有文件，目录须预先存在。首帧只初始化。完整缓存验证了3,889组连续帧对及对应匹配数。

停止ROS节点后，直接运行C++离线工具（无需ROS master）：

```bash
time rosrun lab_6 cache_eval /mnt/lab6_data/results/NEW_cache /mnt/lab6_data/results/NEW_eval 1e-6 12345 2
```

参数依次为缓存前缀、新输出前缀、2D统一阈值、随机种子、每方法超时秒数。生成`_nister.csv`、`_eightpt.csv`、`_twopt.csv`、`_arun.csv`。失败行保留`status`且误差为NaN；统计必须以`status=ok`筛选并另报失败率，不能把失败当零误差。

五点/八点均内点精化，两点固定旋转SVD，Arun在有效3D内点上重估。**这套离线对照不等同于ROS节点中的八点分支**：节点仍保留先前的`1e-5`和不精化设置。复现本目录最终多种子表格请使用`cache_eval`。源码没有因追求排名而改用其他五点算法。

未隔离的一次四方法完整计算实测45.642 s；超时保护版有进程开销，异常种子测试79.771 s。它们不包含SIFT与全系统输入处理，不能用作完整视觉系统实时性证明。

`scripts/experiment/`保存实际用过的批处理脚本，含原机绝对路径和复用轮次，作为溯源文件，不是新机器的一键安装器。最简单的独立复现实验是预先选定五个种子，逐一运行上述命令，每个种子使用新输出前缀。

## 重新绘图

在有matplotlib的Python环境执行：

```bash
python3 lab6/scripts/plot_results.py
```

从已提交的CSV重绘7张PNG及SVG，并生成中文汇总表；无需ROS或bag。旋转指标为`||Rgt-Rest||F`；2D平移为单位方向的欧氏差，Arun为米制欧氏差。成功误差与失败率分开统计。

## 已知限制与来源

- OpenCV3.4与系统cv_bridge依赖的OpenCV4.2存在混用警告；实测关闭特征弹窗可以运行，但没有宣称ABI风险已消除。
- 无RANSAC分支是可选项，未实现；运行必须保持`use_ransac=true`。
- 离线超时保护只在`cache_eval`中，未加到ROS在线节点。五点法随机样本可能触发OpenGV求根异常耗时。
- [`results/provenance.json`](results/provenance.json)记录素材来源、种子协议与特殊说明；原始日志和全部轮次保留。
- [MIT Lab6要求](https://vnav.mit.edu/labs_2023/lab6/exercises.html) · [官方VNAV2023代码](https://github.com/MIT-SPARK/VNAV-labs/tree/VNAV2023/lab6) · [原始README](docs/UPSTREAM_README.md)。

本目录是实验源码与证据归档，不代表课程平台已经接收作业。正式实验报告将按后续要求另行编写和上传。
