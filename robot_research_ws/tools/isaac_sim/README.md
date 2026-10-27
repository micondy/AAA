# Isaac Sim 手动验证入口

该目录保存 Isaac Sim 6.0.1 的项目运行入口。当前主线是自制四轮四转底盘；Nova Carter
只保留为早期接口验证基线，不再继续扩展。

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd /home/scc/isaac-sim-standalone-6.0.1-linux-x86_64
./python.sh \
  /media/scc/Windows_D/AAA/robot_research_ws/tools/isaac_sim/run_carter_warehouse.py
```

S1 需要检查项目使用的 640×480、15 Hz RGB-D 等效相机时增加：

```bash
--enable-nav-rgbd
```

这会从 Nova Carter 前左相机发布：

```text
/sensors/nav_camera/color/image_raw
/sensors/nav_camera/aligned_depth_to_color/image_raw
/sensors/nav_camera/color/camera_info
/sensors/nav_camera/depth/color/points
```

主循环默认使用 `--realtime-factor 1.0` 将仿真限制在不超过实时速度，避免 Isaac 产生数据
的速度超过 RTAB-Map/TF 的消费速度。离线快速物理测试可显式传入
`--realtime-factor 0`，但运行 ROS 2 感知与导航时不要关闭节流。

`--enable-front-rgb` 仅用于核对官方 1920×1200 Hawk 双目 RGB 话题，不作为本项目默认
导航相机配置。

停止仿真时关闭 Isaac Sim 窗口，或在启动终端按 `Ctrl+C`。

## 2026-09-09 S0 实测接口

未启用相机渲染时，官方场景已经提供：

```text
/clock                       rosgraph_msgs/msg/Clock
/cmd_vel                     geometry_msgs/msg/Twist
/chassis/odom                nav_msgs/msg/Odometry
/chassis/imu                 sensor_msgs/msg/Imu
/tf                          tf2_msgs/msg/TFMessage
/front_3d_lidar/lidar_points sensor_msgs/msg/PointCloud2
```

实测 `/clock` 约 57 Hz、`/chassis/odom` 约 59 Hz。向 `/cmd_vel` 发送 0.15 m/s 共 3 秒，
里程计前进约 0.463 m；随后发送 0.30 rad/s 共 3 秒，实际偏航约 0.73 rad，最终速度回到
近零。差异来自官方 Carter 控制图的加速度限制。

脚本会把官方物理里程计改为 `/sim/ground_truth/odom`，并将其真值 TF 改到
`/sim/ground_truth/tf`。官方动态传感器 TF 也改到 `/sim/ground_truth/sensor_tf`；Carter
mapping/navigation profile 在 `/tf_static` 发布已核对的固定相机外参，避免渲染点云领先
动态 TF 引起间歇性 extrapolation。因此标准 TF 中不会出现 Isaac 和 RTAB-Map 同时发布
`odom -> base_link`，相机刚性外参也不再依赖动态时间戳。

## 2026-09-09 S1 实测接口

项目相机四路数据已连续发布，实测 RGB、深度和内参均为 640×480；RGB 编码为 `rgb8`，
深度编码为 `32FC1`，点云为 307200 点。四路消息统一使用
`front_stereo_camera_left_rgb`，其官方 TF 外参为：

```text
base_link -> front_stereo_camera_left_rgb
translation: [0.100, 0.075, 0.346] m
rotation RPY: [-90, 0, -90] deg
```

相机内参实测 `fx=fy=319.2705, cx=320, cy=240`。使用 RTAB-Map `rgbd_sync` 严格同步
（`approx_sync=false`）已生成 `/sim_test/rgbd_image`。GUI 模式并行订阅时 RGB/深度约
11–12.5 Hz，点云约 8 Hz，低于设定的 15 Hz，但足以先进行 0.15 m/s 的短路线建图。

注意：Nova Carter 的 Hawk 原始资产使用已弃用的 `fisheyePolynomial` 投影，而 ROS 2
CameraInfo helper 对该旧模型输出的是零畸变 `plumb_bob` 内参。脚本在项目 RGB-D 模式下
会把这一相机覆盖成标准 `pinhole`，保证渲染图像、深度和 CameraInfo 的投影模型一致。

## RTAB-Map 仿真建图

Isaac Sim 就绪后，在另一个终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash
ros2 launch mobile_navigation_bringup mapping.launch.py \
  start_camera:=false \
  use_sim_time:=true \
  fresh_map:=true \
  database_path:=/media/scc/Windows_D/AAA/robot_research_ws/maps/isaac_warehouse_pinhole_01.db
```

2026-09-09 直线尺度检查：真值前进 1.017 m，视觉估计 1.038 m，误差约 2.1%，视觉
里程计 z 漂移约 6.6 mm。旧鱼眼错误模型生成的
`maps/isaac_warehouse_01.db` 仅作为失败诊断保留，不用于定位或导航。

### 2026-09-12 闭环地图与 Nav2

启动 Carter 后，可用闭环工具执行保守右转路线：

```bash
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash
python3 /media/scc/Windows_D/AAA/robot_research_ws/install/mobile_navigation_bringup/lib/mobile_navigation_bringup/carter_mapping_route.py \
  --edge-x 1.0 --edge-y 0.7 --linear-speed 0.10 --angular-speed 0.22 \
  --turn-direction right
```

已验收数据库和二维地图为：

```text
maps/isaac_carter_loop_right_20260912.db
maps/isaac_carter_loop_right_20260912.pgm
maps/isaac_carter_loop_right_20260912.yaml
results/navigation/carter_loop_right_20260912.csv
results/navigation/carter_loop_right_20260912.json
```

加载定位与 Nav2 时必须使用 Isaac 的仿真时钟，并把安全输出映射到 Carter 的 `/cmd_vel`：

```bash
ros2 launch mobile_navigation_bringup carter_navigation.launch.py \
  database_path:=/media/scc/Windows_D/AAA/robot_research_ws/maps/isaac_carter_loop_right_20260912.db
```

RViz 中以 `map` 为 Fixed Frame，用 “Nav2 Goal” 先发送前方约 0.5 m 的空闲目标。2026-09-12
单目标实测返回 `SUCCEEDED`。该记录使用当时的 Regulated Pure Pursuit 历史基线；当前
自制底盘主配置已经是 Smac State Lattice + MPPI Omni。碰撞监控仍位于最终速度出口，未被
跳过。相机静态 TF 与整帧 RGB-D 配置后的最终复测未出现 collision monitor TF/源超时警告。
连续 10 目标与障碍故障注入仍待验收。

## 自制四轮四转底盘

展开后的独立仿真 URDF 和 Isaac Sim 6.0.1 导入结果位于：

```text
assets/mobile_base/mobile_base_sim.urdf
assets/mobile_base/cad_20260912/mobile_base_sim/mobile_base_sim.usda
```

第一阶段只检查物理模型。四个转向关节保持 0°，四个驱动轮保持零速，不连接实车 CAN：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_base.py
```

无窗口运行 10 秒稳定性测试：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_base.py --headless --test-seconds 10
```

低速运动演示（1 秒静置、0.15 m/s 直行 3 秒、停车转舵、0.25 rad/s 原地旋转
4 秒、停车）：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_base.py --demo
```

2026-09-10 的旧简化几何曾通过直线和原地旋转测试。2026-09-12 已换成自制底盘 CAD
视觉、标准化关节树和简化碰撞体；仿真逆运动学与实车配置现在都使用 CAD 反算初值：
轴距 0.419 m、转向轴轮距 0.450 m、轮半径 0.0855 m，并采用转角限制在 ±90°、必要时
反转轮速的等效方向。这组新参数仍需重新执行运动尺度验收，不能沿用旧模型的 0.6% 结果。

接收 ROS 2 `/cmd_vel` 并等待外部控制：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_base.py --ros
```

`--ros` 同时发布 `/clock`、`/joint_states`、`/swerve_controller/odom`、标准 `/tf`、
`/sim/ground_truth/odom` 和 `/sim/ground_truth/tf`。真值 TF 不写入标准 `/tf`；
早期 Carter 视觉里程计基线中 `odom -> base_link` 只由 RTAB-Map 负责；当前自制底盘
profile 由 `/swerve_controller/odom` 的底盘状态唯一负责该 TF，RTAB-Map 只发布
`map -> odom`。

### 自制底盘 RGB-D、建图与导航

主线仿真使用完整的“自制底盘 + 官方 OpenArm”。胸前 D435i-B 作为操作感知相机保留，
RTAB-Map 使用顶部中央、向下俯视 12° 的独立 D435；它与真机使用同一组上层话题和
`nav_camera_color_optical_frame`：

脚本默认生成 7×5 m 普通室内测试房间，包含四面墙、彩色箱体、立柱和前墙标记，均为
独立程序化场景而非 Carter。只检查整机模型时可传 `--empty-scene` 恢复空旷 Grid。

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /media/scc/Windows_D/AAA/robot_research_ws

/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py --ros --enable-nav-rgbd \
  --nav-camera-width 320 --nav-camera-height 240 --nav-camera-hz 10
```

需要做导航障碍回归时，在上述 Isaac 命令末尾增加
`--enable-nav-obstacle-control`。出现 `nav obstacle control READY` 后，另开终端执行：

```bash
source tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup isaac_obstacle_safety_benchmark.launch.py
```

它会自动放置并移除测试障碍，验证真实渲染点云触发 `FrontStop` 后能够恢复，不会向
`/cmd_vel` 发送底盘运动命令。

障碍控制现提供 `standard`、`low`、`hanging`、`shelf` 四种服务；安全基准自动验收后三种。
低矮障碍放在1.00 m处，用于验证顶部相机在进入近端盲区前能够触发提前停车。

需要验收运动中的障碍侵入和相机断流时，只能在实车驱动关闭、ROS domain 与实车隔离的
条件下运行：

```bash
source tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup dynamic_obstacle_intrusion_benchmark.launch.py \
  allow_motion:=true
```

此命令会直接发布 `/cmd_vel` 并移动仿真底盘；默认不传 `allow_motion:=true` 会拒绝运行。

等待终端出现两条 `READY` 后，另开终端启动在线建图、Nav2、RViz 和网页控制台：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=$HOME/.ros/isaac_mobile_base_online.db \
  rtabmap_mode:=mapping fresh_map:=false
```

该 profile 显式启动统一机器人描述、仿真相机 TF、RTAB-Map 在线增量建图和 Nav2，
不启动真机 RealSense。地图会随机器人移动持续更新，数据库持续写入。默认网页地址是
`http://127.0.0.1:8080/#visualization`。默认续建；只有明确要删除所选数据库内容、建立
全新地图时才使用 `fresh_map:=true`。

只有要冻结已有地图做可重复定位实验时，才切换为定位模式：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=/media/scc/Windows_D/AAA/robot_research_ws/maps/isaac_indoor_external_odom_04.db \
  rtabmap_mode:=localization \
  initial_pose:="0 0 0 0 0 0"
```

网页手动控制发布 `/cmd_vel_web`，Nav2 安全链发布 `/cmd_vel_nav_safe`，优先级仲裁器是
唯一的 `/cmd_vel` 发布者。Isaac 收到转向命令后先让四个转向关节对准目标；误差进入 5°
以内才驱动车轮，避免转向未到位时横拖或朝错误方向运动。RViz 的 Fixed Frame 使用 `map`，
首次只发送前方约 0.3--0.5 m 的空闲目标。当前仍必须有人观察，不能把仿真真值里程计接入
定位结果来“美化”实验。

当前正式仿真地图使用 0.5×0.5 m 右转闭环生成：8/8 段完成，实际路径 1.958 m，平移闭合
误差 2.79 cm。四点验收配置为
`mobile_navigation_bringup/config/isaac_indoor_external_odom_04_waypoints.yaml`；基准工具接受
任意数量的 waypoint，不再要求每次执行 10 点。

另开终端进行低速键盘控制：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.30 \
  -r /cmd_vel:=/cmd_vel_web
```

按住 `i` 前进、`,` 后退、`j/l` 原地左右旋转、大写 `J/L` 横移，其他键停车。
仿真输入限制为纵向 ±0.30 m/s、横向 ±0.20 m/s、转动 ±0.50 rad/s；超过范围会被
截断，指令中断后自动停车。运行导航 profile 时必须重映射到 `/cmd_vel_web`，不要直接
发布 `/cmd_vel`。大幅改变运动方向时先停车转舵，再开始驱动。

当前 URDF 已显式提供 box/cylinder collision 和 CAD 导出质量的近似 inertial；高面数 STL
只用于视觉。CAD 质量未包含最终电池、OpenArm、支架和线缆，因此仍不能用于倾覆、制动距离
或能耗论文数据，整车装配后必须重新称重并标定质心/惯量。

后续按以下顺序验收，避免同时启动过多组件：

1. 用新 CAD 参数重新做静置、直线、原地旋转和横移尺度验收；
2. 对四路 RGB-D、相机 TF 和严格同步做运行时验收；
3. 用自制底盘 profile 建立第一张闭环地图，验证短距离 Nav2 和前方障碍停车；
4. 组合 OpenArm，并依据新增质量重新标定底盘惯量和控制参数。

## 自制底盘 + OpenArm 双臂组合体（M1）

ROS 统一树由 `mobile_manipulator_description` 生成；Isaac 组合层
`assets/mobile_manipulator/mobile_manipulator_official.usda` 直接引用项目内的 NVIDIA 官方
OpenArm 双臂 USD 和自研底盘 USD。官方资产副本、SHA-256 与来源记录均在同一目录。当前
临时安装座位于底盘中心上表面，双臂使用官方零位驱动；不接实车 CAN、不启动 MoveIt、
Nav2 或相机：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py
```

无窗口执行 10 秒 M1 稳定性验收：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py --headless --test-seconds 10
```

脚本会验证 8 个底盘关节、14 个机械臂主关节、两组官方手指 mimic 和组合固定关节，报告
底盘水平漂移、最大横滚/俯仰和最终高度。2026-09-10 实测 10 秒水平漂移 0.150 mm、最大
横滚/俯仰 0°、最终 `base_link z=0.1000 m`，静置验收通过。当前质量模型仍是“官方
OpenArm 物理资产 + 底盘临时密度”，不能用于倾覆极限、制动距离或动力学论文数据；实体
整车称重和 CAD 惯量补齐后需要重新导入底盘 USD。

## 组合体固定 D435i 等效相机（S7-c）

顶部导航 D435 尚未制作支架，组合模型先使用临时位姿 `xyz=[0.10, 0, 1.12] m`、向下
俯视 12°。胸前 D435i-B 不接入导航图。ROS 统一模型直接引用官方
`realsense2_description`；Isaac 运行时相机位于 nominal 彩色光学中心并发布同一接口：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py --ros --enable-nav-rgbd
```

无窗口短测可增加 `--headless --test-seconds 10`。2026-09-11 实测四路消息为：

```text
/sensors/nav_camera/color/image_raw                    rgb8
/sensors/nav_camera/aligned_depth_to_color/image_raw   32FC1
/sensors/nav_camera/color/camera_info                  640x480
/sensors/nav_camera/depth/color/points                 307200 points
frame_id: nav_camera_color_optical_frame
```

同一 ROS 图会发布递增的 `/clock`；实测连续两次取样为 19.233 s 和 20.650 s，因此算法节点
可以安全使用 `use_sim_time:=true`。Clock publisher 显式连接 Isaac simulation time，不能
只触发 publisher 而漏接时间输入，否则会持续发布零时间。

需要 ROS 侧提供相机 nominal TF 时，可只启动描述层，不启动 MoveIt、Nav2、控制器或真机
RealSense 驱动：

```bash
ros2 launch mobile_manipulator_description display.launch.py \
  use_rviz:=false use_joint_state_publisher:=true \
  nav_camera_use_nominal_extrinsics:=true
```

实体 D435i 上车后应保持 `nav_camera_use_nominal_extrinsics:=false`，由 RealSense 驱动发布
内部标定 TF，并用实测值覆盖 `nav_camera_mount_xyz/rpy`。临时位姿不得当作论文外参。

目标感知端到端检查可增加已知 20 cm 绿色方块：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py --enable-nav-rgbd --add-green-target

# 另一个 ROS 2 终端
ros2 launch target_perception color_depth.launch.py \
  output_frame:=nav_camera_color_optical_frame
```

2026-09-11 实测 `green_target_0` 可见前表面深度为 0.76955 m，与相机至方块前表面的
几何真值一致；恢复宽高约 0.2001×0.2025 m，有效深度点 6889、有效率 98.8%。节点输出
`Detection3DArray`、`Detection2DArray`、带协方差位姿和 diagnostics；测试完不需要保存
修改后的 USD，方块和相机均由脚本运行时创建。

`--green-target-depth` 的定义是相机光学中心到方块可见前表面的距离。多距离评测不要连续
重启 Isaac；8 GB 显存机器可能在第二次初始化时卡住。项目提供单进程入口，在一次仿真中
移动同一个目标并按仿真时间分段：

```bash
source tools/source_robot_stack.bash
python3 tools/run_target_distance_benchmark.py --samples 100
```

底层对应 `--green-target-depth-sequence` 与 `--green-target-stage-seconds`。2026-09-11 使用
0.4/0.6/0.8/1.0 m、每档 30 秒和 100 个严格同步样本完成验收，结果见
`results/target_perception/distance_20260911_4x100`。
