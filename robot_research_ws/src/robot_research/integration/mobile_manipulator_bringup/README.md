# mobile_manipulator_bringup

整机的 profile 编排层。它不实现底盘运动学、不复制 OpenArm 驱动，也不修改两个下层项目；
只把组合 URDF、MoveIt、官方 OpenArm 控制器和已有四轮四转控制器组织为一致的运行入口。

## Mock profile

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup mock.launch.py
```

该入口只加载 `mock_components/GenericSystem`，不会访问底盘或 OpenArm CAN。接口为：

```text
/cmd_vel                 geometry_msgs/msg/Twist
/odom                    nav_msgs/msg/Odometry
/joint_states            sensor_msgs/msg/JointState
/tf                      world -> odom -> base_link -> robot links
/sim/ground_truth/odom   nav_msgs/msg/Odometry (mock ideal feedback)
```

`/cmd_vel` 被转发到已有 `/swerve_controller/cmd_vel`；已有控制器的轮速里程计从
`/swerve_controller/odom` 统一转发到 `/odom`，并由本包唯一发布 `odom -> base_link`。
底盘控制器保留 0.25 秒指令超时停车机制。

## Sim perception profile

当前的第一版仿真入口只建立 RGB-D 感知最小闭环：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_perception.launch.py
```

它默认启动 Isaac Sim 组合体与绿色测试目标、统一 `robot_state_publisher` 和
`target_perception`，建立 `base_link -> nav_camera_color_optical_frame`，并将三维检测结果发布到
`base_link`。该入口故意不启动 MoveIt、ros2_control、Nav2、RTAB-Map、RViz 或真机
RealSense，便于单独定位 TF、图像、深度和目标检测问题。

常用的裁剪方式：

```bash
# 只发布统一机器人/相机 TF，不启动 Isaac 和检测器
ros2 launch mobile_manipulator_bringup sim_perception.launch.py \
  start_isaac:=false start_target_perception:=false

# 已经手工打开 Isaac，只接入检测器和统一 TF
ros2 launch mobile_manipulator_bringup sim_perception.launch.py start_isaac:=false

# 无界面自动测试 30 秒
ros2 launch mobile_manipulator_bringup sim_perception.launch.py \
  headless:=true test_seconds:=30

# 已知前表面距离 0.4 m，预热后自动记录 100 帧并退出
ros2 launch mobile_manipulator_bringup sim_perception.launch.py \
  headless:=true run_benchmark:=true green_target_depth:=0.4 \
  benchmark_samples:=100 benchmark_output_csv:=/tmp/target_depth_0p4.csv
```

完整的 0.4/0.6/0.8/1.0 m 评测应使用 `tools/run_target_distance_benchmark.py`。它只启动
一次 Isaac，并在一个进程中移动目标；默认每档保留 30 秒，避免 8 GB 显存机器连续重启
Isaac 时卡在第二次初始化。

`publish_world_tf` 默认关闭，因为将来应由仿真底盘里程计唯一发布 `world/odom -> base_link`；
只有做静止演示时才可临时设为 `true`。仿真使用 D435i 官方 nominal extrinsics；真机 profile
必须改回 `nav_camera_use_nominal_extrinsics:=false`，由 RealSense 驱动发布设备内参 TF。

后续完整 `sim` profile 会在保持标准接口的基础上增加 Isaac 底盘控制和
`/sim/ground_truth/odom`。`real` profile 不发布仿真真值，且只有完成 CAN、零位、急停和低速
限制验收后才允许创建。

在目标感知上增加 Target Belief 及可选 100 帧收敛记录：

```bash
ros2 launch mobile_manipulator_bringup sim_target_belief.launch.py \
  headless:=true test_seconds:=20 run_belief_benchmark:=true \
  benchmark_output_csv:=/tmp/target_belief.csv
```

该入口复用薄 `sim_perception`，不会启动 MoveIt、Nav2 或 RTAB-Map。belief 输入使用
sensor-data QoS，避免与 `target_perception` 的 best-effort 发布器不兼容。

## Isaac 导航与主动感知联合 profile

先单独启动 Isaac（便于观察和重启）：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py \
  --ros --enable-nav-rgbd --add-green-target \
  --nav-camera-width 320 --nav-camera-height 240 --nav-camera-hz 10
```

第二个终端启动 RTAB-Map、Nav2、目标感知、Belief、NBV 和 RViz：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mobile_manipulator_bringup sim_active_perception.launch.py
```

看到目标 belief 和地图后，在第三个终端触发一次规划：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 service call /active_perception/compute_next_best_view std_srvs/srv/Trigger '{}'
```

RViz 同时显示 `/rtabmap/map`、目标 belief、候选箭头和最佳视角。该入口不会自动发送
`NavigateToPose`，因此触发服务不会让底盘移动。2026-09-17 已完成 A3 联合验收；结果见
[`results/active_perception/a3_joint_sim_20260917.md`](../../../../results/active_perception/a3_joint_sim_20260917.md)。

A4 执行器也随该入口启动，但默认处于 `DISARMED`。需要做仿真运动实验时，必须显式传入
`enable_nbv_execution:=true`；计算最佳视角并在 RViz 检查后，再调用：

```bash
ros2 service call /active_perception/execute_best_view std_srvs/srv/Trigger '{}'
```

每个视角只能执行一次，结果写入 `results/active_perception/executions/`。执行过程中可调用
`/active_perception/cancel_view_execution` 停止。当前代码和默认禁用保护已通过测试，完整
Isaac 移动—重新观测验收仍待下一次启动仿真完成。

## RealSense 通用物体感知 profile

插入一台 D435i 后，一条命令启动对齐 RGB-D、Mask R-CNN 通用日常物体分割和 RViz：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup real_instance_perception.launch.py
```

默认使用仓库 `models/` 中已校验的官方 torchvision 权重、CUDA 自动选择，以及
`nav_camera_color_optical_frame` 输出，因此手持相机测试不依赖尚未标定的底盘外参。RViz
的 `General object instances` 面板显示实例 mask、框、类别和置信度。默认相机阶段是
`aligned`，不打开点云或 IMU；需要指定设备时传 `serial_no:=相机序列号`。类别门控的
三维跟踪默认同时启动，并在 `/target_perception/tracked_detections_3d` 发布跨帧基线 ID。

该 profile 默认启用标准权重支持的全部通用 COCO 类别，不包含蘑菇专用识别；具体任务可
通过 `allowed_classes` 白名单收窄。2026-09-15 软件链路与固定 RGB-D 输入已通过，但当次
USB 枚举没有发现 D435i，所以真实相机连续流仍是下一验收项。

正式采集时摆放 2～3 类普通物体，并同时录制原始 RGB-D 与 100 帧评测结果。下面两个输出
路径必须是尚不存在或为空的新目录：

```bash
ros2 launch mobile_manipulator_bringup real_instance_perception.launch.py \
  run_benchmark:=true record_bag:=true \
  benchmark_samples:=100 benchmark_warmup_frames:=15 \
  benchmark_output_dir:=results/target_perception/d435i_general_001 \
  bag_output:=bags/d435i_general_object_001
```

100 个测量帧完成后，评测节点会结束整个 launch；rosbag 同时收到 SIGINT 并写完元数据。
结果目录包含 `frames.csv`、`instances.csv` 和 `summary.json`，原始 bag 可用于离线重复比较
其他模型。不要对同一输出目录重复运行。

## MoveIt 碰撞模型验收

启动 mock profile 后运行：

```bash
ros2 run mobile_manipulator_bringup validate_moveit_collisions
```

该命令通过 MoveIt `/check_state_validity` 服务验证左右臂零位有效，并验证一组固定左臂姿态
会因 `openarm_left_link4 <-> openarm_body_link0` 碰撞被拒绝。静态测试同时保证底盘碰撞
`base_link` 具有简化 collision 几何，并且 SRDF 只放行它与固定相邻的
`openarm_mount_link`，不放行任何机械臂活动 link。CAD 高面数 STL 只用于显示。

2026-09-11 还使用固定随机种子在左右臂关节范围内检查了 6000 个整机状态：1492 个状态因
OpenArm 自碰撞被拒绝，未出现机械臂与底盘接触。这与当时的临时底盘碰撞代理及安装高度
相符，表示手臂无法触及底盘，而不是把底盘碰撞错误地加入允许列表。安装高度或
底盘 CAD 变化后必须重新执行采样并更新回归姿态。

## 通用任务场景

不依赖温室或蘑菇层架的固定底座验证入口：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup task_scene.launch.py
```

该入口使用 MoveIt mock 控制器，不连接 OpenArm CAN，也不启动底盘控制器。默认载入
`config/generic_task_scene.yaml` 中的工作台、后挡板、上层板和遮挡板四个 box。要替换为其他
普通室内场景，只需新建同格式 YAML，并传入：

```bash
ros2 launch mobile_manipulator_bringup task_scene.launch.py \
  scene_config:=/absolute/path/to/my_scene.yaml
```

场景启动后另开终端运行碰撞回归：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 run mobile_manipulator_bringup validate_moveit_collisions
```

验收器必须同时得到四项 PASS：左右臂 home 安全、自碰撞姿态被拒绝、左夹爪与
`work_surface` 接触姿态被拒绝，并核对具体接触对象。开发阶段若修改场景位置，可用固定种子
重新寻找环境碰撞姿态：

```bash
python3 tools/find_scene_collision_pose.py --samples 3000 --seed 20260911 --side left
```
