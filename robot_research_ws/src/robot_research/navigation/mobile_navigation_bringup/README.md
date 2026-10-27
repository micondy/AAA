# mobile_navigation_bringup

固定 D435i 的 RTAB-Map 建图/定位与 Nav2 第一版集成。D435i 点云同时进入局部/全局代价
地图，并在速度输出端提供前向停止区域。第一版 Nav2 只允许前进和旋转，不自动横移或
倒车，因为单个前向相机不能覆盖底盘两侧和后方。

## 数据与控制链

```text
底盘里程计 -> /swerve_controller/odom -> odom -> base_link
固定 D435i RGB-D + 底盘里程计 -> RTAB-Map -> map -> odom
固定 D435i PointCloud2 -> Nav2 costmaps -> Smac State Lattice（Omni）
Smac State Lattice -> MPPI（Omni）-> velocity_smoother -> collision_monitor
  -> /swerve_controller/cmd_vel
```

## 前置条件

1. D435i 已刚性固定在底盘上，并记录序列号。
2. 整机 URDF 或独立静态 TF 正确提供 `base_link -> nav_camera_link`；禁止使用猜测外参。
3. 底盘发布 `/swerve_controller/odom`，并接收 `/swerve_controller/cmd_vel`。
4. RTAB-Map 数据库路径是明确的文件，首次重建才使用 `fresh_map:=true`。
5. 初次实车测试必须低速、架空或有人监护，并保留实体急停。

移动底盘默认明确传入 `odom_source:=external` 后，由底盘发布 `odom -> base_link`，
RTAB-Map 不启动 `rgbd_odometry`，只发布 `map -> odom`。通用 launch 的默认值仍是
`odom_source:=rgbd`，方便手持 D435i 独立测试；实车启动时必须显式选择外部里程计，且禁止
第二个 `odom -> base_link` TF 发布者。

如果完整机器人模型尚未包含固定相机，可以在下面两个 launch 中额外传入：

```text
publish_camera_mount_tf:=true
camera_x:=<实测米> camera_y:=<实测米> camera_z:=<实测米>
camera_roll:=<实测弧度> camera_pitch:=<实测弧度> camera_yaw:=<实测弧度>
```

六个值没有安全默认值；不传 `publish_camera_mount_tf:=true` 时，launch 假定外部机器人模型
已经发布该 TF。

## 建图

先启动底盘，再在 GPU 主机运行：

```bash
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/mobile_base_ws/install/setup.bash
source /media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash

ros2 launch mobile_navigation_bringup mapping.launch.py \
  serial_no:=D435I_SERIAL \
  odom_source:=external odom_topic:=/swerve_controller/odom \
  database_path:=$HOME/.ros/mobile_robot_map.db \
  fresh_map:=true
```

数据库确认可用后，不再传 `fresh_map:=true`。

## 地图定位与导航

```bash
ros2 launch mobile_navigation_bringup navigation.launch.py \
  serial_no:=D435I_SERIAL \
  odom_source:=external odom_topic:=/swerve_controller/odom \
  database_path:=$HOME/.ros/mobile_robot_map.db
```

在 RViz2 中确认 TF、RTAB-Map 地图、局部/全局 costmap 和前向停止多边形正确后，才发送
短距离 `NavigateToPose` 目标。

## Nova Carter 仿真 profile

这两个入口固定 `start_camera:=false` 和 `use_sim_time:=true`；导航入口还把碰撞监控后的
最终速度映射到 Carter `/cmd_vel`：

```bash
ros2 launch mobile_navigation_bringup carter_mapping.launch.py \
  database_path:=$HOME/.ros/isaac_carter_map.db fresh_map:=true

ros2 launch mobile_navigation_bringup carter_navigation.launch.py \
  database_path:=$HOME/.ros/isaac_carter_map.db
```

仿真定位/导航入口不会替 Isaac Sim 启动传感器。必须保持 Isaac Sim 终端运行，并确认
`/clock`、RGB、aligned depth 和 camera info 持续发布后，再启动导航。若 `rgbd_sync` 提示
五秒没有数据，优先检查 Isaac Sim 是否仍在运行，不要先修改同步队列。路径平滑服务器
`smoother_server` 与速度平滑器 `velocity_smoother` 均由本包生命周期管理。

不要把 Carter profile 用于实车；实车继续使用 `mapping.launch.py` 和
`navigation.launch.py`。

## 自制四轮四转底盘仿真 profile（当前主线）

先用 Isaac Sim 的 `run_mobile_manipulator.py --ros --enable-nav-rgbd \
--nav-camera-width 320 --nav-camera-height 240 --nav-camera-hz 10` 启动自制底盘、
官方 OpenArm 和顶部导航 D435。胸前 D435i-B 不进入 RTAB-Map。当前推荐在同一个入口中
同时运行 RTAB-Map 在线增量建图和 Nav2；它会自动启动统一 URDF/TF，固定使用仿真时钟，
且不会误启真机 RealSense：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=$HOME/.ros/isaac_mobile_base_online.db \
  rtabmap_mode:=mapping fresh_map:=false
```

`mapping` 表示机器人移动时地图会继续更新，并非 rosbag 离线建图。默认续建数据库；只有
明确要清空所选数据库并从零建图时才传 `fresh_map:=true`。若实验要求冻结地图，才改用
`rtabmap_mode:=localization`，且数据库必须已经存在。

仿真导航入口默认同时启动网页控制台，并把二维地图映射到 `/rtabmap/map`。浏览器访问
`http://底盘电脑IP:8080/#visualization`；不需要网页时传 `start_web_console:=false`。
网页手动速度发布到 `/cmd_vel_web`，Nav2 经平滑和碰撞监控后的速度发布到
`/cmd_vel_nav_safe`，`cmd_vel_priority_mux` 仲裁后唯一输出 `/cmd_vel` 给 Isaac。收到网页
手动指令后的 0.30 秒内手动控制优先；网页接管前会取消正在执行的 Nav2 任务。运行本
profile 时不要让其他节点直接发布 `/cmd_vel`，否则会绕过仲裁和安全链。

只需要网页显示 Nvblox 三维环境、但仍保持 PointCloud2 导航基线时使用：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  enable_nvblox:=true \
  database_path:=$HOME/.ros/isaac_mobile_base_map.db
```

默认仍使用冻结的 PointCloud2 voxel/obstacle layer。需要测试 Nvblox ESDF 代价地图时增加：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  use_nvblox_costmap:=true \
  database_path:=$HOME/.ros/isaac_mobile_base_map.db
```

该开关会自动启动Nvblox；local costmap改用Nvblox，global costmap保留RTAB-Map静态层并
叠加Nvblox。删除该参数或设为`false`即可回退 PointCloud2 costmap 基线。

当前 Isaac profile 的 Nav2 速度链为 `/cmd_vel_nav` -> `/cmd_vel_smoothed` ->
Collision Monitor -> `/cmd_vel_nav_safe`；再与 `/cmd_vel_web` 一起进入优先级仲裁器，
由仲裁器输出 Isaac 接收的 `/cmd_vel`。仿真 overlay 暂时关闭
`front_depth` 与 `FrontStop`，因此当前只验证指令转发；在启用点云紧急停车前，还需要按
相机盲区和制动距离标定安全区。Nav2 控制器自身碰撞检查和 local/global costmap 始终启用。
Isaac 同时发布理想化的
`/swerve_controller/odom` 与标准 `odom -> base_link`，模拟实车控制器接口；独立的
`/sim/ground_truth/odom` 和 `/sim/ground_truth/tf` 只用于误差评测。RTAB-Map 使用外部
odom 后只发布 `map -> odom`，不会因低纹理画面让连续 TF 中断。两个自制底盘仿真入口均由最外层 profile 直接启动 RViz，
避免嵌套 launch 的同名 `use_rviz` 参数互相覆盖。该仿真 profile 将碰撞点云超时放宽为
2.0 秒，以容忍 Isaac 渲染造成的短暂停帧；实机 `navigation.launch.py` 默认仍为 0.75 秒。
顶部相机仍存在近距离低矮障碍盲区；当前仿真配置不能作为安全验收。实机必须启用并验证
Collision Monitor 的传感器源，重新测量最低可见高度和制动距离，不能照搬仿真参数。

10 个候选目标保存在 `config/isaac_indoor_top_rt_01_waypoints.yaml`。先只预览：

```bash
ros2 run mobile_navigation_bringup navigation_waypoint_benchmark.py
```

在 RViz 确认目标均位于当前可通行区域后，才执行并记录 JSON/CSV：

```bash
ros2 run mobile_navigation_bringup navigation_waypoint_benchmark.py --execute
```

中断后可从指定编号继续，例如 `--execute --start-index 3`。仿真 profile 的位置/角度到点
阈值为 0.08 m/0.15 rad，进度检查半径为 0.08 m。位置阈值收紧后，网页选择附近目标也会
产生可见运动；后续机械臂任务仍需独立精定位。实机通用 profile 保持
0.15 m/0.15 rad 和 0.15 m 进度半径。

确定性 Isaac 场景启动位姿已知时，仿真 profile 默认向 RTAB-Map 提供建图起点
`0 0 0 0 0 0`。若恢复历史场景或从非零存档位姿启动，必须用
`initial_pose:="x y z roll pitch yaw"` 覆盖；通用/实机定位入口默认不设初值。

首轮不要使用 `--continue-on-failure`：目标点在栅格上有净空，不保证目标间路径不会被
在线建图新增的占据栅格或局部点云阻塞。遇到 `PATIENCE_EXCEEDED(104)` 时，应先检查
RViz 的 global/local costmap、目标是否落在已建图区域、机器人定位、全局路径和
`/collision_monitor/front_stop`，再调整目标或由人工确认后执行逃逸动作。
导航仿真使用 320×240@10 Hz，以防 640×480 点云在 Isaac 渲染和 DDS 中积压；640×480
仍是相机/目标感知专项的默认值，不改变实机 D435 配置。

碰撞监控可脱离仿真做确定性回归；该命令只使用 `/safety_test/*` 隔离话题，不会控制
Isaac 或实车：

```bash
ros2 launch mobile_navigation_bringup collision_safety_benchmark.launch.py
```

测试依次验证净空点云放行、`FrontStop` 内障碍归零和点云超过 0.75 秒未更新时归零，结果
写入 `results/navigation/safety/collision_safety_*.json`。这只验证软件安全边界，不替代
实车硬件急停、底盘看门狗和实际停止距离测试。

Isaac 启动时增加 `--enable-nav-obstacle-control` 后，会提供分类障碍服务。下列静态测试把
低矮、悬空和货架障碍依次放到当前底盘正前方 1.00 m，再自动移除，并验证完整的渲染点云
安全链；它不会发送运动命令：

```bash
source tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup isaac_obstacle_safety_benchmark.launch.py
```

正式基线中障碍阶段触发 `STOP:FrontStop` 且输出为 0，移除后输出恢复为 0.15 m/s；结果写入
`results/navigation/safety/isaac_obstacle_safety_*.json`。

动态侵入测试会让仿真底盘以 0.10/0.18 m/s 前进，并直接向通用 `/cmd_vel` 输出，因此只能在
与实车隔离的 Isaac ROS domain 中运行。关闭真车驱动并确认仿真障碍服务存在后，必须显式
确认运动：

```bash
source tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup dynamic_obstacle_intrusion_benchmark.launch.py \
  allow_motion:=true
```

它验证低矮/悬空障碍侵入后的停车延迟、里程计前进量和相机点云断流失效停车，结果写入
`results/navigation/intrusion/dynamic_intrusion_*.json`。默认 `allow_motion:=false`，误启动不会
发送运动命令。这仍是仿真回归，不是实车制动认证。

## 第一版限制

- `nav2.yaml` 的 footprint 仍保守设置为 0.84×0.68 m；CAD 反算本体约 0.62×0.58 m，
  轮组外廓约 0.59×0.60 m。实车必须把支架、线缆和相机计入最外轮廓后再收紧。
- D435i 前向盲区、最低测距、地面飞点和车体自遮挡尚需实测。
- 后退恢复已从行为树删除；相机没有侧后覆盖前不启用 Nav2 全向横移。
- PointCloud2 障碍层是导航避障输入，不替代硬件急停和底盘本地看门狗。
- RTAB-Map、D435i 外参和真实数据库尚未完成实机验收。
- 自制底盘的完整 10 目标成功率、真实点云障碍清除和实车停止距离尚未验收；`FrontStop`
  与点云超时的软件注入、仿真动态侵入回归已通过。
- P01 加初值后成功，但旧版纯 RGB-D odom 在低纹理朝向丢失并使 P02 中止；当前已切换为
  底盘连续 odom + RTAB-Map `map -> odom`，仍需用一个短目标完成运行时回归。
