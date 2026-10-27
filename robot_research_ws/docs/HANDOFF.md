# 移动操作机器人工程交接文档

> 更新日期：2026-10-20  
> 工程根目录：`/media/scc/Windows_D/AAA`  
> 适用对象：接手开发、复现实验、部署仿真或准备实机联调的人员

## 1. 工程一句话说明

本工程是一套基于 ROS 2 Jazzy 的移动操作机器人软件栈，机器人由四轮四转底盘、双
OpenArm、顶部 RGB-D 相机组成。它已经能在 Isaac Sim 中完成机器人仿真、RGB-D 感知、
RTAB-Map 建图/定位、Nav2 点击导航、Nvblox 三维重建、网页监控与控制、双臂 MoveIt 规划、
目标三维感知以及带人工安全门控的主动视角规划。

仿真闭环已经具备，实机底盘、相机外参和整机安全仍需按本文的“未完成与限制”逐项验收。

## 2. 工作区组成

工程不是一个工作区，而是按依赖方向分成两层：

```text
/media/scc/Windows_D/AAA/
├── mobile_base_ws/          # 底盘层：CAN、四轮四转、底盘模型、网页
├── robot_research_ws/       # 科研层：整机、感知、建图、导航、MoveIt、主动感知
└── chassis_4wis/            # 原始 CAD/描述资产，不是运行时工作区
```

必须按以下顺序加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/mobile_base_ws/install/setup.bash
source /media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash
```

科研工作区提供了等价的快捷脚本：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
```

依赖方向只能是 ROS 2 → `mobile_base_ws` → `robot_research_ws`。不要让底盘包反向依赖科研包，
也不要直接修改 `robot_research_ws/src/vendor` 中固定版本的 OpenArm 上游代码。

## 3. 当前能做到什么

### 3.1 四轮四转底盘

- 支持前后、横移、斜移、原地旋转、弧线和边走边转。
- 每个轮组独立计算转角和轮速，目标轮向限制在机械 `-90°～+90°`。
- 后半平面运动使用“转角折叠 180° + 轮速反向”，避免舵轮越过机械限位。
- 大幅换向时先停车、等待轮速归零，再转向；四轮对准后才恢复行走。
- 有转向反馈超时、角度跳变、速度/加速度限制和命令超时停车。
- 支持 mock hardware，不连接 CAN 也能验证完整控制链。
- 已提供达妙转向电机和 ZLAC8015D 行走驱动器的 SocketCAN 硬件接口及独立检查工具。

状态：软件、mock 和 Isaac 已验证；重新机械安装后，四轮零位、转向方向、行走方向和横移姿态已在架空低速条件下核对。落地整车联动、安全速度和里程计精度仍待验收。实物映射、当前零偏、安全联锁和落地清单见 [底盘工程交接文档](../../mobile_base_ws/docs/工程交接文档.md)。

### 3.2 网页控制台

默认地址为 `http://机器人IP:8080`，地图页为 `/#visualization`。网页能够：

- 查看 CAN、关节、里程计、工作模式、安全诊断和事件。
- 在 NORMAL 模式下手动发送前后、横移和旋转速度。
- 在 MAINTENANCE 模式下进行单轮低速测试和转向零位标定。
- 显示 RTAB-Map 二维占据地图、历史轨迹和机器人位姿。
- 在地图上按下选择目标位置，拖动或输入角度设置目标 yaw，并向 Nav2 发送目标。
- 二维地图显示 `0.62 × 0.58 m` 实体车身、四个轮子和 `0.66 × 0.62 m` Nav2 轮廓；
  目标位置也显示完整目标 footprint，而不是只有一个箭头。
- 显示 Nvblox 彩色三维环境点、可拖动旋转/滚轮缩放的视图，以及随 odom 移动的三维底盘、
  四轮、朝向和安全轮廓。
- 切换独立的底盘结构视图，查看四轮实时转向角和轮速。
- 修改网页速度限制、CAN 参数、轮径、轴距、轮距、车身尺寸和安全余量。

网页是实验操作界面，不是实体急停，不应暴露到公网。

### 3.3 RGB-D、建图和定位

- D435i 支持 `basic`、`imu`、`aligned`、`full` 四种启动阶段。
- 支持相机数据健康检查、rosbag2 录制和回放。
- RTAB-Map 支持在线建图、固定 rosbag 离线建图和已有数据库定位。
- TF 所有权明确：底盘/仿真发布 `odom -> base_link`，RTAB-Map 只发布 `map -> odom`。
- Isaac 自制底盘场景已生成正式定位数据库，并完成四点闭环导航验证。

状态：仿真通过；真实 D435i 刚性安装、序列号绑定和 `base_link -> camera` 外参尚待实测。

### 3.4 Nav2 自主导航与安全链

- 支持网页、RViz 和命令行发送 `NavigateToPose`。
- 使用 RTAB-Map 静态地图、局部/全局 costmap、Smac State Lattice（Omni）、
  MPPI（Omni）、速度平滑器和 Collision Monitor。
- 仿真命令链为：

  ```text
  Nav2 controller -> velocity smoother -> collision monitor -> /cmd_vel_nav_safe
                                                        ┐
  Web manual control ----------------------> /cmd_vel_web│
                                                        v
                                           priority mux -> /cmd_vel -> Isaac
  ```

- 手动命令优先于 Nav2；手动命令超时后自动回到安全的零速度/导航状态。
- 仿真目标容差目前为位置 `0.08 m`、朝向 `0.15 rad`。
- 仿真基线 footprint 为 `0.66 × 0.62 m`，每边已包含 `0.02 m` 余量。
- 支持前向点云障碍、动态障碍侵入和点云断流停车的专项 benchmark。
- 支持可选 Nvblox ESDF costmap profile，关闭开关即可回退到 PointCloud2 基线。

注意：地图中“看起来有空地”不等于车能通过。规划器还会计算完整 footprint、余量、栅格
分辨率和转弯空间，不能用继续减小 inflation 的方式掩盖真实几何约束。

### 3.5 Nvblox 三维重建

- 使用单 RGB-D 与 odom TF 实时融合 TSDF、彩色 Mesh 和二维 ESDF slice。
- 网页可直接显示 `/nvblox_node/mesh`，并正确按 Mesh 的全局顶点坐标缓存增量块。
- 仿真已验证低矮、悬空、货架障碍的标记和清除。
- 已完成 600 秒稳定性验证：深度约 10 Hz、ESDF 约 4.7 Hz，显存和温度在可用范围内。

Nvblox 不拥有定位 TF，也不替代 RTAB-Map；默认导航仍以 RTAB-Map 与点云安全链为基线。

### 3.6 目标三维感知

- 支持基于 HSV 的已知颜色几何目标，用于确定性工程验收。
- 支持 torchvision Mask R-CNN 的通用 COCO 实例分割。
- 将 RGB mask 与对齐深度融合，输出 2D/3D detection、目标位姿、尺寸、协方差、深度质量和
  debug 图。
- 支持类别门控的跨帧三维 ID 跟踪。
- Isaac 20 cm 方块恢复尺寸约 `20.01 × 20.25 cm`；0.4～1.0 m 四档各 100 帧均 100% 检出。

通用 COCO 模型不包含项目特定类别；真实 D435i 连续流和真实场景 ID switch 率仍待录包评测。

### 3.7 主动感知 / 下一最佳视角

- 从多帧三维观测维护 Target Belief，融合位置协方差并拒绝乱序和 Mahalanobis 离群点。
- 根据目标、可见性、地图和机器人运动约束生成固定相机底盘 NBV 候选。
- 调用 Nav2 `ComputePathToPose` 检查候选可达性并综合路径长度与预期信息增益评分。
- 发布全部候选和最佳视角供 RViz 检查。
- 执行器默认 DISARMED；必须在启动时允许，并在运行时再次调用服务，才会发送 Nav2 目标。
- 支持目标切换取消、重复/过期拒绝、超时取消和执行前后指标记录。

状态：Isaac 中已完成“移动到新视角—重新观测—融合”的 A4 闭环；当前研究重点是改进视角
收益模型和跨视角系统偏差，而不是继续扩展执行权限。

### 3.8 双 OpenArm 与 MoveIt

- 使用 Enactic 官方 OpenArm v1.0 描述、MoveIt 语义、限位和控制器配置。
- 统一 URDF 组合底盘、双臂和 D435i。
- 支持不访问 CAN 的双臂 MoveIt mock 规划执行。
- Planning Scene 支持工作台、挡板、层板和遮挡物。
- 有确定性的 home、自碰撞、夹爪—工作台碰撞回归。
- 修改安装高度或碰撞几何后，可重新运行固定种子采样检查。

状态：mock 规划和碰撞模型通过；整机双臂真机执行未完成安全验收。

### 3.9 Gaussian Splatting 数据支线

- 从仿真或实机 RGB-D、CameraInfo 和 TF 同步采集 Nerfstudio 数据集。
- 输出 RGB、毫米深度、`transforms.json`、时间轴和稀疏彩色点云。
- 提供离线 Splatfacto 训练/导出封装。

该功能是离线重建支线，不参与实时底盘控制。8 GB GPU 上应与在线 Isaac/Nav2 错峰运行。

### 3.10 草菇棚架感知（当前专项）

- 原始 `Gupeng.STL` 是不可按零件名拆分的整体设备。默认仿真场景改用独立参数化三层架，
  按 4×4 排列；原始 CAD 只保留为对照调试模式。
- `rack_detector` 将 Isaac 语义标签转换为逐架逐层实例掩膜；`rack_geometry` 用对齐深度
  反投影并提取高度、种植面边界、前沿与朝向；`rack_map` 跨帧保留稳定架/层 ID 和遮挡记录。
- `harvest_planner` 输出候选停车 `PoseStamped`，`rack_alignment` 以 15 Hz 输出距离、横向
  和角度误差；当前没有将这些输出接入自动停车控制闭环。
- 仿真使用真值 `odom -> sim_world` 和 D435 等效 RGB-D。单架多视角验收为 223/223 帧
  保持同一架、三层稳定 ID，收到 318 个对齐样本。
- 2026-10-16 的 4×4 连续验收沿四条通道走完 25/25 个脚本路点，最终 16 架/48 层；
  5236 个地图帧中，完整地图只出现一套稳定 ID。最大层高误差 0.018 m，最大架位误差
  0.303 m。里程计包围盒估算未出现重叠，但最小间隙仅 0.045 m，不能据此认定真实导航安全。

这次全通道移动由验收脚本直接发送 `/cmd_vel`，**不是 Nav2 在棚架内自主规划或避障验收**。
另一个联动 profile 已在 2026-10-16 跑通网页手动前进 0.235 m、在线 RTAB-Map
地图、Nvblox 三维场景和网页发送的单个约 0.4 m Nav2 目标（action `SUCCEEDED`，
最终距目标约 0.071 m）。这是同一通道的短距离冒烟，不是 16 架自主巡航。
原 4×4 验收借助 Isaac 语义架号；新增的无架号对照验收已于 2026-10-20 跑通 25/25 路点、16 架/48 层，架/层 ID 碎片和架 ID 切换均为 0。它仍以 Isaac 准确像素分割和真值 TF 为输入，并不等于真实图像分割或 D435i 实测完成。真实层架实例分割、D435i 数据、
蘑菇定位和停车执行均未完成。2026-10-20 已补充 YOLO26n-seg 实体相机推理节点、训练入口和同接口掩膜输出，但尚无真实棚架标注及训练权重，未做 D435i 实测；见 [YOLO26 实体数据接入](RACK_YOLO26_REAL_DATA.md)。本棚架场景已做 nvblox 短时在线联动冒烟，收到非空 Mesh
和 ESDF 切片，但全温室地图质量及 Nav2 代价地图尚未验收。细节、运行命令及话题见
[rack_perception README](../src/robot_research/perception/rack_perception/README.md)，结果见
[4×4 连续验收报告](../results/rack/grid_acceptance.json)及
[网页/Nav2 联动记录](../results/rack/web_nav2_smoke_20261016.json)，无架号实验见
[方法与限制](RACK_ANONYMOUS_ASSOCIATION.md)和[验收报告](../results/rack/grid_anonymous_acceptance_retry_20261020.json)。

## 4. 最常用启动方式

### 4.1 构建

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/mobile_base_ws/install/setup.bash
colcon build --symlink-install
```

### 4.2 Isaac + Nav2 + 网页 + Nvblox

终端 1 启动 Isaac：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py \
  --ros --enable-nav-rgbd \
  --nav-camera-width 320 --nav-camera-height 240 --nav-camera-hz 10
```

终端 2 启动固定地图定位、Nav2、网页和 Nvblox：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=/media/scc/Windows_D/AAA/robot_research_ws/maps/isaac_indoor_external_odom_04.db \
  rtabmap_mode:=localization initial_pose:="0 0 0 0 0 0" \
  enable_nvblox:=true use_rviz:=false
```

然后访问：

```text
http://127.0.0.1:8080/#visualization
```

局域网其他设备使用运行主机 IP 代替 `127.0.0.1`。浏览器缓存旧前端时使用 `Ctrl+F5`。

### 4.3 在线建图

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=$HOME/.ros/isaac_mobile_base_online.db \
  rtabmap_mode:=mapping fresh_map:=false enable_nvblox:=true
```

`fresh_map:=true` 会要求建立新地图。确认数据库目标无误后才使用，避免破坏已有基线。

### 4.4 底盘 mock

```bash
source /media/scc/Windows_D/AAA/mobile_base_ws/install/setup.bash
ros2 launch mobile_base_bringup mock.launch.py
```

### 4.5 实车底盘

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
./start_robot.sh
```

该脚本会配置 CAN 并检查设备。首次或故障后测试必须架空轮组，并遵循
[`mobile_base_ws/启动与实车操作.md`](../../mobile_base_ws/启动与实车操作.md)。

### 4.6 双臂 MoveIt mock

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py
```

### 4.7 主动感知

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_active_perception.launch.py
ros2 service call /active_perception/compute_next_best_view std_srvs/srv/Trigger '{}'
```

默认只规划和显示，不自动移动。不要在未检查候选与路径时启用 NBV execution。

### 4.8 三层棚架仿真

网页控制、在线建图、Nav2 与层架感知一起启动（只开这一个仿真实例）：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
colcon build --symlink-install --packages-select mobile_navigation_bringup mobile_manipulator_bringup
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack_navigation.launch.py
```

浏览器打开 `http://127.0.0.1:8080/#visualization`。网页选择 NORMAL 模式后可手动
低速移动，也可在已建出的二维地图上发送 Nav2 目标；地图随移动在线更新。该入口默认
4×4 独立架、Nvblox 开启，RTAB-Map 数据库为 `~/.ros/rack_grid4x4_online.db`，
重启不会主动清空。网页显示二维地图上的架号、三维环境中的每层稳定 ID 和可滚动的
架/层编号列表；RViz 的 `/rack_map/markers` 仍可用于交叉核对。RViz 固定坐标系选
`sim_world`。只测感知时改用 `sim_rack.launch.py`，单架加 `rack_layout:=single`，
完整 CAD 对照加 `rack_layout:=cad`。无界面启动联动 profile 时应显式给出
`headless:=true test_seconds:=180`。完整复现见专项 README。

## 5. 关键接口

| 接口 | 作用 |
|---|---|
| `/joint_states` | 底盘和双臂关节反馈 |
| `/swerve_controller/odom` | 底盘连续里程计 |
| `/cmd_vel_web` | 网页手动速度输入 |
| `/cmd_vel_nav_safe` | 通用 Isaac 导航 profile 中 Nav2 经 Collision Monitor 后的速度 |
| `/cmd_vel_nav_smoothed`、`/cmd_vel_muxed` | 棚架联动 profile 中，Nav2 平滑速度与网页速度进入 mux 后送 Collision Monitor 的接口 |
| `/cmd_vel` | 最终仿真底盘速度；棚架联动 profile 中由 Collision Monitor 发布 |
| `/rtabmap/map` | 二维占据地图 |
| `/navigate_to_pose` | Nav2 导航 action |
| `/nvblox_node/mesh` | Nvblox 彩色三维 Mesh |
| `/nvblox_node/static_map_slice` | Nvblox 二维 ESDF slice |
| `/target_perception/tracked_detections_3d` | 跨帧三维目标 |
| `/active_perception/target_belief` | 目标融合状态 |
| `/rack_detector/debug/color_mask` | 人可读的层架实例分割图 |
| `/rack_map/model`、`/rack_map/markers` | 稳定层架模型与 RViz 标记 |
| `/rack_alignment/errors` | 停车控制器可使用的误差，当前不直接控制底盘 |

TF 唯一所有者：

| TF | 所有者 |
|---|---|
| `map -> odom` | RTAB-Map |
| `odom -> base_link` | 实车控制器或 Isaac |
| `odom -> sim_world` | 仅棚架仿真中的 Isaac 真值 TF；避免与 RTAB-Map 的 `map -> odom` 争夺 odom 父节点 |
| `base_link -> robot links` | `robot_state_publisher` |
| `base_link -> nav_camera_*` | 统一整机描述；真机设备内部 TF 由 RealSense 驱动负责 |

## 6. 配置与数据在哪里

| 内容 | 位置 |
|---|---|
| 底盘首次配置模板 | `mobile_base_ws/src/mobile_base_bringup/config/chassis.yaml` |
| 网页保存后的实机配置 | `~/.config/mobile_base/chassis.yaml` |
| Nav2 参数 | `robot_research_ws/src/robot_research/navigation/mobile_navigation_bringup/config/` |
| 正式 Isaac 地图 | `robot_research_ws/maps/` |
| Isaac 启动脚本 | `robot_research_ws/tools/isaac_sim/` |
| 感知模型 | `robot_research_ws/models/` |
| 实验结果 | `robot_research_ws/results/` |
| rosbag / 数据集 | `bags/`、`datasets/`，通常不进入 Git |

注意：网页加载的是 `~/.config/mobile_base/chassis.yaml`（如存在），不是始终加载仓库模板。
当前运行配置可能与 Git 中的默认轴距、轮距和轮径不同，定位几何问题时必须先查询
`GET /api/chassis` 或检查该用户配置文件。

## 7. 验证与排障

### 7.1 测试

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
colcon test
colcon test-result --verbose

cd /media/scc/Windows_D/AAA/robot_research_ws
colcon test
colcon test-result --verbose
python3 tools/check_architecture_docs.py
```

修改单个包时优先使用 `--packages-select`。网页 footprint/三维车体与底盘配置已有静态测试；
导航参数、launch profile 和 22 包架构目录也有自动检查。

### 7.2 快速运行检查

```bash
ros2 node list
ros2 topic hz /sensors/nav_camera/color/image_raw
ros2 topic hz /sensors/nav_camera/aligned_depth_to_color/image_raw
ros2 topic hz /rtabmap/rgbd_image
ros2 topic info /cmd_vel_smoothed
ros2 topic info /cmd_vel_nav_safe
curl -fsS http://127.0.0.1:8080/api/status
curl -fsS http://127.0.0.1:8080/api/scene3d
```

### 7.3 常见问题

- 网页点击目标不动：检查 Nav2 action 是否 active、地图坐标是否可达、目标是否已落在位置容差内、
  `/cmd_vel_smoothed -> collision_monitor -> /cmd_vel_nav_safe -> mux -> /cmd_vel` 是否完整。
- 空地不能规划：先看完整 footprint、未知区域、障碍伪影和通道宽度，不要只看机器人中心点。
- 网页仍只有箭头：强制刷新浏览器，确认加载 `/app.js?v=10`。
- Nvblox 没有三维数据：检查 RGB、深度、CameraInfo、相机 TF 和 `/nvblox_node/mesh`。
- RTAB-Map 有同步警告：先测四条话题频率；不能只根据一条周期性 warning 判断数据中断。
- 找不到包：确认加载顺序是 ROS 2 → 底盘工作区 → 科研工作区。
- RealSense 无数据：先用 `lsusb` 确认 USB 层是否枚举到设备。

## 8. 已知限制和下一步优先级

1. **实机安全优先**：完成急停、断电、轮子方向、机械零位、速度上限和不同载荷制动距离验收。
2. **相机安装与外参**：固定 D435i、绑定序列号并标定真实 `base_link -> nav_camera`。
3. **Collision Monitor 标定**：仿真速度链已完整，但点云停车区必须按真实盲区和制动距离重新启用/标定。
4. **实机导航闭环**：低速验证 RTAB 定位、唯一 TF、单点导航、动态障碍和断流停车。
5. **两台额外 D435**：机械位置尚未确定，不应提前创建虚假腕部相机 TF。
6. **目标感知实测**：录制真实 RGB-D，评估检测率、深度误差和跨帧 ID switch。
7. **主动视角研究**：当前执行闭环已完成，下一步是收益模型和跨视角系统偏差，不应先扩大自动执行权限。
8. **双臂真机**：MoveIt mock 通过不代表硬件安全；真机执行需单独完成控制器、限位和碰撞验收。
9. **棚架专项下一阶段**：无架号仿真关联已通过；下一步采集真实层架图像并训练实例分割，接 D435i 与定位，在噪声和遮挡下重测多架 ID；停车规划与实时对齐先单独验收，再由用户后续改进导航。
   当前脚本路线最小估算间隙只有 4.5 cm，不应作为实机可通行证明。

## 9. 安全和停止原则

- 网页、Nav2、Collision Monitor 都不能替代实体急停和人工断电。
- 实机电机运动前必须架空对应轮组、现场有人监护并准备断电。
- 主动感知执行默认关闭，不能为了演示删除双门控。
- 不要让 Nvblox、RTAB-Map 或感知失效绕过原始传感器安全链。
- 停止某个 ROS/Isaac 进程使用该终端的 `Ctrl+C`；**停止程序不等于关闭计算机**。
- 只有用户明确要求关闭主机，且实机已按操作手册停车断电时，才执行系统关机。

## 10. 文档阅读顺序

1. 本文：快速交接与能力边界。
2. [架构导航](architecture/README.md)：系统图、依赖、TF、安全边界。
3. [运行时数据流](architecture/RUNTIME_FLOWS.md)：建图、导航、网页、Nvblox、主动感知时序。
4. [ROS 包目录](architecture/PACKAGE_CATALOG.md)：22 个包的职责和稳定接口。
5. [软件边界与实施状态](ARCHITECTURE.md)：详细完成项和历史决策。
6. [底盘启动与实车操作](../../mobile_base_ws/启动与实车操作.md)：上电、CAN、停车和关机。
7. 各功能实施计划：`NAVIGATION_IMPLEMENTATION_PLAN.md`、`NVBLOX_IMPLEMENTATION_PLAN.md`、
   `ACTIVE_PERCEPTION_IMPLEMENTATION_PLAN.md` 等。

## 11. 修改工程时的交接要求

- 新增/删除包、顶层 launch、稳定接口、TF 所有者或安全链时，同步更新 `docs/architecture/`。
- 参数只在 YAML 和所属包 README 维护，避免在多个文档复制后产生不一致。
- 新实验结果放入 `results/`，说明运行条件、输入版本、输出指标和 PASS/FAIL 标准。
- 不覆盖正式地图、数据库、bag 或既有结果；新运行使用新的、有含义的文件名。
- 工作区可能包含大量尚未提交的用户改动。修改前先看 `git status`，不要 reset、checkout 或删除
  不属于当前任务的文件。
