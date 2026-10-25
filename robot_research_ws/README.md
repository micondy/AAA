# robot_research_ws

> 新接手本工程请先看
> [移动操作机器人工程交接文档](docs/HANDOFF.md)：其中集中说明当前能做什么、启动方式、
> 能力边界、已知限制和下一步优先级。
>
> 新成员或准备修改跨包接口时，先看
> [移动操作机器人架构导航](docs/architecture/README.md)：其中包含 Mermaid 系统图、
> 运行时序、全部 ROS 包职责和文档维护规则。

ROS 2 Jazzy 科研算法工作区。硬件包含一台 D435i 和两台 D435；当前代码包含 D435i 单机
驱动入口、实验数据工具和 RTAB-Map 基线，多相机启动与验收尚未实现；
当前分两条可并行主线：固定 D435i 的通用室内建图、地图定位、Nav2 与前向点云障碍；
以及“自制底盘 + OpenArm + 计划固定安装的 D435i”的统一仿真骨架和主动感知；另外两台 D435
尚未确定机械安装位置，不预设为腕部相机；
本工作区同时作为顶层研究 overlay，组合 OpenArm、整机模型和论文算法。OpenArm 的描述、
ROS 2、MoveIt 2、硬件接口和 CAN 库全部来自工程内固定版本的 Enactic 官方源码，不引用
任何个人 OpenArm 工作区。

## 当前包

- `d435i_bringup`：D435i 分阶段启动、参数与 RViz2 配置。
- `experiment_tools`：相机健康检查、rosbag 录制与回放。
- `rtabmap_bringup`：RTAB-Map RGB-D 离线/在线建图与数据库重定位。
- `nvblox_bringup`：默认关闭的单 RGB-D 局部 TSDF、Mesh 与 ESDF 重建层；不拥有定位，
  第一阶段不接管 Nav2 costmap。
- `gaussian_splatting_bringup`：从仿真或实机 RGB-D、CameraInfo 与 TF 采集 Nerfstudio
  数据集，生成毫米深度、时间轴和稀疏彩色点云，并封装离线 Splatfacto 训练/导出入口。
- `mobile_navigation_bringup`：固定 D435i 建图、地图定位、Nav2 与前向点云障碍。
- `target_perception`：相机来源无关的 RGB-D 目标观测；已有通用 COCO 日常物体实例分割，
  输出标准 2D/3D detection、带协方差位姿、深度质量指标和可视化叠加图。
- `rack_perception`：Isaac 三层棚架实例掩膜、RGB-D 几何提取、跨帧稳定架/层 ID、
  停车位姿候选与 15 Hz 实时对齐误差；[使用与验收说明](src/robot_research/perception/rack_perception/README.md)。
- `active_perception`：关注目标选择、目标多帧高斯 belief、协方差融合、离群门控、可见性、
  固定相机底盘 NBV 候选生成、地图过滤和 Nav2 路径评分。
- `mobile_manipulator_description`：自研底盘、Enactic 官方 OpenArm v1.0 与官方 D435i
  描述的统一模型；相机位姿当前是可替换的临时值。
- `mobile_manipulator_moveit_config`：复用 Enactic 官方 MoveIt/ros2_control 参数的整机
  双臂规划与安全 mock 启动入口。
- `mobile_manipulator_bringup`：整机 profile 编排与稳定接口；mock 入口统一底盘、双臂、
  `/cmd_vel`、`/odom`、TF 和仿真真值，薄 `sim_perception` 入口统一 Isaac RGB-D、相机 TF
  与目标感知。

## 构建

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

验证：

```bash
colcon test
colcon test-result --verbose
```

## 当前工程状态

- D435i 独立驱动、数据检查、rosbag 和 RTAB-Map 基线已经具备；实机固定安装与外参尚未
  完成。
- 自制四轮四转底盘、官方 OpenArm v1.0 和官方 D435i 描述已经组成统一 URDF；MoveIt
  mock 规划执行、底盘低速控制和碰撞回归已经通过。
- Isaac Sim 中的组合机器人、前向 RGB-D、递增 `/clock` 和已知尺寸测试物体已经可运行。
- `target_perception` 已完成第一版相机来源无关的目标三维观测。20 cm 仿真方块恢复尺寸约
  为 20.01 × 20.25 cm，可见表面深度与仿真真值一致。
- 通用 Mask R-CNN 前端已在 RTX 5090 和 ROS 2 RGB-D 链路中通过：固定样例发布两个 person
  三维实例；默认类别是常见 COCO 物体，不含蘑菇。真实 D435i 连续流尚待相机重新接入。
- `sim_perception` 一键入口已经运行验收：`base_link -> nav_camera_color_optical_frame` 唯一
  可用，目标检测已从相机坐标转换到 `base_link`；已知方块前表面中心恢复为
  `(1.1002, 0.0012, 0.4301) m`。
- 单进程多距离基准已完成：0.4、0.6、0.8、1.0 m 各 100 帧均 100% 检出，位置 RMSE
  分别为 0.834、1.754、1.716、3.083 mm；逐帧 CSV 和汇总保存在
  [`results/target_perception/distance_20260911_4x100`](results/target_perception/distance_20260911_4x100)。
- 通用 MoveIt Planning Scene 已加入工作台、后挡板、上层板和遮挡板；左右臂 home 安全、
  固定自碰撞姿态和夹爪—工作台碰撞姿态均通过运行态回归。
- `active_perception` Target Belief 第一版已完成：两帧合成观测使位置方差约减半，明显离群
  观测由 Mahalanobis 门控拒绝；Isaac 100 帧连续流位置 RMSE 为 1.660 mm，最终协方差迹
  为 `3.48e-6 m²`，结果见
  [`results/active_perception/belief_20260911_clean_100.json`](results/active_perception/belief_20260911_clean_100.json)。
- A3 联合仿真已通过：自制底盘、顶部 RGB-D、RTAB-Map、Nav2、目标跟踪、Belief 与 NBV
  在同一运行链中接通；36 个候选中有 3 个进入 Nav2 路径查询，最佳路径 2.954 m、预期信息
  增益 0.9210 nats、总分 0.9877。当前只输出最佳视角，不自动驱动底盘。
- A4 已加入默认禁用的最佳视角安全执行器：只有启动时显式允许且运行中再次调用执行服务
  才会发送 Nav2 目标；支持过期/重复拒绝、目标切换取消、超时取消以及运动前后 Belief
  指标 JSON。首次运动被 `FrontStop` 正确安全停止，因此候选半径已增至 1.2/1.4/1.6 m；
  随后修复单个 Nav2 路径请求阻塞整轮规划的问题；第三次运行进一步证明只检查候选终点
  不够，现已加入整条路径相对目标的 1.1 m 净空过滤。第四次运行确认该过滤能在运动前拒绝
  危险路径，并定位到合成绿方块与场景绿长凳的颜色目标混淆；仿真 profile 已收窄专用 HSV
  范围。2026-09-19 最终闭环在 49.18 s 内到达、0 次恢复并融合 126 个新观测，A4 工程验收
  PASS；但该新视角的协方差迹反而增大，视角收益模型与跨视角系统偏差仍是下一研究任务。记录见
  [`results/active_perception/a4_joint_sim_20260917.md`](results/active_perception/a4_joint_sim_20260917.md)。
- 当前没有腕部相机；两台额外 D435 也没有分配安装位置，不创建对应的虚假 TF 或模型。
- Isaac ROS nvblox 4.5 已接入为可选局部重建及Nav2 ESDF profile；Isaac 室内静止场景已确认
  10.1 Hz 深度融合、TSDF/Mesh 与 4.7 Hz ESDF 输出；1.126 m短矩形路线8/8完成，移动
  监测中Mesh 3.88 Hz、ESDF 4.64 Hz且内容非空。600秒稳定性验收也已PASS：深度最大
  间断0.184 s，显存平均3305 MB/峰值3452 MB，温度峰值66 °C。默认仍使用RTAB-Map
  定位和现有Nav2点云障碍层；可切换的Nvblox Nav2 profile已完成4/4点功能初测及低矮、
  悬空、货架障碍的ESDF标记/清除。
- 草菇棚架专项已在 Isaac 中用 4×4 独立三层架取代不可拆分的整体 `Gupeng.STL` 作为默认
  场景。单架多视角和全阵列脚本巡行已通过：后者为 25/25 路点、16 架/48 层、完整地图
  一套稳定 ID；[2026-10-16 验收报告](results/rack/grid_acceptance.json)记录了误差与间隙。
  这里是仿真真值语义 + 真值 TF 的感知验收，不是实机视觉、Nav2 自动停车或碰撞安全验收；
  脚本路线最小估算间隙仅约 4.5 cm，导航方案由后续工作改进。
  `sim_rack_navigation.launch.py` 现可一次启动棚架、网页、RTAB-Map 在线地图、Nav2 和
  Nvblox；[联动冒烟记录](results/rack/web_nav2_smoke_20261016.json)证明网页手动控制和
  单个同通道短距离 Nav2 目标跑通，尚非全阵列 Nav2 自主巡行验收。

## D435i

```bash
# RGB、深度、IMU、对齐深度和点云
ros2 launch d435i_bringup camera.launch.py stage:=full

# 不启动 RViz2，适合远程或性能检查
ros2 launch d435i_bringup camera.launch.py stage:=basic use_rviz:=false

# 数据健康检查
ros2 launch experiment_tools monitor_camera.launch.py stage:=full

# 录制与回放；每次录制请换一个新的输出目录
ros2 launch experiment_tools record_camera.launch.py \
  stage:=full output:=bags/d435i_001
ros2 launch experiment_tools play_camera.launch.py bag:=bags/d435i_001
```

D435i 当前仍按独立手持传感器运行，RViz2 固定坐标系是 `camera_link`。整机描述已经为将来
刚性安装提供 `nav_camera_mount_xyz/rpy` 参数，但默认 `0.32 0 0.42`、`0 0 0` 仅是仿真
临时值，不是实测外参；底盘上机前必须量取并标定。

如果启动日志提示 `No RealSense devices were found`，先运行 `lsusb` 确认 USB 层是否出现 RealSense 设备；这类提示不是 launch 包名或 ROS 话题配置错误。

## 目标三维感知基线

推荐使用已验收的一键最小入口：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_perception.launch.py
```

该入口启动 Isaac 组合体、等效 RGB-D、20 cm 绿色测试方块、统一相机 TF 和目标感知，输出
默认在 `base_link`。它不会连接 CAN，也不会启动 MoveIt、Nav2、RTAB-Map、RViz 或真机
RealSense。更细的开关见
[`mobile_manipulator_bringup/README.md`](src/robot_research/integration/mobile_manipulator_bringup/README.md)。

主要输出：

```text
/target_perception/detections_3d
/target_perception/tracked_detections_3d
/target_perception/detections_2d
/target_perception/target_pose
/target_perception/metrics
/target_perception/debug/mask
/target_perception/debug/instances
/target_perception/debug/overlay
```

当前绿色几何体检测只是用于验证 RGB-D、TF、三维反投影、协方差和实验接口的工程基线，
不作为论文创新点。参数和验收边界见
[`target_perception/README.md`](src/robot_research/perception/target_perception/README.md)。

真实 D435i 的通用物体分割与 RViz 一键入口：

```bash
ros2 launch mobile_manipulator_bringup real_instance_perception.launch.py
```

## RTAB-Map RGB-D 基线

建议先对固定 rosbag 重复跑离线建图，再接实时相机：

```bash
# 离线建图；确认数据库路径后，首次重建才显式传 fresh_map:=true
ros2 launch rtabmap_bringup offline.launch.py \
  bag:=bags/d435i_001 database_path:=$HOME/.ros/d435i_001.db fresh_map:=true

# 在线建图
ros2 launch rtabmap_bringup online.launch.py \
  database_path:=$HOME/.ros/room_001.db fresh_map:=true

# 加载已有数据库定位；文件不存在时会拒绝启动
ros2 launch rtabmap_bringup localization.launch.py \
  database_path:=$HOME/.ros/room_001.db
```

默认不会删除数据库。当前基线使用 RGB、对齐深度和相机内参，不直接使用尚未经过姿态
滤波的 D435i IMU。完成情况和实机验收项见
[RTAB-Map RGB-D 基线实施计划](docs/RTABMAP_IMPLEMENTATION_PLAN.md)。

## 固定 D435i 建图与导航

D435i 刚性安装并提供实测 `base_link -> nav_camera_link` TF 后：

```bash
# 建图
ros2 launch mobile_navigation_bringup mapping.launch.py \
  serial_no:=D435I_SERIAL \
  odom_source:=external odom_topic:=/swerve_controller/odom \
  database_path:=$HOME/.ros/mobile_robot_map.db fresh_map:=true

# 加载数据库定位并启动 Nav2/前向障碍
ros2 launch mobile_navigation_bringup navigation.launch.py \
  serial_no:=D435I_SERIAL \
  odom_source:=external odom_topic:=/swerve_controller/odom \
  database_path:=$HOME/.ros/mobile_robot_map.db
```

第一版只使用前向运动模型，不允许 Nav2 自动横移，也删除了无后向视野条件下的倒车恢复。
逐项实机验收见[固定 D435i 导航实施计划](docs/NAVIGATION_IMPLEMENTATION_PLAN.md)。

当前自制底盘仿真推荐同时运行 RTAB-Map 在线增量建图和 Nav2，地图会随运动更新：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=$HOME/.ros/isaac_mobile_base_online.db \
  rtabmap_mode:=mapping fresh_map:=false
```

只有固定地图回归时才使用已有正式地图和定位模式。该地图使用底盘外部 odom 建立，
0.5×0.5 m 闭环 8/8 完成，平移闭合误差 2.79 cm：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  database_path:=/media/scc/Windows_D/AAA/robot_research_ws/maps/isaac_indoor_external_odom_04.db \
  rtabmap_mode:=localization initial_pose:="0 0 0 0 0 0"

python3 install/mobile_navigation_bringup/lib/mobile_navigation_bringup/navigation_waypoint_benchmark.py \
  --config install/mobile_navigation_bringup/share/mobile_navigation_bringup/config/isaac_indoor_external_odom_04_waypoints.yaml
```

第二条命令默认只预览四点；在 RViz 检查地图、代价地图和目标位置后增加 `--execute` 才会
实际发送。工具现已支持任意数量的 waypoint，也可用 `--start-index/--end-index` 只跑一段。

底盘电池正在安装但整车安全尚未验收；此前已用 Isaac Sim 官方 Carter 和单前向 RGB-D
等效相机验证上层
RTAB-Map、定位、Nav2 最小闭环；后续仿真切换到自制四轮四转底盘与底盘固定 D435i
等效相机。仿真阶段划分、接口契约和验收标准见
[Isaac Sim 感知—建图—导航闭环验证计划](docs/ISAAC_SIM_VALIDATION_PLAN.md)。

2026-09-12 已在 Nova Carter 仓库场景完成短闭环建图、数据库重载定位及一次约 0.5 m 的
`NavigateToPose`：Nav2 返回 `SUCCEEDED`。可用地图位于
`maps/isaac_carter_loop_right_20260912.{db,pgm,yaml}`。Carter 只保留为历史冒烟记录；自制
底盘仿真已另行完成四点导航、分类障碍标记/清除、动态侵入和点云断流停车。仿真结果仍不能
替代实车安全验收。

## 与底盘联调

需要同一终端同时访问底盘和科研包时，按依赖顺序加载：

```bash
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/mobile_base_ws/install/setup.bash
source /media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash
```

也可以使用固定顺序脚本：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
```

`openarm_description 1.0.4`、`openarm_ros2 0.9.2` 和 `openarm_can 1.3.4` 都固定在
`src/vendor`，来源见 [`src/vendor/README.md`](src/vendor/README.md)。加载后所有 OpenArm
ament 包都应指向 `robot_research_ws/install`；`openarm_can` 是纯 CMake 包，通过工程内
`install/openarm_can/lib/cmake/OpenArmCAN` 提供。科研数据目录 `bags/`、colcon 产物和
缓存默认不会提交。

## 规划边界

当前必须完成的基础功能、导航基线的冻结条件、P01/P02 测试点含义，以及后续功能边界见
[工程范围、基础功能与后续完善边界](docs/PROJECT_SCOPE_AND_BASELINE.md)。

后续按可运行实验逐步增加整机启动、目标感知、主动视角和任务编排。固定
D435i 导航链做到可重复到达任务区即可；不等待整套导航完成再开始仿真中的 OpenArm 主动
感知。移动操作任务、OpenArm 组合、整机描述与系统 bringup 归属本工作区的顶层组合层。
只有出现可运行节点、配置或 launch 时才创建 ROS 包，不预建空包；整机描述、MoveIt
mock 配置、统一整机 mock bringup、碰撞回归工具、固定 D435i 参数入口和仿真等效 RGB-D
以及相机安装方式无关的目标观测接口已经建立。

尚未完成：D435i 刚性安装和外参、RTAB-Map 实机验收、Nav2/前向障碍实机验收、目标三维
感知实机标定、Target Belief 实机/移动坐标系评测和主动视角。详见
[架构与实施状态](docs/ARCHITECTURE.md)和
[主动感知工程框架与实施计划](docs/ACTIVE_PERCEPTION_IMPLEMENTATION_PLAN.md)。

## 接下来按顺序实施

1. **[已完成] 薄 Isaac Sim profile 与统一 TF**：入口只启动 Isaac、组合机器人描述、
   nominal 相机光学坐标和目标感知，不拉起 MoveIt/Nav2；`/clock`、相机 TF 和
   `base_link` 三维检测已完成端到端验收。
2. **[仿真完成，实机待测] 多距离误差与重复性评测**：单个 Isaac 进程内依次移动已知
   目标，0.4、0.6、0.8、1.0 m 各记录 100 个严格同步样本；已输出位置/尺寸误差、标准差、
   有效深度率、失败率和 CSV/JSON。正式实机实验时仍需保存对应 rosbag。
3. **[已完成] 通用任务场景进入 Planning Scene**：工作台、后挡板、上层板和遮挡板 box
   已载入；安全 home、自碰撞和夹爪—工作台环境碰撞均有确定性回归。暂不做复杂在线重建。
4. **[仿真基线完成，实机待测] Target Belief**：已实现固定坐标系中的多帧高斯融合、
   协方差、接受/拒绝次数、可见性、乱序拒绝和 Mahalanobis 离群门控；Isaac 100 帧收敛
   数据已固化。
5. **[仿真导航基线已冻结，实机待测] RTAB-Map + Nav2**：Carter 已停止扩展；
   自制底盘已具备 odom/TF、顶部 RGB-D、普通室内地图定位和 Nav2 单目标闭环。低负载
   320×240@10 Hz profile 已验证，`FrontStop` 障碍与 0.75 秒点云断流的软件安全注入通过；
   真实 Isaac RGB-D 障碍放置—停止—移除—恢复也已通过。显式启动初值后 P01 成功；
   P02 曾暴露纯视觉 odom 在低纹理朝向丢失的问题；现已改为底盘连续 odom 发布
   `odom -> base_link`、RTAB-Map 只发布 `map -> odom`。切换后 P01→P02 连续回归 2/2
   成功，P02 后 odom 约 60 Hz 且完整 TF 连续。外部 odom 正式建图路线 8/8 完成，
   1.958 m 路线平移闭合误差 2.79 cm，数据库为
   `maps/isaac_indoor_external_odom_04.db`。新地图四点验收 4/4 成功，总耗时 92.2 秒、
   0 次恢复；结果为
   `results/navigation/waypoints/navigation_waypoints_20260915_100232.json`。动态低矮/悬空
   障碍侵入后前进0.0095～0.0299 m，相机断流0.594 s后fail-safe停车；这些仅为仿真值。
6. **固定相机的底盘 NBV**：围绕目标生成有限底盘候选，通过障碍、Nav2 可达性、预期可见
   性、belief 不确定性下降和运动代价评分；当前没有腕部相机，所以暂不生成机械臂观察
   候选。
7. **实机支线**：D435i 刚性安装后测量并标定 `base_link -> nav_camera_link`，再做普通室内
   RTAB-Map、定位、Nav2 低速单点导航和前向障碍验收。安装电池不等于具备自主运动条件，
   必须先完成急停、看门狗、架空轮向和人工低速制动测试。

普通目标检测/分割、RGB-D 三维融合、多实例评测、跨帧基线 ID、关注目标锁定和单目标
Target Belief 接入和几何可见性/遮挡软件核心已完成。目标切换时会先重置旧 Belief，下一
帧才接收新目标观测，避免混合两个对象。固定相机底盘候选生成、地图过滤、可见性/NBV
评分和 Nav2 `ComputePathToPose` 客户端的软件实现及联合仿真均已通过。下一步是受控执行
最佳视角、到达后重新观测并记录不确定性变化。通用
多目标跟踪仍属于 `target_perception`，暂不建设多目标 belief 数据库，也不增加语义大
模型、三相机融合或腕部相机。

具体接口、模型后端选择和完成条件见
[普通目标检测/分割与 RGB-D 三维观测基线](docs/PERCEPTION_BASELINE_PLAN.md)。

移动底盘、OpenArm、MoveIt 2、Nav2 与安全联锁的总体方案见
[移动机器人项目当前进展](../移动机器人项目当前进展.md)。
