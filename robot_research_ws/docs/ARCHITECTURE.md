# 软件边界与实施状态

> 本文件保留详细实施状态和历史决策。用于快速理解系统的图形化入口是
> [移动操作机器人架构导航](architecture/README.md)，包职责和稳定接口见
> [ROS 包目录](architecture/PACKAGE_CATALOG.md)。

## 代码层与工作区职责

`mobile_base_ws` 只负责底盘：CAN 驱动、四轮四转控制、底盘模型、底盘配置、网页调试与底盘启动。

OpenArm 软件栈全部固定在 `robot_research_ws/src/vendor`，不引用 `/home/scc/ros2_ws` 或
任何个人 fork：`openarm_description 1.0.4`、`openarm_ros2 0.9.2`、
`openarm_can 1.3.4`。详细仓库、tag 和 commit 见 `src/vendor/README.md`。集成代码不修改
这些 vendor 快照。Isaac Sim 侧使用项目内复制的 NVIDIA 官方
`OpenArm/openarm_bimanual/openarm_bimanual.usd`，来源和校验值见
`assets/mobile_manipulator/OFFICIAL_OPENARM_SOURCE.md`。

`robot_research_ws` 是顶层研究 overlay，负责 D435i + 2×D435、定位、建图、导航、整机
组合、MoveIt 整机配置、目标三维感知、主动视角、任务编排和实验评估。无需再创建
`robot_system_ws`。workspace 表示构建依赖层，功能边界由 package 表达。

标准加载顺序：ROS 2 → `mobile_base_ws` → `robot_research_ws`。OpenArm 的描述、MoveIt
配置、bringup、硬件接口和 CAN 库均由最后一层提供。用 `ros2 pkg prefix` 验证 OpenArm
ament 包必须指向 `robot_research_ws/install`；纯 CMake 的 `openarm_can` 则检查
`install/openarm_can/lib/cmake/OpenArmCAN/OpenArmCANConfig.cmake`。

## 当前数据链

```text
固定 D435i (nav_camera) -> RGB-D/IMU -> RTAB-Map -> Nav2（当前主线）
固定 D435i PointCloud2 -> Nav2 costmaps/collision monitor
固定 D435i + odom TF -> Nvblox 局部 TSDF/Mesh/ESDF（可选，默认关闭）
固定 D435i + odom TF -> Gaussian 数据采集（可与 Nvblox 并行）-> 离线 3DGS/4DGS
固定 D435i (nav_camera) -> RGB-D -> 通用实例分割 -> 跨帧 3D ID -> 3D target belief
两台未分配 D435 -> 安装与标定完成后再决定 observation/context 角色
三台相机 -> health monitor / rosbag2 / calibration / benchmark
3D target belief -> active view planner
Isaac 棚架语义/深度 + 真值 TF -> rack_perception -> 稳定架/层模型、停车候选与对齐误差
```

目标感知以后使用 `task_frame` 下的任务局部地图。它不依赖 RTAB-Map、Nvblox 或 Nav2 在线，
必须能够使用固定 rosbag 独立回放和评测。全局地图以后只负责把机器人送到任务区域附近，不能
成为单帧感知、多视角融合或主动视角算法的隐式依赖。

## 已实现

- 官方 ROS 2 RealSense 二进制驱动入口，支持 `basic`、`imu`、`aligned`、`full` 四阶段。
- D435i 参数：深度和彩色均为 848x480@30，点云使用彩色纹理。
- 独立 RViz2、数据频率检查、rosbag2 录制与回放。
- RViz2、健康监测和 rosbag2 共用 `camera_namespace`、`camera_name` 与 `stage`；
  `basic/imu` 记录原始深度，`aligned/full` 记录对齐深度。
- 与底盘描述解耦，独立相机实验不再伪造 `base_link -> camera_link`。
- RTAB-Map RGB-D 基线代码：离线/在线建图、数据库定位、初始参数与 RViz2。
- 固定 D435i 的 `mobile_navigation_bringup`：通用室内建图、数据库定位、Nav2、前向点云
  costmap、速度平滑和碰撞停止区。
- `mobile_manipulator_description`：自制底盘与 Enactic 官方 OpenArm v1.0 的统一 URDF；
  NVIDIA 官方 OpenArm USD 与底盘的 Isaac 组合层已通过 10 秒静置物理验收。
- `mobile_manipulator_moveit_config`：直接复用 Enactic 官方 SRDF 语义、IK、限位、MoveIt
  控制器映射和 ros2_control 配置；补充集成层仿真加速度限制后，双臂规划执行已通过，
  默认不加载 CAN 硬件插件。
- `mobile_manipulator_bringup`：统一整机 mock profile 已通过。单一 controller manager
  同时加载既有四轮四转控制器和官方 OpenArm 控制器，发布 `/joint_states`；接口适配节点
  提供 `/cmd_vel`、`/odom`、`world -> odom -> base_link` 和
  `/sim/ground_truth/odom`。0.05 m/s 短时前进、超时停车和底盘移动后的右臂规划均已验收。
- MoveIt 碰撞回归：左右臂零位有效，固定左臂姿态因
  `openarm_left_link4 <-> openarm_body_link0` 接触被拒绝；底盘代理只与固定相邻 link
  进入 Allowed Collision Matrix。关节范围内 6000 个确定性样本未发现手臂能够触及当前
  临时安装高度下的底盘代理。
- 固定 D435i 软件模型：直接组合官方 `realsense2_description`，提供可关闭、安装位姿可
  参数化的 `nav_camera`。真机模式由 RealSense 驱动拥有内部标定 TF，仿真模式由 URDF
  提供 nominal optical frames，避免双发布。
- 自制底盘 + 官方 OpenArm 的 Isaac 组合体已增加 640×480@15 Hz 前向 RGB-D；实测发布
  `rgb8`、`32FC1` 深度、CameraInfo 和 307200 点云，统一 frame ID 为
  `nav_camera_color_optical_frame`。
- `target_perception` 已建立相机来源无关的绿色几何体基线：同步 RGB、对齐深度和内参，
  用 HSV 区域、MAD 深度离群值过滤及针孔反投影输出标准 2D/3D detection、带协方差位姿和
  diagnostics。Isaac 20 cm 方块端到端恢复为约 0.2001×0.2025 m，有效深度率 98.8%。
- `target_perception` 已增加完整 COCO 通用实例分割、mask-depth 三维融合、可视化叠加、
  多实例 100 帧评测器和类别门控三维跟踪基线。原始 detection 保留帧内 ID，跟踪输出使用
  `/target_perception/tracked_detections_3d`；真实 D435i 的 ID switch 率仍待录包评测。
- `mobile_manipulator_bringup/sim_perception.launch.py` 已建立薄仿真感知 profile，仅启动
  Isaac RGB-D、统一机器人描述和目标检测。运行验收得到
  `base_link -> nav_camera_color_optical_frame = [0.3306, 0.0325, 0.4325] m`，检测结果已
  正确转换到 `base_link`；该入口不启动 MoveIt、Nav2、RTAB-Map、RViz 或真机相机。
- 多距离评测已实现为单 Isaac 进程内目标移动，避免 8 GB 显存机器重复初始化卡住。0.4、
  0.6、0.8、1.0 m 各 100 帧的检测率均为 100%，位置 RMSE 为 0.834～3.083 mm；这只是
  确定性渲染基线，真实深度噪声和外参误差仍须实机评估。
- 通用 Planning Scene 已用 YAML 定义工作台、后挡板、上层板和遮挡板，并经 MoveIt 服务
  回读；左右臂 home 有效，自碰撞姿态和左夹爪—工作台碰撞姿态均被确定性拒绝。
- `active_perception` 已创建第一版 Target Belief：在固定坐标系递推融合位置与协方差，
  拒绝乱序/离群观测，并发布接受/拒绝次数和短时可见性。Isaac 100 帧连续流 RMSE 为
  1.660 mm，最终协方差迹为 `3.48e-6 m²`。
- `nvblox_bringup` 已建立单顶部 RGB-D 的局部重建 profile，复用既有 `odom` TF，输出
  TSDF Mesh 与二维 ESDF slice；仿真静止、移动闭环和600秒资源/断流稳定性均已通过。
  Nav2另有默认关闭的 `use_nvblox_costmap` profile：局部代价地图使用ESDF，全局代价地图
  保留RTAB-Map静态层并叠加ESDF；原点云安全监控不变，关闭开关即回退冻结基线。
  仿真已验证低矮、悬空和货架障碍的ESDF标记/清除；顶部相机近端低矮盲区通过1.10 m
  前向停车区提前防护。低矮/悬空障碍动态侵入及0.75 s点云超时停车也已通过仿真回归；
  实机仍需重新测量最低可见高度和不同载荷下的制动距离。
- `gaussian_splatting_bringup` 已提供仿真/实机共用的 RGB-D + CameraInfo + TF 同步采集，
  输出 Nerfstudio `transforms.json`、毫米深度与 RGB-D 稀疏 PLY；采集可与 Nvblox 并行，
  Splatfacto 训练必须放在独立 Python 环境并与 8 GB GPU 上的在线导航错峰运行。每帧保留
  纳秒时间戳，为后续多视角 4DGS 转换预留时间轴。
- `rack_perception` 与 `sim_rack.launch.py` 已完成 Isaac 三层棚架单架和 4×4 参数化场景
  仿真基线。2026-10-16 连续脚本巡行 25/25 路点，最终 16 架/48 层，完整地图只出现
  一套稳定 ID；最大层高误差 0.018 m，架位误差 0.303 m。脚本直接发 `/cmd_vel`，
  不是 Nav2 自主通行验收；最小估算间隙 0.045 m，真实视觉和安全仍待验证。
  `sim_rack_navigation.launch.py` 另提供网页控制、RTAB-Map 在线地图、Nav2 与 Nvblox
  联动；已通过单个同通道短距离 Nav2 目标冒烟，不代表全阵列导航验收。

## 未实现及建议顺序

1. 刚性安装 D435i，按序列号固定为 `nav_camera`，完成数据和外参实机验收。
2. 安装 RTAB-Map 运行时，用普通室内闭环路线完成离线/在线建图和数据库重载。
3. 验收地图定位、唯一 TF、Nav2 低速单点导航和 D435i 前向障碍。
4. 再验收两台 D435 及多相机带宽、丢帧、外参和深度干扰。
5. [完成] 为整机 mock 增加故意碰撞/非碰撞测试位姿，验证 Planning Scene 过滤。
6. [完成] 已把底盘固定 RGB-D 加入 Isaac 组合体，并建立只启动所需组件的薄 Isaac Sim
   profile；未创建尚不存在的腕部相机 TF。
7. [完成] 已实现单帧目标三维感知、`base_link` 输出、0.4～1.0 m 多距离评测和通用
   Planning Scene 碰撞回归。
8. [完成] Target Belief 纯算法、合成 ROS 消息和 Isaac 100 帧连续流收敛评测通过。
9. [完成] 把 CAD 修正后的自制底盘 USD 接入 odom/TF、固定 RGB-D、RTAB-Map 和 Nav2，
   完成建图、导航与前方障碍安全验收。
10. [完成] 固定相机底盘主动视角候选、路径评分及人工双门控执行闭环。
11. [仿真完成，实机待测] 验收可选 Nvblox 单相机 TSDF/Mesh/ESDF；下一步新增独立Nav2
    A/B profile，不覆盖现有点云costmap。
12. [棚架仿真完成，实机待测] 训练真实层架实例分割并接 D435i，去除 Isaac 架号和真值 TF
    后复测多架关联；停车位姿、对齐误差与导航控制的闭环由后续阶段实施。

新增 package 按可运行里程碑创建：先
`mobile_manipulator_description`、`mobile_manipulator_moveit_config`、
`mobile_manipulator_bringup`，之后才创建 `target_perception`、`active_perception` 和
`mobile_manipulation_tasks`。belief、可操作性评价和 whole-body 候选第一版均作为
`active_perception` 内部模块，不单独拆包。

底盘尚未安装电池期间，Nova Carter 已完成 RTAB-Map、地图定位和 Nav2 最小闭环验证，
现只保留为历史对照。当前改用自制四轮四转 USD 与底盘固定 RGB-D 等效相机继续建图、
导航和前向障碍闭环；该阶段仍不替代真实相机和底盘安全的实机验收。详细步骤见
[Isaac Sim 感知—建图—导航闭环验证计划](ISAAC_SIM_VALIDATION_PLAN.md)。

不计划增加网页控制租约平台或大量占位包；这些都不是当前单人科研阶段的必要条件。

RTAB-Map 的逐项状态、验收指标和执行记录见
[RTAB-Map RGB-D 基线实施计划](RTABMAP_IMPLEMENTATION_PLAN.md)。
固定 D435i 到 Nav2 的逐项状态见
[固定 D435i 导航实施计划](NAVIGATION_IMPLEMENTATION_PLAN.md)。

完整的移动底盘—OpenArm—感知—导航—操作架构见根目录
[移动机器人项目当前进展](../../移动机器人项目当前进展.md)。
主动感知工程的 package 边界、完成定义和当前实施顺序见
[移动操作主动感知工程框架与实施计划](ACTIVE_PERCEPTION_IMPLEMENTATION_PLAN.md)。
