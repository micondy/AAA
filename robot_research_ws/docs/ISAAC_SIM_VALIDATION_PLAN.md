# Isaac Sim 感知—建图—导航闭环验证计划

> 决策日期：2026-09-08  
> 计划开始：2026-09-09  
> 当前状态：自制四轮四转底盘外部 odom、RTAB-Map 与 Nav2 单目标闭环已通过  
> 适用平台：Isaac Sim 6.0.1、ROS 2 Jazzy、单台 8 GB 级笔记本 GPU

说明：第 1～6 节保留最初 Carter 验证方案作为历史基线；当前实施主线是第 6 节后半的
自制底盘 S7。当前 TF 所有权以 S7 记录为准，不再以 Carter 的纯视觉里程计为准。
2026-10-27 起，主导航配置已切换为 Smac State Lattice（Omni）+ MPPI Omni；下文的
Regulated Pure Pursuit 与差速描述仅表示 Carter 早期验证环境，不代表当前自制底盘配置。

## 1. 目的与结论

第一阶段不导入自制四轮四转底盘，不复刻 D435i 的具体外观和噪声模型。使用 Isaac Sim
自带移动机器人、简单仓库和一个前向 RGB-D 等效相机，优先回答下面的问题：

> 当前 `robot_research_ws` 的 RTAB-Map、地图定位、Nav2、深度点云障碍和安全速度链能否
> 形成完整闭环？

选择官方现成载体可以先隔离自制底盘模型、四轮四转运动学、CAN 和电机调试造成的干扰。
仿真验收通过后，上层接口保持不变，再分别接入自制底盘仿真模型和真实机器人。

第一版选择：

- 机器人：Isaac Sim 自带 Nova Carter ROS 2 资产；
- 环境：Simple Warehouse；
- 传感器：仅启用一个前向 RGB-D 等效相机；
- 运动模型：差速，只使用 `linear.x` 和 `angular.z`；
- 定位链：RTAB-Map RGB-D 里程计发布 `odom -> base_link`，RTAB-Map 发布
  `map -> odom`；
- 导航链：使用 Nav2 Regulated Pure Pursuit 前向差速基线；
- 障碍链：前向深度点云进入 Nav2 costmap 和 collision monitor；
- 时间源：全部 ROS 2 节点使用 Isaac Sim 的 `/clock`，设置 `use_sim_time:=true`。

本机已发现的官方资产：

```text
/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd
/Isaac/Environments/Simple_Warehouse/full_warehouse.usd
```

本机参考脚本：

```text
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/
  standalone_examples/benchmarks/benchmark_robots_nova_carter_ros2.py
```

## 2. 验证边界

### 2.1 本阶段能够验证

- Isaac RGB、深度、相机内参和点云是否能进入现有 ROS 2 接口；
- RTAB-Map RGB-D 里程计、闭环检测、二维占据地图及数据库保存；
- 加载数据库后的地图定位；
- Nav2 全局规划、Regulated Pure Pursuit 局部控制、速度平滑和目标到达；
- 深度点云对局部/全局 costmap 的标记和清除；
- 前方障碍进入停止区后，collision monitor 是否截断速度；
- `/clock`、QoS、TF、话题名称和 launch 参数是否正确；
- 同一上层软件在“仿真传感器”和“真实 D435i”之间切换是否足够薄。

### 2.2 本阶段不能替代的实机验证

- 四轮四转的转角分配、横移、斜移和真实运动学误差；
- CAN 驱动、电机死区、通信中断、本地看门狗和急停；
- 真实 D435/D435i 的深度噪声、曝光、反光、USB 带宽和温漂；
- 轮胎打滑、结构振动、相机支架外参漂移和真实停车距离；
- OpenArm 载荷变化、高重心和倾覆风险。

因此，仿真通过的结论应写成“上层感知—导航软件闭环可运行”，不能直接写成“实车系统
已经验证”。

## 3. 最小系统架构

```text
Isaac Sim / Simple Warehouse
├── Nova Carter 物理底盘
├── 前向 RGB-D 等效相机
├── /clock
├── RGB / depth / camera_info / PointCloud2
├── 真实物理位姿 -> /sim/ground_truth/odom（只评测，不发布 TF）
└── 接收仿真底盘 /cmd_vel
          ▲
          │ 话题、帧名与 QoS 适配
          │
robot_research_ws
├── rgbd_sync
├── rgbd_odometry -> /rtabmap/odom + odom -> base_link
├── rtabmap -> /rtabmap/map + map -> odom
├── Nav2 Regulated Pure Pursuit（前向差速）
├── velocity_smoother
└── collision_monitor -> /swerve_controller/cmd_vel
          │
          └── 仿真适配 -> Isaac /cmd_vel
```

关键原则：同一条 TF 只能有一个发布者。第一阶段需要验证视觉里程计，因此 Isaac 的真实
物理里程计只发布到独立评测话题，不发布 `odom -> base_link`；该 TF 由 RTAB-Map RGB-D
odometry 唯一发布。真实物理位姿用于计算误差，不参与导航闭环。

## 4. ROS 2 接口契约

Isaac 内部原始话题以实际 ROS graph 为准，不在实现前猜测。适配层必须向现有研究工作区
提供以下稳定接口：

| 功能 | `robot_research_ws` 侧固定接口 | 要求 |
|---|---|---|
| 彩色图 | `/sensors/nav_camera/color/image_raw` | `sensor_msgs/Image` |
| 深度图 | `/sensors/nav_camera/aligned_depth_to_color/image_raw` | 与彩色像素对齐 |
| 相机内参 | `/sensors/nav_camera/color/camera_info` | 与图像分辨率一致 |
| 深度点云 | `/sensors/nav_camera/depth/color/points` | `sensor_msgs/PointCloud2` |
| 安全速度输出 | `/swerve_controller/cmd_vel` | `geometry_msgs/Twist` |
| Isaac 速度输入 | `/cmd_vel` | 由适配层转发安全速度 |
| 视觉里程计 | `/rtabmap/odom` | RTAB-Map 发布，Nav2 使用 |
| 仿真真值 | `/sim/ground_truth/odom` | 仅记录和计算误差 |
| 时间 | `/clock` | 所有算法节点启用仿真时间 |

TF 的目标结构为：

```text
map                         # RTAB-Map 发布 map -> odom
└── odom                    # RTAB-Map RGB-D odometry 发布 odom -> base_link
    └── base_link
        └── nav_camera_link # Isaac/机器人描述提供唯一刚性外参
            └── optical frames
```

若 Nova Carter 资产使用其他 chassis/camera 帧名，应在 Isaac ROS helper 的 frame ID 或整机
描述入口统一改名。只在没有第二条父子路径时使用静态 TF，禁止通过多条静态别名拼出闭环。

## 5. 资源与性能约束

便携开发机按 8 GB 显存预算设计：

- 一台机器人、一个简单仓库；
- 只开一个前向 RGB-D 等效相机；
- 初始使用 640×480、15 Hz，闭环稳定后再试 848×480、30 Hz；
- 不启用 RTX 高质量渲染、域随机化、多机器人；
- 关闭 Nova Carter 不参与本试验的多组 Hawk/Owl 相机和雷达；
- RTAB-Map、Nav2、RViz2 分阶段启动，先区分显存问题和 ROS 接口问题。

实验室 RTX 5090 后续用于三相机、复杂光照、传感器噪声、域随机化和批量消融，不作为
第一天闭环成功的前提。

## 6. 实施阶段

### S0：环境和 ROS 2 Bridge 体检

任务：

- 启动 Isaac Sim 6.0.1，确认 GPU 渲染和 Physics 正常；
- 使用 ROS 2 Jazzy 环境启用 ROS 2 Bridge；
- 加载 Simple Warehouse 和 Nova Carter ROS 2 资产；
- 确认仿真开始/暂停、`/clock` 和基础 TF；
- 使用键盘或一次性 `cmd_vel` 让 Carter 前进、旋转和停止。

通过条件：连续运行 10 分钟不崩溃，仿真时间单调增长，车辆能够稳定响应速度并在零速度
后停止。

### S1：单前向 RGB-D 等效相机

任务：

- 在车体前方增加或保留一台相机；
- 发布 RGB、depth、camera_info 和由深度生成的 PointCloud2；
- 将 frame ID 和话题统一到第 4 节接口；
- 检查图像时间戳、深度单位、相机内参和点云坐标方向；
- 验证点云能转换到 `base_link`。

通过条件：四路传感器话题连续 5 分钟；RGB、深度与内参尺寸一致；前方 1 m 方箱在点云
中的位置和尺度正确；无未来时间戳和 TF extrapolation 错误。

### S2：适配现有 launch，不启动真实 D435i

计划代码位置：

```text
robot_research_ws/src/robot_research/simulation/isaac_sim_bringup/
```

仅在开始实现并能提供可运行 launch、配置和测试时创建该包。预期工作：

- 增加仿真话题/速度适配；
- 让 mapping/navigation 入口在 `start_camera:=false` 时不要求真实相机序列号；
- 加入 `use_sim_time:=true` 的仿真入口；
- 为 Carter 使用独立 `nav2_sim.yaml`，只修改 footprint、速度和适配话题；
- 保持 RTAB-Map 和 Nav2 核心配置与实机版本共用，避免复制整套 launch；
- 添加 TF 单发布者、话题类型、仿真时间和危险速度绕过的静态测试。

通过条件：拔掉 D435i 后仍能启动算法链；系统中不存在 `realsense2_camera` 节点；所有算法
节点使用 `/clock`；Nav2 最终速度只能经过 collision monitor 到达仿真底盘。

### S3：RTAB-Map 仿真建图

任务：

- 在仓库中人工低速走一条短闭环；
- 第一遍采用约 0.15 m/s、0.3 rad/s，避免运动过快；
- 保存独立数据库，例如 `maps/isaac_warehouse_01.db`；
- 同时记录 `/sim/ground_truth/odom`、`/rtabmap/odom`、TF 和诊断信息；
- 检查墙体尺度、地图重影、闭环校正和里程计重置。

通过条件：完成一条闭环且无自动重置；地图不存在明显双墙/重复楼层；数据库可连续三次
加载；用真值计算轨迹误差并记录，而不是只凭 RViz 截图判断。

### S4：地图定位与 Nav2

任务：

- 重启仿真并把机器人放在已建图区域；
- 加载 S3 数据库进入 localization 模式；
- 确认 `map -> odom -> base_link` 唯一且连续；
- 先发送近距离目标，再执行多个目标点；
- 检查全局路径、局部轨迹、目标到达和失败恢复。

通过条件：连续完成 10 个可达目标，成功率至少 90%；不存在盲区倒车；定位短暂失效时不
继续高速运动；每次结果均保存起终点误差和耗时。

### S5：前向障碍和安全速度链

任务：

- 在 0.5、1.0、2.0 m 处分别放置方箱；
- 检查局部 voxel layer、全局 obstacle layer 的标记和清除；
- 将障碍移入前向停止区，检查 collision monitor 状态与最终速度；
- 暂停传感器发布，验证点云超时后的行为；
- 确认不能绕过安全输出直接向 Isaac 底盘发导航速度。

通过条件：障碍被正确加入和清除；侵入停止区后最终速度为零；传感器超时不会继续发出
非零导航速度；无地面或机器人自身形成的永久假障碍。

### S6：结果固化与下一阶段决策

产物：

- 可重复启动的场景/脚本、ROS launch 和参数文件；
- 一份成功 rosbag 和一份故障注入 rosbag；
- 建图数据库、地图截图、TF 图和 ROS graph；
- 轨迹误差、导航成功率、停止距离、CPU/GPU/显存和节点频率表；
- 已知问题与实机对应关系。

S7 把 Nova Carter 换成自制四轮四转 USD。S7 只替换机器人模型和底盘控制适配，不重写
RGB-D、RTAB-Map、Nav2 或评测工具。2026-09-10 已完成 S7-a：URDF 导入、物理稳定性、
四轮运动学、低速运动和 ROS `/cmd_vel`；S7-b 已完成 NVIDIA 官方 OpenArm USD 的项目内
固化、底盘装配和静置稳定性；整机 MoveIt/mock 控制与碰撞回归已经完成。S7-c 已在组合体
增加与官方 D435i nominal 光学坐标一致的底盘固定 RGB-D，并验证四路 ROS 消息。S7-d 已
完成目标观测基线；S7-e 已完成薄 Isaac profile、统一相机 TF 和 `base_link` 输出的端到端
验收。当前没有腕部相机，不创建虚假的腕部传感器模型。多距离重复性评测和通用 Planning
Scene 碰撞回归和 Target Belief 连续流收敛评测已经完成；Carter 已完成 S3 闭环地图与
S4 单目标最小导航，作为上层链路历史基线，到此停止扩展。2026-09-12 已根据自制底盘 CAD
重新建立 REP-103 关节树、显式碰撞/惯量和 0.419×0.450 m 运动学，并导入新版 USD。
自制底盘 profile 已补齐控制器接口 odom/TF 与独立真值，并复用同一套 RGB-D、RTAB-Map
与 Nav2 完成最小导航闭环；独立 `FrontStop` 和点云超时安全注入已通过。P01→P02 旧故障
连续回归已经通过，外部 odom 正式闭环地图也已生成；下一步执行四点代表目标统计、
真实仿真点云障碍清除与 rosbag。主动感知的整体实施顺序见
[`ACTIVE_PERCEPTION_IMPLEMENTATION_PLAN.md`](ACTIVE_PERCEPTION_IMPLEMENTATION_PLAN.md)。

## 7. 下一步执行顺序

1. [完成] 新版自制底盘 USD 静置与运动尺度回归；
2. [完成] 发布 `/sim/ground_truth/odom`、关节状态及一致 TF；
3. [完成] 接入顶部固定 RGB-D、建立普通室内数据库并固化 10 个候选目标；
4. [完成] 已接入约 60 Hz 底盘连续 odom，RTAB-Map 只负责 `map -> odom`；P01→P02
   连续回归均成功，低纹理故障关闭；外部 odom 正式闭环地图已生成；
5. [完成] 新地图四点代表目标 4/4 成功、0 次恢复；
6. [完成] 独立验证 `FrontStop` 停止区和 0.75 秒点云超时不会保留非零速度；
7. [障碍安全链完成] 真实仿真点云已验证障碍触发停止、移除后恢复；下一步固化成功/故障
   rosbag；
8. [导航已通过] 先完成普通目标检测/分割感知基线，再进入固定相机底盘 NBV。

## 8. 验收记录模板

| 日期 | 阶段 | 场景/版本 | 结果 | 指标或错误 | 数据路径 | 下一步 |
|---|---|---|---|---|---|---|
| 2026-09-09 | S0 | Nova Carter Warehouse / Isaac Sim 6.0.1 | 核心通过 | `/clock` 约 57 Hz；0.15 m/s×3 s 前进 0.463 m；0.30 rad/s×3 s 实际转向约 0.73 rad；累计多次稳定运行，单次连续 10 分钟验收待补 | `tools/isaac_sim/run_carter_warehouse.py` | S1 |
| 2026-09-09 | S1 | 前左相机 / 640×480@15 Hz 设置 | 核心通过 | RGB、32FC1 depth、info、307200 点云均正常；严格 RGB-D 同步通过；官方 `base_link -> front_stereo_camera_left_rgb` TF 可用；GUI 实际约 8–12.5 Hz；方箱尺度专项验收待补 | `tools/isaac_sim/README.md` | 隔离真值 TF，启动 RTAB-Map |
| 2026-09-09 | S2 | 现有 mapping launch + Isaac 话题 | 核心通过 | `start_camera:=false` 无 RealSense；真值改到 `/sim/ground_truth/*`；标准 `/tf` 中 RTAB-Map 唯一发布 `odom -> base_link`；补齐空相机序列号默认值 | `src/robot_research/navigation/mobile_navigation_bringup/launch/` | 针孔相机重跑 S3 |
| 2026-09-09 | S3-a | Hawk 旧鱼眼模型 | 不通过，已保留诊断数据库 | 成功生成 263×323、0.05 m 地图和 395 MB 数据库，但短路线后视觉里程计 z 漂移约 1.32 m；原因为 `fisheyePolynomial` 渲染与零畸变 `plumb_bob` 内参不一致 | `maps/isaac_warehouse_01.db` | 覆盖为 pinhole 后重跑 |
| 2026-09-09 | S3-b | Hawk 覆盖为 pinhole | 直线尺度通过，闭环待跑 | 真值直行 1.017 m，视觉估计 1.038 m，误差约 2.1%；z 漂移约 6.6 mm；生成 106×176、0.05 m 地图 | `maps/isaac_warehouse_pinhole_01.db` | 人工低速闭环并保存数据库 |
| 2026-09-10 | S7-a | 自制四轮四转底盘 / Isaac Sim 6.0.1 | 通过 | 静置水平漂移约 0.29 mm；0.45 m 直行实测 0.4473 m；原地旋转和 `/cmd_vel` 超时停车正常 | `tools/isaac_sim/run_mobile_base.py` | 装配 OpenArm 与底盘固定 RGB-D |
| 2026-09-10 | S7-b | 自制底盘 + NVIDIA 官方 OpenArm 双臂 USD | 模型装配通过 | 10 s 静置：水平漂移 0.150 mm，最大横滚/俯仰 0°，最终 base z=0.1000 m；官方手指 mimic schema 保留 | `assets/mobile_manipulator/mobile_manipulator_official.usda` | 整机 MoveIt/mock 控制 |
| 2026-09-11 | S7-c | 组合体固定 RGB-D / 640×480@15 Hz | 接口通过 | 四路话题可发现；frame=`nav_camera_color_optical_frame`；RGB=`rgb8`；depth=`32FC1`；点云=307200 点；`/clock` 已验证递增；10 s 物理稳定性仍通过；5 分钟连续性待专项验收 | `tools/isaac_sim/run_mobile_manipulator.py` | 薄 sim profile 与目标观测接口 |
| 2026-09-11 | S7-d | 20 cm 绿色方块 + `target_perception` | 单帧尺度通过 | 恢复尺寸约 0.2001×0.2025 m；可见前表面深度 0.76955 m 与几何真值一致；6889 个有效深度点、有效率 98.8%；多距离重复性待测 | `src/robot_research/perception/target_perception` | 固定场景 rosbag 与 0.4～1.0 m 误差表 |
| 2026-09-11 | S7-e | `sim_perception` 薄 profile | 端到端通过 | TF `[0.3306,0.0325,0.4325] m`；检测 frame=`base_link`，目标前表面中心 `(1.1002,0.0012,0.4301) m`；未启动 MoveIt/Nav2/RTAB-Map/真机相机 | `mobile_manipulator_bringup/launch/sim_perception.launch.py` | 多距离误差与重复性评测 |
| 2026-09-11 | S7-f | 单进程多距离 RGB-D 基准 | 仿真通过 | 0.4/0.6/0.8/1.0 m 各 100/100 检出；位置 RMSE 0.834/1.754/1.716/3.083 mm；有效深度率 98.1%～100%；Isaac 稳定并干净退出 | `results/target_perception/distance_20260911_4x100` | 已知任务环境 Planning Scene |
| 2026-09-11 | S7-g | 通用 MoveIt Planning Scene | 通过 | 四个 box 由服务回读；左右臂 home 安全；固定自碰撞与夹爪—工作台碰撞姿态被拒绝 | `mobile_manipulator_bringup/config/generic_task_scene.yaml` | Target Belief 连续流评测 |
| 2026-09-11 | S7-h | Isaac Target Belief / 100 帧 | 通过 | 位置 RMSE 1.660 mm；最终误差 1.660 mm；协方差迹由 4.02e-6 降至 3.48e-6 m²；所有进程干净退出 | `results/active_perception/belief_20260911_clean_100` | 固定相机底盘 NBV 候选与评分 |
| 2026-09-12 | S3-c | Nova Carter pinhole RGB-D 闭环 | 通过 | 1.0×0.7 m 右转路线 8/8 段完成；导出 239×297、0.05 m/格地图；左转大路线曾接触货架，失败记录保留 | `maps/isaac_carter_loop_right_20260912.db`、`results/navigation/carter_loop_right_20260912.json` | S4 单目标基线 |
| 2026-09-12 | S4-a | 数据库定位 + Nav2 单目标 | 最小闭环通过 | 修复仿真时钟、Jazzy voxel 参数、Isaac 点云 `height`、动态相机 TF 与 RGB-D 位移补偿；RPP 约 0.5 m 目标返回 `SUCCEEDED`，最终复测无安全层 TF/超时警告；10 目标统计未做 | `maps/isaac_carter_loop_right_20260912.{pgm,yaml}` | S4-b 10 目标，随后 S5 |
| 2026-09-13 | S7-i | 自制底盘外部 odom + RTAB-Map/Nav2 | 单目标通过 | 无 `rgbd_odometry`；底盘 odom 约 60 Hz；P01 39.4 s、1 次恢复并成功；低纹理结束位姿 `map -> base_link` 连续 | `results/navigation/waypoints/navigation_waypoints_20260913_192318.json` | 只跑 P02，再分批补齐统计 |
| 2026-09-13 | S7-j | P01→P02 低纹理故障回归 | 通过 | P01 39.4 s、1 次恢复；P02 6.1 s、0 次恢复；2/2 成功；P02 后 odom 约 60 Hz 且完整 TF 连续 | `results/navigation/waypoints/navigation_waypoints_20260913_193700.json` | 外部 odom 正式重建图 |
| 2026-09-13 | S7-k | 自制底盘外部 odom 正式闭环建图 | 通过 | 0.5×0.5 m 右转路线 8/8；路径 1.958 m；平移闭合误差 2.79 cm；数据库 228 帧、15087 特征，优化连通图 122 位姿 | `maps/isaac_indoor_external_odom_04.db`、`results/navigation/mapping/isaac_external_odom_formal_loop_20260913.json` | 新地图四点导航验收 |
| 2026-09-15 | S7-l | 外部 odom 新地图四点导航 | 通过 | E01～E04 全部 `SUCCEEDED`；4/4 成功；总耗时 92.2 s；0 次恢复；运行后 odom 采样窗口平均约 56 Hz | `results/navigation/waypoints/navigation_waypoints_20260915_100232.json` | 冻结仿真导航基线，进入普通目标感知 |
| 2026-09-09 | S1 | 待填写 | 待执行 |  |  |  |
| 2026-09-09 | S2 | 待填写 | 待执行 |  |  |  |

### 2026-09-09 执行记录

- Isaac Sim 6.0.1、ROS 2 Jazzy、RTX 5060 Laptop 8 GB 启动正常；
- 官方 `carter_warehouse_navigation.usd` 成功加载，首次 RTX shader 编译耗时较长；
- `/clock` 约 57 Hz，`/chassis/odom` 约 59 Hz；
- 0.15 m/s 前进 3 秒，实测里程计位移约 0.463 m；
- 0.30 rad/s 原地旋转 3 秒，实测偏航约 0.73 rad；
- 停止命令后线速度、角速度均回到近零；
- 已增加持续运行入口 `tools/isaac_sim/run_carter_warehouse.py`，它不自行发布速度；
- S0 核心功能通过，连续 10 分钟稳定性观察进行中；S1 尚未开始。

## 9. 完成定义

只有同时满足以下条件，才能认为“当前上层方案在仿真中可用”：

- 建图、保存、重启和地图定位均可重复；
- Nav2 可完成当前新地图的 4 个代表目标点测试；
- 新障碍能够标记、移除后能够清除；
- 前向停止区和传感器超时测试通过；
- TF 无多发布者，算法统一使用仿真时间；
- ROS graph 中只替换了传感器和底盘接口，上层算法没有为 Carter 写专用分支；
- 所有结论有日志、rosbag 或指标文件支撑。

截至 2026-09-15 已达到自制底盘外部 odom 的 P01→P02 连续回归、正式闭环建图、新地图
四点导航 4/4 和确定性障碍安全回归，仿真导航基线冻结。正式论文实机实验仍需在电池、
刚性相机外参和硬件急停具备后补录 rosbag；当前主线进入普通目标检测/分割感知基线。
