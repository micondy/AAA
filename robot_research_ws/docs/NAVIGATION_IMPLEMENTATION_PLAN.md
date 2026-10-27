# 固定 D435i 建图、定位、Nav2 与前向障碍实施计划

更新日期：2026-10-27。

> 状态更新：本文的 N1～N5 前向差速方案保留为早期安全验收记录。当前主导航已切换为
> Smac State Lattice（官方 Omni lattice）+ MPPI Omni，并放开 `vy`。前置 D435i 对侧后方
> 动态障碍仍有盲区，因此全向实车测试必须限速、有人监护，且不能替代实体急停。

当前正在安装底盘电池，但整车供电、急停和看门狗尚未验收，暂不适合落地自主移动。实车
N1～N5 之前，先按
[Isaac Sim 感知—建图—导航闭环验证计划](ISAAC_SIM_VALIDATION_PLAN.md)使用官方 Nova
Carter 和单前向 RGB-D 等效相机验证同一套上层链路。仿真使用 `use_sim_time:=true`、
`start_camera:=false`。当前自制底盘 profile 由底盘里程计唯一发布 `odom -> base_link`，
RTAB-Map 只发布 `map -> odom`。

## 目标

以刚性安装在底盘上的 D435i 为唯一导航相机，完成通用室内环境的：

1. RTAB-Map RGB-D 建图；
2. 加载数据库后的地图定位；
3. Nav2 `NavigateToPose`；
4. D435i 前向 PointCloud2 动静态障碍更新；
5. Nav2 到自研四轮四转控制器的安全速度链。

早期 N1～N5 阶段只使用前向相机并按 `DiffDrive` 模型验收。当前软件已进入全向算法阶段，
但侧向/后向感知覆盖尚未补齐，不能把“算法可输出横移”理解为“实车已具备全向安全能力”。

## 软件链

```text
固定 D435i RGB-D
  + /swerve_controller/odom -> RTAB-Map mapping/localization -> map -> odom
底盘控制器 -> /swerve_controller/odom + odom -> base_link
  -> /rtabmap/map -> Nav2 global costmap

固定 D435i PointCloud2
  -> Nav2 local voxel layer
  -> Nav2 global obstacle layer
  -> collision_monitor 前向停止区

NavigateToPose
  -> Smac State Lattice（Omni）
  -> MPPI（Omni）
  -> velocity_smoother
  -> collision_monitor
  -> /swerve_controller/cmd_vel
```

## 已完成代码

- [x] RTAB-Map 在线、离线和数据库定位基础 launch。
- [x] RTAB-Map 相机序列号和相机 stage 透传。
- [x] `mobile_navigation_bringup` 建图入口。
- [x] `mobile_navigation_bringup` 定位 + Nav2 入口。
- [x] D435i 点云进入局部 voxel layer 和全局 obstacle layer。
- [x] 保守 footprint、MPPI 全车轮廓代价、全向速度平滑和前向碰撞停止区。
- [x] 无倒车恢复的第一版行为树。
- [x] YAML/XML/launch 静态测试和 colcon 构建。
- [x] Nova Carter 1.0×0.7 m 右转闭环建图并保存 239×297、0.05 m/格地图。
- [x] Nova Carter 数据库重载定位和单次约 0.5 m `NavigateToPose`（`SUCCEEDED`）。
- [x] 仿真速度链切换到 `/cmd_vel`，并修复 Isaac 点云无 `height` 字段的问题。
- [x] Carter 固定相机外参改由 `/tf_static` 发布；整帧 RGB-D 关闭扫描式底盘位移补偿。
- [x] Carter 到此作为上层最小闭环历史基线，不再扩展专用测试。
- [x] RTAB-Map 支持 `odom_source:=external`，外部 odom 模式不启动 `rgbd_odometry`。
- [x] Isaac 自制底盘发布 `/swerve_controller/odom` 与 `odom -> base_link`，真值保持隔离。
- [x] 自制四轮四转 USD 已完成 odom/TF、外部 odom 正式闭环地图、四点导航统计和安全专项
  验收；仿真导航基线冻结，实机验收待硬件条件具备。
- [x] 仿真动态障碍侵入：低矮障碍0.10/0.18 m/s、悬空障碍0.18 m/s均停车；相机断流按
  fail-safe归零，结果见`results/navigation/intrusion/`。

## 实机前必须完成

- [ ] 安装并确认 `ros-jazzy-rtabmap-ros`。
- [ ] 记录固定 D435i 序列号和固件版本。
- [ ] 测量相机安装位姿并提供唯一 `base_link -> nav_camera_link` TF。
- [ ] 复测包含轮胎、突出件和安全余量的真实 footprint。
- [ ] 实车确认底盘 `/swerve_controller/cmd_vel` 和 `/swerve_controller/odom`。
- [ ] 实车确认只有底盘状态估计发布 `odom -> base_link` TF。
- [ ] 实测 D435i 点云中的地面、车体自遮挡、最小距离和飞点。

## 分阶段验收

### N1：固定相机

- 静止 10 分钟无持续掉线；
- RGB、aligned depth、camera info、point cloud 频率稳定；
- 相机安装无松动，TF 数值有测量记录；
- 点云转换到 `base_link` 后方向、高度和距离正确。

### N2：建图

- 先推行/遥控低速闭环，不启动 Nav2；
- 地图墙体方向和尺度正确，无重复楼层或明显点云拖影；
- 回到起点的平移误差低于路线长度 2%；
- 数据库保存并连续三次可重载。

### N3：地图定位

- 加载已有数据库后发布 `/rtabmap/map`；
- `map -> odom -> base_link -> nav_camera_link` 连续且无双发布者；
- 遮挡相机后能够安全停止，视觉恢复后能够重新定位；
- 静止时地图位姿不存在持续明显跳变。

### N4：前向障碍

- 在 0.5、1.0、2.0 m 放置纸箱，局部和全局 costmap 均正确标记；
- 移除纸箱后障碍能够被射线清除；
- 地面和机器人自身不产生持续假障碍；
- 障碍进入前向停止区时，collision monitor 输出零速度；
- 相机点云超时后系统不继续发送非零导航速度。

### N5：Nav2

- 先架空检查速度话题和轮向，再落地短距离测试；
- 依次验证直行 0.5 m、转向、单目标点、连续 10 个目标点；
- 不出现自动横移和盲区倒车；
- 记录成功率、定位丢失次数、最小障碍距离、路径误差和停止距离。

## 当前限制

- 仿真当前使用理想化底盘 odom；实车轮速/转角 odom 与 D435i IMU 融合尚未验收。
- 前向 D435i 不能保证侧后方安全；虽然当前 Nav2 已启用全向规划控制，实车仍需限速监护，
  并应在补齐侧后感知后再做无人全向运行验收。
- `nav2.yaml` footprint 来自当前 URDF 外形的保守估计，不是最终实测值。
- PointCloud2 障碍和 collision monitor 不能替代实体急停及底盘本地通信看门狗。
- Nova Carter 仿真只能验证差速模式下的上层闭环，不能验证自研四轮四转控制和真实制动。
- Isaac profile 必须显式传入 `use_sim_time:=true`；否则传感器/TF 的仿真时间与节点墙上时间
  不同域，Nav2 无法可靠激活。

## 2026-09-12 仿真基线记录

- 闭环路线：1.0×0.7 m、右转、直线 0.10 m/s、转向 0.22 rad/s，8/8 段完成；
- 数据库：`maps/isaac_carter_loop_right_20260912.db`；
- 二维地图：`maps/isaac_carter_loop_right_20260912.{pgm,yaml}`，239×297，0.05 m/格；
- 路线记录：`results/navigation/carter_loop_right_20260912.{csv,json}`；
- 地图定位：`map -> odom -> base_link` 可用；
- 单目标：约 0.5 m 前向目标成功，终点进入 0.15 m 容差，Nav2 返回 `error_code=0`；
- 最终复测：动态 `/tf` 不再包含传感器树，`/tf_static` 仅由
  `carter_nav_camera_static_tf` 发布固定相机外参；目标过程中无 collision monitor TF/超时
  警告；
- 尚未验收：自制底盘 profile 的连续多目标成功率、动态/静态障碍、清除、停止距离和
  点云超时；这些结果不能由 Carter 代替。

## 2026-09-13 自制底盘导航基线

- 数据库：`maps/isaac_indoor_top_rt_01.db`，562 个节点、23 个全局闭环、26 个局部空间闭环；
- RTAB-Map 已固定为平面优化，`map -> base_link` 的 z、roll、pitch 均为零；
- 当时 Nav2 的 planner、simple smoother、Regulated Pure Pursuit、velocity smoother 和
  collision monitor 均由 lifecycle manager 激活；该条仅为 2026-09-13 历史基线，当前控制器
  已替换为 MPPI Omni；
- 从约 `(-2.49, 2.18, -28.9 deg)` 导航到 `(-1.89, 2.54, 31 deg)`，底盘实际移动约
  0.55 m，`NavigateToPose` 返回 `SUCCEEDED`（error_code=0）；
- Isaac RGB-D 偶发约 1.5 秒帧间隔会触发 0.75 秒传感器超时。仅自制底盘仿真 profile
  放宽为 2.0 秒；实机默认值保持 0.75 秒；
- 该段 10 点记录仅保留为旧地图历史测试；当前主线改用外部 odom 新地图做四点验收。

### 10 目标首轮记录

- 已导出 `maps/isaac_indoor_top_rt_01.pgm/.yaml`，并从同一连通自由区筛选 10 个候选点；
  配置与叠加图分别为
  `mobile_navigation_bringup/config/isaac_indoor_top_rt_01_waypoints.yaml` 和
  `maps/isaac_indoor_top_rt_01_waypoints.png`；
- P01 成功，89.9 秒、4 次恢复；P02 成功，8.7 秒、0 次恢复；
- P03 首次在距目标约 0.16 m、偏航误差约 0.35 rad 处超时。仿真粗导航阈值调整为
  0.20 m/0.35 rad 后，续跑立即成功；实机阈值未变；
- P04 从 P03 出发后在约 `(-0.95, 3.55, -0.81 rad)` 停止，目标为
  `(-0.655, 3.239, -0.825 rad)`；控制器返回 `PATIENCE_EXCEEDED(104)`，累计 13 次恢复；
- 结论：候选目标栅格本身具有 0.65 m 净空，不代表目标间的在线路径一定安全。P04 失败应
  作为“前向点云阻塞且第一版禁止盲倒车”的安全回归样本；下一次续跑前需在 RViz 检查
  局部 voxel layer、FrontStop 点和全局路径，必要时更换 P04 或增加人工确认的逃逸位姿。
- 单帧诊断确认停止区内有 2064 个场景点，最近 x=0.634 m，位于右前边缘
  y=-0.36~-0.325 m、高度 0.459~0.616 m；人工沿已走路径倒退后停止区点数恢复为 0，
  最近前向障碍为 1.075 m。保持停止多边形不变，将后续顺序改为先绕地图西北侧。
- 绕行后的 P04 成功（36.6 秒、0 恢复），但 P05 再次因在线障碍停止；同时 costmap 日志
  持续丢弃时间戳早于 TF 缓存的点云。导航 profile 后续改用 320×240@10 Hz 仿真 RGB-D，
  降低到原 640×480@15 Hz 约六分之一的点云吞吐量；相机感知专项仍保留原分辨率。
- 低负载 profile 重新启动后，点云实测平均约 3 Hz、最大间隔 1.585 秒，RTAB-Map
  处理/消息延迟主要降至约 0.017~0.050 秒；发送约 0.3 m 的短目标后底盘实际运动，
  `NavigateToPose` 返回 `SUCCEEDED`。costmap 偶发旧帧丢弃尚未完全消失，列为后续
  QoS/渲染节流专项，不阻塞当前最小导航闭环。

### 安全故障注入记录

- 2026-09-13 增加独立 `collision_safety_benchmark.launch.py`，不依赖 Isaac 或实车，
  只向 `/safety_test/*` 注入受控速度和 `base_link` 点云；
- 净空阶段 20 个输出样本保持 0.15 m/s；`FrontStop` 内放置 7 个点后稳定阶段 20 个输出
  样本均为 0，并收到 `STOP:FrontStop`；
- 恢复净空后重新放行；停止点云发布超过 0.75 秒后收到 `STOP:invalid source`，最终速度
  为 0；正式结果为
  `results/navigation/safety/collision_safety_20260913_185437.json`；
- 真实仿真点云障碍标记/清除已经完成；下一步仅补齐新地图四点统计和成功/故障数据。
  实机阶段仍必须测量实际停止距离并验证硬件急停和底盘本地看门狗。
- 同日为 Isaac profile 增加 `/sim/nav_obstacle/set_enabled`：0.45×0.45×0.80 m 障碍启用
  时位于当前底盘正前方 0.75 m，禁用时隐藏并移出场景。真实
  `Isaac RGB-D -> PointCloud2 -> collision_monitor` 回归通过：初始净空输出 0.15 m/s，
  放置障碍后触发 `STOP:FrontStop` 且输出为 0，移除后恢复 0.15 m/s；结果为
  `results/navigation/safety/isaac_obstacle_safety_20260913_190237.json`。

### 多目标复测与定位结论

- 无启动初值时，重复纹理导致 RTAB-Map 把世界位姿错误匹配到 P03 附近；P01 在 48.5 秒
  后中止。通用定位入口现支持可选 `initial_pose`，自制底盘确定性 Isaac profile 默认使用
  建图起点 `-2.49 2.18 0 0 0 -0.504`，实机入口默认仍为空；
- 加入初值后，启动 `map -> base_link` 实测为 `(-2.495, 2.181, -28.8°)`；P01 在
  15.3 秒内成功且 0 次恢复，结果为
  `results/navigation/waypoints/navigation_waypoints_20260913_191117.json`；
- 随后 P02 在 6.1 秒内中止。日志表明 P01 朝向结束后 RGB-D 视觉特征降到 3～10 个，
  `odom -> base_link` 停在仿真 103.6 秒，而相机继续到约 125 秒，产生约 20 秒 TF 断层；
  这不是目标点或碰撞监控失败；
- 工程结论：固定相机的纯 RGB-D 里程计可用于手持建图基线，但不应作为移动底盘 Nav2 的
  唯一连续 odom。下一项改为接入底盘轮速/转角 odom，并在实机融合 D435i IMU；RTAB-Map
  只负责 `map -> odom` 的全局校正。仿真可使用带明确“理想化”标记的底盘 odom 接口，但
  `/sim/ground_truth/*` 仍只用于误差评测，不能混入论文算法指标。

### 外部底盘 odom 回归

- 2026-09-13，Isaac 已发布 `/swerve_controller/odom` 和标准 `odom -> base_link`；
  `/sim/ground_truth/odom`、`/sim/ground_truth/tf` 继续独立发布，仅供评测；
- 自制底盘 mapping/navigation profile 自动选择 `odom_source:=external`，启动进程中没有
  `rgbd_odometry`；RTAB-Map 同步订阅外部 odom 与 RGB-D，只负责 `map -> odom`；
- 使用与候选目标绑定的 `maps/isaac_indoor_top_rt_01.db`，P01 单目标在 39.4 秒内成功，
  1 次恢复，结果为
  `results/navigation/waypoints/navigation_waypoints_20260913_192318.json`；
- 到达低纹理朝向后继续观测，底盘 odom 平均约 60 Hz，`map -> base_link` 连续，无旧版
  约 20 秒 TF 断层；
- 随后按旧故障条件连续执行 P01→P02：P01 39.4 秒、1 次恢复，P02 6.1 秒、0 次恢复，
  两者均成功。P02 后 odom 约 60 Hz，完整 TF 连续。结果为
  `results/navigation/waypoints/navigation_waypoints_20260913_193700.json`；
- 旧低纹理连续里程计故障至此关闭；
- 随后以 0.08 m/s、0.15 rad/s 执行 0.5×0.5 m 右转闭环建图，8/8 段完成，实际路径
  1.958 m，平移闭合误差 2.79 cm。数据库
  `maps/isaac_indoor_external_odom_04.db` 包含 228 帧、15087 个特征；优化后的连通图有
  122 个关键位姿。路线指标为
  `results/navigation/mapping/isaac_external_odom_formal_loop_20260913.json`；
- 第一条 0.8×0.8 m 左转路线因接近蓝色箱体出现 0.346 m 闭合误差，只作为失败样本保留；
  150 秒限时复测在第 8 段被仿真自动退出中断，也不计为通过；
- 下一项使用 `config/isaac_indoor_external_odom_04_waypoints.yaml` 做四点导航验收并保存
  成功/故障数据。基准工具已支持任意数量的 waypoint，不再强制每次运行 10 点。

### 新地图四点导航验收

- 2026-09-15，以 `initial_pose="0 0 0 0 0 0"` 重新加载
  `maps/isaac_indoor_external_odom_04.db`，RTAB-Map 成功恢复 122 个连通关键位姿；
- `E01_east`、`E02_southeast`、`E03_southwest`、`E04_return` 均返回 `SUCCEEDED`，耗时分别
  为 1.3、19.2、38.0、33.7 秒；4/4 成功，总耗时 92.2 秒，0 次恢复；
- 运行后外部 odom 采样窗口平均约 56 Hz；导航过程中完整 TF 持续可用；
- 正式结果为 `results/navigation/waypoints/navigation_waypoints_20260915_100232.json` 和
  同名 CSV。结合已有确定性障碍/断流故障记录，仿真导航基线至此冻结；
- 后续除非主动感知闭环暴露明确导航接口问题，不再扩展 Carter、不再反复调 Nav2 参数。
