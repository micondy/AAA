# rack_perception

单层几何提取、三层稳定建模、停车位姿和近距离实时对齐的 ROS 2 Jazzy 最小闭环。

## 进展快照（2026-10-20；下表仿真/网页运行数据采于 2026-10-16）

| 范围 | 结果 | 边界 |
|---|---|---|
| 单架多视角 | 223/223 地图帧保持同一架、三层稳定 ID；318 个对齐样本 | Isaac 语义标签与真值 TF |
| 4×4 全通道 | 25/25 脚本路点，最终 16 架/48 层；完整地图只有一套稳定 ID | 脚本直接发送 `/cmd_vel`，不是 Nav2 自主导航 |
| 三维精度 | 最大层高误差 0.018 m，最大架位误差 0.303 m | 对比参数化场景真值，尚非实机精度 |
| 通行估算 | 包围盒重叠样本 0，最小间隙 0.045 m | 余量很小，仅里程计与静态几何估算，非碰撞安全证明 |
| 网页 + 在线地图 + Nav2 | 网页手动前进 0.235 m；单个约 0.4 m 导航目标 `SUCCEEDED`，终点误差 0.071 m | 同一通道短距离冒烟，未验证 4×4 自主巡航 |

全程结果见 [`results/rack/grid_acceptance.json`](../../../../results/rack/grid_acceptance.json)。
相关层架与整机描述单元测试共 12 项，2026-10-16 复核全部通过。
目前已实现 YOLO26n-seg 真实相机推理入口，但缺少真实层架标注和训练权重；D435i 实测尚未完成。
无 Isaac 架号提示的仿真多架关联已通过。停车位姿到 Nav2/控制器的执行仍未验收；
这些输出现在只供观察和后续算法接入。
`enable_nvblox:=true` 是可选入口。2026-10-16 的短时联动冒烟验证收到非空 Mesh
和含已知单元的 ESDF 切片，记录见
[`results/rack/online_mapping_smoke_20261016.json`](../../../../results/rack/online_mapping_smoke_20261016.json)；
这不等于已完成全温室地图质量或 Nav2 代价地图验收。
网页/Nav2 联动详情见
[`results/rack/web_nav2_smoke_20261016.json`](../../../../results/rack/web_nav2_smoke_20261016.json)。

## 当前范围

- `rack_detector`：把 Isaac Sim 语义分割及标签字典转换成 `mono16` 掩膜。默认 4×4 场景
  用 11/12/13、21/22/23 等分别表示每架的三层，10001 以上表示立柱；单架模式兼容
  1/2/3 和 100。实体相机改用独立 `rack_yolo_detector`，输出相同接口，仿真路径不受影响。

- `rack_geometry`：同步掩膜、对齐深度与内参，转换到配置的地图坐标系（仿真 `sim_world`、实体默认 `odom`），拟合每层高度、边界、长轴和
  朝向相机一侧的前沿。
- `rack_map`：按平面位置和高度关联多帧观测，生成 `rack_0001`、`layer_0001` 等稳定 ID；
  当前帧不可见时保留对象并将 `observed=false`。
- `harvest_planner`：按目标层和可配置离架距离发布 Nav2 可用的 `PoseStamped` 停车位姿。
- `rack_alignment`：15 Hz 发布当前距离、距离误差、横向误差和角度误差。

实体 YOLO26n-seg 的数据格式、训练与启动命令见[专项文档](../../../../docs/RACK_YOLO26_REAL_DATA.md)。

## 无架号关联实验

`anonymous_instances:=true` 是对照实验入口：Isaac 仍提供各床面的准确像素分割，但
`rack_detector` 每帧重新分配 1～9999 的床面临时掩膜 ID；立柱也使用临时 ID。发布给
后端的 `/rack_detector/instances` 不包含原始语义 ID 或 `rack_XX_layer_Y` 标签。
`rack_geometry` 只按已知三层高度做几何过滤，不再从掩膜号解码层号。
`rack_map` 在该模式关闭 `source_id // 10` 架号捷径，以地图位置和层高门控、长轴弱评分，
对同帧观测与已有架子做一对一最小代价分配；连续三帧看见三层才确认新架，遮挡记录保留。
这是“准确分割、未知架号”的关联消融，**不是**真实图像分割或实机定位验收。

从关闭状态复现（一个终端启动仿真，另一个终端启动验收脚本）：

```bash
source tools/source_robot_stack.bash
colcon build --symlink-install --packages-select rack_perception mobile_manipulator_bringup
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py \
  anonymous_instances:=true headless:=true test_seconds:=900

source tools/source_robot_stack.bash
python3 tools/run_rack_grid_acceptance.py --anonymous --duration 720 \
  --output results/rack/grid_anonymous_acceptance_retry_20261020.json
```

第二条命令须在另一个终端执行。验收脚本中的真值位置**仅用于评分**，不传回检测、几何
或关联节点。报告额外包含每个实物关联到多少稳定架/层 ID，以及一个稳定架 ID 是否误关联
到多个实物。路线依然由测试脚本发 `/cmd_vel`，不代表 Nav2 自主通行或碰撞安全。
2026-10-20 完整重测通过：25/25 路点、16 架/48 层，架/层 ID 碎片及架 ID 切换均为 0。
实现边界、失败案例和验收细节见
[`docs/RACK_ANONYMOUS_ASSOCIATION.md`](../../../../docs/RACK_ANONYMOUS_ASSOCIATION.md)。

## 运行

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
colcon build --symlink-install --packages-select rack_perception mobile_manipulator_bringup
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py
```

默认场景是 4×4 共 16 个独立三层架：单架 0.80 × 2.00 × 1.68 m，架间横向通道
1.80 m、端部通道 1.40 m，机器人初始位于阵列外侧。仿真总览相机会自动对准整个阵列。
要同时启动网页手动控制、RTAB-Map 在线建图、Nav2、Collision Monitor 和默认开启的
Nvblox，请用以下**单一入口**，不要再另开一份 `sim_rack.launch.py`：

```bash
source tools/source_robot_stack.bash
colcon build --symlink-install --packages-select mobile_navigation_bringup mobile_manipulator_bringup
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack_navigation.launch.py
```

网页地址为 `http://127.0.0.1:8080/#visualization`；选 NORMAL 后可低速手动移动并在
逐步生成的 RTAB-Map 二维地图上发送 Nav2 目标。在线地图数据库默认保存在
`~/.ros/rack_grid4x4_online.db`，重启不会自动清空。网页三维场景使用 Nvblox Mesh；
架号会叠加在网页二维地图上，每层稳定 ID 会显示在三维场景和编号列表中；网页通过
TF 将 `sim_world` 模型分别转到地图与 Mesh 坐标，缺少 TF 时暂不绘制对应叠加层。
该 profile 的速度链为网页 `/cmd_vel_web` 或 Nav2 `/cmd_vel_nav_smoothed` →
`/cmd_vel_muxed` → Collision Monitor → `/cmd_vel`。可用
`headless:=true test_seconds:=180` 做无界面短测。如果已有单独的棚架仿真，可设
`start_rack_stack:=false` 仅附加网页和导航，但不要重复启动 Isaac。

单架感知和原始 CAD 对照仍使用上面的 `sim_rack.launch.py`：

```bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py rack_layout:=single
ros2 launch mobile_manipulator_bringup sim_rack.launch.py rack_layout:=cad
```

需要同时生成 TSDF/ESDF 和 Nav2 可用地图切片时加 `enable_nvblox:=true`；默认关闭以减少
8 GB 显存机器上的负载：

```bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py enable_nvblox:=true
```

无界面冒烟：

```bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py headless:=true test_seconds:=20
```

单架多视角稳定 ID 自动验收（启动仿真时添加 `rack_layout:=single test_seconds:=40`，
另一个终端运行）：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
python3 /media/scc/Windows_D/AAA/robot_research_ws/tools/run_rack_acceptance.py
```

结果写入 `results/rack/acceptance.json`。脚本移动底盘、偏航并改变距离，验收锁定后始终只有
一个层架、三个固定层 ID，且实时距离随运动变化。

4×4 全通道连续验收（一个终端启动无界面仿真，另一个终端运行脚本）：

```bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py headless:=true test_seconds:=800
python3 tools/run_rack_grid_acceptance.py --duration 660
```

脚本依次经过四条通道的 25 个路点，在每架前转向观测，报告写入
`results/rack/grid_acceptance.json`。通过条件包括最终 16 架/48 层、完整地图帧中只有
一套稳定 ID、层高与位置误差阈值，以及底盘包围盒与架子不相交。这里的底盘检查是
里程计轨迹对静态架子的几何间隙估算，不是物理碰撞传感器测试。

RViz 固定坐标系设为 `sim_world`，添加 `MarkerArray` 话题 `/rack_map/markers`。仿真通过
真值 `odom → sim_world` 连接定位与层架地图，避免与 RTAB-Map 的 `map → odom`
争夺 `odom` 父节点；层高仍以地面为零点。检查：

```bash
ros2 topic echo /rack_map/model
ros2 topic echo /rack_alignment/status
ros2 topic echo /harvest_planner/parking_pose
```

`/rack_detector/instance_mask` 是 `mono16` 数值图，不是普通彩色照片。直接用图像查看器时，
1、2、3 这些很小的像素值通常会显示成全黑。调试时应在 RViz 添加 `Image`，选择
`/rack_detector/debug/color_mask`；它把三层和立柱转换成了容易辨认的颜色。也可以先确认数据链：

```bash
ros2 topic hz /rack_detector/instance_mask
ros2 topic echo --once /sensors/nav_camera/semantic_labels --field data
ros2 topic echo --once /rack_detector/instances --field data
ros2 topic echo --once /rack_map/model --field data
```

这里的“语义标签转实例掩膜”是：Isaac 为像素给出 `rack_01_layer_1`、
`rack_01_layer_2` 等语义类别，`rack_detector` 根据同帧标签字典把它们变成后端约定的
16 位实例 ID 图。十位以上表示架号，个位表示层号，例如 11/12/13 是 1 号架三层，
21/22/23 是 2 号架三层；立柱使用 10001 以上的 ID。这样多个架子同时入镜时仍分别提取几何。
切换到 `single` 模式时继续兼容原来的 1、2、3 层 ID。真实相机阶段则由实例分割模型产生
同样的独立实例 ID。默认 4×4 模式的跨帧关联仍利用 Isaac 架号提示；
`anonymous_instances:=true` 模式已在仿真中完成无架号多架关联验收，但像素分割仍来自 Isaac，
尚未验证真实图像与 D435i；后续接入实机时需替换检测器并重新验证。

`/rack_alignment/errors` 使用 `TwistStamped` 承载误差而不是直接控制底盘：
`linear.x=距离误差(m)`、`linear.y=沿层架的横向误差(m)`、`angular.z=航向误差(rad)`。

## CAD 资产说明

`Gupeng.STL` 是完整合并设备，不能在不破坏几何的情况下自动拆出单个架子，因此不再作为默认
导航场景。默认 `grid4x4` 使用按其三层结构重建的独立架子单元；每个单元具有独立碰撞体和语义
实例，可供机器人在通道内移动。完整 CAD 仍保留为 `rack_layout:=cad` 调试选项，缓存位于
`assets/rack/Gupeng.usd`。重新导出 STL 后可重建缓存：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/convert_rack_stl.py \
  /media/scc/Windows_D/AAA/Gupeng.STL assets/rack/Gupeng.usd
```

这个 STL 内部没有可恢复的 SolidWorks 零件名，所以 CAD 调试模式在三条主要种植带
上叠加了薄的语义真值面，分别标记 `rack_layer_1` 至 `rack_layer_3`；RGB-D 深度仍与真实 CAD
表面共位。可用 `rack_asset:=/绝对路径/其他.usd` 指定另一 CAD 资产。

2026-10-15 的 CAD 验收结果保存在 `results/rack/acceptance_cad.json`：263/263 地图帧均为
同一个层架和三个稳定层 ID，锁定后无重复/丢失帧，并收到 342 个实时对齐样本。

## 话题契约

| 话题 | 类型 | 含义 |
|---|---|---|
| `/rack_detector/instance_mask` | `sensor_msgs/Image` | 每层和立柱掩膜 |
| `/rack_detector/debug/color_mask` | `sensor_msgs/Image` | 便于 rqt/RViz 查看的人眼彩色掩膜 |
| `/rack_geometry/observations` | `std_msgs/String` | `rack_observations/v1` JSON |
| `/rack_map/model` | `std_msgs/String` | `rack_map/v1` JSON，可靠且 transient-local |
| `/rack_map/markers` | `visualization_msgs/MarkerArray` | 层边界、前沿法向、稳定 ID |
| `/harvest_planner/parking_pose` | `geometry_msgs/PoseStamped` | 目标停车位姿 |
| `/rack_alignment/errors` | `geometry_msgs/TwistStamped` | 控制器输入误差 |
| `/rack_alignment/status` | `diagnostic_msgs/DiagnosticArray` | 人可读距离与角度状态 |
