# Nvblox 局部三维地图实施计划

更新日期：2026-09-21。

## 定位

Nvblox 是可选的 GPU 局部三维感知后端，不是 SLAM 或全局定位替代品。RTAB-Map继续维护
`map -> odom`，底盘/视觉里程计维护 `odom -> base_link`；Nvblox消费该 TF 与顶部 D435i
的 RGB-D，在 `odom` 中建立有界 TSDF、Mesh 和 ESDF。关闭 Nvblox 后，建图、定位、Nav2、
MoveIt、目标感知和主动感知必须仍能运行。

## 分阶段范围

### N1：Mapping-only（仿真已完成）

- [x] 新建独立 `nvblox_bringup` 包；
- [x] 复用 `/sensors/nav_camera` RGB、对齐深度和内参；
- [x] 使用既有 TF，不启动第二套视觉里程计；
- [x] 限制为单相机、5 cm voxel、6 m 局部半径；
- [x] 输出 Mesh、ESDF pointcloud 和 map slice；
- [x] 在自制底盘 Isaac mapping/navigation profile 中加入默认关闭开关；
- [x] 使用 Isaac 室内静止场景确认深度、TSDF、Mesh 与ESDF端到端融合；
- [x] 执行短矩形移动闭环，验证移动时的非空Mesh、ESDF与距离切片；
- [x] 执行10分钟壁钟运行，记录图像间断、GPU显存/利用率/温度和输出频率；
- [ ] 固定 D435i 后使用实机录包重复验收。

### N2：Nav2 A/B（当前）

- [x] 新建独立 Nav2参数覆盖文件，不覆盖现有点云障碍基线；
- [x] 提供默认关闭、可一键回退的 Nvblox costmap profile；
- [x] 仿真确认 local/global costmap 均实际订阅 ESDF slice；
- [x] 使用既有四点路线完成点云基线与 Nvblox profile 初步A/B；
- [x] 验证低矮、悬空和货架类障碍的点云停车及ESDF标记/清除；
- [x] 仿真记录分类障碍标记/清除、动态停车距离和相机断流行为；
- [ ] 同批次重复 A/B 成功率、最小净空与 GPU 负载，及全部实机指标；
- [x] 仅作为可选 navigation profile，不设为默认值。

### N3：移动操作接口（以后）

- [ ] 向 MoveIt或任务局部地图提供动态环境几何；
- [ ] 为 NBV 候选提供遮挡和净空查询；
- [ ] 明确安装和标定后再评估第二、第三台D435融合。

## 当前运行入口

先分别启动 Isaac Sim 与既有 RTAB-Map/导航链，再启用同一入口中的开关：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_mapping.launch.py \
  enable_nvblox:=true database_path:=maps/isaac_nvblox_test.db
```

仅调试 Nvblox（相机和 TF 已由其他进程提供）：

```bash
ros2 launch nvblox_bringup mapping.launch.py \
  use_sim_time:=true input_qos:=DEFAULT start_camera:=false
```

仿真中启用可选 Nav2 ESDF 障碍层（该开关会自动启动 Nvblox）：

```bash
ros2 launch mobile_navigation_bringup mobile_base_sim_navigation.launch.py \
  use_nvblox_costmap:=true \
  database_path:=maps/isaac_indoor_external_odom_04.db \
  initial_pose:="0 0 0 0 0 0"
```

不传 `use_nvblox_costmap` 或显式设为 `false`，即回到原来的点云 voxel/obstacle layer。

真实相机不能手持进行此项测试；必须刚性安装并具有连续、唯一的相机 TF。

## N1通过标准

1. `/nvblox_node/mesh` 与 `/nvblox_node/static_esdf_pointcloud` 持续更新；
2. 地面、墙面和大型障碍的尺度与方向正确，无明显随运动重影；
3. 运行10分钟没有持续 TF lookup错误或节点退出；
4. 相机短时断流后能够继续融合，且不会影响 Nav2现有安全链；
5. 记录 RTX 5090 与目标 Jetson 上的频率、GPU显存和温度；
6. N1通过前，不修改当前 Nav2 costmap配置。

## 2026-09-19 首次运行记录

环境为现有自制底盘 + 官方 OpenArm Isaac 室内场景，顶部等效 RGB-D 为320×240@10 Hz。
首次只启动相机与 Nvblox 时，图像回调存在但没有相机 TF，未发生 TSDF 融合；补充正式入口
中的统一 robot description 后复测通过。这说明独立调试入口也必须同时提供完整
`odom -> base_link -> nav_camera_color_optical_frame`，不能只检查图像话题存在。

复测结果：

- 深度回调和实际融合均为299帧，稳定窗口频率10.1 Hz；
- TSDF实际融合299帧；颜色融合100帧，稳定窗口3.4 Hz；
- ESDF稳定窗口4.7 Hz，平均端到端延迟约76 ms；
- 深度融合平均延迟约34 ms；
- Mesh、`static_esdf_pointcloud` 与 `static_map_slice` 均发布 `odom` frame消息；
- 仿真、Nvblox与 robot_state_publisher 均已正常退出。

这次只证明固定姿态的端到端数据链和GPU计算成立，尚不等于移动建图、重影、显存或长期
稳定性验收通过。

## 2026-09-19 移动闭环记录

新增 `nvblox_mapping_monitor.py`，同时监测深度、Mesh、ESDF pointcloud、距离切片和底盘
里程。报告要求地图内容非空，避免只凭话题存在或空Header误判通过。

第一次0.45×0.35 m路线因Isaac预设运行时间先结束，在第7段按设计超时；监测到的1.301 m
移动三维重建仍为PASS，但路线结果保留为失败记录。第二次使用0.30×0.25 m短矩形完成8/8
段，结果如下：

- 路线状态：completed；真值路线1.126 m；闭合平移误差8.55 cm；
- 45.13 s监测状态：PASS；监测累计里程1.113 m；
- 深度9.93 Hz、Mesh 3.88 Hz、ESDF pointcloud与map slice均为4.64 Hz；
- 最大550个Mesh block、25505个顶点；
- ESDF最大11365点，距离切片最大11365个已观测单元；
- 仿真最大倾斜0.001°，所有进程已关闭。

记录：

- `results/nvblox/moving_loop_20260919_2215.json`
- `results/nvblox/moving_route_20260919_2215.json`
- `results/nvblox/moving_route_20260919_2215.csv`

路线JSON中的旧字段名 `rtabmap_*` 表示命令行指定的 `estimate_topic`；本次该话题实际为
`/swerve_controller/odom`，并未启动RTAB-Map。

## 2026-09-19 十分钟稳定性记录

运行600.12秒壁钟，Isaac运行620仿真秒。开始阶段再次执行0.30×0.25 m短矩形并完成8/8
段；随后保持RGB-D和重建持续运行。监测结果为PASS：

- 深度5912帧，9.998 Hz，最大帧间断0.184 s；
- Mesh 2282条，3.861 Hz，最大间断0.551 s；
- ESDF pointcloud和map slice各2733条，4.622 Hz，最大间断0.284 s；
- 最大570个Mesh block、25109个顶点、11847个ESDF点/已观测slice单元；
- 596次GPU采样：显存平均3305 MB、峰值3452 MB；
- GPU利用率平均64.8%、峰值97%；温度平均63.1 °C、峰值66 °C；
- 累计底盘里程2.710 m，仿真最大倾斜0.002°；
- Nvblox、Isaac和robot_state_publisher均正常退出。

稳定性开始阶段的闭环路线长1.145 m，平移闭合误差4.79 cm。另一次扩大到1.0×0.7 m的
路线在第三段被场景可用空间限制，按30秒超时停车；该结果保留为失败记录，不计入通过项。

记录：

- `results/nvblox/stability_10min_20260919_2218.json`
- `results/nvblox/stability_route_20260919_2218.{json,csv}`
- `results/nvblox/stability_route_long_20260919_2218.{json,csv}`（预期保留的失败记录）

至此N1仿真软件验收完成。真实D435i仍须在刚性安装和外参标定后另行验收；N2只能新增为
可选Nav2 A/B profile，不能覆盖已经冻结的点云障碍基线。

## 2026-09-19 N2 首次 Nav2 A/B 记录

新增两个小型参数覆盖文件。默认的 `costmap_baseline_overlay.yaml` 不修改既有配置；只有
`use_nvblox_costmap:=true` 时才加载 `costmap_nvblox_overlay.yaml` 并自动启动 Nvblox：

- local costmap：`nvblox_layer + inflation_layer`；
- global costmap：保留 RTAB-Map `static_layer`，叠加 `nvblox_layer + inflation_layer`；
- `map -> odom` 和全局定位仍由 RTAB-Map负责；
- collision monitor仍直接使用原始点云，Nvblox异常不会绕过最后一道停车保护。

运行时参数和ROS图已确认：两个NvbloxCostmapLayer均成功初始化，两个costmap均订阅
`/nvblox_node/static_map_slice`。使用与点云基线相同的四点配置，Nvblox profile 4/4成功，
0次恢复；总导航动作耗时45.27秒。历史点云基线同为4/4、0次恢复，总耗时92.25秒。
两次运行并非同批次同步实验，到点剩余距离也不同，因此当前只能证明功能兼容，不能宣称
Nvblox速度更快或更安全。正式性能结论要等待三类障碍的同批次重复试验。

记录：

- `results/nvblox/nav2_ab/nvblox/navigation_waypoints_20260919_223713.{json,csv}`；
- 对照：`results/navigation/waypoints/navigation_waypoints_20260915_100232.{json,csv}`。

## 2026-09-21 分类障碍与视场盲区记录

Isaac障碍控制已扩展为低矮（16 cm）、悬空横杆和货架三类可重复服务。首次将低矮障碍放在
0.75 m处时，顶部相机因安装高度1.12 m、俯角12°而看不到该物体，安全门未停车；该失败
记录保留为 `results/navigation/safety/isaac_obstacle_safety_20260921_114124.json`。

没有通过增高测试物体掩盖问题，而是把前向停车多边形从0.70 m扩展到1.10 m，让低矮障碍
在1.00 m、尚未进入近端盲区时触发。复测结果：

- 低矮、悬空、货架均触发 `FrontStop`，输出速度为0；
- 三类障碍移除后均恢复0.15 m/s测试输入；
- 正式安全结果为 `results/navigation/safety/isaac_obstacle_safety_20260921_114428.json`。

随后直接统计Nvblox local costmap前方ROI（x=0.30～1.20 m，|y|≤0.55 m）的致命栅格：

| 障碍 | 新增致命栅格 | 移除8秒后的残留 |
|---|---:|---:|
| 低矮 | 155 | 0 |
| 悬空 | 366 | 0 |
| 货架 | 365 | 0 |

结果为PASS，见
`results/nvblox/nav2_ab/nvblox_costmap_obstacles_20260921_115026.json`。这证明三类几何在当前
固定视角下能够被ESDF代价地图标记并清除，但尚未替代实机最低可见高度和制动距离测试。

## 2026-09-21 动态侵入与断流记录

使用独立安全测试输入和真实 Isaac 渲染点云，让仿真底盘先运动，再在正前方1.00 m插入
障碍。低矮障碍在0.10/0.18 m/s下分别于0.066/0.137 s内把输出归零，侵入后前进量分别为
0.0095/0.0299 m；悬空障碍在0.18 m/s下于0.120 s内归零，前进0.0269 m。按仿真几何和
里程计估算，最终净空分别为0.345/0.325/0.353 m。停止点云转发后0.594 s归零，小于
0.75 s安全源超时。正式结果为
`results/navigation/intrusion/dynamic_intrusion_20260921_122038.json`。

上述净空是障碍服务放置距离、模型尺寸和里程计前进量的运动学估算，并非接触传感器实测。
该测试验证原始点云 collision monitor 最后防线，不证明 Nvblox 本身的制动性能，也不能
替代实车急停、控制器看门狗和不同载荷下的停车距离测试。
