# ROS 包目录

状态含义：`运行基线` 表示已有可重复软件/仿真验证；`实机待验` 表示接口完成但真实硬件验收
未闭环；`上游固定` 表示 vendor 快照，只通过集成层使用。

## `mobile_base_ws`

| 包 | 职责 | 主要入口与接口 | 状态 / 禁止承担 |
|---|---|---|---|
| `mobile_base_description` | 四轮四转底盘 Xacro/URDF、ros2_control 与传感器安装接口 | `description.launch.py`、`robot_description`、TF | 运行基线；不包含驱动逻辑 |
| `mobile_base_driver` | 达妙转向、ZLAC 行走、SocketCAN、ros2_control hardware interface | `can0/can1`、关节 state/command interfaces | 实机待整车验收；不做运动学 |
| `mobile_base_swerve_controller` | 四轮独立转向运动学、对准门控、限速、里程计 | 订阅 Twist；发布 `/swerve_controller/odom` | mock 运行基线；不直接访问 CAN |
| `mobile_base_bringup` | 底盘配置、controller manager、实机/mock 启动、标定工具 | `robot.launch.py`、`mock.launch.py`、`chassis.yaml` | 编排层；不复制算法实现 |
| `mobile_base_web_console` | 局域网监控、手动控制、维护、Nav2 目标、2D/3D 显示 | HTTP 8080、`/cmd_vel`、`/navigate_to_pose`、Nvblox Mesh | 运行基线；不是实体急停 |

## `robot_research_ws/src/robot_research`

| 分层 | 包 | 职责 | 主要入口与接口 | 状态 / 边界 |
|---|---|---|---|---|
| sensing | `d435i_bringup` | 单 D435i 分阶段 RGB、深度、IMU、点云启动 | `camera.launch.py`；`basic/imu/aligned/full` | 软件基线；外参实机待验 |
| evaluation | `experiment_tools` | 相机健康监测、rosbag 录制/回放、QoS 固化 | `monitor/record/play_camera.launch.py` | 运行基线；不实现感知算法 |
| localization | `rtabmap_bringup` | RGB-D 建图、数据库定位、map/odom TF | `online/offline/localization.launch.py` | 仿真基线；实机待验 |
| mapping | `nvblox_bringup` | 可选 TSDF、Mesh、ESDF 局部地图 | `/nvblox_node/mesh`、`static_map_slice` | 仿真通过；不拥有定位 TF |
| mapping | `gaussian_splatting_bringup` | 标定 RGB-D 轨迹采集、Nerfstudio 数据和离线训练封装 | `capture.launch.py`、dataset recorder | 数据支线；不进实时控制闭环 |
| navigation | `mobile_navigation_bringup` | RTAB 定位 + Nav2 + 点云/ESDF costmap + 安全评测 | `mapping.launch.py`、`navigation.launch.py`、仿真入口 | 仿真运行基线；实机待验 |
| perception | `target_perception` | RGB-D 目标检测、实例分割、三维反投影与跟踪 | detections、target pose、debug topics | 仿真基线；不决定机器人运动 |
| perception | `rack_perception` | Isaac 层架掩膜、RGB-D 层几何、稳定架/层 ID、停车候选与对齐误差 | `sim_rack.launch.py`、联动入口 `sim_rack_navigation.launch.py`；`/rack_map/model`、`/rack_alignment/errors` | 4×4 仿真基线；真值语义辅助，不直接控制底盘 |
| planning | `active_perception` | 目标选择、Belief、多视角候选、路径评分与执行门控 | belief/NBV launch、plan/execute/cancel services | 仿真闭环；执行默认关闭 |
| integration | `mobile_manipulator_description` | 底盘、双 OpenArm、D435i 的统一 URDF | `display.launch.py`、统一 TF/robot_description | 运行基线；相机实测外参待填 |
| integration | `mobile_manipulator_moveit_config` | 双臂 MoveIt 配置、控制器映射、碰撞模型 | `demo.launch.py`、MoveGroup actions/services | mock 基线；不负责底盘导航 |
| integration | `mobile_manipulator_bringup` | 实机/mock/Isaac profile 与稳定接口、任务场景 | `sim_*`、`mock.launch.py`、task scene | 顶层编排；不复制下层算法 |

## `robot_research_ws/src/vendor`

| 包 | 来源职责 | 集成规则 |
|---|---|---|
| `openarm_can` | OpenArm SocketCAN 底层库 | 固定版本；上游修改需单独记录 |
| `openarm_description` | OpenArm URDF 与 mesh | 由整机 description 组合，不直接改 vendor mesh |
| `openarm` | OpenArm ROS 元包 | 仅依赖聚合 |
| `openarm_hardware` | OpenArm ros2_control 硬件插件 | 真机 profile 才加载 |
| `openarm_bringup` | OpenArm 控制器启动 | 由整机 bringup 选择性复用 |
| `openarm_bimanual_moveit_config` | 上游双臂 MoveIt 语义与参数 | 集成配置复用并覆盖必要差异 |

版本、上游仓库与提交哈希以 [`src/vendor/README.md`](../../src/vendor/README.md) 为准。

## 非 ROS 包资产

| 目录 | 功能 | 维护规则 |
|---|---|---|
| `robot_research_ws/tools/isaac_sim` | Isaac 场景、机器人、传感器与测试物体脚本 | 接口变化同步更新运行流图 |
| `robot_research_ws/assets` | USD 和整机资产来源记录 | 保留来源、版本与校验信息 |
| `robot_research_ws/maps` | RTAB 数据库与导出地图 | 文件名包含场景/odom 来源，不覆盖基线 |
| `robot_research_ws/results` | 评测 JSON/CSV/说明 | 结果不是运行参数来源 |
| `chassis_4wis` | 原始底盘描述/CAD 资产 | 运行时使用已集成的 description 包 |

## 稳定接口速查

| 接口 | 生产者 | 主要消费者 |
|---|---|---|
| `/joint_states` | ros2_control joint state broadcaster / Isaac | 描述、MoveIt、网页、诊断 |
| `/swerve_controller/odom` | 四轮四转控制器 / Isaac 等效底盘 | RTAB-Map、Nav2、网页、评测 |
| `/cmd_vel` | 安全链最终输出或仿真直通 | 底盘 / Isaac |
| `/rtabmap/map` | RTAB-Map | Nav2 static layer、网页 2D 地图 |
| `/navigate_to_pose` | Nav2 BT Navigator | 网页、NBV executor、评测工具 |
| `/nvblox_node/mesh` | Nvblox | RViz、网页 3D 环境 |
| `/nvblox_node/static_map_slice` | Nvblox | 可选 Nav2 Nvblox costmap layer |
| `/target_perception/tracked_detections_3d` | target_perception | target selector / belief |
| `/active_perception/target_belief` | active_perception | NBV planner / executor / 评测 |
| `/rack_detector/instance_mask` | rack_detector | rack_geometry、调试工具 |
| `/rack_map/model`、`/rack_map/markers` | rack_map | harvest_planner、rack_alignment、RViz |
| `/rack_alignment/errors` | rack_alignment | 调试与未来的停车控制器；当前未接入自动控制 |

接口改名必须同时搜索 launch remap、YAML、网页配置、测试和本文档。
