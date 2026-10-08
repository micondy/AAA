# 移动操作主动感知工程框架与实施计划

> 决策日期：2026-09-10  
> 当前状态：A1～A3 已完成；固定相机底盘 NBV 已通过 Isaac/Nav2 联合仿真  
> 目标：建立可重复开展论文实验的科研平台，不扩张为产品级通用机器人软件

## 1. 架构决策

不再新建 `robot_system_ws`。采用两个稳定下层工作区和一个顶层研究 overlay：

```text
/opt/ros/jazzy                         # ROS 2 系统 underlay
        ↓
/media/scc/Windows_D/AAA/mobile_base_ws # 自制底盘 underlay
        ↓
/media/scc/Windows_D/AAA/robot_research_ws
  └── src/vendor/                       # 固定版本 Enactic 官方 OpenArm 全栈
      ├── openarm_description           # 1.0.4
      ├── openarm_ros2                  # 0.9.2，含官方 MoveIt/bringup/hardware
      └── openarm_can                   # 1.3.4
                                        # 整机组合、感知、导航和论文算法
```

`workspace` 是构建和依赖层，不是功能模块。功能边界由 ROS 2 package 表达。OpenArm
描述、MoveIt、bringup、hardware 和 CAN 均来自 Enactic 官方固定版本；顶层只在集成包中
增加底盘安装关系和组合配置，不修改官方源码。Isaac Sim 官方双臂 USD 已复制到
`assets/mobile_manipulator/openarm_bimanual_official`，避免依赖个人目录或误用实机 fork。

标准加载顺序为：

```bash
source /opt/ros/jazzy/setup.bash
source /media/scc/Windows_D/AAA/mobile_base_ws/install/setup.bash
source /media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash
```

已提供同等功能的固定入口：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
```

所有 `openarm_*` 包都必须从 `robot_research_ws/install` 解析，并由自动测试核验。禁止在
该工程的 source 脚本、package、launch 或配置中引用个人 OpenArm 工作区。

这不是唯一的行业目录模板，但符合主流 ROS 2 项目的责任拆分方式：`ros2_control` 官方示例
分别展示硬件接口、机器人描述、控制器配置和 bringup；MoveIt 2 官方建议为机器人建立独立
的 `*_moveit_config` colcon package。参考：

- [ros2_control Jazzy demos](https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html)
- [MoveIt 2 configuration package](https://moveit.picknik.ai/main/doc/how_to_guides/moveit_configuration/moveit_configuration_tutorial.html)

Enactic 官方 OpenArm 已按 `openarm_description`、`openarm_hardware`、
`openarm_bringup` 和 `openarm_bimanual_moveit_config` 分包；`openarm_hardware` 依赖官方
`openarm_can`。顶层复用这些官方包，不重新实现 OpenArm 驱动和控制器。

## 2. Package 边界

### 2.1 已有 package

| Package | 职责 |
|---|---|
| `d435i_bringup` | 固定 D435i 驱动、参数和独立验收 |
| `rtabmap_bringup` | RGB-D 建图和数据库定位基线 |
| `mobile_navigation_bringup` | Nav2、点云障碍和安全速度链 |
| `experiment_tools` | 健康检查、rosbag 和实验评测工具 |

### 2.2 按可运行里程碑新增

| 顺序 | Package | 创建条件与职责 |
|---:|---|---|
| 1 | `mobile_manipulator_description` | 已创建；底盘、官方 OpenArm、参数化支架和官方 D435i 描述通过 Xacro/URDF 测试 |
| 2 | `mobile_manipulator_moveit_config` | 已创建；复用官方 OpenArm SRDF/IK/限位/控制器配置，MoveIt mock 闭环通过 |
| 3 | `mobile_manipulator_bringup` | 已创建；mock profile 统一整机控制器、标准底盘接口、TF 和理想真值 |
| 4 | `target_perception` | 已创建；从 RealSense/Isaac 统一 RGB-D 接口输出标准 2D/3D detection、位姿和质量指标 |
| 5 | `active_perception` | 已创建；Target Belief 已运行，后续加入候选生成、过滤和评分 |
| 6 | `mobile_manipulation_tasks` | 能执行 observe-plan-act-verify 状态机时创建 |

第一版不单独创建 `target_belief`、`manipulability_evaluator`、`whole_body_planner`、
`safety_supervisor` 或 `isaac_sim_bringup` 空包：

- belief、可见性、信息增益和可操作性评价先作为 `active_perception` 内部库；
- 底盘—机械臂候选也先放入 `active_perception`，出现独立复用需求后再拆；
- 整机安全状态先由 `mobile_manipulation_tasks` 的显式状态和底层看门狗共同保证；
- Isaac Sim 启动先归入 `mobile_manipulator_bringup` 的 `sim` profile。

## 3. 整机模型与相机角色

统一模型只组合现有 Xacro，不修改下层包：

```text
base_link
├── 四个转向关节和四个车轮关节
├── openarm_mount_link
│   └── OpenArm 关节链
├── nav_camera_mount_link
│   └── nav_camera_link              # 固定 D435i
```

当前 OpenArm 手腕没有安装 D435，也没有确认腕部相机支架。两台额外 D435 第一阶段均不
写入整机 URDF，不提前指定为 `wrist_camera` 或 `context_camera`。安装方案确定后，统一用
`observation_camera` 作为算法侧角色名，其机械父节点可以是 `base_link`、独立云台或
OpenArm 末端；感知算法不得依赖具体父 link 名。相机、支架和 OpenArm 相对底盘的临时尺寸
必须标记为 `provisional`，实机碰撞规划和论文实验前替换为测量与标定结果。

这会影响论文方法边界：没有可随机械臂运动的相机时，机械臂不能产生主动观察视角，第一版
主动视角只能由移动底盘携带固定 D435i 实现；若以后给末端安装一台 D435，才启用机械臂
NBV 候选。目标三维观测、belief 和评分接口保持不变，因此两条路线不需要重写感知代码。

## 4. 稳定接口

优先复用标准接口：

| 功能 | 接口 |
|---|---|
| 底盘速度 | `/cmd_vel` 或安全链最终输出，`geometry_msgs/msg/Twist` |
| 底盘里程计 | `/odom`，`nav_msgs/msg/Odometry` |
| 关节状态 | `/joint_states`，`sensor_msgs/msg/JointState` |
| 机械臂轨迹 | `control_msgs/action/FollowJointTrajectory` |
| 目标三维检测 | `vision_msgs/msg/Detection3DArray` |
| 带协方差位姿 | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| 导航 | Nav2 标准 action |
| 操作规划 | MoveIt 2 标准 action/service |

只有当标准消息不能表达实例 ID、遮挡率、深度有效率、belief 版本、候选收益分解和恢复状态
时，才增加一个小型接口 package。模型内部张量不能成为跨 package 接口。

## 5. 框架完成定义

满足下列门槛后停止扩展基础设施，进入主动感知算法和批量实验。

### G0：整机模型

- 一个 Xacro 同时包含底盘、OpenArm 和当前实际使用的一台固定 D435i；
- `check_urdf` 通过，TF 无多父节点、闭环和重复发布者；
- `sim/mock/real` 只改变硬件和参数，不改变 link、joint 和 topic 名。

### G1：底盘

- 仿真和实机均能响应同一速度接口；
- 前进、横移、斜移和原地旋转方向正确；
- 有里程计、关节状态、限速和命令超时停车；
- Nav2 只需可靠到达任务区域，不作为论文创新。

### G2：OpenArm 与 MoveIt 2

- mock/仿真中可规划并执行给定末端位姿；
- IK、关节限位、自碰撞和已知场景障碍过滤生效；
- 一组至少 20 个测试位姿的规划执行成功率达到 90%；
- 底盘运动时机械臂处于允许的收拢状态，操作时底盘被锁定。

### G3：固定 RGB-D 与目标观测

- RGB、对齐深度、CameraInfo 与 TF 连续、时间戳一致；
- 使用 AprilTag、彩色物体或单类分割输出目标三维位置和有效点统计；
- 0.4～1.0 m 范围内静态目标的第一版定位误差达到约 10～20 mm；
- 同一 rosbag 可重复得到相同结果。

### G4：主动视角闭环

- 生成有限候选相机位姿而不是连续 6D 盲搜；
- MoveIt IK 和 Planning Scene 自动过滤不可达/碰撞候选；
- 输出每个候选的可见性、不确定性、可操作性和运动代价；
- 使用当前可用载体移动观察相机后重新观测，并证明目标定位或操作成功率改善；未安装腕部
  相机时载体仅为底盘，安装并标定后才加入机械臂候选。

### G5：四轮底盘加入论文方法

- 规划层只搜索 `(x_b, y_b, yaw_b, q_arm)`，不直接优化四个轮转角；
- 比较差速约束和四轮转向横移/斜移在窄空间中的效果；
- 执行层第一版采用底盘与机械臂分时运动，每次动作后重新观测；
- 完成固定视角、随机 NBV、仅可见性、机械臂 NBV 和全身方法的消融。

## 6. 实施阶段

### M1：统一仿真骨架（完成）

1. [完成] NVIDIA 官方 OpenArm USD 已复制进项目并装配到自制底盘；
2. [完成] `mobile_manipulator_description` 已建立，使用参数化临时安装尺寸；
3. [完成] 建立整机 MoveIt 配置；mock 模式硬编码为假硬件，不访问 CAN；
4. [完成] 建立 `mobile_manipulator_bringup` mock profile，统一发布 `/joint_states`、
   `/cmd_vel`、`/odom`、`world -> odom -> base_link` 和 `/sim/ground_truth/odom`；
5. [完成] 底盘低速运动、超时停车及底盘移动后的机械臂规划已通过；左右臂安全零位和
   固定自碰撞姿态已由 `/check_state_validity` 回归工具验收。底盘代理只对固定相邻 link
   放行；6000 个确定性样本未发现机械臂可触及当前高度的底盘代理。

M1 已转入 M2；没有创建不存在的 `wrist_camera`。

### M2：固定相机与简单目标

1. [软件完成，实机待测] 把固定 D435i 作为 `observation_camera`：统一模型使用官方
   `realsense2_description`，安装位姿可参数化；Isaac 等效相机已按 640×480、15 Hz 发布
   同名 RGB、对齐深度、CameraInfo 和点云。两台 D435 的安装位置确认前不加入统一模型；
2. [仿真基线完成，实机待测] `target_perception` 使用标准 `Detection3DArray`、
   `Detection2DArray` 和 `PoseWithCovarianceStamped`；20 cm 绿色方块端到端恢复尺寸为
   0.2001×0.2025 m，可见表面深度与仿真真值一致，有效深度率 98.8%；
3. [完成] 建立薄 `sim_perception` profile，仅编排 Isaac RGB-D、统一 URDF/相机 TF 和
   目标感知；检测结果在 `base_link` 中恢复到 `(1.1002, 0.0012, 0.4301) m`，不启动
   MoveIt、Nav2、RTAB-Map、RViz 或真机 RealSense；
4. [仿真完成，实机 rosbag 待录] 单 Isaac 进程依次设置 0.4、0.6、0.8、1.0 m 前表面
   距离，各保存 100 个严格同步样本；检测率均为 100%，位置 RMSE 为
   0.834/1.754/1.716/3.083 mm，结果包含逐帧 CSV、JSON 和汇总表；
5. [完成] 加入通用工作台、后挡板、上层板和遮挡板 box Planning Scene；MoveIt 回读四个
   物体，左右臂 home 有效，固定自碰撞和夹爪—工作台碰撞姿态均被拒绝。

M2 软件侧闭环完成。真实 D435i 刚性安装后，再按同一距离协议录制实机 rosbag。

### M3：载体 NBV

1. [仿真基线完成，实机待测] 在固定坐标系融合 Target Belief，维护目标均值、位置
   协方差、接受/拒绝计数和短时可见性；完成乱序和 Mahalanobis 离群门控；Isaac 100 帧
   连续流位置 RMSE 为 1.660 mm，最终协方差迹为 `3.48e-6 m²`；
2. [完成] 从 `/target_perception/tracked_detections_3d` 中稳定选择一个关注目标，转换为
   `/active_perception/selected_target_pose` 并复用单目标滤波器；通用多目标关联仍由感知
   包负责；
3. [联合仿真通过] 围绕目标生成约 30～60 个有限候选视角；当前相机固定在底盘时只生成底盘候选，
   若以后安装并标定腕部相机再生成机械臂候选；
4. [联合仿真通过] 完成地图障碍、Nav2 可达性和轨迹代价过滤；
5. [软件完成，实机参数待标定] 完成可见性、遮挡、信息收益和运动代价的可解释评分；
6. [工程闭环通过，质量收益待优化] 经显式双重确认执行最佳视角并完成重新观测；继续验证并优化误差/协方差下降。

M3 的目标选择、几何可见性、有限底盘候选、占据栅格过滤、分项评分和 Nav2 路径 action
客户端均已在现有 `active_perception` package 内完成，并通过算法回归与联合仿真。联合测试
生成 36 个候选，3 个进入 Nav2 查询，最佳路径 2.954 m、信息增益 0.9210 nats、总分
0.9877。M3-6 已完成一次工程闭环；下一项是解释并降低跨视角均值偏移与协方差增长。只有研究问题需要同时规划多个目标时，才增加
多目标任务状态和自定义消息。真实相机安装后补噪声参数标定与实机 rosbag。

2026-09-17 已加入 A4 安全执行器：默认禁用，启动参数和运行时服务构成双重确认；拒绝过期、
重复和并发视角，目标切换或导航超时时取消；成功后等待新观测并保存成对 Belief 指标。
软件构建与默认禁用冒烟测试当时已通过；最终运动闭环结果见下文 2026-09-19 记录。

同日第一次运动测试中，底盘约移动 0.96 m 后由 `FrontStop` 正确停车，执行器在 120 s 后
取消导航并保存失败 JSON；这验证了安全失败路径，但没有完成重新观测。候选距离因此从
0.6/0.8/1.0 m 增至 1.2/1.4/1.6 m。第二次运行又发现单个 `ComputePathToPose` 长时间不
返回会阻塞候选批次，现已加入每候选 8 s 超时、取消及迟到回调隔离。下一次联合仿真必须
恢复有限时间规划。第三次运行中终点距离已经足够，但路径途中仍进入 `FrontStop`；因此新增
整条路径相对目标的 1.1 m 最小净空过滤。当时约定必须同时满足安全到达与至少 10 个
新观测才可将 A4 标记为完成；该条件已在 2026-09-19 的最终运行中满足。详见
[`results/active_perception/a4_joint_sim_20260917.md`](../results/active_perception/a4_joint_sim_20260917.md)。

2026-09-19 最终运行中，检测/跟踪统一到连续 `odom` 坐标系，Nav2 使用姿态进展检查，仿真
驱动补偿低速纯旋转死区。底盘 49.18 s 到达最佳观察位、0 次恢复，到达后融合 126 个新观测
并输出 `SUCCEEDED` JSON，A4 工程闭环标记完成。协方差迹由 `7.97e-6` 增至 `4.28e-5`，
因此“估计质量提升”仍未通过，转为下一阶段的标定与评分模型研究问题。

### M4：底盘—机械臂联合候选

1. 在 `task_frame` 中增加少量底盘横移、斜移和偏航候选；
2. 对每个底盘候选生成机械臂观察候选；
3. 联合评价观测收益、操作可达性和执行代价；
4. 用固定遮挡等级场景完成消融和重复试验。

### M5：系统补全

1. 将已经具备的 RTAB-Map/Nav2 接到“到达任务区”之前；
2. 将局部在线点云或体素障碍接入 MoveIt Planning Scene；
3. 加入操作结果验证和失败恢复；
4. 最后评估第二台固定 D435、多相机融合和 Jetson 部署是否对论文有必要。

## 7. 明确不做或后置

- 不做统一承担导航、语义、碰撞和目标 belief 的万能三维地图；
- 不在第一版做底盘与机械臂同步运动；
- 不以通用开放词汇、VLA 或大量类别训练作为前置条件；
- 不为已有驱动和控制器重新包装一份副本；
- 不为了目录整齐创建没有节点、配置、launch 或测试的 package；
- 不在固定底座 NBV 跑通之前优化复杂全身连续空间。

## 8. 每个新增 package 的最低要求

一个 package 只有同时满足以下条件才允许进入主线：

1. 有明确单一职责和输入/输出；
2. 至少一个可运行节点、库、配置或 launch；
3. 有 README 和最小启动示例；
4. 有静态测试或可重复运行验收；
5. 仿真与实机差异由参数/profile 表达；
6. 失败时安全停止并给出可诊断日志。
