# 运行时数据流

本页描述“启动后发生什么”。接口名以当前默认 profile 为准；具体参数仍以各包 YAML 和
launch 文件为事实来源。

## 建图与定位

```mermaid
sequenceDiagram
    participant C as D435i / Isaac RGB-D
    participant B as 底盘里程计
    participant R as RTAB-Map
    participant TF as TF Tree
    participant M as OccupancyGrid / DB

    C->>R: RGB + aligned depth + CameraInfo
    B->>TF: odom -> base_link
    TF->>R: 相机位姿
    R->>M: /rtabmap/map 与数据库
    R->>TF: map -> odom
    Note over R: 建图模式写数据库；定位模式只加载并校正
```

## 网页点击导航

```mermaid
sequenceDiagram
    actor U as 用户
    participant W as Web Console
    participant N as NavigateToPose
    participant P as Planner / Costmap
    participant C as Controller
    participant S as Velocity Smoother
    participant X as Collision Monitor
    participant B as 底盘或 Isaac

    U->>W: 地图按下选择 x,y
    U->>W: 拖动或输入目标 yaw
    W->>N: map 坐标 Pose
    N->>P: 规划路径
    P-->>N: Path
    N->>C: FollowPath
    C->>S: cmd_vel_nav
    S->>X: cmd_vel_smoothed
    X->>B: /cmd_vel
    Note over X: Isaac profile 保留转发链；障碍源须单独标定后启用
    B-->>W: odom / joint_states / 状态
```

位置到达容差与障碍膨胀是两件事：前者决定离目标多近算完成，后者决定规划时离障碍多远。

## Nvblox 三维环境到网页

```mermaid
sequenceDiagram
    participant C as RGB-D + CameraInfo
    participant TF as odom 到相机 TF
    participant N as nvblox_node
    participant W as Web Console Backend
    participant UI as Browser Canvas

    C->>N: 深度与彩色图像
    TF->>N: 每帧相机位姿
    N->>N: TSDF 融合与 Mesh 更新
    N->>W: /nvblox_node/mesh 增量块
    W->>W: 缓存块并限制采样点数
    UI->>W: GET /api/scene3d?revision=N
    W-->>UI: 彩色 3D 顶点与版本
    UI->>UI: 旋转、透视投影、缩放
```

网页只做显示，不反向影响 Nvblox 或 Nav2。没有 RGB-D、TF 或 Mesh 时显示“等待 Nvblox”，
不会把二维栅格伪装成三维地图。

## 目标感知与主动视角

```mermaid
sequenceDiagram
    participant C as RGB-D
    participant D as Instance / RGB-D Detector
    participant T as Target Selector
    participant B as Target Belief
    participant V as NBV Planner
    participant N as Nav2
    participant E as NBV Executor

    C->>D: 同步 RGB、深度、内参
    D->>T: 多实例 2D/3D detections
    T->>B: 锁定目标观测
    B->>V: 位姿、协方差、可见性
    V->>N: ComputePathToPose 候选查询
    N-->>V: 路径与可达性
    V-->>E: 最佳观察位姿
    Note over E: 默认不自动执行
    E->>N: 二次授权后的 NavigateToPose
    N-->>E: 结果 / 取消 / 超时
```

## 控制模式状态机

```mermaid
stateDiagram-v2
    [*] --> DISABLED
    DISABLED --> NORMAL: 启用底盘控制器
    NORMAL --> DISABLED: 停止或故障
    DISABLED --> MAINTENANCE: 确认轮组架空且反馈健康
    MAINTENANCE --> DISABLED: 退出、失焦或超时
    MAINTENANCE --> MAINTENANCE: 单轮限幅命令
    NORMAL --> NORMAL: 网页 / Nav2 速度命令
```

Isaac 直控 profile 不进入硬件维护模式。

## 典型启动组合

| 场景 | 顶层入口 | 包含 | 明确不包含 |
|---|---|---|---|
| 实车底盘 | `mobile_base_bringup robot.launch.py` | CAN、ros2_control、描述、可选网页 | 感知与导航算法 |
| 底盘 mock | `mobile_base_bringup mock.launch.py` | mock hardware、控制器、描述 | CAN 实机 |
| 实机导航 | `mobile_navigation_bringup navigation.launch.py` | 相机、RTAB 定位、Nav2、安全链 | Nvblox 默认关闭 |
| Isaac 导航 | `mobile_base_sim_navigation.launch.py` | 描述、RTAB 定位、Nav2、网页；可选 Nvblox | Isaac 进程本体、CAN 实机 |
| Nvblox | `nvblox_bringup mapping.launch.py` | TSDF/Mesh/ESDF | 定位所有权 |
| 感知最小链 | `sim_perception.launch.py` | Isaac RGB-D、描述、目标感知 | MoveIt、Nav2 |
| 主动感知 | `sim_active_perception.launch.py` | 感知、Belief、NBV、可选执行 | 默认自动运动 |
| 双臂规划 | `mobile_manipulator_moveit_config demo.launch.py` | MoveIt、Planning Scene、控制器 | 自主导航 |
