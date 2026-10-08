# rtabmap_bringup

D435i 的 RTAB-Map RGB-D 基线。当前使用彩色图、对齐深度和相机内参，不使用未经姿态
滤波的 D435i 原始 IMU。

移动底盘模式同时设置 `Reg/Force3DoF=true` 和 `Optimizer/Slam2D=true`，保证视觉里程计
及地图优化只输出平面 `x/y/yaw`，避免 RGB-D 的高度、横滚和俯仰漂移进入 Nav2。

安装 Jazzy 二进制依赖：

```bash
sudo apt update
sudo apt install ros-jazzy-rtabmap-ros
```

在线建图：

```bash
ros2 launch rtabmap_bringup online.launch.py \
  fresh_map:=true database_path:=$HOME/.ros/room_001.db
```

实车底盘与网页联合运行时，先启动底盘，再使用专用入口：

```bash
ros2 launch rtabmap_bringup mobile_base_online.launch.py \
  database_path:=$HOME/.ros/mobile_base_online.db
```

该入口强制使用实时 D435i 和 `/swerve_controller/odom`，地图发布在
`/rtabmap/map`；默认 `fresh_map:=false`，不会删除已有数据库。

离线 rosbag 建图：

```bash
ros2 launch rtabmap_bringup offline.launch.py \
  bag:=bags/d435i_001 database_path:=$HOME/.ros/d435i_001.db
```

加载已有数据库定位：

```bash
ros2 launch rtabmap_bringup localization.launch.py \
  database_path:=$HOME/.ros/room_001.db
```

`fresh_map:=true` 会删除所选数据库后重新建图，只应在确认输出路径后使用。相机安装到
机器人后，把 `base_frame` 改为 `base_link`，并由整机描述提供
`base_link -> camera_link`。底盘能提供连续里程计时使用
`odom_source:=external odom_topic:=/swerve_controller/odom`：此时不会启动
`rgbd_odometry`，底盘负责 `odom -> base_link`，RTAB-Map 只负责 `map -> odom`。
手持相机或尚无底盘里程计时保留默认的 `odom_source:=rgbd`。

## 手持冒烟测试（2026-09-08）

使用 D435i `346122070466` 完成约 60 秒手持建图，数据库保存在
`maps/handheld_test_01.db`。数据库包含 48 个关键帧、8554 个视觉词袋特征，累计
里程约 4.74 m，并记录到 6 条全局回环约束，证明 RGB-D、视觉里程计、建图、TF
与数据库保存链路可用。

本次手持运动中视觉里程计自动重置 27 次，最终形成 14 个会话/子地图，因此该结果
只作为功能冒烟测试，不能作为 Nav2 地图。日志中的主要原因是快速六自由度运动及
部分画面特征不足。相机固定到底盘后，应使用平面低速运动并引入轮速里程计，再进行
正式地图验收。
