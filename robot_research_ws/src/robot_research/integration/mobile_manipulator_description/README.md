# Mobile manipulator description

这个包把现有的四轮四转底盘、官方 OpenArm v1.0 双臂和官方 RealSense D435i 描述连接成一棵 URDF 树。OpenArm 描述固定使用项目 `src/vendor/openarm_description` 中的 Enactic 官方 tag `1.0.4`（commit `5db5232`），集成包不修改任何上游模型，也不负责启动控制器或相机驱动。

## 当前安装假设

- 根坐标系：`base_link`
- 临时安装座：`openarm_mount_link`，250 × 250 × 40 mm，2 kg
- 安装座底面位于底盘上表面：默认 `openarm_mount_xyz:=0,0,0.18`
- OpenArm 朝向与底盘 `+X` 前方对齐：默认 `openarm_mount_rpy:=0,0,0`
- 双臂和双手均启用；OpenArm 的 ROS 2 Control 默认关闭
- 顶部导航相机：`nav_camera`（D435），临时安装位姿为 `xyz=0.10,0,1.12`、
  `rpy=0,0.20944,0`（向下俯视 12°）
- 真机默认只由 URDF 发布 `base_link` 到相机安装根部的 TF；相机内部标定 TF 由 `realsense2_camera` 发布

安装座和相机位姿都是仿真集成临时值。实体 CAD 和相机支架确定后，应只修改这里的安装参数与安装座几何，不修改 `mobile_base_description`、`openarm_description` 或 `realsense2_description`。

## 构建与查看

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
colcon build --symlink-install \
  --packages-select openarm_description mobile_manipulator_description \
  --allow-overriding openarm_description
source install/setup.bash
ros2 pkg prefix openarm_description  # 应指向 robot_research_ws/install
ros2 launch mobile_manipulator_description display.launch.py
```

安装高度或方向可临时覆盖：

```bash
ros2 launch mobile_manipulator_description display.launch.py \
  openarm_mount_xyz:="0 0 0.20" openarm_mount_rpy:="0 0 1.5708"
```

相机安装位姿同样可覆盖；尚未装相机时也可完全关闭：

```bash
ros2 launch mobile_manipulator_description display.launch.py \
  nav_camera_mount_xyz:="0.30 0 0.38" nav_camera_mount_rpy:="0 0 0"

ros2 launch mobile_manipulator_description display.launch.py \
  nav_camera_enabled:=false
```

Isaac Sim 等不运行真机驱动的环境应传入
`nav_camera_use_nominal_extrinsics:=true`，由 URDF 提供标准的彩色、深度和 IMU 光学坐标系。真机不要启用该参数，以免和 RealSense 驱动重复发布内部 TF。OpenArm 胸前的 D435i-B
保留给操作感知，不接入当前 RTAB-Map 导航链。

## 边界

当前相机安装位姿未经实测，不能用于最终外参标定。MoveIt 联合控制、真机外参标定和 Nav2 联动属于后续里程碑。
