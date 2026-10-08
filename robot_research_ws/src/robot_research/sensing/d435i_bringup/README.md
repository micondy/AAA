# d435i_bringup

D435i 独立启动、参数与 RViz2 配置。

```bash
ros2 launch d435i_bringup camera.launch.py stage:=basic use_rviz:=false
ros2 launch d435i_bringup camera.launch.py stage:=imu use_rviz:=false
ros2 launch d435i_bringup camera.launch.py stage:=aligned use_rviz:=false
ros2 launch d435i_bringup camera.launch.py stage:=full
```

可用参数包括 `serial_no`、`params_file`、`reset_on_start`、`camera_name` 和 `camera_namespace`。独立运行时 RViz2 固定坐标系为 `camera_link`。

## 实机基线（2026-09-08）

- D435i 序列号：`346122070466`
- 固件：`5.17.3.10`
- USB：`3.2`
- RGB / Depth：`848x480@30`
- Accel / Gyro：`100 Hz / 200 Hz`
- 推荐建图阶段：`stage:=aligned`；Nav2 需要前方点云时再使用
  `stage:=full`

RealSense `/diagnostics` 的实测值为 Depth `29.90 Hz`、Color
`30.00 Hz`、Accel `100.00 Hz`、Gyro `200.30 Hz`。对原始大图像运行
`ros2 topic hz` 会受 Python 反序列化速度影响而明显低报，应优先以驱动诊断值
判断采集帧率。

相机当前报告 `IMU Calibration is not available`，驱动会采用默认 IMU 内外参。
这不阻塞 RGB-D RTAB-Map 建图，但在正式使用 IMU 融合前需单独完成静置噪声、
轴向与温漂标定。
