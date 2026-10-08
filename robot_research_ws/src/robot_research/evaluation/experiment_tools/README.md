# experiment_tools

阶段 A 的可复现实验工具。目前提供 D435i 录包和在线帧率/时间戳检查。

相机启动后，在另一个终端运行：

```bash
ros2 run experiment_tools camera_health_monitor
ros2 launch experiment_tools monitor_camera.launch.py stage:=full
ros2 launch experiment_tools record_camera.launch.py \
  stage:=full output:=bags/camera_001
```

`monitor_camera.launch.py` 和 `record_camera.launch.py` 的 `stage` 必须与
`d435i_bringup` 一致。`basic/imu` 自动使用原始深度，`aligned/full` 自动使用对齐深度；
`basic` 不要求 IMU。也可直接运行节点并手工指定主题：

```bash
ros2 run experiment_tools camera_health_monitor --ros-args \
  -p depth_topic:=/camera/camera/depth/image_rect_raw -p require_imu:=false
```

自定义相机名称时，三个入口使用相同参数：

```bash
ros2 launch d435i_bringup camera.launch.py \
  camera_namespace:=robot1/sensors camera_name:=front stage:=full
ros2 launch experiment_tools monitor_camera.launch.py \
  camera_namespace:=robot1/sensors camera_name:=front stage:=full
ros2 launch experiment_tools record_camera.launch.py \
  camera_namespace:=robot1/sensors camera_name:=front stage:=full \
  output:=bags/front_001
```

录制流程建议包含约 10 秒静止、缓慢平移、绕三个轴旋转和绕物体一圈。用 `Ctrl-C`
正常结束录制，随后检查：

```bash
ros2 bag info bags/camera_001
ros2 launch experiment_tools play_camera.launch.py bag:=bags/camera_001
```

回放时用 `use_sim_time:=true` 启动消费 rosbag 的节点。
