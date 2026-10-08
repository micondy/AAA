# nvblox_bringup

可选的单 RGB-D 局部三维重建层，建立 TSDF、Mesh 和二维 ESDF slice；它不负责
SLAM、里程计或全局定位。Nav2可通过 `mobile_navigation_bringup` 的默认关闭 profile
消费ESDF slice。系统必须已有
`odom -> base_link -> nav_camera_color_optical_frame` TF。

默认输入：

```text
/sensors/nav_camera/color/image_raw
/sensors/nav_camera/aligned_depth_to_color/image_raw
/sensors/nav_camera/color/camera_info
```

默认主要输出：

```text
/nvblox_node/mesh
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
```

## 已有相机/仿真数据

```bash
ros2 launch nvblox_bringup mapping.launch.py \
  use_sim_time:=true input_qos:=DEFAULT start_camera:=false
```

## 固定安装的真实 D435i

只有整机 TF 已经由 robot description 发布时才使用此入口：

```bash
ros2 launch nvblox_bringup mapping.launch.py \
  start_camera:=true serial_no:=D435I_SERIAL
```

手持相机没有稳定的 `odom -> camera` 位姿，不能用来建立一致的 nvblox 地图。当前只支持
顶部导航 D435i；另外两台 D435 在安装和标定完成前不进入此 profile。

## 验收边界

1. RGB、对齐深度、CameraInfo 和 TF 连续可用；
2. RViz 能看到持续更新的 Mesh 和 ESDF slice；
3. nvblox关闭或异常时不影响 RTAB-Map、Nav2 和机器人控制；
4. 记录处理频率、GPU显存和相机丢帧；
5. Nav2 A/B profile必须保持默认关闭，不能直接替换当前点云障碍层。

移动映射验收可复用现有低速闭环路线，同时运行监测器：

```bash
ros2 run nvblox_bringup nvblox_mapping_monitor.py \
  --duration 60 --minimum-path-length 1.0 \
  --output results/nvblox/mapping_run.json
```

报告不仅检查话题是否存在，还要求 Mesh含顶点、ESDF点云非空、距离切片含已观测单元，
并累计底盘里程、最大深度断流间隔以及GPU显存/利用率/温度，避免把空消息误判为通过。
