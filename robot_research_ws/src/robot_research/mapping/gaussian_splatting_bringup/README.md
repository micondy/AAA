# gaussian_splatting_bringup

将 ROS 2 的对齐 RGB-D、CameraInfo 和 TF 轨迹采集成 Nerfstudio 数据集。在线节点只做
同步、抽帧、PNG/JSON 写盘和稀疏种子点采样，可以和 nvblox 同时订阅同一相机；3DGS
训练放在独立 Nerfstudio 环境中离线执行，不占用导航运行时的 GPU。

输出目录包含：

```text
images/*.png             RGB
depth/*.png              uint16 毫米深度
transforms.json          OPENCV 内参、OpenGL 相机位姿、原始时间戳
sparse_pc.ply            RGB-D 生成的彩色初始化点云
```

## Isaac Sim 采集

先启动 `run_mobile_manipulator.py --ros --enable-nav-rgbd`，然后：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch gaussian_splatting_bringup sim_capture.launch.py \
  output_dir:=/media/scc/Windows_D/AAA/robot_research_ws/datasets/gs/room_01
```

按 `Ctrl+C` 时会写出最终 `sparse_pc.ply`。输出目录必须是新目录，采集器拒绝覆盖已有
`transforms.json`。

## 真实 D435i 采集

相机必须固定在机器人上，并且系统必须持续提供 `odom -> 彩色光学帧` TF。只启动手持相机、
没有全局运动位姿时不能重建一致场景。

```bash
ros2 launch d435i_bringup camera.launch.py stage:=aligned use_rviz:=false
ros2 launch gaussian_splatting_bringup real_capture.launch.py \
  output_dir:=/media/scc/Windows_D/AAA/robot_research_ws/datasets/gs/real_room_01
```

若相机话题名称不同，可覆盖 `color_topic`、`depth_topic`、`camera_info_topic` 和
`global_frame`。缓慢移动并保持曝光稳定；默认至少平移 3 cm 或旋转 3° 才保存下一帧。

### 整机移动建图（一条命令）

固定在底盘上的 D435i 使用下面的入口。它统一启动整机 URDF、`sensors/nav_camera`
RealSense、RTAB-Map RGB-D 里程计/SLAM、nvblox、高斯采集和 RViz：

```bash
ros2 launch gaussian_splatting_bringup real_mobile_mapping.launch.py \
  output_dir:=/media/scc/Windows_D/AAA/robot_research_ws/datasets/gs/real_room_01
```

默认 TF 所有权为：URDF 发布 `base_link -> nav_camera_link`，RealSense 发布相机内部光学
TF，RGB-D 里程计发布 `odom -> base_link`，RTAB-Map 发布 `map -> odom`。实测支架后必须用
`camera_mount_xyz`、`camera_mount_rpy` 覆盖临时安装值。若底盘驱动已经稳定发布里程计，改用：

```bash
ros2 launch gaussian_splatting_bringup real_mobile_mapping.launch.py \
  odom_source:=external publish_odom_tf:=false \
  odom_topic:=/swerve_controller/odom \
  output_dir:=/path/to/new/dataset
```

任何时刻只能有一个节点发布 `odom -> base_link`。采集结束用 `Ctrl+C`，以便写出最终
`sparse_pc.ply`。

## 与 nvblox 同时运行

可以。两者都是相机话题订阅者，nvblox继续在 GPU 上实时生成 TSDF/ESDF，采集器主要消耗
CPU、磁盘和 DDS 带宽。推荐 320×240@10 Hz（仿真）或限制保存帧率，并观察深度断流。
不要在 8 GB GPU 上同时运行 Isaac、nvblox 和 Splatfacto 训练；先在线采集，停止仿真/导航
后再训练。

## 训练和导出

在独立 Conda 环境安装 Nerfstudio；不要把它的 PyTorch/CUDA 依赖装入 ROS 环境。激活环境后：

```bash
ros2 run gaussian_splatting_bringup train_gaussian \
  /media/scc/Windows_D/AAA/robot_research_ws/datasets/gs/room_01

ros2 run gaussian_splatting_bringup export_gaussian \
  /path/to/nerfstudio/output/config.yml \
  --output-dir /media/scc/Windows_D/AAA/robot_research_ws/results/gaussian_splatting/room_01
```

也可用 `train_gaussian ... --dry-run` 只检查数据集并打印命令。Splatfacto 会使用
`sparse_pc.ply` 初始化；深度文件同时保留给深度监督方法和质量检查。

## 4D 扩展边界

每帧已写入 `timestamp_ns` 和 `time_seconds`。后续 4DGS 导出器可直接使用时间轴，但动态场景
仍需要同步多视角、对象遮罩或 Isaac 真值对象轨迹；当前单移动相机入口先保证静态 3DGS。

## 本机已验证的真实数据预览

`real_mobile_20261005_0633` 已完成 3000 步 Splatfacto 训练（44 帧，约 28.3 万个高斯）。
初版结果位于 `results/gaussian_splatting/real_mobile_20261005_0633/`：
`comparison.jpg` 左侧为采集图，右侧为高斯渲染；`splat.ply` 为高斯模型。
这是静态 3DGS，采集中的移动人物会产生拖影；初版边缘也较模糊。

训练环境位于 `/home/scc/.local/share/robot-research/gs-preview`，复用本机 `yolo26`
环境已有的 PyTorch 2.13/CUDA 13，其余组件安装在独立 venv 中。
已验证 Nerfstudio 1.1.5 和 gsplat 1.5.3 能在 RTX 5060 上编译、训练和导出。
本次使用的模型配置：

```text
/home/scc/.local/share/robot-research/outputs/real_mobile_20261005_0633/splatfacto/2026-10-05_065829/config.yml
```

重新打开查看器（PyTorch 加载开关仅用于本机自己训练产生的可信 checkpoint）：

```bash
source /home/scc/.local/share/robot-research/gs-preview/bin/activate
TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 CUDA_HOME=/usr/local/cuda-13.0 ns-viewer \
  --load-config /home/scc/.local/share/robot-research/outputs/real_mobile_20261005_0633/splatfacto/2026-10-05_065829/config.yml \
  --viewer.websocket-port 7007
```

浏览器打开 `http://localhost:7007`。`tools/render_gaussian_preview.py` 可从配置导出
采集视角的对比图、MP4 和 GIF；无系统 ffmpeg 时使用 `imageio-ffmpeg` 提供的程序。
