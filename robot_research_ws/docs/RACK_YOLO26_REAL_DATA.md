# 实体棚架 YOLO26n-seg 接入

状态（2026-10-20）：**推理节点、训练入口和 ROS 接口已实现；已有 Isaac 合成标注与仿真权重，但尚无真实棚架多边形标注，也未完成实体检测验收。** 仿真权重只能用于验证数据和 ROS 管线，不应当作实体 D435i 模型。第一次合成采集缺少正确的环境光，训练图上半部发黑，该数据集与权重仅保留为失败对照。现已给棚架场景加日光穹顶和太阳光，并重新采集、训练和验证；结果见 [仿真数据记录](RACK_YOLO26_SYNTHETIC.md)。

## 数据与标签

使用 Ultralytics 实例分割格式。复制 `src/robot_research/perception/rack_perception/config/rack_dataset.example.yaml` 为真实数据根目录的 `data.yaml`，把 `path` 改成绝对路径。目录为 `images/train`、`images/val`、`labels/train`、`labels/val`；同名图像和 `.txt` 配对。每个可见对象占一行：`类别 x1 y1 x2 y2 ...`，坐标归一化到 0～1，至少三个顶点。[官方格式](https://docs.ultralytics.com/datasets/segment)

类别固定为 `0: bed`（**每层种植床分别标一个实例**）、`1: post`（每根可见立柱分别标一个实例）。不要把架号或层号写成训练类别；它们由 `rack_map` 基于三维几何关联。遮挡的不可见部分不要凭空补全。建议按采集日期/位置划分训练与验证，不要把同一段连续视频的邻帧随机分到两边。保留空场景、斜视角、遮挡、不同距离和光照的样本。

## 环境与训练

本机 ROS 2 使用 Python 3.12，GPU 为 RTX 5060 Laptop 8 GB。已在工作区 `.venvs/rack_yolo` 建独立虚拟环境，安装并验证 `ultralytics==8.4.183`、NumPy 1.26.4、OpenCV 4.10.0.84；它继承 ROS、PyTorch CUDA 系统包，`pip check` 通过。重建环境可用：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
python3 -m venv --system-site-packages .venvs/rack_yolo
.venvs/rack_yolo/bin/python -m pip install 'numpy==1.26.4' 'opencv-python==4.10.0.84' 'ultralytics==8.4.183'
```

标注到位后，在**不运行 Isaac/Nvblox** 时训练，避免 8 GB 显存争用：

```bash
source tools/source_robot_stack.bash
.venvs/rack_yolo/bin/python tools/train_rack_yolo26.py \
  --data /绝对路径/rack_dataset/data.yaml --device 0 --batch 4 --imgsz 640
```

脚本会先检查两类标签和训练/验证目录，缺少标注时拒绝开训。训练从官方 `yolo26n-seg.pt` 迁移学习，权重通常在 `runs/rack_yolo26/bed_post/weights/best.pt`（重复运行可能自动加后缀）；实际以训练输出为准。训练前核对 Ultralytics AGPL/商业许可。[官方训练说明](https://docs.ultralytics.com/modes/train)

## ROS 运行与查看

终端 1 启动实体 D435i 的彩色和对齐深度流：

```bash
source tools/source_robot_stack.bash
ros2 launch d435i_bringup camera.launch.py \
  camera_namespace:=sensors camera_name:=nav_camera stage:=aligned use_rviz:=false
```

终端 2 启动棚架推理、几何和稳定 ID 地图：

```bash
source tools/source_robot_stack.bash
ros2 launch rack_perception rack_real_yolo.launch.py \
  python_executable:=/media/scc/Windows_D/AAA/robot_research_ws/.venvs/rack_yolo/bin/python \
  weights_path:=/绝对路径/best.pt output_frame:=odom
```

检测器要求权重类别**恰好**为 `bed`、`post`；缺权重或拿 COCO 通用权重会明确失败，不会产生貌似有效的床面地图。它订阅 `/sensors/nav_camera/color/image_raw`，发布 `/rack_detector/instance_mask`（`mono16`，每帧临时 ID）、`/rack_detector/debug/color_mask` 和 `/rack_detector/instances`。`rack_geometry` 使用同名 D435i 对齐深度和内参；`rack_map` 禁用 Isaac 架号提示。节点以最新帧优先、目标最多 10 Hz 推理；实际频率需实测，未承诺 10 Hz。

```bash
ros2 topic hz /rack_detector/instance_mask
ros2 topic echo --once /rack_detector/instances --field data
ros2 topic echo --once /rack_map/model --field data
rqt_image_view /rack_detector/debug/color_mask
```

`rack_geometry` 必须能在图像时间查到 **`odom -> nav_camera_color_optical_frame`** 的完整 TF 链，否则只会跳帧；这份 launch **不启动定位、TF 发布器、Nav2 或底盘控制**。若只想先检查二维分割，可加 `start_geometry:=false start_map:=false`。仿真中的同帧/跨帧关联距离、三层确认门槛只是实机初值，需用实体数据重测高度、前沿、ID switch、漏检和推理延迟。
