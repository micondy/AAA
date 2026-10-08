# Target perception

`target_perception` 是主动感知链的第一版可重复目标观测基线。它不启动相机，也不依赖
RealSense 或 Isaac Sim 的具体节点；只订阅统一的彩色图、对齐深度和 CameraInfo。

当前 `color_depth_detector` 检测最大的指定 HSV 色块，用目标区域内的有效深度点进行 MAD
离群值过滤和针孔反投影。它用于验证三维观测、TF、数据记录和后续 belief 接口，不作为
最终语义识别算法。

2026-09-15 已把二维前端与 RGB-D 几何拆开：`estimate_mask_target()` 接受任意二值实例
mask、网络置信度、对齐深度和内参，继续输出相同的三维位置、尺寸、协方差与质量指标。
因此后续普通检测/分割模型只需替换“生成 mask 与类别”的前端，不重写深度反投影、TF、
`Detection3DArray` 或 Target Belief 接口。HSV 节点继续作为确定性回归后端保留。

## 通用日常物体实例分割

`instance_segmentation` 使用 torchvision Mask R-CNN COCO 基线，默认启用标准权重所学的
全部通用 COCO 类别（人、交通工具、杯瓶、家具、电子设备等），不包含蘑菇专用类别。它
支持同帧多个实例，并把每个 mask 与对齐深度融合为 `Detection3DArray`。任务需要时可在
实验参数文件中把 `allowed_classes` 从 `["*"]` 改为明确的类别白名单。

模型绝不隐式下载。准备好本地权重后启动：

```bash
ros2 launch target_perception instance_segmentation.launch.py \
  weights_path:=/absolute/path/maskrcnn_resnet50_fpn_v2_coco.pth \
  device:=cuda
```

`/target_perception/debug/instances` 是 `16UC1` 实例标签图，0 表示背景，1、2……对应当帧
检测顺序。默认不发布单目标 `target_pose`，防止不同类别进入同一个 Target Belief；需要
跟踪某一类时显式设置 `primary_class_id:=cup` 等。原始检测 ID 仍是帧内 ID；
`instance_tracking` 使用类别门控与三维距离关联，另行发布跨帧基线 ID。

`/target_perception/debug/overlay` 是供人检查的 BGR 叠加图，包含彩色实例 mask、二维框、
类别和置信度；算法和评测不要从这张调试图反向读取数据。

## 接口

输入：

```text
/sensors/nav_camera/color/image_raw
/sensors/nav_camera/aligned_depth_to_color/image_raw
/sensors/nav_camera/color/camera_info
```

输出：

```text
/target_perception/detections_3d  vision_msgs/msg/Detection3DArray
/target_perception/tracked_detections_3d vision_msgs/msg/Detection3DArray
/target_perception/detections_2d  vision_msgs/msg/Detection2DArray
/target_perception/target_pose    geometry_msgs/msg/PoseWithCovarianceStamped
/target_perception/metrics        diagnostic_msgs/msg/DiagnosticArray
/target_perception/debug/instances sensor_msgs/msg/Image (16UC1)
/target_perception/debug/overlay   sensor_msgs/msg/Image (bgr8)
```

`Detection3DArray` 中包含帧内目标 ID、类别、位置协方差、近似三维包围盒和置信度。通用
实例分割节点的 metrics 记录模型实例数、深度有效实例数、融合耗时、设备和权重哈希；HSV
节点另外记录 `valid_depth_points`、`valid_depth_ratio`、色块面积和几何质量分数。不要把
原始帧内 ID 当作对象身份；需要稳定 ID 时订阅 `tracked_detections_3d`。

## 跨帧 ID 基线

`instance_tracking` 只关联类别相同且三维距离不超过 0.35 m 的实例，默认允许丢失 5 帧。
它已经用 ROS 消息验证将原始 `person_01` 改写为稳定的 `person_0001`。这是一条可解释的
工程基线，适合静态或低速普通物体；同类物体交叉、相机快速运动和长时间遮挡仍可能换 ID。
论文阶段应在真实 rosbag 上比较三维距离、mask IoU 和外观特征关联，而不是把当前贪心关联
包装成创新点。

单独启动：

```bash
ros2 launch target_perception instance_tracking.launch.py
```

仓库已验证的官方权重位于：

```text
models/maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth
SHA256 73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e
```

2026-09-15 的 ROS 2 端到端冒烟测试使用真实 RGB 样例与 1.5 m 合成对齐深度，正确发布了
两个 person 三维实例。该测试证明软件链路连通；下一项仍是用真实 D435i 对普通物体录制
固定 rosbag 并做 100 帧回归。

通用多实例评测器同步记录 `Detection3DArray` 和 diagnostics，输出 `frames.csv`、
`instances.csv` 与 `summary.json`。汇总包含帧检出率、每帧实例数、模型实例到深度有效实例
的转化率、类别计数、置信度，以及推理和融合耗时的均值、标准差、P50、P95 和最大值。
输出目录非空时拒绝启动，不会覆盖已有实验。

## 运行

构建后启动节点：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
colcon build --symlink-install --packages-select target_perception
source install/setup.bash
ros2 launch target_perception color_depth.launch.py
```

仿真时增加 `use_sim_time:=true`。若尚未发布相机到 `base_link` 的 TF，可先在相机光学坐标
系中验收算法：

```bash
ros2 launch target_perception color_depth.launch.py \
  use_sim_time:=true output_frame:=nav_camera_color_optical_frame
```

### 已完成的仿真验收

2026-09-11 使用 Isaac Sim 中的 20 cm 绿色方块完成端到端测试：恢复宽高约
0.2001×0.2025 m；可见前表面深度为 0.76955 m，与仿真几何真值一致；有效深度点 6889，
有效率 98.8%。这只证明当前接口和几何基线正确，多距离重复性及真实 D435i 误差仍需单独
验收。

单个距离的自动记录入口（距离指相机光学中心到方块可见前表面）：

```bash
ros2 launch mobile_manipulator_bringup sim_perception.launch.py \
  headless:=true run_benchmark:=true green_target_depth:=0.4 \
  benchmark_samples:=100 \
  benchmark_output_csv:=/tmp/target_depth_0p4.csv
```

记录器忽略预热帧后，同步记录 detection 与 diagnostics，生成逐帧 CSV 及同名
`.summary.json`；记录完成后 Isaac 按评测所需的仿真时长自行结束。0.4、0.6、0.8、1.0 m
四距离批量入口只启动一次 Isaac，在同一场景内移动目标：

```bash
source tools/source_robot_stack.bash
python3 tools/run_target_distance_benchmark.py --samples 100
```

每次运行创建独立时间戳目录，并生成 `distance_summary.csv`，不会覆盖已有非空目录。

2026-09-11 的正式仿真基线使用每档 15 帧预热、100 帧记录和 30 秒阶段时长。四档检测率均
为 100%，位置 RMSE 分别为 0.834、1.754、1.716、3.083 mm，有效深度率为
98.1%～100%。结果位于
`results/target_perception/distance_20260911_4x100`。仿真渲染高度确定性，因此该结果只用
作软件和几何回归；真实 D435i 仍需使用相同协议录制 rosbag 并评测噪声、温漂与外参误差。

默认检测绿色物体。修改 HSV 阈值时复制
`config/color_depth.yaml` 为实验专用参数文件，不要为不同目标颜色复制节点代码。红色跨越
OpenCV Hue 的 0° 边界时可以设置 `hsv_lower[0] > hsv_upper[0]`，节点会自动组合两段色域。

## 验收边界

- 深度必须已经对齐到彩色图，两个图像尺寸必须一致；
- 支持 RealSense 常用 `16UC1` 毫米深度和 Isaac `32FC1` 米深度；
- 请求的 `output_frame` TF 不可用时丢弃本帧并发布 ERROR diagnostics，不伪造坐标；
- 未检测到目标时发布空 Detection 数组和 WARN diagnostics；
- 目标方向当前未知，姿态协方差相应设为大值；
- 实机论文数据前需使用已知尺寸目标在 0.4–1.0 m 完成误差和重复性标定。
