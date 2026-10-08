# 普通目标检测/分割与 RGB-D 三维观测基线

> 状态：通用 COCO 实例分割与 RGB-D 三维融合已端到端通过；下一步是真实 D435i 场景回归  
> 当前原则：检测器只是可替换基线，论文创新放在 belief、可见性和主动视角，不放在堆模型

## 1. 最小目标

第一版只需完成：

1. 从统一 RGB 话题输出目标类别、置信度、二维框和单目标实例 mask；
2. 使用对齐深度与内参，将 mask 稳健反投影为三维位置、尺寸和协方差；
3. 保持现有 `/target_perception/detections_3d`、`target_pose`、`metrics` 输出不变；
4. 同一套节点可接 Isaac、D435i 和 rosbag；
5. 对静态目标记录检测率、三维误差、推理延迟和有效深度率。

当前不做开放词汇、视觉语言模型、全场景语义地图和端到端控制。

## 2. 软件边界

```text
RGB
  -> 可替换实例分割前端（class、score、bbox、mask）
  -> 通用 mask + aligned depth 几何
  -> Detection3DArray / PoseWithCovarianceStamped / metrics
  -> Target Belief
  -> 后续底盘 NBV
```

2026-09-15 已增加 `estimate_mask_target()`。它接受任意二值 mask 与前端置信度，完成 MAD
深度离群过滤、针孔反投影、尺寸估计和协方差计算。原 HSV 绿色方块节点已经改为调用同一
函数，所以后续替换前端不会产生第二套三维几何代码。

## 3. 后端选择

开发机当前已有 PyTorch 2.11、torchvision 0.26 和 TensorRT 10.13，没有安装 Ultralytics
或 ONNX Runtime。第一版优先使用 torchvision 的 COCO 预训练实例分割模型作为通用、
可复现基线：

- 5090/笔记本开发：PyTorch 后端；
- Jetson 部署：接口稳定后再导出 ONNX/TensorRT；
- HSV 后端：只用于无网络、无模型权重时的确定性几何回归。

模型权重不提交到仓库；通过参数传入本地路径并记录哈希、类别表和推理配置。没有权重时
节点必须明确拒绝启动，不能静默退回 HSV 后端。

## 4. 实施顺序

1. [已完成] 抽出通用 mask-depth 三维几何并增加单元测试；
2. [已完成] 实例分割前端数据结构与 torchvision 输出解析，已用伪造多实例输出测试；
3. [已完成] 增加 ROS 2 节点，发布标准二维/三维检测、实例标签图、诊断指标，并通过
   `primary_class_id` 防止不同类别混入同一 belief；
4. [已完成] 准备官方 torchvision COCO 权重，并完成 GPU、真实图片和 ROS 2 端到端冒烟测试；
5. [工具已完成、待相机] 使用 D435i 采集含 2～3 类普通物体的固定 rosbag，完成 100 帧
   检出率、深度有效率、类别计数和延迟统计；评测器与一键录包入口已通过合成 ROS 消息闭环；
6. 在 Isaac 中放置 2～3 类普通物体，补充具有几何真值的三维误差统计；
7. [基线已完成、待实流调参] 已增加类别门控的三维距离跨帧关联；使用真实 rosbag 验证
   ID switch 后，再按稳定对象分别接入 Target Belief；
8. 再进入固定顶部相机的底盘 NBV。

## 5. 已验证基线（2026-09-15）

- 权重：`models/maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth`（178 MB）；
- SHA256：`73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e`；
- RTX 5090、CUDA、320×240 全黑图：模型加载 1.874 s，单帧推理 0.653 s；
- 官方 PyTorch Grace Hopper 样例图：推理 0.398 s，检出 person 0.9992、tie 0.9334、
  person 0.7901；默认启用权重支持的全部通用 COCO 类别；
- ROS 2 RGB + 对齐深度 + CameraInfo 端到端：发布 2 个 person 三维实例，恒定 1.5 m
  测试深度均恢复为 1.500 m，输出 frame 为 `test_camera_frame`。

以上只验证模型、消息同步、mask-depth 融合与 ROS 接口连通，并不代表真实相机精度。实际
D435i 测试需要使用真实深度、遮挡和光照变化，不能用这组恒定深度数据替代。

2026-09-16 通用评测器运行态冒烟测试记录 3 个测量帧，其中两个双实例帧和一个空检测帧；
输出正确汇总 `person: 2`、`cup: 2`、4 个深度有效实例和 2/3 帧检出率。测试产物位于
`/tmp/general_object_benchmark_ros_smoke_20260916`，只用于软件回归，不作为论文实验数据。

## 6. 第一版完成条件

- 至少一个非颜色规则的实例分割模型能够端到端运行；
- 同时存在两个目标时分别输出，不再只选择最大色块；
- mask 无有效深度、TF 缺失和模型文件缺失都有明确错误；
- 3D 输出和 Target Belief 无需修改即可消费新观测；
- 固定数据回归有 JSON/CSV 指标，且模型版本与权重哈希可追踪；
- Jetson 优化不阻塞算法闭环，先保证开发机正确性。
