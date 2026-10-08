# active_perception

主动感知算法层。当前已实现关注目标锁定、Target Belief、几何可见性、地图遮挡、固定相机
底盘候选生成和 Nav2 可达性/NBV 评分。节点只做决策，不会自行驱动机器人。

## 关注目标锁定

从 `/target_perception/tracked_detections_3d` 中按稳定 ID 或类别锁定一个对象。按类别选择时，
首次选择置信度最高的匹配对象；锁定后不会因为另一对象置信度更高而跳转。目标短时丢失时
保留 ID，超过 `max_missed_frames` 后才释放。

选择器与 Belief 一起启动：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch active_perception selected_target_belief.launch.py \
  target_class:=cup belief_frame:=base_link
```

也可以指定准确的稳定 ID：

```bash
ros2 launch active_perception selected_target_belief.launch.py \
  target_id:=cup_0001 belief_frame:=base_link
```

运行中可以切换目标：

```bash
ros2 param set /target_selector target_id bottle_0001
```

目标改变时，选择器先发布新 ID，Belief 清空旧状态；下一帧才发布新目标位姿，防止两个
对象的观测混入同一滤波器。输出为：

```text
/active_perception/selected_target_id       std_msgs/msg/String
/active_perception/selected_target_pose     geometry_msgs/msg/PoseWithCovarianceStamped
/active_perception/selected_target_visible  std_msgs/msg/Bool
/active_perception/selected_target_detection vision_msgs/msg/Detection3DArray
```

`selected_target_detection` 只包含一个对象，并保留稳定 ID、类别、三维包围盒、位姿和协方差，
供后续可见性与 NBV 模块使用。

## 几何可见性与遮挡模型

`active_perception.visibility` 是无 ROS 依赖的候选视角评价核心。给定针孔相机内参、目标在
候选相机光学坐标系中的中心和三维尺寸，可计算：

- 目标三维包围盒在图像中的完整/裁剪投影；
- 视锥内比例、投影像素面积与画面占比；
- 观察距离和可选表面法向入射余弦；
- 使用对齐深度图计算的前景遮挡率与有效深度率；
- 随距离、可见比例和遮挡变化的预计三维观测协方差；
- 当前 belief 与预计观测之间的高斯信息增益（单位 nats）。

最小调用方式：

```python
import numpy as np
from active_perception.visibility import PinholeCamera, evaluate_view

camera = PinholeCamera(640, 480, 615.0, 615.0, 320.0, 240.0)
result = evaluate_view(
    camera,
    center_camera=(0.0, 0.0, 1.5),
    size_xyz=(0.2, 0.2, 0.3),
    prior_covariance=np.eye(3) * 0.01,
)
```

第一版把目标包围盒近似为候选相机坐标系轴对齐盒；没有表面法向时不施加入射角惩罚。
深度图遮挡只适用于当前真实视角；未来候选位置的遮挡由 A3 使用占据地图射线检查。默认
噪声系数只是 D435 类相机的可解释初值，论文实机数据前必须重新标定。

## 固定相机底盘 NBV

`active_perception.nbv` 生成围绕目标的有限底盘候选，并完成圆形 footprint、未知栅格、
地图净空、二维视线遮挡、候选可见性、信息增益、路径长度和转向代价的分项评分。
`nbv_planner` 节点逐个调用 Nav2 `ComputePathToPose`，不会直接发送 `NavigateToPose`，因此
触发计算不会驱动机器人。每个候选路径查询默认最多等待 8 秒；超时请求会取消并作为不可达
候选继续处理，避免一个 Nav2 请求阻塞整轮 NBV。候选终点通过检查后，还要求 Nav2 整条
路径与目标中心至少保持 1.1 m，而不是只检查终点，以免路径中途穿过目标的停止区。

在目标锁定、Belief、地图、CameraInfo、TF 和 Nav2 planner 都在线后启动：

```bash
ros2 launch active_perception active_perception_stack.launch.py \
  use_sim_time:=true belief_frame:=map target_class:=cup
ros2 service call /active_perception/compute_next_best_view std_srvs/srv/Trigger '{}'
```

如果选择器和 Belief 已由其他入口启动，也可以只运行 `nbv_planner.launch.py`。

与自制底盘 Isaac 仿真联合运行时，推荐直接使用整机入口：

```bash
ros2 launch mobile_manipulator_bringup sim_active_perception.launch.py
ros2 service call /active_perception/compute_next_best_view std_srvs/srv/Trigger '{}'
```

2026-09-17 联合验收中，默认 36 个候选有 3 个通过初筛并完成 Nav2 路径查询，最佳候选
路径 2.954 m、预计信息增益 0.9210 nats、总分 0.9877。该结果验证 A3“选择视角”，不等于
机器人已经移动；导航执行与重新观测属于 A4。

输出：

```text
/active_perception/candidate_view_poses    geometry_msgs/msg/PoseArray
/active_perception/candidate_view_markers  visualization_msgs/msg/MarkerArray
/active_perception/best_view_pose          geometry_msgs/msg/PoseStamped
/active_perception/nbv_diagnostics         diagnostic_msgs/msg/DiagnosticArray
```

当前默认半径为 1.2、1.4、1.6 m，每圈 12 个候选，共 36 个。A3 首次只规划验收使用过
0.6、0.8、1.0 m；A4 首次运动发现 1.0 m 观察位会因目标深度点进入 0.70 m `FrontStop`
区域而停车，因此增加了目标尺寸和深度噪声余量。diagnostics 保留最佳候选的总分、
信息增益、可见比例、净空、地图遮挡率、Nav2 路径长度、转向量和可行候选数。当前二维地图
射线遮挡是保守近似，不能代替未来的三维场景遮挡；真实机器人半径、相机外参和权重都需
在实机验收后标定。

## 最佳视角安全执行器

`nbv_executor` 将 A3 的“选择视角”与 A4 的“执行视角”分开。默认
`execution_enabled=false`，误调用执行服务不会移动。仿真运动测试必须显式使能：

```bash
ros2 launch mobile_manipulator_bringup sim_active_perception.launch.py \
  enable_nbv_execution:=true
```

随后先计算并在 RViz 检查目标，再于 120 秒内明确执行：

```bash
ros2 service call /active_perception/compute_next_best_view std_srvs/srv/Trigger '{}'
ros2 service call /active_perception/execute_best_view std_srvs/srv/Trigger '{}'
```

紧急取消当前目标：

```bash
ros2 service call /active_perception/cancel_view_execution std_srvs/srv/Trigger '{}'
```

执行器拒绝过期视角、同一视角重复执行和并发目标；目标 ID 在运动中变化时取消导航，导航
超时也会取消。成功到达后等待至少 10 个新观测，并将导航状态、恢复次数、运动前后协方差
迹、观测数和 belief 位置变化写入 JSON。协方差下降只代表滤波器内部不确定性下降，不能
替代有仿真真值或人工标注的精度评测。

首次 A4 联调的可复现实验结论和后续修正见
[`results/active_perception/a4_joint_sim_20260917.md`](../../../../results/active_perception/a4_joint_sim_20260917.md)。

## Target Belief

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch active_perception belief.launch.py belief_frame:=base_link
```

Isaac 连续流与自动收敛记录：

```bash
ros2 launch mobile_manipulator_bringup sim_target_belief.launch.py \
  headless:=true test_seconds:=45 run_belief_benchmark:=true \
  benchmark_samples:=100 benchmark_warmup_frames:=15 \
  benchmark_output_csv:=results/active_perception/belief_100.csv
```

记录器生成逐帧 CSV 和同名 JSON 汇总，包含位置 RMSE、最终误差、初始/最终协方差迹及其
下降比例。

输入与输出：

```text
/target_perception/target_pose                 PoseWithCovarianceStamped
/active_perception/target_belief               PoseWithCovarianceStamped
/active_perception/target_observation_count    UInt32
/active_perception/target_rejected_count       UInt32
/active_perception/target_visible              Bool
/active_perception/reset_target_belief         std_srvs/srv/Trigger
```

节点先把每帧观测转换到 `belief_frame`，再用观测协方差做静态目标高斯递推融合。默认使用
三自由度 99% 卡方阈值（Mahalanobis²=11.344867）拒绝离群观测；协方差强制对称并限制特征值，
时间戳倒退的消息也会被拒绝。`target_visible` 表示最近 0.5 秒内是否收到被接受的观测，
并不等同于语义识别置信度。输入订阅使用 ROS 2 sensor-data QoS，与 RealSense、Isaac 和
`target_perception` 的 best-effort 图像处理链兼容。

固定底座实验可使用 `base_link`。底盘开始运动后，必须把 `belief_frame` 改成稳定的
`task_frame`、`map` 或 `odom`，并保证该坐标系到相机观测坐标的 TF 在消息时间戳处可用；
否则不能把移动底盘坐标中的历史观测直接平均。

可视化时使用专用 RViz 配置，它会自动加载目标 belief、放大 40 倍的位置协方差、RGB-D
点云和目标 mask：

```bash
rviz2 -d $(ros2 pkg prefix --share active_perception)/rviz/belief.rviz \
  --ros-args -p use_sim_time:=true
```
