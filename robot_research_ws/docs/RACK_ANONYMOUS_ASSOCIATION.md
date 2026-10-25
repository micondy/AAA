# 层架无真值架号关联实验

本实验回答一个独立问题：在准确的床面像素分割已知、但**不把 Isaac 架号和层号传给几何/关联后端**时，多视角观察能否维持同一架及各层的稳定 ID。它不测试真实图像分割、D435i 外参、定位误差或 Nav2 全通道通行。

## 数据边界

- Isaac 仍生成 `rack_XX_layer_Y` 语义图，供仿真前端分开相邻床面；这属于受控的“分割上限”输入。
- `rack_detector` 的 `anonymous_instances:=true` 每帧把可见床面随机映射到 1～9999 的临时 `mono16` ID，立柱映射到 10000～65535；`/rack_detector/instances` 只写 `bed`/`post` 和像素数，不写原始语义 ID/标签。临时 ID 不可用作跨帧关联。
- `rack_geometry` 对每个临时掩膜做深度反投影，并用参数化三层的高度集合 `[0.55, 1.05, 1.55] m` 排除夹在层面之间的深度碎片；不使用 `mask_id % 10` 推断层号。
- `rack_map` 在该模式设置 `use_source_instance_hints=false`。同帧分组使用独立的 `group_distance_m=1.1`，以组中心约束距离，并禁止同高度床面并入一个组；跨帧地标关联使用 `rack_distance_m=1.8`，以 XY 距离、层高做门控，局部 PCA 长轴只作弱评分，再用一对一最小代价分配。同一已关联架内按高度合并层对象，避免局部可见面积变化生成新层。候选连续见到 3 帧且积累 3 层才发布稳定架 ID；未观测到的旧对象保留并标 `observed=false`。
- 验收脚本里的 4×4 真实坐标只用于**输出评分**，不会发送给 detector、geometry 或 map。它统计同一实物对应多个稳定 ID（fragment）、同一稳定 ID 对应多个实物（switch），并检查 16 架/48 层、几何误差和路线进度。

这是同一次运行内的稳定 ID；`rack_map` 重启后从空地图和计数器重新开始。跨重启 ID 持久化需要另做地图保存、重载与重定位。

## 运行

从全部关闭的状态，在终端 1：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash
colcon build --symlink-install --packages-select rack_perception mobile_manipulator_bringup
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py \
  anonymous_instances:=true headless:=true test_seconds:=900
```

仿真出现 `/rack_map/model` 后，终端 2：

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
python3 /media/scc/Windows_D/AAA/robot_research_ws/tools/run_rack_grid_acceptance.py \
  --anonymous --duration 720 \
  --output /media/scc/Windows_D/AAA/robot_research_ws/results/rack/grid_anonymous_acceptance_retry_20261020.json
```

脚本直接向仿真 `/cmd_vel` 发低速命令，仅用于感知验收。**不要**在网页/Nav2 联动仿真还开着时再启动这个独立仿真。

## 判定边界

通过条件除了 25/25 路点、最终 16 架/48 层、完整地图 ID 集合一致、层高/位置误差和无包围盒重叠外，还要求 16 个真实架与 48 个真实层都被关联，且架/层 ID fragment 与架 ID switch 为零。几何匹配门槛目前针对参数化 4×4 场景，不应直接套到真实棚架。

## 2026-10-20 完整重测

报告：`results/rack/grid_anonymous_acceptance_retry_20261020.json`。结果为 **通过**：25/25 路点、最终 16 架/48 层、4131 个地图帧、完整地图稳定 ID 集合只有 1 套；16 个真实架和 48 个真实层均曾关联，架 ID fragment/switch 与层 ID fragment 均为 0。最大层高误差 0.0163 m，最大架位误差 0.3024 m。包围盒重叠风险样本 0，但最小估算间隙仅 0.0463 m，不是安全通行证明。路线耗时约 593 s，测试脚本直接发 `/cmd_vel`，没有验证 Nav2 全程自主导航。

保留两个失败报告，便于复现改动缘由：`grid_anonymous_acceptance_20261019.json` 在 2/25 中断，局部 PCA 长轴的硬门控导致重复架 ID；`grid_anonymous_acceptance_20261020.json` 在 3/25 中断，同帧分组距离与跨帧关联距离共用 1.8 m，邻架片段被串组，一个架出现超过三层。最终修复是弱化朝向评分、将分组距离独立为 1.1 m、禁止同高度重复入组，并在已关联架内按高度合并层对象。两个失败报告均未覆盖。
