# Nvblox Nav2 A/B 初测

2026-09-19使用相同四点配置验证两种costmap profile：

| profile | 成功 | 恢复 | 四目标动作总耗时 |
|---|---:|---:|---:|
| PointCloud2基线（2026-09-15） | 4/4 | 0 | 92.25 s |
| Nvblox ESDF（2026-09-19） | 4/4 | 0 | 45.27 s |

Nvblox运行时已确认local costmap加载`nvblox_layer + inflation_layer`，global costmap加载
`static_layer + nvblox_layer + inflation_layer`；两个插件都订阅
`/nvblox_node/static_map_slice`。

这不是严格的速度对照：两轮日期不同，仿真状态和到点剩余距离也不同。因此本记录只证明
Nvblox profile能够加载并完成既有路线，不据此宣称性能优于点云基线。

2026-09-21又完成了分类障碍静态A/B。`nvblox_costmap_obstacles_20260921_115026.json`直接
统计前方ROI的致命栅格：低矮、悬空、货架分别新增155、366、365格，移除8秒后残留均为0。
三类障碍也全部通过独立PointCloud2 collision monitor停车/恢复测试。

低矮障碍首次在0.75 m处未被顶部相机看到，证明存在近端盲区；正式配置将停车观察区扩到
1.10 m，使其在1.00 m处提前停车。真实机器人仍需测量最低可见高度和实际制动距离，不能
仅凭仿真结果认为近地感知已经完全解决。
