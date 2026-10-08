# RTAB-Map RGB-D 基线实施计划

更新日期：2026-09-08。

## 目标

使用 D435i 的 RGB、对齐深度和相机内参完成可重复的 RTAB-Map RGB-D SLAM，支持在线
建图、离线 rosbag 建图以及加载数据库重新定位。本链路现在是固定 D435i 导航的当前主线；
目标三维感知、目标 belief 和主动视角仍保持独立。RTAB-Map 自身验收阶段不接 Nvblox 或
语义分割，完成地图定位后再由 `mobile_navigation_bringup` 接入 Nav2。

## 数据链

```text
d435i_bringup stage:=aligned
  -> color/image_raw
  -> aligned_depth_to_color/image_raw
  -> color/camera_info
  -> rtabmap_sync/rgbd_sync
  -> rtabmap_odom/rgbd_odometry
  -> odom_rgbd_image（复用视觉特征）
  -> rtabmap_slam/rtabmap
  -> odom + map + mapData + RTAB-Map database
```

D435i 合并 IMU 当前只有角速度和线加速度，不直接作为姿态测量进入基线。后续先增加
IMU 姿态滤波和标定，再决定接入 RTAB-Map 或 `robot_localization`。

## 已完成

- [x] 创建 `rtabmap_bringup` ROS 2 包。
- [x] 添加独立 RGB-D 同步、里程计和图优化节点编排。
- [x] SLAM 复用 RGB-D odometry 的特征输出，避免重复提取视觉特征。
- [x] 添加 `online.launch.py` 在线建图入口。
- [x] 添加 `offline.launch.py` rosbag 离线建图入口。
- [x] 添加 `localization.launch.py` 已有数据库定位入口。
- [x] 支持 camera namespace/name、map/odom/base frame 参数。
- [x] 区分 mapping 和 localization 的 RTAB-Map 内存模式。
- [x] 默认不删除数据库；只有显式 `fresh_map:=true` 才重新建图。
- [x] 添加 RGB-D 里程计和占据地图初始参数。
- [x] 添加 RTAB-Map 地图、轨迹、TF 和点云 RViz 配置。
- [x] 添加配置解析、模式和相机话题单元测试。

## 尚未完成

- [ ] 在 Ubuntu 24.04 + ROS 2 Jazzy 安装 `ros-jazzy-rtabmap-ros` 并完成构建。
- [ ] 使用真实 D435i 验证在线建图。
- [ ] 使用标准 D435i rosbag 验证离线建图。
- [ ] 确认 RGB 与对齐深度能够持续精确同步。
- [ ] 测量视觉里程计频率、丢失次数、处理延迟、CPU 和内存占用。
- [ ] 完成闭环路线测试并记录闭环前后漂移。
- [ ] 验证数据库保存、重新加载和丢失后的重定位。
- [ ] 相机固定后标定 `base_link -> camera_link`。
- [ ] 接入轮速里程计、IMU 姿态滤波和 `robot_localization`。
- [ ] 接入 Nav2 全局地图。
- [x] 创建与 RTAB-Map 解耦的 Nvblox 单相机局部 TSDF/ESDF 启动层；运行验收和 Nav2
  A/B 接入仍单独进行。

实例分割、目标 belief 和主动视角属于独立的任务局部感知链，不列为本 RTAB-Map 计划的
验收项。

## 验收标准

1. 同一 rosbag 连续运行三次均能完成，节点不崩溃。
2. RGB-D odometry 输出平均频率不低于 10 Hz。
3. 标准闭环过程中不发生无法自动恢复的持续跟踪丢失。
4. 回到起点后的平移闭环误差低于路线长度的 2%，并记录真实测量方法。
5. `/rtabmap/map`、`/rtabmap/mapData` 和 `/rtabmap/mapPath` 可在 RViz2 查看。
6. 生成的数据库可重新加载并完成重定位。
7. 记录所用代码版本、参数文件、相机配置、rosbag 名称和测试环境。

## 执行记录

| 日期 | 内容 | 结果 |
|---|---|---|
| 2026-09-08 | 创建第一版包、配置、launch、RViz 和单元测试 | 已完成静态实现 |
| 2026-09-08 | Python AST、XML、行长与 Git 差异检查 | 已通过 |
| 2026-09-08 | Python 单元测试 | 未执行：Windows 环境缺少 pytest |
| 待定 | Ubuntu 24.04 / ROS 2 Jazzy 构建与测试 | 未执行 |
| 待定 | D435i rosbag 离线建图 | 未执行 |
| 待定 | D435i 在线建图与重定位 | 未执行 |

## 后续顺序

```text
RTAB-Map 离线基线
-> RTAB-Map 在线建图
-> 已有地图定位
-> 与轮式里程计/状态估计建立唯一 TF 链
-> Nav2 将机器人送到目标区域附近
-> 按实际需要评估 Isaac Visual SLAM 或 Nvblox
```

RTAB-Map 第一版既提供里程计也提供全局图优化。以后切换到 Isaac Visual SLAM 时，保留
RTAB-Map 的回环和全局地图能力，替换局部视觉里程计来源，而不是同时维护两套全局地图。
目标实例和不确定性保存在独立的任务局部地图中，不直接写入 RTAB-Map 数据库。
