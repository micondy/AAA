# Omni State Lattice planner benchmark

该包只评测 Nav2 全局路径规划，不发布 `cmd_vel`，也不修改
`mobile_navigation_bringup/config/nav2.yaml`。研究对象是 0.66 x 0.62 m 的矩形导航
footprint（另加 0.02 m padding）在 SE(2) 状态空间中的可达性。

## 已提供的地图

四张地图均为 200 x 200 栅格、0.05 m/格、10 x 10 m：

- `open_space`：开阔区加单个中心障碍；
- `l_corridor`：1.05 m 宽 L 形通道；
- `dense_obstacles`：交错密集障碍；
- `narrow_irregular`：三个错位窄门及非对称障碍。

`config/scenarios.yaml` 为每张地图保存两个确定性起终点。地图由代码生成，可以用下面的
命令完全复现：

```bash
ros2 run omni_lattice_benchmark generate_benchmark_maps \
  --output src/omni_lattice_benchmark/maps
```

Nav2 Jazzy 本身还安装了 `depot`、`tb3_sandbox` 和 `warehouse` 三张示例地图，一般位于
`/opt/ros/jazzy/share/nav2_bringup/maps/`。它们用于外部复核；四张专项地图用于解释不同
原语产生差异的原因。工程已有的 `maps/isaac_indoor_top_rt_01.yaml` 用于最终真实场景综合
验证。

## 规划器矩阵

`config/planner_matrix.yaml` 声明五组实验：

1. NavFn A*；
2. SmacPlanner2D；
3. Nav2 官方 Omni State Lattice；
4. 八方向平移加原地旋转的非耦合消融组；
5. 八方向平移乘以 `-1/0/+1` 姿态格变化的耦合组。

官方原语固定到 Nav2 Jazzy commit
`426682ff88405a99617294f5cc8401a61aa5fa50`，下载时校验 SHA-256，避免上游变化破坏
实验可重复性。自定义 lattice 均使用与官方相同的 16 个非均匀姿态格；非耦合组每个姿态
10 条原语，耦合组每个姿态 26 条原语。

重新生成原语：

```bash
ros2 run omni_lattice_benchmark generate_coupled_lattices \
  --output src/omni_lattice_benchmark/lattice
ros2 run omni_lattice_benchmark fetch_official_omni_lattice \
  --output src/omni_lattice_benchmark/lattice/nav2_official_omni_16_5cm_r0.5.json
```

## 构建

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select omni_lattice_benchmark
source install/setup.bash
```

## 单组测试

终端一启动地图和规划器，例如官方 Omni + L 形通道：

```bash
PKG=$(ros2 pkg prefix omni_lattice_benchmark)/share/omni_lattice_benchmark
ros2 launch omni_lattice_benchmark planner_benchmark.launch.py \
  map:=$PKG/maps/l_corridor.yaml \
  planner_profile:=$PKG/config/planner_smac_lattice.yaml \
  lattice_file:=$PKG/lattice/nav2_official_omni_16_5cm_r0.5.json
```

终端二只请求路径，不会控制机器人：

```bash
ros2 run omni_lattice_benchmark planner_benchmark \
  --scenario l_corridor \
  --planner-label smac_lattice_official_omni \
  --repeat 10
```

切换到耦合原语时，只替换：

```text
lattice_file:=$PKG/lattice/omni_coupled_16_5cm.json
--planner-label smac_lattice_coupled
```

NavFn 与 SmacPlanner2D 不需要 `lattice_file`，分别传入 `planner_navfn.yaml` 和
`planner_smac_2d.yaml`。

每次运行输出 JSON 和 CSV，记录成功状态、规划耗时、平移路径长度、总姿态变化量、路径
位姿数、原地旋转段数，以及路径首末端相对请求位姿的误差。Action 即使返回成功，若路径
末端没有到达目标，也会被标记为 `INVALID_ENDPOINT`，不会计入成功率。默认输出目录为
`results/navigation/omni_lattice/`。

## 实验边界

- 当前阶段比较的是全局规划能力，不是轨迹跟踪能力。
- NavFn/SmacPlanner2D 不搜索车身姿态，只作为二维基线。
- 主工程当前采用官方 Omni State Lattice 与 MPPI Omni；本工作区只负责全局规划对比，
  不作为主工程的运行依赖。侧后感知覆盖完成前，全向实车测试仍须限速并有人监护。
- 自定义耦合原语暂不施加轮角、轮速和转向速率约束，符合第一阶段研究假设。
- Nav2 Jazzy 的中间碰撞采样还需通过旋转扫掠专项测试确认；在确认前，规划成功不等同于
  实车可安全执行。
