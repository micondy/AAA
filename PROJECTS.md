# 工程与工作区地图

仓库顶层是整套机器人项目的汇总入口，不是单一 ROS 2 工作区。日常开发只需根据任务进入下列三个活跃工作区。

## 活跃工作区

| 目录 | 定位 | ROS 版本 | 日常用途 |
| --- | --- | --- | --- |
| `robot_research_ws/` | 主科研工作区 | Jazzy | 整机模型、RealSense、RTAB-Map、Nav2、感知、仿真和实验评测 |
| `mobile_base_ws/` | 底盘部署工作区 | Humble | Jetson 实车、CAN、四轮四转控制器和网页控制 |
| `omni_lattice_benchmark_ws/` | 独立规划实验工作区 | Jazzy | NavFn、Smac 2D 与 Omni State Lattice 的可复现实验和历史结果 |

主导航运行代码位于 `robot_research_ws`。规划器对比实验位于 `omni_lattice_benchmark_ws`，二者不互相依赖；主导航只自带运行所需的官方 Omni lattice 文件。

## 支撑内容

| 目录 | 内容 | 处理原则 |
| --- | --- | --- |
| `chassis_4wis/` | 四轮四转底盘相关源码或机械资料 | 保留，修改前确认硬件版本 |
| `greenhouse_4wis_thesis/` | 论文源文件 | 与运行代码分开维护 |
| `abc/` | FishBot、YDLidar 等参考/上游仓库 | 视为外部参考，不混入主工作区构建 |
| `archives/`、`配置软件文档等/` | 本地安装包和资料 | 已忽略，不上传 GitHub |
| `robot_research_ws/maps/`、`datasets/`、`models/`、`results/` | 地图、数据、模型和实验输出 | 本地保存，按需单独归档 |

## Git 边界

顶层仓库为 `micondy/AAA` 的工程快照。`robot_research_ws/`、`mobile_base_ws/` 以及 `abc/` 下若干外部项目还保留各自的 `.git` 历史。因此执行 Git 命令前先确认当前根目录：

```bash
git rev-parse --show-toplevel
```

- 更新整套工程快照：在仓库顶层执行 Git 命令。
- 检查科研或底盘工作区的本地历史：进入对应工作区后执行。
- 不要在不清楚边界时使用 `git add -A`，应显式指定待提交路径。

## 构建入口

三个工作区分别构建，不要从仓库顶层运行 `colcon build`：

```bash
cd robot_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

cd ../omni_lattice_benchmark_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

`mobile_base_ws` 的 Jetson/Humble 构建和启动方式以该工作区自己的 `README.md` 为准。
