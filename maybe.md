| 方向 | 需要解决的问题 | 可能形成的创新 |
|---|---|---|
| 1. 自适应运动模式 | 狭窄空间如何高效通过 | 四轮四转运动模式自动切换 |
| 2. 智能路径规划 | 如何规划安全、高效的路线 | 考虑转向代价、空间约束的规划算法 |
| 3. 视觉语义建图 | 如何理解棚架、通道、障碍物 | 结构化语义地图 |
| 4. 自主识别与编号 | 如何知道第几排、第几架、第几层 | 基于视觉的实例关联与自动编号 |
| 5. 主动感知 | 遮挡、识别不准确怎么办 | 机器人主动调整观察位置 |
| 6. 自主任务决策 | 如何知道下一步该去哪里 | 感知、导航与任务调度联合优化 |




| 算法 | 类型 | 主要特点 |
|---|---|---|
| Dijkstra | 全局规划 | 最短路径搜索，通常搜索范围较大 |
| A* | 全局规划 | 引入启发函数，通常比Dijkstra更高效 |
| Hybrid A* | 全局规划 | 考虑车体朝向和运动学约束 |
| RRT* | 路径规划 | 适合连续空间和复杂约束 |
| DWA | 局部规划 | 采样速度，考虑避障和运动约束 |
| TEB | 局部轨迹优化 | 同时优化轨迹空间形状和时间 |
| MPPI | 局部控制 | 预测多种控制序列并优化代价 |

| 模块 | 初期方案 | 后续研究 |
|---|---|---|
| 全局路径规划 | Smac 2D A* | Smac State Lattice |
| 局部控制 | MPPI | 自定义代价函数 |
| 避障 | Nav2 Costmap | 结合3D语义信息 |
| 底盘控制 | 四轮逆运动学 | 自适应运动模式 |




1.当前环境，草菇层架。



我筛选出了 **10篇与你的四轮四转机器人直接相关的论文**，涵盖2015—2026年。

重点按照你的实际硬件条件筛选：

- 四轮独立驱动、独立转向（4WID-4WIS / 4WIDS）。
- 转向角度限制：-90°～+90°。
- 考虑转向速度、驱动速度、加速度等物理约束。
- 研究路径规划、轨迹优化、MPC、MPPI控制。
- 最终应用于温室狭窄空间导航。

## 一、最值得你阅读的6篇论文

**1\. 有限转向角下的运动规划与控制** (最相关)

2025 · IEEE ICAR · Liu 等

Motion Planning and Control of an Overactuated 4-Wheel Drive with Constrained Independent Steering

研究无法连续360°转向的四轮独立转向机器人，处理转向不连续、路径跟踪、避障以及停车重置车轮方向的问题。

[论文全文](https://arxiv.org/abs/2510.19054) · [会议版本](https://doi.org/10.1109/ICAR65334.2025.11338621)

与你的±90°底盘限制最接近。[arxiv.org](https://arxiv.org/abs/2510.19054)[dblp.dagstuhl.de](https://dblp.dagstuhl.de/rec/conf/icar/LiuHGA25.html)

---

**2\. 四轮四转的MPPI导航算法**

2024 · IEEE IROS · Aoki 等

Switching Sampling Space of Model Predictive Path-Integral Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation

研究四轮四转8个控制输入带来的高维优化问题，通过切换MPPI采样空间兼顾效率与避障安全。

[论文](https://arxiv.org/abs/2409.08648) · [开源代码](https://github.com/MizuhoAOKI/mppi_swerve_drive_ros) · [视频与项目主页](https://mizuhoaoki.com/projects/iros2024.html)

这篇有开源代码，非常适合作为研究基线。[mizuhoaoki.com](https://mizuhoaoki.com/projects/iros2024.html)

---

**3\. 考虑转向角限制和连续性的MPC控制**

2015 · 日本机械学会论文集 · Hagimori 等

Model Predictive Tracking Control for Wheeled Mobile Robots Considering Limitation and Continuity of Steering Angles

研究转向角受限、车轮突然翻转、转向连续性与驱动轮速度限制，使用MPC实现轨迹跟踪。

[论文（可获取PDF）](https://www.jstage.jst.go.jp/article/transjsme/81/825/81_14-00484/_article/-char/en)

研究你提出的角度与运动连续性问题，属于重要的早期工作。[jstage.jst.go.jp](https://www.jstage.jst.go.jp/article/transjsme/81/825/81_14-00484/_article/-char/en)

---

**4\. 具有过约束特性的四轮四转轨迹优化**

2026 · Journal of Systems and Control Engineering · Pang 等

Over-constrained Trajectory Planning of 4WIS-4WID Robots with Model Reconstruction and Iterative Optimal Control

研究四轮转向与驱动变量耦合、轨迹优化、模型重构和安全走廊约束。

[期刊论文](https://doi.org/10.1177/09596518261479625)

2026年9月11日在线发表，部分全文受限。[journals.sagepub.com](https://journals.sagepub.com/doi/10.1177/09596518261479625)

---

**5\. 有转向策略约束的MPC轨迹跟踪**

2022 · Mechatronics · 期刊论文

Trajectory Tracking of Redundantly Actuated Mobile Robot by MPC Velocity Control under Steering Strategy Constraint

针对四轮独立转向机器人的冗余驱动、多运动模式及计算复杂度，提出转向策略约束下的MPC速度控制。

[期刊论文](https://doi.org/10.1016/j.mechatronics.2022.102779)

适合研究四轮运动模式协调和计算效率。[sciencedirect.com](https://www.sciencedirect.com/science/article/pii/S0957415822000289)

---

**6\. 四轮四转运动学、动力学与控制设计**

2015 · The Journal of Engineering · Lee 等

Kinematics, Dynamics and Control Design of 4WIS4WID Mobile Robots

系统推导四轮独立转向和驱动的运动学、动力学模型，讨论非线性控制及滑模控制。

[开放获取论文](https://doi.org/10.1049/joe.2014.0241)

推荐作为建模入门。[ietresearch.onlinelibrary.wiley.com](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/joe.2014.0241)

## 二、另外4篇值得参考的研究

| 年份 / 期刊 | 论文 | 研究重点 |
|--------|--------|--------|
| 2025 · 名古屋大学博士论文 | [Nullspace MPC](https://mizuhoaoki.com/projects/nullspace_mpc.html) | 分层优化任务优先级，狭窄空间连续运动，有开源代码 [mizuhoaoki.com](https://mizuhoaoki.com/projects/nullspace_mpc.html) |
| 2022 · ISA Transactions | [MPC-based High-speed Trajectory Tracking for 4WIS Robot](https://doi.org/10.1016/j.isatra.2021.05.018) | 高速轨迹跟踪与MPC控制 [sciencedirect.com](https://www.sciencedirect.com/science/article/abs/pii/S0019057821002731) |
| 2026 · Robotics and Autonomous Systems | [Unified Motion Control of 4WIS-4WID WMR with Unlimited Steering and Load Transfer Consideration](https://doi.org/10.1016/j.robot.2026.105341) | 无限转向、轮胎滑移和载荷转移，适合对照研究 [sciencedirect.com](https://www.sciencedirect.com/science/article/abs/pii/S092188902600014X) |
| 2026 · International Journal of Robust and Nonlinear Control | [Friction-Aware Force-Realizable Safety Control for Four-Wheel Independently Steered and Driven Mobile Robots](https://doi.org/10.1002/rnc.70772) | 摩擦力、车轮转矩约束与安全控制 [onlinelibrary.wiley.com](https://onlinelibrary.wiley.com/doi/10.1002/rnc.70772?af=R) |

此外，还有一篇2021年的综述：[Towards Autonomous Driving: Review and Perspectives on Configuration and Control of Four-Wheel Independent Drive/Steering Electric Vehicles](https://doi.org/10.3390/act10080184)，可用于系统了解这一领域。[mdpi.com](https://www.mdpi.com/2076-0825/10/8/184)

## 三、针对你的底盘，建议按什么顺序读？

## 1

**2015 · 运动学与动力学建模**

首先弄清楚四轮转向角、驱动速度与底盘运动之间的关系。

---

## 2

**2015 · 有限转向角MPC**

重点理解转向角限制、车轮翻转和速度连续性。

---

## 3

**2025 · ICAR有限转向运动规划**

与你的±90°约束最相关，重点分析还有哪些问题没有解决。

---

## 4

**2024 · IROS MPPI 与 2025 Nullspace MPC**

学习采样优化、实时避障和运动效率优化，研究开源实现。

---

## 5

**2026 · 过约束轨迹优化**

了解较新的方法如何处理四轮驱动和转向耦合问题。

## 四、目前可以判断出的研究空缺

这批文献表明，四轮四转控制已经有相当多的工作，尤其是：

- 转向限位与角度不连续：2015、2025年已有专门研究。
- MPC/MPPI：2022—2025年已有多种方法。
- 四轮过驱动轨迹优化：2026年仍有新论文。

因此，**不能把加入转向角、速度约束本身当作创新**。

对你的温室机器人，我更建议检查一个具体问题：

在 **±90°转向范围、有限转向角速度、狭窄通道和车轮允许正反转**的条件下，现有MPPI或MPC是否会产生不必要的停车、转向切换或低效率轨迹？

如果存在且能够量化，就可以进一步研究连续可执行的轨迹优化方法。

另外，上述 IROS 2024 和 Nullspace MPC 的开源项目主要基于 ROS Noetic，与你目前的 ROS 2 Jazzy 环境不同，适合先研究算法，再考虑移植。

**建议优先精读第1、2、3篇**。它们最能帮你判断有限转向角四轮四转控制还能在哪里形成自己的创新。