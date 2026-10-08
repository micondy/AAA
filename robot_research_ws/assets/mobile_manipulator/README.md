# Mobile manipulator assets

本目录是自研四轮四转底盘与官方 OpenArm 双臂的整体仿真资源：

```text
mobile_manipulator_official.usda       # 唯一的 Isaac Sim 组合入口
mobile_manipulator_official.urdf       # ROS 2 统一树的展开快照
openarm_bimanual_official/             # NVIDIA 官方 OpenArm USD，保持原样
OFFICIAL_OPENARM_SOURCE.md             # 来源、commit 和 SHA-256
../mobile_base/mobile_base_sim/        # 自研底盘 USD
```

组合层增加一块 250 × 250 × 40 mm 临时安装板，并将 OpenArm body 固定在底盘
`base_link` 上方 0.22 m。该尺寸只用于首轮仿真，实体支架完成后必须替换为测量值。

启动可视化仿真：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py
```

执行自动稳定性验收：

```bash
/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh \
  tools/isaac_sim/run_mobile_manipulator.py --headless --test-seconds 10
```

不要直接编辑 `openarm_bimanual_official/`。安装关系、相机和场景扩展应写在组合层或启动脚本中。
