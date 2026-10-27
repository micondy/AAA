# Omni State Lattice Benchmark Workspace

独立的 Nav2 全向 State Lattice 规划器实验工作区。这里包含基准地图、运动原语生成器、规划器矩阵、自动测试和历史实验结果；不属于机器人运行工程。

```bash
cd /media/scc/Windows_D/AAA/omni_lattice_benchmark_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
```

实验包位于 `src/omni_lattice_benchmark/`，结果位于 `results/navigation/omni_lattice/`。
