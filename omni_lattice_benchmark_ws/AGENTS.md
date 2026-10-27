# Omni lattice benchmark workspace

Standalone ROS 2 Jazzy experiment workspace. It is not a runtime dependency of the robot.

- Source package: `src/omni_lattice_benchmark/`
- Planner matrix/scenarios: `src/omni_lattice_benchmark/config/`
- Motion primitives: `src/omni_lattice_benchmark/lattice/`
- Synthetic maps: `src/omni_lattice_benchmark/maps/`
- Historical output: `results/navigation/omni_lattice/`

Default work should ignore historical results and generated `build/install/log`. Inspect results only for an explicit analysis request.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test
```
