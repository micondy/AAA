# Robot research workspace

ROS 2 Jazzy primary research workspace. Scope searches to the relevant package under `src/robot_research/`.

## Package routing

- `sensing/d435i_bringup`: RealSense launch and health
- `localization/rtabmap_bringup`: odometry, mapping, localization
- `navigation/mobile_navigation_bringup`: Nav2 runtime and safety chain
- `mapping/nvblox_bringup`, `mapping/gaussian_splatting_bringup`: mapping research
- `perception/*`: rack and target perception
- `planning/active_perception`: belief, NBV, execution
- `integration/*`: combined robot description, bringup, MoveIt
- `evaluation/experiment_tools`: recording and benchmarks
- `tools/isaac_sim`: simulator entry points

Do not inspect `src/vendor`, data/model/result directories, or generated workspaces unless the request names them. For runtime truth, prefer launch/config/source/tests over dated plans.

Build and test narrowly:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <package>
colcon test --packages-select <package>
```

This directory has its own Git history with unrelated local changes; preserve them.
