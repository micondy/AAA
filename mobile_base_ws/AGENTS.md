# Mobile base workspace

ROS 2 Humble deployment workspace for the Jetson and physical 4WIS base.

- `mobile_base_driver`: hardware/CAN interfaces
- `mobile_base_swerve_controller`: chassis kinematics and control
- `mobile_base_description`: URDF and meshes
- `mobile_base_bringup`: launch, chassis configuration, deployment helpers
- `mobile_base_web_console`: manual control UI

Use package-local README, config, source, and tests. Skip `build/`, `install/`, `log/`, `tmp/`, `.vscode/`, and PDFs unless explicitly requested. Do not mix Jazzy assumptions from `robot_research_ws` into this Humble workspace.

Build/test only the touched package and its dependencies. This directory has its own Git history and existing local changes; do not stage or revert unrelated files.
