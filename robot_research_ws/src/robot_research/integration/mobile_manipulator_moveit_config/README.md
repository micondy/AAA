# mobile_manipulator_moveit_config

MoveIt 2 integration for the custom four-wheel-steering chassis and Enactic's official bimanual
OpenArm. This package is intentionally a thin composition layer:

- the URDF comes from `mobile_manipulator_description`, which mounts the untouched official
  `openarm_description` model on the chassis;
- SRDF groups, named states, end effectors, and the OpenArm collision matrix are included directly
  from the official `openarm_bimanual_moveit_config` package;
- kinematics, MoveIt controller mappings, and ros2_control controller parameters are loaded
  directly from the official packages at launch time;
- the integration package keeps a local `joint_limits.yaml` that preserves the official velocity
  limits and adds conservative acceleration limits required by MoveIt's trajectory time
  parameterization; the vendored official packages remain untouched;
- this first milestone plans only the two arms and grippers. Nav2 remains responsible for base
  motion; whole-body base/arm candidate planning is added later at the task layer.

The mock launch hard-codes `mock_components/GenericSystem` for both the chassis and OpenArm. It does
not load `openarm_hardware` and cannot transmit CAN commands:

```bash
source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py
```

The real-hardware chain remains the official one:

```text
MoveIt 2 -> official trajectory controllers -> official openarm_hardware -> official openarm_can
```

A separate hardware bringup will be added only after CAN interface names, arm zeroing, emergency
stop behavior, and low-speed limits are verified on the assembled robot.
