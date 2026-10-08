# Vendored upstream packages

This directory contains clean, unmodified source snapshots from Enactic. Nothing is copied from
`/home/scc/ros2_ws` or any personal OpenArm fork.

`openarm_description/`:

```text
repository: https://github.com/enactic/openarm_description.git
tag:        1.0.4
commit:     5db5232d4bbf7396222437a568c625176bac1139
license:    Apache-2.0
```

`openarm_ros2/`:

```text
repository: https://github.com/enactic/openarm_ros2.git
tag:        0.9.2
commit:     73ef89838763496b94da30ede38fc92218bea18e
license:    Apache-2.0
contains:   openarm, openarm_hardware, openarm_bringup,
            openarm_bimanual_moveit_config
```

`openarm_can/`:

```text
repository: https://github.com/enactic/openarm_can.git
tag:        1.3.4
commit:     a30364622ca939b8c8a741167c317b3e95e38a49
license:    Apache-2.0
```

These snapshots are built in this workspace so every OpenArm ROS package resolves to an official,
pinned source. Do not add robot-specific edits inside them. Put the chassis mounting model in
`mobile_manipulator_description`, and put combined-robot planning changes in
`mobile_manipulator_moveit_config`.
