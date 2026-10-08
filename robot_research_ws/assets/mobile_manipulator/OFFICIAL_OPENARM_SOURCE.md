# Official OpenArm asset provenance

本目录中的 `openarm_bimanual_official/` 是从本机 Isaac Sim 官方机器人资产副本完整复制而来，原始路径为：

```text
/home/scc/a1/OpenArm/openarm_bimanual/
```

NVIDIA Isaac Sim 6.0 的机器人资产目录将其列为 `OpenArm/openarm_bimanual/openarm_bimanual.usd`，许可证为 Apache-2.0。项目没有修改该目录内的 USD 文件；移动底盘安装关系只写在相邻的 `mobile_manipulator_official.usda` 组合层中。

主要文件 SHA-256：

```text
a5b18e8292d70b7d7767c49137c7f4bc9eea2d697466be7d34bbb335f2b3478e  openarm_bimanual.usd
55d129950a453f859f1e795ab06b2c2d810f71e66051ca6d4f57ef0f0e887532  configuration/openarm_bimanual_base.usd
33f73a2426e232a514aad09139da527a4c645822a4fd59c851725567377fad94  configuration/openarm_bimanual_physics.usd
be7bed93ad1d4950223434004ffba28637f3bafaa47a746cafd4fd71ab60a8f0  configuration/openarm_bimanual_sensor.usd
```

ROS 2 侧另外固定使用 Enactic 官方 `openarm_description` tag `1.0.4`、commit `5db5232d4bbf7396222437a568c625176bac1139`，位于：

```text
robot_research_ws/src/vendor/openarm_description/
```

ROS 侧同样只使用 `robot_research_ws/src/vendor` 中固定版本的 Enactic 官方源码；工程不
source、引用或复制任何个人 OpenArm 工作区。
