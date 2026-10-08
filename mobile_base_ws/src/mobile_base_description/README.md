# mobile_base_description

纯四轮四转底盘模型和 `ros2_control` 硬件参数入口，不包含 D435i 或其他上装传感器。

## 当前模型

- ROS 运动学使用重新建立的 REP-103 坐标：`base_link` 的 X 向前、Y 向左、Z 向上；
- 转向关节绕 Z，行走关节绕 Y，不继承 SolidWorks 导出文件中的异常关节轴；
- 默认轴距 `0.419 m`、转向轴轮距 `0.450 m`、轮胎半径 `0.0855 m`；这些是 CAD
  反算初值，实车落地前必须复测和标定；
- `chassis_4wis` 网格只用于 visual；collision 使用 box/cylinder，避免 MoveIt 与 Isaac
  对约 38 万个 STL 三角面做实时碰撞；
- CAD 中车轮中心相对转向轴向外约 `0.043 m`；原始左前轮关节导出损坏，因此左前视觉
  复用同型左后轮网格，不使用错误的关节变换。

网格来源和处理原则见 `meshes/chassis_4wis_cad/README.md`。原始目录
`/media/scc/Windows_D/AAA/chassis_4wis` 保持不变，便于日后重新导出和核对。

```bash
ros2 launch mobile_base_description description.launch.py
```

只看简化几何可运行：

```bash
ros2 launch mobile_base_description joint_preview.launch.py use_cad_visuals:=false
```

传感器固定到底盘后，其安装外参由科研/整机集成工作区维护。
