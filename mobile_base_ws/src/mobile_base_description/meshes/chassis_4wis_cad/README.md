# chassis_4wis CAD meshes

来源：`/media/scc/Windows_D/AAA/chassis_4wis/meshes`，复制日期 2026-09-12。

这些二进制 STL 保持原始顶点不变。坐标重心、轴方向及零件偏置由
`urdf/mobile_base.urdf.xacro` 的 visual origin 修正。它们不用于 collision。

原始 SolidWorks URDF 有两个不能直接用于机器人控制的问题：

1. CAD 坐标不是 ROS REP-103 坐标，零件 mesh 还包含较大的局部偏置；
2. 左前轮关节导出异常，零位下轮心比其余三轮前移约 0.23 m。

因此本包保留 CAD 外观，但重新定义标准化关节树和简化碰撞体。
