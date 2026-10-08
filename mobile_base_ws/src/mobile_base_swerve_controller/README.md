# mobile_base_swerve_controller

四轮四转运动学控制包，负责 `/cmd_vel` 分解、转向最短路径、命令超时停车和轮速里程计。
它只通过标准 `ros2_control` 接口访问关节，不直接依赖 CAN 驱动实现。

完整安装、配置、标定、网页和实机流程统一维护在工程根目录
[`README.md`](../../README.md)，当前完成情况见
[`docs/IMPLEMENTATION_STATUS.md`](../../docs/IMPLEMENTATION_STATUS.md)。
