# mobile_base_bringup

Mobile Base 的顶层配置、Xacro、launch、RViz 和现场测试工具。本包只负责组合，不实现
CAN 驱动或四轮运动学。

完整安装、配置、标定、网页和实机流程统一维护在工程根目录
[`README.md`](../../README.md)，当前完成情况见
[`docs/IMPLEMENTATION_STATUS.md`](../../docs/IMPLEMENTATION_STATUS.md)。

```bash
ros2 launch mobile_base_bringup mock.launch.py
ros2 launch mobile_base_bringup chassis.launch.py
ros2 launch mobile_base_bringup robot.launch.py
ros2 launch mobile_base_bringup operator.launch.py
ros2 run mobile_base_bringup configure_damiao_mode.py read
ros2 run mobile_base_bringup calibrate_damiao_steering.py read
ros2 run mobile_base_bringup test_zlac8015d.py --interface can0 --node 5 status
ros2 run mobile_base_bringup test_zlac8015d.py --interface can0 --node 6 status
```
