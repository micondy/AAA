# 双主机部署

底盘机和科研机使用相同 ROS 2 Jazzy 版本、相同 `ROS_DOMAIN_ID`，并保证局域网 DDS 组播或已配置的 DDS discovery 可达。

底盘机：

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_base_bringup robot.launch.py
```

科研机：

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch d435i_bringup camera.launch.py stage:=full
```

同一台电脑同时运行时分别在两个终端 source 对应工作区即可。需要在科研节点中使用底盘消息/接口时，先 source `mobile_base_ws/install/setup.bash`，再 source `robot_research_ws/install/setup.bash`。目前没有统一整机 launch；固定传感器并确定 TF 后再创建独立集成包。
