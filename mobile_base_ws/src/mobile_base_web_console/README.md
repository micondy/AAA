# mobile_base_web_console

四轮四转底盘的中文局域网配置、诊断、整车控制、单轮维护和转向零位标定网页。

完整使用方法统一维护在工程根目录 [`README.md`](../../README.md)，当前完成情况见
[`docs/IMPLEMENTATION_STATUS.md`](../../docs/IMPLEMENTATION_STATUS.md)。

```bash
export MOBILE_BASE_CONFIG_DIR=/opt/mobile_base_ws/config
export MOBILE_BASE_WEB_TOKEN='替换为足够长的随机字符串'
ros2 launch mobile_base_web_console web_console.launch.py
```

浏览器访问 `http://底盘电脑IP:8080`。配置文件与底盘节点共用同一份 `chassis.yaml`。
顶部导航将状态总览、地图与 3D、手动控制、维护调试和参数配置分成五个界面。
地图页订阅 `web_console.yaml` 中配置的 `topics.map`（默认 `/rtabmap/map`，类型
`nav_msgs/msg/OccupancyGrid`），收到 SLAM 栅格后显示墙体和障碍物；没有地图话题时
自动退化为里程计轨迹网格。按下地图选择目标位置，继续拖动可设置目标朝向，也可在
“目标朝向”输入框中直接填写角度。

三维环境视图订阅 `topics.nvblox_mesh`（默认 `/nvblox_node/mesh`），把增量 Mesh
降采样为适合局域网页面的彩色三维点视图；可拖动旋转、滚轮缩放，并可切换回根据
`/joint_states` 联动的 3D 底盘结构。没有启动 nvblox 或 RGB-D 数据中断时，页面会明确
显示“等待 Nvblox Mesh”，不会把二维代价地图误标为三维环境。
该研究版不使用控制租约：正常模式点击一次启用整车控制，再用鼠标操作摇杆和旋转滑条；
再次点击、立即停车、页面失焦、断连或超时会停车。单轮维护仍必须按住执行。
访问令牌仍可选配。网页不是实体急停，不能暴露到公网。
