# mobile_base_ws

> 整个移动操作机器人（底盘、双臂、感知、建图、导航与网页）的统一结构见
> [架构导航](../robot_research_ws/docs/architecture/README.md)；逐包职责见
> [ROS 包目录](../robot_research_ws/docs/architecture/PACKAGE_CATALOG.md)。

按钮与摇杆采用统一的 ±90°有界舵轮规划。横移目标落在等价的 ±90°边界且转向路程近似相同时，会按实物辨向结果选择四轮向轴距外侧展开的姿态：左前 -90°、右前 +90°、左后 +90°、右后 -90°；若一侧路径明显更短，仍优先选择最近路径。
停车换向以实际轮速反馈为依据；具体操作及限制见操作手册。

> 每次上电、故障检查和实车测试的可复制命令请直接查看：
> [启动与实车操作.md](启动与实车操作.md)

ROS 2 Jazzy 四轮四转底盘工作区。这里只维护底盘硬件、控制器、模型、配置、网页调试和启动；

## 有效 ROS 2 包

- `mobile_base_driver`：达妙 DM4340P 转向电机、ZLAC8015D 行走驱动器及 SocketCAN 硬件接口。
- `mobile_base_swerve_controller`：四轮四转运动学、速度命令和轮速里程计。
- `mobile_base_bringup`：统一 YAML、实车/mock 启动、标定和硬件测试工具。
- `mobile_base_web_console`：局域网页配置、诊断和单轮维护。
- `mobile_base_description`：不含传感器的纯底盘 Xacro/URDF。

## 当前实机配置

| 用途 | 接口 | 参数 | 设备映射 |
|---|---|---|---|
| 行走 | `can0` | 经典 CAN 500 kbps | ZLAC 节点 5=前轴、节点 6=后轴；通道 1=左轮、通道 2=右轮 |
| 转向 | `can1` | 经典 CAN 1 Mbps | 左前=`02/12`、右前=`01/11`、左后=`04/14`、右后=`03/13` |

2026-08-31 已将四轮当时的机械朝向标为软件零位，并设置转向相对零位左右各 90°、总范围 180°。
轮子最终正反方向和安全速度仍需完成低速验收。包内 [`chassis.yaml`](src/mobile_base_bringup/config/chassis.yaml) 是首次启动模板。
网页首次保存后，运行配置位于 `~/.config/mobile_base/chassis.yaml`，后续实车启动会自动优先读取它；
也可用 `MOBILE_BASE_CONFIG_DIR` 指定可移动部署目录。

## 构建

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## CAN 与启动

推荐每次上电直接运行工作空间根目录的一键脚本。它会配置两路 CAN，验证四台达妙
`pos_vel` 模式和中菱节点 5/6；任何设备超时都会中止，不会带故障启动整车：

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
./start_robot.sh
```

如需网页显示实时 SLAM 地图，底盘启动后在第二个终端运行：

```bash
cd /media/scc/Windows_D/AAA/mobile_base_ws
./start_online_slam.sh
```

完整的启动、停止、在线 SLAM 和关机步骤见[启动与实车操作](启动与实车操作.md)。

先确认 Linux 枚举出的 `can0/can1` 与上表一致，再配置接口：

```bash
ros2 run mobile_base_bringup setup_can.sh
ip -details -statistics link show can0
ip -details -statistics link show can1
```

启动实车底盘和网页后端：

```bash
ros2 launch mobile_base_bringup robot.launch.py
```

浏览器访问 `http://底盘电脑IP:8080`。网页不再使用多人控制租约：正常模式点击一次启用整车控制，再用鼠标操作；再次点击、立即停车、页面失焦或命令超时会停车。网页用于实验调试，不是实体急停，不能暴露到公网。

### 网页整车控制

实车启动命令默认同时启动网页后端（`start_web_console:=true`）。网页显示“服务在线”后：

1. 检查 `can0`、`can1`、八个关节反馈和安全诊断。
2. 点击“正常整车模式”，确认状态变为 `NORMAL`。
3. 拖动圆形摇杆设置前后速度 `vx` 和横移速度 `vy`。
4. 调整“旋转速度”滑条设置角速度 `wz`。
5. 点击“启用整车控制”后，可松开鼠标操作摇杆和旋转滑条；再次点击或按“立即停车”归零并停止发送。

页面失焦、切换标签页、关闭页面或网络中断也会停止发送，底层控制器超过 0.25 秒收不到新命令会自动归零。

网页摇杆和旋转滑条可以同时使用，所以当前四轮四转控制器支持：

| 动作 | 网页输入 | 控制器行为 |
|---|---|---|
| 前进/后退 | 摇杆前/后，`wz=0` | 四轮趋向车身前后方向 |
| 左右横移（蟹行） | 摇杆左/右，`wz=0` | 四轮趋向横向 |
| 任意斜移 | 摇杆拖到任意方向 | 四轮平行指向目标平移方向 |
| 原地自转 | 摇杆居中，仅设置 `wz` | 四轮沿各自绕车体中心的切线布置 |
| 弧线转弯 | 设置 `vx` 或 `vy`，同时设置 `wz` | 每轮按自身位置计算角度和轮速 |
| 边走边转 | 同时改变摇杆和旋转滑条 | 合成完整全向底盘速度 |

这里“原地掉头”就是 `vx=0、vy=0、wz≠0` 的原地自转。网页提供前进、后退、左右横移、左右前斜行、左右原地旋转和全部归零预设，也可以通过摇杆与旋转滑条组合产生任意低速命令。

> 软件运动学和 mock 控制链已经实现；四轮整车联动尚未实机验收。首次测试必须架空四轮、降低网页速度上限，并先完成四个转向零位和行走方向确认。若轮子朝向不正确，不能直接落地测试原地自转。

四轮运动学遇到目标方向位于机械后半平面时，会把转向目标折叠 180°并反转对应行走轮速度，保证转向目标始终位于 `-90°～+90°`。维护模式会将越过绝对限位的目标自动修正到最近限位，并显示修正结果。

控制器只在机械限位内选择最近的等价轮向，禁止对实际角取模绕过限位。大幅换向先下发零轮速并保持轮角，四轮速度均低于 0.05 rad/s 持续 0.15 秒后才开始转向，对准后按加速度限制恢复行走。转向目标速率取四轮配置速度中的最小值（当前 0.15 rad/s）。

网页的“整车速度倍率”独立控制动作速度；按钮和摇杆都发送 vx/vy/wz。横移到达 ±90°边界且两条等价路径的代价相差不超过 5°时，前轴选择左前 -90°、右前 +90°，后轴选择左后 +90°、右后 -90°，使带 43 mm 偏置的四个轮心都向轴距外侧展开；路径差异更大时最近路径仍然优先。不同轮的角度和轮速符号可以不同，但实际滚动速度矢量必须一致。反向横移优先保持轮角并反转轮速。

整车控制增加了 5° 转向对准门限：收到横移、斜行或原地旋转命令后，四个转向先自动到达运动学目标；任一参与运动的轮子误差超过 5° 时，四个行走轮保持零速，对准后再统一放行。网页提供前进、横移、斜行、原地旋转和归零预设，预设只设置目标，不自动启用运动。

单轮维护支持两种输入：`绝对实际目标角` 直接输入相对标定零位的目标角，`相对当前角` 在按下瞬间的反馈角上叠加。两种方式超出 ±90° 时都会自动修正到最近限位并在页面提示，不再以弹窗中断。四台转向默认命令速度已从 0.5 降为 0.15 rad/s。

不接硬件的控制链验证：

```bash
ros2 launch mobile_base_bringup mock.launch.py
```

## 独立硬件检查

检查两个 ZLAC 节点：

```bash
ros2 run mobile_base_bringup test_zlac8015d.py --interface can0 --node 5 status
ros2 run mobile_base_bringup test_zlac8015d.py --interface can0 --node 6 status
```

读取达妙模式和绝对角度：

```bash
ros2 run mobile_base_bringup configure_damiao_mode.py --config src/mobile_base_bringup/config/chassis.yaml --interface can1 read
ros2 run mobile_base_bringup calibrate_damiao_steering.py --config src/mobile_base_bringup/config/chassis.yaml --interface can1 read
```

会让电机运动的命令不集中复制到总 README；执行前请阅读脚本 `--help`，架空对应轮组并准备断电。

## 测试与 Git

```bash
colcon test --packages-select \
  mobile_base_description mobile_base_driver mobile_base_swerve_controller \
  mobile_base_web_console mobile_base_bringup
colcon test-result --verbose

git status
git log --oneline --decorate -5
```

仓库使用 `main` 分支，`build/`、`install/`、`log/`、rosbag 和本地历史副本不会进入 Git。

## 相关文档

- [实施状态与未完成项](docs/IMPLEMENTATION_STATUS.md)
- [双主机部署](docs/TWO_HOST_DEPLOYMENT.md)
- [底层协议说明](src/mobile_base_bringup/docs/protocol_notes.md)
- [科研工作区说明](../robot_research_ws/README.md)
- [移动机器人项目当前进展](../移动机器人项目当前进展.md)
