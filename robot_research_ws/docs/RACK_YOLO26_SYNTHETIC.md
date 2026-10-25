# Isaac 合成标注与 YOLO26n-seg：光照修正记录

日期：2026-10-20。目标是用 Isaac 的语义标签生成 `bed`/`post` 实例分割训练集，再以独立视觉模型替换在线仿真标签。它**不是**实体相机验收。

## 已发现的问题

第一次采集的 `datasets/rack_yolo_synthetic_20261020` 使用 4×4 层架和网格地板，但没有棚架专用环境光。首张训练图的灰度中位数仅 12/255，约 50% 像素灰度低于 12，画面上半部是黑色。基于它训练的 `models/rack_yolo26n_sim_20261020.pt` 与 `results/rack/yolo26_synthetic_validation_20261020.json` **仅作失败对照，不应部署**；即使 ROS 在线冒烟能发出掩膜，也不代表视觉模型可用。

## 修正

`tools/isaac_sim/run_mobile_manipulator.py` 在添加棚架时创建固定的 DomeLight 和 DistantLight，保证无人值守 RGB 渲染有光，黑色背景变为可见的浅灰天光。光照只改变渲染，不改变层架几何、语义标签或导航算法。`tools/record_rack_yolo_dataset.py` 新增亮度门槛：灰度中位数至少 35/255、灰度小于 12 的像素占比至多 20%；不满足则跳过并在 manifest 中记录拒收数量。可用 `--minimum-median-luma` 和 `--maximum-black-fraction` 调整。

少量 QA 样张保存在 `datasets/rack_yolo_light_qa_20261020/images/train`。人工检查第 3 张：层面、立柱及地面均可辨；其灰度中位数 196/255，黑像素占比 0。亮图训练/验证集独立放在 `datasets/rack_yolo_synthetic_lit_20261020`，没有覆盖暗图和旧权重。两组各 160 张；全组灰度中位数最低分别为 193、192/255，最大近黑像素比例分别约 0.010%、0.011%。训练集有 2403 个 `bed`、5713 个 `post` 标签；验证集有 2372 个 `bed`、5570 个 `post` 标签。

## 运行方式

先启动仿真，再同时运行数据记录器和现有底盘巡检脚本；训练、验证时关闭 Isaac 以避免显存争用。训练和验证要使用不同 `rack_visual_seed`，但独立的相机轨迹/场景也很重要，不能把连续邻帧随机分割成训练/验证。

```bash
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack.launch.py \
  headless:=true test_seconds:=165 rack_visual_seed:=1

# 另一个终端
source tools/source_robot_stack.bash
python3 tools/record_rack_yolo_dataset.py \
  --output datasets/rack_yolo_synthetic_lit_20261020 \
  --split train --visual-seed 1 --max-frames 160 --duration-s 150

# 可并行驱动相机经过层架；它不是导航算法修改
python3 tools/run_rack_grid_acceptance.py --duration 140 \
  --output results/rack/yolo_lit_train_drive_20261020.json --anonymous
```

验证集用 `--split val --visual-seed 2` 并启动 `rack_visual_seed:=2` 的新仿真进程。训练入口是 `tools/train_rack_yolo26.py`，评估入口是 `tools/evaluate_rack_yolo26.py`。

## 本次训练与结果

使用官方 `yolo26n-seg.pt` 迁移训练 30 轮，640 像素、batch 4，权重复制为 `models/rack_yolo26n_sim_lit_20261020.pt`（SHA-256 `3c083f5be2f9dd559c307cef18c7933bf7016f4077057a07e6de3d9a7e321f6a`）。独立评估结果在 `results/rack/yolo26_synthetic_lit_validation_20261020.json`，预测图在 `results/rack/yolo26_validation_plots/holdout-2/`。

| 验证集分割指标 | 暗图旧版 | 亮图新版 |
|---|---:|---:|
| 床面 mAP50 | 0.652 | 0.681 |
| 床面召回率 | 0.608 | 0.658 |
| 立柱 mAP50 | 0.258 | 0.258 |
| 立柱召回率 | 0.282 | 0.305 |

结论：**光照/图像质量缺陷已修正，立柱分割质量仍不足**。立柱细、远处像素少，当前可视化有漏检与大量重叠预测。训练/验证仍共享同一种简化架体和相近巡检路径，指标不能作为实机泛化证据；真实棚架需另采集标注并实测。旧暗图权重不得用于演示或部署。此修正没有改动 Nav2、底盘控制或地图算法。

使用亮图权重做 40 秒 ROS 在线冒烟，记录在 `results/rack/yolo26_lit_online_smoke_20261020.json`：收到 175 帧实例掩膜（测得约 4.37 Hz）、163 帧几何和 162 帧地图，链路通过。这个检查只证明节点能互通；观察中最多出现 2 架、6 层，**没有**完成 4×4 全场景稳定 ID 或导航验收。相关 rack_perception 测试 22 项通过。

## 网页可视化与短距离实测

`sim_rack_navigation.launch.py` 现在可透传 `use_yolo`、权重路径及 Python 路径到层架仿真。不改动 Nav2 算法。一次启动 Isaac 图形窗口、网页、在线 RTAB-Map 和 YOLO；Nvblox 默认不启动，需要对照点图时显式加 `enable_nvblox:=true`：

```bash
source tools/source_robot_stack.bash
ros2 launch mobile_manipulator_bringup sim_rack_navigation.launch.py \
  headless:=false anonymous_instances:=true use_yolo:=true \
  yolo_weights_path:=/media/scc/Windows_D/AAA/robot_research_ws/models/rack_yolo26n_sim_lit_20261020.pt \
  yolo_python_executable:=/media/scc/Windows_D/AAA/robot_research_ws/.venvs/rack_yolo/bin/python
```

本机打开 `http://127.0.0.1:8080/#visualization`。网页「地图与 3D」页现在增加 RGB 与实例分割两幅实时画面，分别来自 `/sensors/nav_camera/color/image_raw` 和 `/rack_detector/debug/color_mask`；图像接口是 `/api/camera/rgb.jpg`、`/api/camera/mask.jpg`。若不在本机，使用机器人电脑的局域网 IP。Isaac 窗口显示全局场景；网页显示机器人相机实际输入、YOLO 输出、在线地图和稳定 ID。当前网页未设置访问令牌，只能在可信局域网使用，不能映射到公网。

2026-10-20 单架移动试验：`results/rack/yolo_lit_single_motion_acceptance_20261020.json` 记录 159 帧三层模型，稳定 ID 始终为 `rack_0001` + `layer_0001`～`0003`，无锁定后无效帧，对齐误差 376 次更新；距离误差变化 0.0479 m，略低于现有验收阈值 0.05 m，因此该脚本**判定未通过**。4×4 网页联动中，网页控制机器人前进约 0.347 m 后停车，RGB/掩膜、在线二维地图和 Nvblox 三维地图继续更新；网页当时观察到 4 架、12 层。前进后的 RGB、分割快照分别保存为 `results/rack/yolo_lit_web_rgb_after_20261020.jpg`、`results/rack/yolo_lit_web_mask_after_20261020.jpg`。随后 20 秒在线观察 `results/rack/yolo_lit_web_live_observer_20261020.json` 收到 89 帧掩膜、88 帧几何和 85 帧地图，最大 4 架、12 层。此短测不是 16 架完整巡航验收，也不能说明所有架号无跳变。短测后速度归零、网页模式复位为 `DISABLED`；要人工控制需在网页手动切换 NORMAL。

查看原始 RGB 样张：`datasets/rack_yolo_synthetic_lit_20261020/images/train/frame_000000.png`。查看标签：同名 `labels/train/frame_000000.txt`。查看模型预测叠图：`results/rack/yolo26_validation_plots/holdout-2/val_batch0_pred.jpg`。亮图数据的样张不是实体相机照片。

## 2026-10-21：网页语义三维层架

网页「地图与 3D」的默认视图现改为「层架模型」。数据链路为 YOLO 实例分割 → 深度反投影/床面拟合 → `rack_map` 稳定 ID → TF 转至 odom → `/api/racks` 的 `scene_racks` → 网页绘制床面、前沿、编号和底盘。绿色表示当前观测，蓝灰表示遮挡后保留；四角虚线立柱仅由床面角点和层高推测，**不是立柱的直接测量结果**。该模型是已观测层架的结构化视图，未观测区域不凭空补齐，也不是完整的 TSDF/ESDF 或可用于碰撞判断的占据地图。二维在线地图和 Nav2 仍用既有 RTAB-Map/代价地图链路；本次没有修改导航算法。Nvblox 点图保留为可选按钮，用于与几何视图对照，但不再是默认显示或默认启动项。

本机持续运行命令即上方 `headless:=false` 命令，网页地址 `http://127.0.0.1:8080/#visualization`。`headless:=true` 且 `test_seconds:=0` 的 Isaac 脚本会按 10 秒短测自动结束；无界面短测需显式指定 `test_seconds`。2026-10-21 实测：Nvblox 关闭时，网页接口从初始 2 架、6 层更新到 4 架、12 层，含四角床面边界、前沿、稳定 ID，RGB/分割接口均为 HTTP 200，里程计在线。网页包全部 26 项测试、层架感知包 22 项测试及 JavaScript 语法检查通过。尚未验证 16 架全场景稳定性；当前环境无可连接浏览器，故尚未完成截图级画面验收。

### 层架在地图上倾斜的修正

4×4 仿真架子的真实长轴沿世界 Y 轴（90°），但 RGB-D 只看到部分床面时，逐帧 PCA 的角度曾偏到约 57°–126°；`rack_map` 平滑这些偏差后，网页会把床面画成斜四边形。这里不是 Isaac 架体旋转，也不是 Nvblox 点图造成的。`fit_layer()` 现支持可选 `axis_prior_yaw_rad`；仿真配置 `rack_sim.yaml` 明确开启 90° 平行层架先验，仍使用分割与深度估计中心、高度、可见范围及前沿，没有借用 Isaac 的架号。未知朝向的实机场景保持默认关闭先验，需要从真实多视角观测估计。网页二维地图同时增加了床面轮廓，便于与三维模型和占据地图核对；导航代价地图及 Nav2 参数未修改。重启后的在线接口初始 2 架、6 层全部为 90.0°，床面长边 X 位移为 0。此约束只修正朝向，不保证部分遮挡时的床面尺寸/中心无误，也不表示已经完成全场景建图。

### 重新运行后的地图质量检查

2026-10-21 重新启动图形仿真、YOLO、RTAB-Map、Nav2 和网页，数据库为 `results/rack/rack_semantic_rerun_20261021.db`。Nav2 将底盘从原点移动到约 `(1.73, 0.02)` m，目标 `(1.8, 0.0)` m，状态 `SUCCEEDED`，末速度为零。占据地图从 120×203 扩至 168×207（0.05 m/格），但该包围盒内仅约 25.1% 栅格已知，不能视作整棚完成。层架模型从 2 架、6 层扩为 4 架、12 层，角度均为 90°；相对仿真真值，近排两架中心误差约 0.228、0.263 m，远排两架约 0.028、0.033 m。可见稳定 ID 为 `rack_0001`、`rack_0002`、`rack_0006`、`rack_0007`，编号跳号提示中途曾分配临时轨迹 ID，不能据此宣称跨帧关联已完全可靠。这次只是约 1.7 m 的短距离在线检查，未覆盖 16 架，也未做全程 ID 跳变统计；目前的地图只适合感知/建图调试，不应直接作为精确采摘位姿依据。

### 三维视图的空间基准

此前网页三维视图使用装饰性网格，网格并不对应地图坐标；鼠标旋转/缩放后无法判断 X/Y 方向、原点或米制距离。现在网格按 `scene_frame` 的世界坐标绘制，标出 `(0,0)`、X/Y 方向、局部米制刻度、视图中心坐标和坐标系名称。Nvblox 关闭且 SLAM 地图可用时，语义三维视图直接使用 `map` 坐标系，与左侧二维地图同基准；地图尚未出现时退回 `odom`，Nvblox 开启时与其 Mesh 使用同一坐标系。增加「正北俯视」（X 向右、Y 向上，拖动平移）、「跟随底盘」和「全图复位」，斜视仍可拖动旋转和滚轮缩放。正北俯视仅显示每架的最上层轮廓，避免三层重叠标签；切回斜视可见分层结构。这些是显示和相机控制改动，不改变 Nav2 或层架跟踪算法，也不会把未知区域变成已建图区域。网页测试和脚本语法检查通过；当前环境的浏览器连接不可用，未做截图级 UI 验收。

随后将坐标刻度统一改为厘米：俯视图沿 X/Y 显示世界坐标数值和刻线；斜视图在地面两条参考线上直接标出 `X 250 cm`、`Y 150 cm` 等坐标，并保留 `Z 高度 / cm` 尺。撤掉右下角抽象的 XYZ 小方向标。网格随缩放在 200、100、50、25、10 cm 间自动切换；坐标数值的文字间隔自动稀疏化，避免缩小时重叠。视图中心坐标和比例尺也均以 cm 显示；透视斜视的比例尺标注“中心处”，因为像素/厘米随深度变化，需要精确量距时应切到正北俯视。该改动只影响网页绘制，不改变 ROS 地图、导航或层架跟踪算法。

二维 SLAM 图也改用同样的厘米标尺和坐标基准。地图可用时，地图中心和车体取 `map_pose`，仅叠加 `map` 坐标下的层架与轨迹；`map_pose` 临时不可用时不把 `odom` 车体硬叠到 `map` 栅格上。地图与里程计来源切换时清空旧轨迹，避免不同坐标系连成一条线。顶部/左侧分别显示 X/Y 世界坐标，网格随缩放在 200、100、50、25、10 cm 间切换；地图可放大到 62.5 cm 视野，此时原始占用栅格的单元边界清晰可见。地图状态列出 `map`/`odom`、中心坐标、网格间隔和原始栅格分辨率；鼠标移到地图上显示坐标、可通行/障碍/未知分类以及占用百分比。这里展示的是原始 SLAM 栅格细节，并未凭空提高地图真实分辨率或改变导航算法。

验证：网页包 29 项测试通过；脚本语法、`map`/`odom` 位姿选择、栅格值读取和二维画布厘米刻度均通过运行时检查；在线网页已提供新版脚本。当前内置浏览器未连接，因此尚未完成截图级的排版验收。
