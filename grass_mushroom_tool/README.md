# 草菇采集台（grass_mushroom_tool）

现场采集用的图形化界面：**拍什么、记什么、文件叫什么**都由界面决定，不会再出现
`IMG_0421.jpg`、`微信图片_2026.jpg` 这种事后无法对应的文件名。

这是一个**完全独立的工具**：

- 不放进 `robot_research_ws`、`mobile_base_ws` 或任何现有工作区，不新增 ROS 包、
  不改动任何现有文件；
- 基础照片采集与测量服务可使用 Python 3 标准库运行；RGB-D 快照需要 ROS 2 与 NumPy，
  数据查看器需要 NumPy/Pillow，视频回放预览还需要支持 H.264 编码的 OpenCV；
- 原始数据保存在 `data/` 与 `bags/`，检查报告保存在 `reports/`；这些目录均不上传 Git。
  需要迁移数据时请单独复制，预览 MP4 不能替代包含精确深度的原始 MCAP。

---

## 30 秒上手

```bash
cd grass_mushroom_tool
./start.sh --open          # 会自动打开浏览器；也可以只跑 python3 app.py
```

- 电脑访问：`http://127.0.0.1:8770`
- 手机访问：启动时终端会打印 `http://<局域网IP>:8770`，手机连同一个 Wi-Fi 直接打开。

第一次进去先到「设置」新建会话：填日期、采集员，菇房按 `名称,排数,层数` 一行一间，例如

```
A,12,3
B,10,3
```

之后所有照片都会自动落到 `data/grass_mushroom_日期/` 里，并按下面的规则命名。

---

## 完整命令行（build / source / 启动）

**先分清两件事**：本工具是纯 Python，**不需要 build**；需要 `build` + `source` 的只有
ROS 包（相机驱动）。如果你要用 D435i 快照或 rosbag 录制，按下面走；只拍照记测量的话
跳过本节。

### 终端 A：构建并启动相机

```bash
# 1) 构建（只有改过包才需要；install/ 已存在可直接跳过）
cd /media/scc/Windows_D/AAA/robot_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select d435i_bringup
# 全量构建用：colcon build --symlink-install

# 2) source（每开一个新终端都要做一次）
source install/setup.bash
# 如果底盘/整机也要一起用，改用：
# source tools/source_robot_stack.bash

# 3) 启动 D435i（stage:=full 才有对齐深度和点云）
ros2 launch d435i_bringup camera.launch.py \
  camera_namespace:=sensors camera_name:=nav_camera \
  serial_no:=346122070466 stage:=full use_rviz:=false
```

### 终端 B：source 同一套环境，再启动采集台

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws
source tools/source_robot_stack.bash        # /opt/ros/jazzy → mobile_base → robot_research

cd /media/scc/Windows_D/AAA/grass_mushroom_tool
python3 app.py --open
```

或者仍用启动脚本，把 overlay 传给它（脚本会先 source 再起服务）：

```bash
cd /media/scc/Windows_D/AAA/grass_mushroom_tool
GM_SETUP=/media/scc/Windows_D/AAA/robot_research_ws/install/setup.bash ./start.sh --open
```

### 一行版（终端 B 用）

```bash
cd /media/scc/Windows_D/AAA/robot_research_ws && source /opt/ros/jazzy/setup.bash && source tools/source_robot_stack.bash && cd /media/scc/Windows_D/AAA/grass_mushroom_tool && python3 app.py --open
```

### 验证环境是否真的生效

```bash
ros2 pkg prefix d435i_bringup                                  # 能打印 install/d435i_bringup 即已 source
ros2 topic hz /sensors/nav_camera/color/image_raw              # 有频率说明相机在出图
```

界面顶栏出现 **「相机 在线 640×480」** 就说明快照可用；显示「相机 无数据」说明相机
没起来或 `--camera-prefix` 不对。

> 录包默认落在本工具的 `bags/`，想存到工作区就在启动时加
> `--bags /media/scc/Windows_D/AAA/robot_research_ws/bags`。

---

## 命名规则

| 内容 | 文件名示例 | 说明 |
| --- | --- | --- |
| 样本原位远照 | `s007-inplace-far.jpg` | 保留位置与周围环境 |
| 样本原位近照 | `s007-inplace-near.jpg` | 草菇清晰完整 |
| 原位左右补充 | `s007-inplace-left-2.jpg` | 密集/遮挡时补拍，重拍自动加序号 |
| 采摘后整簇 | `s007-picked-cluster.jpg` | 成簇样本先拍整簇 |
| 正面 / 反面 / 底部 | `s007-picked-top.jpg`<br>`s007-picked-back.jpg`<br>`s007-picked-bottom.jpg` | 编号牌 + 直尺同框 |
| 簇内个体 | `s007-picked-member-2.jpg` | 分开后逐个拍，可多次 |
| 标尺特写 | `s007-picked-ruler.jpg` | 直尺与编号牌参照 |
| 秤读数 | `s007-picked-weight.jpg` | 电子秤数字清晰 |
| 菇房全景 | `env-A-overview.jpg` | 每间菇房一张 |
| 棚架全排 | `env-A-r02-row.jpg` | 每排一张 |
| 培养床层 | `env-A-r02-l01-bed.jpg` | 每排每层一张 |
| 空床 / 通道 / 现场 | `env-A-r02-l03-empty-bed.jpg` 等 | 可选 |
| D435i 彩色 | `s007-d435i-color.png` | 848×480 RGB |
| D435i 深度 | `s007-d435i-depth.png` | 16 位 PNG，单位毫米，**已对齐到彩色**（与彩色逐像素对应） |
| D435i 点云 | `s007-d435i-points.ply` | 二进制 PLY，彩色光学系，带 RGB，由同帧 RGB-D 生成，默认两个像素方向各步进 2 |
| D435i 参数 | `s007-d435i-meta.json` | 时间戳、对齐方式与 Δt、宽高、彩色/深度内参 K/D、点云帧与点数 |

规则要点：

1. **编号前缀唯一**：`S007` 就代表同一个样本，原位照片、采摘后照片、深度快照、
   CSV 里的那一行，全都靠这个编号对应。
2. **槽位固定顺序**：界面就是按 原位远照 → 原位近照 →（左/右）→ 采摘后整簇 →
   正面 → 反面 → 底部 →（簇内个体）→ 标尺 → 秤读数 排列，不会漏拍。
3. **不覆盖**：同一槽位重拍写 `-2`、`-3`，旧照片保留。
4. **中文菇房名可用**：`A 房` 会写成 `env-A-房-...`，空格等特殊字符统一换成 `-`。

### 目录结构

**数据默认存在工具目录下的 `data/`**，本机就是：

```
/media/scc/Windows_D/AAA/grass_mushroom_tool/data/grass_mushroom_20261105/
```

启动时终端会打印「数据目录」，也可以 `--data` 指到移动硬盘/SSD。影射如下：

```
data/
  current.txt                       当前会话
  grass_mushroom_20261105/
    session.json                    日期、采集员、菇房配置、编号游标
    samples.json                    样本索引（权威数据）
    environment.json                环境照片索引
    measurements.csv                人工测量表
    photos.csv                      全部照片清单
    environment.csv                 环境照片清单
    samples/
      S001/ s001-inplace-far.jpg ...
      S002/ ...
    environment/
      A/ env-A-overview.jpg ...
```

---

## 离线 RGB-D 数据集检查工具

不需要 ROS 或连接相机。依赖 Python 3、NumPy、Pillow。只读取原始数据，生成可直接用浏览器打开的报告文件夹。

```bash
cd /media/scc/Windows_D/AAA/grass_mushroom_tool
# 默认读取 data/ 下所有会话，生成后打开浏览器
python3 review_dataset.py --open
# 仅查看一个会话内的 S002
python3 review_dataset.py --data data/grass_mushroom_20261105 --sample S002 --open
# 也可以直接读取单个样本文件夹，并指定报告位置
python3 review_dataset.py --data data/grass_mushroom_20261105/samples/S002 --output reports/S002_review --open
```

默认报告：`reports/dataset_review/index.html`。图像已内嵌，可直接本地打开，避免 `file://` 图片导致画布 `The operation is insecure`。旧报告需要重新生成。支持按会话、样本、组号切换，RGB/深度叠加、调色标、点击读取 Z 毫米值、拖框统计非零比例/中位数/P10–P90，以及逐组 CSV。图像按需载入，支持不同分辨率。导出后新增的数据需要重新运行命令读取。

仅显示 `aligned_to_color: true`，并通过实际尺寸、元数据尺寸、光学坐标系、毫米单位检查的组；未对齐、损坏或不完整的数据会跳过，原因写入 `inspection.json` 并在页面显示。深度读数无损保留，RGB 查看副本使用 JPEG。检查数据是否可用仍需要看菇体局部，非零比例不等于测量准确率；时间差优先从原始时间戳重新计算，未知值标为未知。

报告文件夹可整体复制后离线查看；原始文件链接依赖原数据相对位置保持不变。工具不读取或验证 PLY 点云，也不修改既有数据。新的快照要求 RGB 与对齐深度的原始时间戳完全相同；点云由该对图像及彩色内参生成，记录相同时间戳。已有历史数据保持原样。

---

## 现场怎么用（对照拍摄方案）

1. **到菇房，先扫环境**：「环境」页选菇房 → `菇房全景` 拍一张 → 逐排点 `棚架全排`
   → 逐层点 `培养床层`（下面的方格子矩阵会实时打勾，缺哪层一眼看出来）。
   顺便拍空床、通道、现场环境。
2. **看到草菇就新建样本**：「样本」页点「＋ 新建样本」，界面给出大号编号 `001`，
   把编号写在牌子上，然后按槽位依次拍照：
   - 先拍 `原位远照`、`原位近照`（这就是“稍远一张 + 近距离一张”）；
   - 特别密集、大小混合、遮挡严重时补 `左向补充`、`右向补充`；
   - 采摘后放编号牌 + 直尺，拍 `正面`、`反面`、`底部`；
   - 成簇的先拍 `采摘后整簇`，能分开的话用 `簇内个体` 逐个拍；
   - 最后拍 `秤读数`。
3. **填三个数**：总高度 mm、最大宽度 mm、重量 g，顺手选「单个 / 成簇」，备注写
   破损、变色之类的观察。成熟度、颜色、形态等全部留到事后按照片分类。
4. **D435i 抓一张**：把相机对准样本停留几秒，点「抓取 RGB-D 快照」，会同时存
   彩色、深度、点云和相机参数，编号与手机照片一致。
5. **进度页看还缺什么**：每个样本缺哪些必需槽位、每间菇房拍了几排几层，一目了然。

> 每张照片保存后，界面都会提示实际文件名，例如 `已保存 s007-picked-bottom.jpg`，
> 便于现场核对。

---

## 手机当相机（推荐）

界面是移动优先的：手机浏览器打开 `http://电脑IP:8770`，每个「拍照」按钮会直接
调起手机相机，拍完立即上传并按规则命名，避免“先拍一堆、事后改名”的麻烦。

- 手机与电脑必须在同一局域网；终端启动时会列出**所有**可用地址（含网卡名），
  在「设置 → 手机访问地址」里也能看到并复制。
- 电脑连的是**手机热点**时：电脑是热点客户端，手机要用电脑在热点网段里的地址
  （`10.x` / `192.168.x`），不要用代理软件产生的 `198.18.x`（工具已自动过滤）。
- 浏览器只在 HTTPS 或 localhost 下允许网页直接调用摄像头，所以这里用的是系统
  相机（`capture="environment"`），走局域网 HTTP 也能正常用。
- 也支持从相册选图（一次可以多选，会自动加 `-2`、`-3`）。

---

## D435i 与 rosbag（可选）

不装 ROS 也能用拍照与测量；装了 ROS 2 并启动相机后，界面会多出两项能力。

```bash
# 1. 启动相机（任意方式，只要话题存在即可）
ros2 launch d435i_bringup camera.launch.py \
  camera_namespace:=sensors camera_name:=nav_camera \
  serial_no:=346122070466 stage:=full use_rviz:=false

# 2. 启动本工具（会自动载入 /opt/ros/*/setup.bash）
./start.sh
# 想连工作区 overlay 一起 source：
./start_with_stack.sh /media/scc/Windows_D/AAA/robot_research_ws

# 相机话题前缀不是 /sensors/nav_camera 时：
./start.sh --camera-prefix /camera
```

- 「进度」页的 **开始录制** 等价于
  `ros2 bag record -a --include-hidden-topics -o bags/grass_mushroom_日期-时间`，
  彩色、深度、点云、IMU 全部进包；停止时发 SIGINT，让 rosbag 正常写完
  `metadata.yaml`（不要直接断电/杀进程）。
- 顶栏「相机 在线」表示最近 5 秒内有彩色图像；「相机 无数据」说明相机没起来或
  话题前缀不对。
- 快照保存的是**最新完整同步帧**，所以对准目标停留两三秒再点按钮。

用 `--no-ros` 启动可完全关闭这两项能力。

### 手机取景器（用手机看 D435i、按快门抓图）

D435i 是 USB 3 设备（要 5 Gbps 带宽和供电），手机上既没有驱动也没有接口，所以
**不是手机直连相机**，而是：Jetson/电脑插相机 → 跑本工具 → 手机连同一 Wi-Fi 打开网页。
效果就是“手机连着相机”：

1. 「样本」页最上面就是取景框，**左边彩色、右边对齐深度伪彩**（红=近，蓝=远，黑=无效）。
2. 三个模式：`彩色 + 深度` / `仅彩色` / `仅深度`。
3. 对准草菇后点 **📸 抓取 RGB-D**，一次落 4 个文件（color / depth / points / meta），
   编号与当前样本一致。
4. 手机自己的相机仍用各槽位的「拍照」按钮，两者互不干扰。

带宽不够或 Jetson 较弱时，可以直接把图片地址改成低配（浏览器地址栏或 `<img>` 里）：

```
/stream/color.mjpg?w=480&fps=6&q=60
```

| 参数 | 默认 | 范围 | 说明 |
| --- | --- | --- | --- |
| `w` | 640 | 240–1280 | 输出宽度（等比缩放） |
| `fps` | 10 | 2–20 | 推流帧率上限 |
| `q` | 70 | 40–90 | JPEG 质量 |

> 多个手机可以同时连同一路流（每一路独立编码）。Jetson 上同时开 2–3 路没问题，
> 再多会挤占 CPU；抓取动作不受推流影响。


### 深度怎么和 RGB 对齐

**是的，深度对齐到彩色**，一帧快照里彩色和深度是同一视角、同一分辨率、逐像素对应：

| 项目 | 做法 |
| --- | --- |
| 采集方式 | 相机用 `stage:=full` → `align_depth.enable=true`，额外发布 `aligned_depth_to_color/image_raw` |
| 订阅话题 | `color/image_raw` + `aligned_depth_to_color/image_raw`（+ 各自 `camera_info`） |
| 分辨率 | 848×480@30（彩色与深度同分辨率，对齐后不缩放），深度编码 `16UC1`，1 单位 = 1 mm，空洞 = 0 |
| 配对 | 相机需开启 `enable_sync`；工具按 `header.stamp` 的整数纳秒严格匹配，要求 Δt=0。缓存多帧以容纳回调先后到达，2.5 秒内没有完整同步对就报错，绝不按接收时间凑帧 |
| 落盘 | `s007-d435i-color.png` 与 `s007-d435i-depth.png` 尺寸完全一致；`meta.json` 记录 `alignment.aligned_to_color / dt_ms`、彩色与深度内参 |
| 3D | 使用该对 RGB-D 与彩色内参 K 反投影生成 PLY，位于彩色光学系、单位米。点云和 RGB/深度记录同一时间戳，不再取驱动的最新点云。非零畸变、内参缺失或不匹配时报错 |
| 保存要求 | 快照只保存 `aligned_depth_to_color/image_raw`，绝不回退到原始深度；未收到对齐深度、数据超过 5 秒未更新、时间戳配对失败或尺寸/坐标系不一致时，报错且不写入快照文件。`depth/image_rect_raw` 仅供预览/诊断，预览可见不代表对齐深度已就绪 |

这样做的原因：草菇尺寸估计、成簇分割、遮挡判断都依赖“像素 ↔ 距离”的直接对应；
对齐后的深度可以直接当作彩色图的 Z 通道用，不用再手写 `depth_to_color` 外参变换。
原始（未对齐）深度和对齐深度在 `-a` 录的 bag 里都保留了，事后要做对比也有数据。

> 默认 `stride=2` 表示宽、高方向都每隔 2 个像素取样，约保留 1/4 的像素位置；
> 实际文件体积取决于图像内容、深度有效率及点云数量。
> 想换抽样率就改 `app.py` 里 `snapshot(..., stride=2)`，文件仍然同名。

### 一个样本可以拍多组 RGB-D

同一个编号可以反复抓取（换角度、换遮挡状态、拍摄前后各一组都行），
**第二组起文件名自动加 `-2`、`-3`，不会覆盖**：

```
s001-d435i-color.png      s001-d435i-color-2.png
s001-d435i-depth.png      s001-d435i-depth-2.png
s001-d435i-points.ply     s001-d435i-points-2.ply
s001-d435i-meta.json      s001-d435i-meta-2.json     # 内含 "group": 2
```

---

## 导出的 CSV

`measurements.csv`（Excel 可直接打开，带 BOM）：

| 列 | 说明 |
| --- | --- |
| 编号 | `S001` |
| 菇房 / 棚架 / 层 | 样本所在位置 |
| 生长形态 | 单个 / 成簇 |
| 簇内个数 | 成簇时填写 |
| 总高度_mm / 最大宽度_mm / 重量_g | 人工测量 |
| 照片张数 | 该样本已拍张数（含 D435i 文件） |
| 缺拍槽位 | 还缺哪些必需照片 |
| 备注 | 现场观察 |
| 采集时间 | 建档时间 |

`photos.csv` 是逐张清单（文件名、槽位、来源、时间、相对路径），`environment.csv`
是环境照片清单 —— 事后做标注、划分训练集时直接读这两张表即可。

---

## 快捷键（电脑端）

| 按键 | 作用 |
| --- | --- |
| `1` – `9` `0` | 依次触发第 1–10 个拍摄槽位 |
| `N` | 新建样本 |

---

## 装到手机桌面（PWA）

网页自带 `manifest.webmanifest` + `sw.js`，无需应用商店，也不会弹安装包：

- **Android Chrome**：地址栏右侧「安装」或菜单 →「添加到主屏幕」。
- **iOS Safari**：分享 →「添加到主屏幕」。
- 打开后是全屏、无地址栏，图标是蘑菇；数据仍然走局域网，和浏览器里完全一样。

> 仅在 HTTPS 或 localhost 下浏览器才允许安装；局域网 HTTP 打开时仍然可以正常使用，
> 只是可能没有「安装」入口（各机型策略不同）。

---

## 部署到 Jetson

工具本身**纯 Python，不需要编译**，整个目录拷过去即可；Jetson 只多需要相机相关的 ROS 环境。

```bash
# 1) 依赖（JetPack 一般已带 numpy / Pillow；缺了就补）
sudo apt install -y python3-pil python3-numpy
# 若用 pip：python3 -m pip install --user pillow

# 2) 拷过去
rsync -a --exclude data --exclude bags grass_mushroom_tool/ jetson:~/grass_mushroom_tool/

# 3) 在 Jetson 上启动（Humble/Jazzy 都行，脚本会自动找 /opt/ros/*/setup.bash）
cd ~/grass_mushroom_tool
./start.sh --data /mnt/ssd/grass --bags /mnt/ssd/grass_bags
# 要连自定义工作区的相机包：GM_SETUP=/path/to/ws/install/setup.bash ./start.sh
```

启动后终端会打印 `http://<Jetson 局域网IP>:8770`，手机浏览器打开即可；Jetson 的
eMMC 通常很小，建议像上面那样把 `--data/--bags` 指到 SSD 或 U 盘。

性能参考：640 宽、10 fps 的 RGB-D 合成流在 Orin 系列上占用很低；Xavier/Nano 建议
用 `/stream/rgbd.mjpg?w=480&fps=6`。抓取快照是单帧操作，与推流互不影响。

---

## 依赖

| 依赖 | 是否必需 | 缺失时 |
| --- | --- | --- |
| Python ≥ 3.9 | 必需 | 无法运行 |
| Pillow | 可选 | 缩略图不生成，直接用原图显示 |
| ROS 2 + rclpy + numpy | 可选 | 只保留拍照与测量，D435i 快照/录包按钮置灰 |

---

## 常见问题

**手机打不开页面？**

按顺序排查：

1. **用对地址**。启动时终端会列出这台电脑的所有局域网地址，界面「设置 → 手机访问
   地址」里也有同样的列表。手机开热点时，电脑是**客户端**，要用无线网卡那一条
   （形如 `10.x.x.x` / `192.168.x.x`）：

   ```
   本机访问: http://127.0.0.1:8770
   手机访问（与电脑同一 Wi-Fi / 同一热点下任选其一）:
     http://10.47.238.71:8770   [wlp128s20f3 · 无线]
   ```
2. **不要用 `198.18.x.x`、`172.17.x.x` 这类地址**。开着 Clash / Mihomo / WireGuard
   等代理或 VPN 时，系统会多出 TUN 虚拟网卡（`198.18.0.1`），手机永远连不上它。
   工具已自动排除这类网卡；如果你换了别的代理软件仍看到奇怪地址，用
   `ip -4 -o addr show` 自己核对。
3. **先验证连通性**：手机浏览器打开 `http://<电脑局域网IP>:8770/api/ping`，
   返回 `{"ok": true, ...}` 说明网络通了。
4. **防火墙**：`sudo ufw allow 8770/tcp`（firewalld 用
   `sudo firewall-cmd --add-port=8770/tcp`）。
5. **Android 热点隔离**：个别机型的「AP 隔离」会阻断访问，换个办法：
   - 让**电脑开热点**，手机连电脑；
   - 用 USB 数据线共享网络（手机开「USB 网络共享」），电脑会多一块 `usb0/enp*` 网卡，
     就用那块网卡的地址；
   - Android 调试可用 `adb reverse tcp:8770 tcp:8770`，然后手机直接开
     `http://127.0.0.1:8770`。
6. 确认启动时用的是默认 `--host 0.0.0.0`（不是 127.0.0.1）。

**端口被占用？**
`./start.sh --port 8780`。

**想把数据放到移动硬盘？**
`./start.sh --data /media/xxx/草菇采集 --bags /media/xxx/bags`。

**照片拍错了？**
在界面上点缩略图右上角的 `✕` 删除，索引与文件会一起清掉；或者直接用文件管理器删，
再刷新界面（索引里会残留一条，目前建议在界面里删）。

**换一台电脑继续采集？**
整个 `data/` 目录拷过去，进入「设置 → 切换会话」。

**想让手机和电脑同时用同一个会话？**
两端打开同一个地址即可，后台是同一个服务、同一份数据；界面每 6 秒自动同步。

---

## 目录一览

```
grass_mushroom_tool/
  app.py              HTTP 服务与接口（标准库实现）
  start.sh            启动脚本（自动载入 /opt/ros/*，可选 GM_SETUP）
  start_with_stack.sh 启动脚本（顺带 source 指定工作区的 overlay）
  review_dataset.py   已对齐 RGB-D 数据集检查报告
  preview_rgbd.py     原始录制转彩色/深度并排 MP4 预览
  extract_rgbd.py     可选的手动抽帧命令（网页不提供入口）
  config/            可选高分辨率相机参数
  gmkit/
    naming.py         命名规则（唯一真源）
    store.py          会话/样本/照片存储与 CSV 导出
    media.py          PNG 写盘与缩略图
    rosbridge.py      rosbag 录制 + 严格同步快照 + 同帧点云
    video.py          原始 RGB-D 录制话题及可选离线抽帧
    playback.py       同步 RGB-D 播放预览与后台任务
  web/
    index.html  app.js  style.css
    manifest.webmanifest  sw.js  icon-192.png  icon-512.png
  tests/
    test_*.py         命名、存储、同步、HTTP 长连接及视频拖动回归测试
  data/  bags/        本地原始数据，不上传 Git
  reports/            本地检查报告，不上传 Git
```

## RGB-D 原始录制与直接回放

「样本」页取景区下方提供 **RGB-D 录制 / 回放**：

1. 点击「开始 RGB-D 连续录制」，结束时点击「停止 RGB-D 录制并收尾」。
2. 在「已完成录制」中选择一段录制，点击「生成可播放预览」。
3. 生成后直接在页面播放、暂停、拖动进度。不会导出照片，也不会生成点云文件。

原始 MCAP 保存在 `bags/rgbd_video/`，保留 RGB、16 位深度、内参与时间戳。未压缩的 848×480@30 RGB-D 每秒约 60 MB，约 10 秒达到 574 MiB 属正常体积；高分辨率会进一步增加存储量。预览单独保存在 `bags/rgbd_video/previews/`，为 H.264 MP4，左边彩色、右边深度伪彩。预览是有损的观看副本，不能从中读取精确毫米值；原始 MCAP 始终保持不变。

生成预览需要 ROS 2、NumPy 和带 H.264 编码支持的 OpenCV。默认深度色标固定为 150–1000 mm，黑色表示无效深度。只使用 header 时间戳完全相同的 RGB-D 对，按相机时间戳确定播放帧率；时间空隙以重复画面保留时长，统计保存在预览旁的 JSON 中。网页支持 HTTP Range，可拖动进度。

也可以单独生成预览：

```bash
source /opt/ros/jazzy/setup.bash
cd /media/scc/Windows_D/AAA/grass_mushroom_tool
python3 preview_rgbd.py --bag bags/rgbd_video/你的录制名 \
  --output bags/rgbd_video/previews/你的录制名-rgbd.mp4
```

网页不再提供抽帧入口。已有抽帧文件不会被自动删除；需要时仍可手动使用 `extract_rgbd.py`，输出必须指定新目录。

---

## 相机参数与故障排查

默认示例使用 848×480@30。`config/d435i_1280x720.yaml` 和
`config/d435i_1280x720_15fps.yaml` 为可选的 1280×720 参数，使用方法见文件注释；
实际启动分辨率以相机话题及快照元数据为准，所有采集模式均需开启对齐及 `enable_sync`。

- 查看器报 `The operation is insecure`：旧报告使用本地图片进行画布读取，可能被浏览器阻止。重新运行 `review_dataset.py` 生成内嵌图像的新报告。
- 停止录制后偶发 HTTP 501：旧版没有消费停止请求中的 JSON 请求体，HTTP 长连接会把残留字节当成下一次请求方法。当前版本统一读取并缓存请求体；更新后需重启采集服务并刷新网页。
- 预览生成失败：检查 ROS 环境、OpenCV 的 H.264 编码器和录制是否已经正常收尾。原始 MCAP 保持不变；错误原因会显示在页面。
- 只想保留原视频：仅操作开始/停止录制即可，预览按需生成，不自动抽帧。已有历史抽帧文件不会自动删除。

## 测试

```bash
python3 -m pytest tests -q
```
