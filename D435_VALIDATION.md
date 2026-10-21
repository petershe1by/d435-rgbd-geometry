# 笔记本接入 D435 的实机验证步骤

以下命令用于 Windows PowerShell。在当前笔记本上已有可加载的 `.venv`；2026-10-07 检查时未枚举到 RealSense 设备。本次仅验证日志统计工具，尚未实际连接 D435。

## 1. 连接与设备检查

用支持 USB 3 数据传输的线把 D435 直接接入笔记本 USB 3.x 口，优先使用相机配套线。避免只支持充电的线，首次验证先不用扩展坞。笔记本接电源，关闭不必要的后台负载。

从 [RealSense 官方 SDK 发布页](https://github.com/realsenseai/librealsense/releases/latest) 获取 Windows RealSense Viewer，先检查设备是否可见、USB 连接类型、RGB 和深度是否能同时稳定开启。对应的 [Windows 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_windows.md) 和 [入门说明](https://www.realsenseai.com/get-started-depth-camera/) 可供参考。

代码默认同时申请 640×480、15 fps 的 RGB 和深度流。确认 USB 3.x 与实际采集正常后，可以改为 30 fps。深度可用的模式不一定适用于 RGB；在 Viewer 查找两路共同支持的组合，再修改 `config.json` 的 `width`、`height`、`fps` 并重启。测试代码前关闭 Viewer 和其他占用相机的程序。

```powershell
# 在克隆后的项目目录打开 PowerShell
.\.venv\Scripts\python.exe -c "import pyrealsense2 as rs; ds=rs.context().query_devices(); print('devices:',len(ds)); [print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in ds]"
```

接上相机后应能列出 D435。多个 RealSense 可在后面的命令加 `--serial 实际序列号` 选择设备。

枚举到设备不等于能收到视频帧。进一步执行下列命令，检查实际 USB 类型、支持的流配置及 5 帧采集；不保存图像、不覆盖工位标定：

```powershell
.\.venv\Scripts\python.exe check_environment.py --profiles --capture
```

如果是在另一台笔记本复制运行，请复制源码和配置，重新创建该机器的虚拟环境，勿直接复制现有 `.venv`。使用安装了 Tk 的 64 位 Python；当前项目已在 Python 3.13.3 测试。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --only-binary=:all: -r requirements-lock.txt
.\.venv\Scripts\python.exe app.py --demo
```

当前笔记本可直接运行最后一条无相机演示，确认 GUI 正常。

## 2. 固定相机并标定空桌面

固定 D435 斜俯视工位，使目标顶部和部分侧面可见。先以相机到桌面约 0.4–0.9 m 作为布置起点，再根据深度完整性调整；30 cm 高物体的顶部也必须在有效测距范围内。使用平整、哑光桌面、稳定漫射光，先用不透明、单色、不反光物体。

清空桌面，保持工位照明稳定，执行：

```powershell
.\.venv\Scripts\python.exe app.py --calibrate-plane --workspace d435_workspace.npz
```

程序等待曝光稳定、拟合桌面，打印保存位置与 plane RMS 后退出。RMS 越大越应检查桌面是否平整、是否包含墙壁/杂物，以及深度是否有明显孔洞；它不是完整物体尺寸误差。也可先打开实时界面，按 `r` 框选工位区域、保存参数，再清空桌面按 `b` 标定。

相机移动、流分辨率改变或背景明显变化后重做空桌面标定。给不同工位分别保存 workspace，不要沿用错误姿态的标定。

## 3. 检查识别与测量

```powershell
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --preset balanced --output live_balanced.jsonl
```

先只放一个已用卡尺测过的正方体或圆柱，等待连续确认，检查形状、颜色、尺寸和深度图；再增加到 3 个、6 个分开的物体。不要让手持续进入 ROI。结束时在图像窗口按 `q`。

默认出现 RGB、深度和已分析帧三个窗口，以及中文调参面板。目标要完整落在 ROI 内，尺寸字段不是 `null` 才表示已确认。`unknown` 需要结合质量标记排查，不应为了显示结果直接降低所有门限。

长宽高单位 mm，顺序 L≥W、H 为桌面法向高度。横放圆柱的 H 是直径。三角形物体的 L/W 是约定的最小面积外接矩形尺寸，不是任意两条三角边长；真值需采用同一定义。

现场调参按顺序进行：ROI → 空桌面标定 → 最小离桌高度/面积 → 表面噪声 → 颜色阈值 → 分类与确认门限。摄像头采集配置修改后要重启。每个正式测试保持参数不变，使用不同输出文件，启动会覆盖同名日志。

## 4. 测实时性能

保持场景和物体数量固定，运行约 60 秒后按 q，先测带界面的实际使用体验：

```powershell
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --preset balanced --quiet --output speed_balanced.jsonl
.\.venv\Scripts\python.exe evaluate_run.py speed_balanced.jsonl
```

再测无界面运行，`--frames 1800` 是采集帧数上限，不是识别结果数；实际时长取决于采集速度：

```powershell
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --preset balanced --headless --quiet --frames 1800 --output speed_balanced_headless.jsonl
.\.venv\Scripts\python.exe evaluate_run.py speed_balanced_headless.jsonl
```

固定目标数量和布局，分别将 `balanced` 改为 `fast`、`precise`，同时更换输出文件名。各模式重复 2–3 次；不要并行运行占用同一 D435 的进程。开始前做短时间运行，让相机曝光及系统负载稳定。统计默认只跳过前 3 条识别结果，可用 `--skip 10` 增加跳过条数。

重点比较：

- `processing_ms`：单次算法处理平均值、P95、最大值，包含周期全面搜索，不只看稳定帧。
- `result_output_hz`：程序真正输出识别结果的频率。
- `host_receipt_to_output_ms`：主机收到帧到输出结果的帧龄；不包括此前曝光、USB 传输和显示器延迟。
- `unknown_object_fraction`：已检出目标中未确认的比例；不能用它推算漏检率。
- `frames_without_detections`：无检出结果帧数；空桌面本来就应无目标。
- 任务管理器中 Python 的 CPU、内存和磁盘负载，以及长时间运行的温升影响。

30 fps 采集不等于 30 fps 识别。默认实时模式取最新可用帧，跳过未处理的中间帧属于工作方式。不要用 `--every-frame` 测实时吞吐率，它会让采集等待识别。

日志没有人工真值，统计工具不能计算正确率和尺寸误差。离线回放的输出频率也不是实时摄像头吞吐率。

## 5. 录制相同数据公平比较准确度

相机和桌面标定保持固定，录一段短视频，包含静止、移动后停稳、不同摆放位置和适当遮挡。RGB-D `.bag` 占空间，先检查剩余磁盘空间，按 q 停止录制。

```powershell
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --record scene.bag --quiet --output recorded_live.jsonl
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --bag scene.bag --preset balanced --headless --quiet --output replay_balanced.jsonl
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --bag scene.bag --preset precise --headless --quiet --output replay_precise.jsonl
.\.venv\Scripts\python.exe evaluate_run.py replay_balanced.jsonl
.\.venv\Scripts\python.exe evaluate_run.py replay_precise.jsonl
```

两次回放都逐帧分析，便于在相同画面上比较形状、尺寸和稳定性。工位标定和使用的配置必须一起保存；调参后重放测试集，并另留一组未用于调参的样品验收。

准备每类 2–3 个尺寸/颜色的样品，在桌面中央、边缘和不同平面内转角测量；先单物体，再多物体。记录真实类别、颜色、尺寸、放置方向，以及对应的画面区间。统计分类正确率、漏检/unknown、颜色正确率、L/W/H 平均误差与 P95、静止抖动、移动后确认时间；漏检和 unknown 不能从统计分母删掉。重复帧不应当作独立样品证明泛化。

## 6. USB 侧视留到单 D435 验收后

确认单相机工作正常，再固定普通 USB 侧视相机，按 README 运行棋盘格标定：

```powershell
.\.venv\Scripts\python.exe calibrate_stereo.py --usb 1 --cols 9 --rows 6 --square-mm 25
.\.venv\Scripts\python.exe app.py --usb 1 --stereo stereo_calibration.json --workspace dual_workspace.npz --calibrate-plane
.\.venv\Scripts\python.exe app.py --usb 1 --stereo stereo_calibration.json --workspace dual_workspace.npz --output dual_live.jsonl
```

USB 索引、内角点数量和格子实际边长按设备/标定板填写。单 D435 与双视角分开标定、分开统计。普通 USB 不提供与 D435 的硬件同步，先验证静止和缓慢运动物体。

## 常见故障

| 现象 | 检查顺序 |
| --- | --- |
| `No device connected` 或设备数为 0 | Viewer 能否看到 → 换 USB 3 数据线/端口 → 检查连接及 Windows 相机权限 |
| Viewer 正常但 Python 无法启动流 | 关闭 Viewer/相机应用 → 查看支持的 RGB+深度共同流配置 → 核对 Python 环境 |
| `Couldn't resolve requests` | 核对两路共同支持的尺寸/帧率；当前配置为 640×480、15 fps，不把 848×480 深度模式直接用于 RGB |
| 流已启动但 `Frame didn't arrive` | 换 USB 3 数据线和笔记本 USB 3 直连端口 → 重新插拔 → Viewer 分别及同时检查 RGB/深度出图 → 关闭 Viewer 后重试采集自检 |
| 桌面有大量碎片或物体不出现 | 空桌面重新标定 → 检查 ROI/有效深度 → 调高度和面积门限 |
| 经常 unknown / 尺寸 null | 查看质量标记 → 检查截断/遮挡/深度孔洞 → 使用受支持形状与姿态 → 再调噪声及评分 |
| 颜色偏差 | 稳定漫射光 → 等曝光/白平衡稳定 → 调颜色阈值；先不以彩色显示窗口的视觉印象代替真实颜色标签 |
| 识别慢 | 分别测 1/3/6 个目标 → 比较 headless → 快速预设/缩小 ROI → 查 CPU 与温升 |
| `ModuleNotFoundError` | 始终用项目 `.venv\Scripts\python.exe`，在该环境安装 requirements |

当前 33 项合成/模拟回归测试可通过下面的命令自行复查；它们不替代真实相机验收：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```
