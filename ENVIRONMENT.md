# 本文件夹运行环境

本文前半部分是原开发电脑的部署记录，不表示克隆仓库后已自动安装环境。新电脑先运行 `install_environment.ps1`（STEP 仿真加 `-Simulation`），或按 [README](README.md) 手动创建虚拟环境。历史日志和 `.venv` 均不随仓库分发。

部署日期：2026-10-07。第三方依赖均安装在项目 `.venv` 内，`include-system-site-packages = false`，不读取全局第三方包。虚拟环境使用本机已安装的 64 位 Python 3.13.3 基础解释器及其 Tk 标准库；它不是可直接复制到另一台机器的便携 Python。

| 组件 | 已部署版本 |
| --- | --- |
| Python | 3.13.3，64 位 |
| NumPy | 2.1.2 |
| SciPy | 1.16.0 |
| OpenCV | opencv-python 4.11.0.86，包含图形界面支持 |
| RealSense SDK Python | pyrealsense2 2.58.4.10922 |
| Tk | 8.6，使用本机 Python 自带组件 |

四个第三方包的版本固定在 `requirements-lock.txt`，原 `requirements.txt` 保留宽版本范围供后续兼容性开发。正式恢复部署优先使用固定版本。

2026-10-21 新增 STEP 仿真扩展：本项目 `.venv` 另安装 `cadquery-ocp-novtk==7.9.3.1`、`cadquery-ocp-proxy==7.9.3.1`，上述四个核心依赖版本未改变。启动 `.\.venv\Scripts\python.exe step_simulator.py`，详细见 [STEP 仿真说明](STEP_SIMULATION.md)。新机器部署仿真使用安装脚本的 `-Simulation` 开关。

部署验证：`pip check` 通过，四个依赖均从本地 `.venv` 加载，Tk/OpenCV 窗口创建与关闭通过，33 项回归测试通过。一键安装与启动脚本实际运行通过；6 帧端到端合成演示完成，末帧六类形状均确认并输出尺寸。源码语法检查通过。演示记录保存在 `deployment_demo.jsonl`。上述检查不代表真实 D435 采集或实物测量精度验收。

## 直接运行

在本文件夹打开 PowerShell，无需 activate：

```powershell
# 无相机演示，默认带调参面板
.\.venv\Scripts\python.exe app.py --demo

# D435 实时运行；空桌面按 b 标定，再放入物体
.\.venv\Scripts\python.exe app.py

# 环境与 GUI 自检；未连接 D435 会提示，但不当作依赖部署失败
.\.venv\Scripts\python.exe check_environment.py --gui

# 强制要求连接相机，用于接入后的检查
.\.venv\Scripts\python.exe check_environment.py --require-camera

# 查询 USB 和支持模式，并实际读取 5 帧，不保存图像
.\.venv\Scripts\python.exe check_environment.py --profiles --capture

# 回归测试
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

也可以使用启动脚本：

```powershell
.\run.ps1 demo
.\run.ps1 camera
.\run.ps1 calibrate --workspace d435_workspace.npz
.\run.ps1 check --gui
.\run.ps1 test
```

`run.ps1` 会以脚本所在文件夹为工作目录，并传递额外参数。`camera` 模式不强制选择 workspace，默认使用 `workspace_calibration.npz`；创建了其他名字的标定后，启动时也要传同一 `--workspace`。

如果 PowerShell 执行策略阻止 `.ps1`，可继续使用上面的 Python 直接启动命令，或仅对该次进程执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run.ps1 demo
```

无需更改系统永久执行策略。

## 恢复或在另一台机器部署

本机已部署完成，不需要重复安装。在另一台 Windows 机器上先安装带 Tcl/Tk 的 64 位 Python；安装脚本接受 Python 3.11–3.13，当前实际验证版本为 3.13.3。复制源码和配置后在新机器重新生成 `.venv`。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\install_environment.ps1
```

若 `python` 指向的解释器不合适，明确指定可执行文件：

```powershell
.\install_environment.ps1 -Python 'C:\实际安装位置\python.exe'
```

脚本会创建或更新项目虚拟环境、从 pip 配置的软件源下载固定版本二进制包、运行 `pip check` 和 GUI/SDK 自检。不会安装到全局 Python，不修改相机固件，不删除项目数据。没有合适的基础 Python 时，需要先安装 Python，再运行脚本。

## D435 连接

Python SDK 已部署，可运行 RealSense 采集代码。环境自检会列出连接设备；当前部署时未连接 D435，故尚未验证真实设备的视频流。

相机接入、桌面标定、实机性能和准确度验证见 [D435 验证指南](D435_VALIDATION.md)。RealSense Viewer 是可选的相机诊断工具，不是运行该 Python 项目的必要 Python 依赖；本次没有安装系统级 Viewer。
