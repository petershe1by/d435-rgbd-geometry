# D435 启动排查记录

日期：2026-10-08。

## 已确认事实

- SDK 枚举到 RealSense D435，固件 5.15.0.2，当前连接由相机报告为 USB 2.1。
- 原配置同时申请 RGB 和深度 848×480、30 fps；当前设备的 RGB BGR8 没有 848×480 模式，USB 2.1 下深度 848×480 也不支持 30 fps。因此原 `Couldn't resolve requests` 有明确的流配置原因。
- 当前设备两路都列出 640×480 的 6/15/30 fps 模式，SDK 对三个组合均能解析。这只验证配置可解析，不保证实际视频传输成功。
- 实际测试 640×480 双路 15 fps：能启动，但 5 秒内未收到帧。进一步测试深度单路 15 fps、RGB 单路 15 fps、双路 6 fps：都能启动，分别等待 10 秒后仍未收到帧。
- Windows 两个 RealSense 相机设备状态均为 OK；查询到的用户级和机器级相机访问值均为 Allow。这不能排除其他驱动/进程/策略问题，也不能据此认定原因只在 USB 线。

## 已完成的软件修复

- `config.json` 及默认配置改为 RGB/深度 640×480、15 fps。
- 保留明确指定的采集尺寸，不自动切换分辨率，避免误用原工位/双相机标定。
- 配置解析失败时提示请求的尺寸/帧率和检查项；实际收帧超时则显示 USB 类型，并区分“设备能启动”和“设备能出图”。
- `check_environment.py` 增加 `--profiles` 和 `--capture`，支持列出 USB 类型/模式及按配置实际采集 5 帧；不保存图像，不修改标定。
- 36 项回归测试全部通过，包括新增的配置解析失败、真实错误保留和超时资源释放测试。

## 下一步实机操作

1. 关闭相机应用，拔下 D435；换已知支持 USB 3 数据传输的线，直连笔记本 USB 3 端口，暂时去掉扩展坞。
2. 用 RealSense Viewer 确认实际 USB 类型及 RGB/深度分别、同时出图。USB 3.x 连接应报告为 3.x；仅枚举到设备不算采集通过。
3. 关闭 Viewer 后运行：

```powershell
.\.venv\Scripts\python.exe check_environment.py --profiles --capture
```

4. 如果打印五帧 RGB/深度尺寸，且没有 FAILED，则清空桌面重新标定并识别：

```powershell
.\.venv\Scripts\python.exe app.py --calibrate-plane --workspace d435_workspace.npz
.\.venv\Scripts\python.exe app.py --workspace d435_workspace.npz --preset balanced --quiet --output live_test.jsonl
```

若 Viewer 同样不出图，应先继续排查数据线、端口、Windows 相机访问及驱动/SDK/固件兼容性；本次没有更改固件或 Windows 权限。若 Viewer 出图但 Python 不出图，保留 Viewer 流配置和采集自检报错，作为下一步 SDK 差异定位依据。

参考：[RealSense 官方对齐示例](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py) 使用 640×480 深度和 RGB，并通过 SDK 对齐；这是配置选择的参考，不是当前连接已恢复的证据。
