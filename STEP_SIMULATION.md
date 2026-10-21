# STEP 虚拟 RGB-D 工位

仓库自带六类示例位于 `models/examples/`，不包含本机生成的用户物块库、外部 CAD 或历史 `simulations/` 记录。文中这类文件需在本机生成。可移植的十二类仿真参数位于 `configs/simulation_extended.json`。

已在本项目中加入独立 STEP 仿真入口，无需 D435。流程是：**STEP CAD → 三角网格 → 带可见性/遮挡的虚拟 RGB 和深度 → 原有分割、模型拟合、颜色判断、跨帧确认 → 长宽高及形状输出**。

模型文件名、模型外接尺寸和 CAD 拓扑不会交给识别器作为答案；实际识别使用渲染得到的像素与深度。它可以用于算法调试和样品几何验证，不能据此保证实拍准确率。

## 启动导入面板

在项目目录打开 PowerShell：

```powershell
.\.venv\Scripts\python.exe step_simulator.py
```

或使用启动脚本：

```powershell
.\run.ps1 simulate
```

已准备 `models/user_blocks/basic_six.json` 时，默认加载用户物块及同目录 `config_extended.json`（不存在时回退 `config_40mm.json`）；否则加载六类示例。当前接入的 50 模型库及验证说明见 [用户物块库](USER_BLOCKS.md)，可在工具栏点“物块库”选择模型。近距离缺面、球体及六棱形识别修复见 [识别修复说明](RECOGNITION_FIX.md)。测试其他文件：

1. 点“新建场景”，清空示例。
2. 点“导入 STEP”，可选择一个或多个 `.step` / `.stp` 文件。
3. 选择物体，设置桌面 X/Y、绕 X/Y/Z 的角度、颜色和缩放倍率。长度单位为 mm，角度为度；选中物体的修改会在预览/启动时自动应用，也可点“应用所选物体”。
4. 设置虚拟相机的方位角、俯视角、距离、水平视场，以及深度噪声和孔洞比例。
5. 点“更新预览”，检查物体是否都在视野内、尺寸是否合理。底部显示的是**从 CAD 网格计算的参考尺寸**，不是识别结果。
6. 勾选“扩展 12 类仿真预设”和“第二路 RGB”可识别球体及规则四/五/六棱形；点“自动取景”检查有效测距和边缘，启动时也会自动取景。物块库生成的场景默认带第二路 RGB。
7. 点“启动仿真识别”，出现虚拟 RGB、深度、第二路 RGB、已分析结果和原有算法调参面板。等待至少三次有效观测后看确认结果。unknown 会显示候选及原因；输入不足时不会强制确认。

修改场景后，请先在识别窗口按 q 退出，再启动新场景。编辑器关闭后已启动的识别窗口仍会保留。每次启动的场景快照、参数 `.config.json`、JSONL 结果及控制台日志分别保存在 `simulations/`，文件名独立生成；算法面板保存的是本次参数快照。

仿真读取已知虚拟桌面，不需要清空桌面按 b。仿真中 b 键不写入真实相机的标定文件。对真实 D435 继续使用原来的 `python app.py`。

## 无界面运行和示例

项目已经生成 `models/examples/` 下六个 STEP 实体及 `scene.json`。它们由 Open CASCADE 的 CAD 实体独立建模，渲染不使用原 `Primitive` 的凸包射线求交。

```powershell
# 六类 STEP，带 GUI 和调参
.\.venv\Scripts\python.exe app.py --scene models/examples/scene.json

# 离线逐帧测试，不受相机连接影响
.\.venv\Scripts\python.exe app.py --scene models/examples/scene.json --headless --every-frame --frames 20 --output my_step_run.jsonl

# 统计识别处理耗时，不能把它当真实 D435 采集帧率
.\.venv\Scripts\python.exe evaluate_run.py my_step_run.jsonl
```

`simulations/step_rgb.png`、`step_depth.png` 是渲染示例，`step_recognition.png` 是实际识别器标注预览。`step_results.jsonl` 和 `validation_summary.json` 保存六帧验证记录与汇总。此验证六个示例在确认阶段后均输出了正确类别，但样本数量很小、形状理想，不能当作泛化测试。

若需要在新位置生成示例，可执行下面的命令。脚本会拒绝覆盖已有目标文件：

```powershell
.\.venv\Scripts\python.exe make_step_examples.py --output models/my_examples
```

## 场景文件格式

可以用面板保存 JSON，也可以手写。相对模型路径以场景文件所在目录为基准；面板保存绝对模型路径以便本机复用。移动到其他电脑时请一起复制模型并修改路径。

```json
{
  "camera": {
    "azimuth_deg": -90,
    "elevation_deg": 52,
    "distance_mm": 850,
    "target_mm": [0, 0, 35],
    "horizontal_fov_deg": 58
  },
  "noise_sigma_mm": 0.8,
  "hole_fraction": 0.02,
  "seed": 1,
  "objects": [
    {
      "file": "my_block.step",
      "name": "我的物块",
      "position_mm": [0, 0],
      "rotation_deg": [0, 0, 30],
      "color_rgb": [220, 45, 35],
      "scale": 1,
      "yaw_speed_deg_s": 0
    }
  ]
}
```

`rotation_deg` 依次为绕 X/Y/Z 的欧拉角。物体按当前姿态的底部最低点放到桌面，X/Y 是当前姿态包围范围的中心。一个 STEP 文件中的所有几何体作为一个刚性目标；装配体内的相对几何保留，但不会自动拆成多个检测对象。

可选 `secondary_camera` 与 `camera` 使用相同字段，设为 `null` 关闭。编辑器“第二路 RGB”默认生成一个俯视补充视角，以已知虚拟外参进行图像轮廓融合；实际识别仍只有主相机一路深度。`camera.distance_mm` 是相机到观察点的距离，不能直接当作每个物体表面的深度。

`yaw_speed_deg_s` 可以生成绕桌面法向旋转的视频；时间由帧号/配置 fps 决定。渲染或识别变慢时并不保证与真实墙钟时间同步，没有惯性、重力或碰撞模拟。静态场景缓存无噪声渲染结果，每帧重新采样噪声和孔洞。

其他可选参数：`mesh_tolerance_mm` 默认 0.15 mm，越小曲面网格越细、导入越慢；`shading` 默认 0.12；`rgb_noise_sigma` 默认 1；`table_color_rgb` 默认灰色。每个文件限制 10 万个三角形，超限时需简化 CAD 或适当增大三角化公差。复杂工程装配不适合作为轻量实时测试对象。

## 单位、颜色和深度

- STEP 声明的源单位由 OCCT 读取并转换到 mm，再变为内部米制。毫米和英寸文件的 75 mm 方块往返测试已通过。`scale` 是额外缩放，不要仅因源文件是英寸再乘 25.4。
- 当前导入几何，不读取 STEP 的材质、面颜色或纹理。RGB 颜色由场景或颜色选择器指定；因此颜色测试验证的是视觉代码，不是在读取模型的颜色标签。
- 深度是相机光学 Z，单位 m，与现有 RealSense 输入约定一致，不是欧氏距离。三角形使用透视正确的 z-buffer 保留遮挡、孔洞和凹陷；不把模型替换成凸包来渲染。
- 预览中的参考长宽高来自三角化网格，与 CAD 精确曲面仍有三角化误差；不是三维任意方向最小体积包围盒。最终识别尺寸来自所启用的几何模型拟合，不直接读取参考尺寸。

## 依赖和恢复安装

当前项目 `.venv` 已安装 CAD 扩展，原四个依赖版本未改变。STEP 功能使用 `cadquery-ocp-novtk==7.9.3.1` 和配套 proxy，不安装 VTK、Open3D、PyBullet 或完整物理仿真平台。

新机器恢复：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\install_environment.ps1 -Simulation
```

已有基础环境只安装扩展：

```powershell
.\.venv\Scripts\python.exe -m pip install --only-binary=:all: --constraint requirements-lock.txt -r requirements-simulation.txt
```

参考 [OCP 无 VTK 发行包](https://pypi.org/project/cadquery-ocp-novtk/7.9.3.1/) 与 [OCCT 官方三角化说明](https://github.com/Open-Cascade-SAS/OCCT/blob/master/dox/user_guides/mesh/mesh.md)。

## 验证边界

当前 50 项测试通过，覆盖原六类、扩展模型、单位、光学深度、遮挡/孔洞、确定性噪声、真实标定保护、外部库导入、双视角外参、自动取景及 Tk 操作。12 个基础 CAD 和截图场景重放的验证见 [识别修复说明](RECOGNITION_FIX.md)。

默认六类，可选扩展到 12 类：另含球体、正四/五/六棱锥、正五/六棱柱。薄壁、镂空、组合体、非规则多边形、偏顶锥体、任意倾斜姿态仍未完整建模。噪声及孔洞模型简化，虚拟桌面、双相机外参和同步已知；不能代替真实 D435 标定和独立实物验收。
