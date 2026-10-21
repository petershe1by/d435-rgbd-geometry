# 识别不出与误分类修复

这是本地用户模型的历史修复记录；文中的原始场景、外部 STEP 和逐帧日志未随仓库分发。仓库可直接运行 `models/examples/scene.json`，预览图见 [识别示例](docs/images/step-recognition.png)，十二类参数见 `configs/simulation_extended.json`。

## 截图问题

读取本次实际场景 `scene_20261021_135314_f3080b.json` 和结果，确认物体为 A01 正方体、A03 圆柱、A04 球体、A05 圆锥、A09 正六棱锥、A12 六棱柱。

1. 相机到观察点设为 240 mm，不代表每个表面深度都大于 200 mm。前排目标的部分光学 Z 小于配置下限，被分割过滤，造成缺面、低深度质量及圆柱分成两个目标。
2. 原六类模型没有球体、正六棱锥或六棱柱，无法正确给出这些类别，可能误匹配成圆锥/圆柱。
3. 增加相似类别后，圆锥与多棱锥的分数更接近，单视角证据不足。尖顶高度的采样百分位约束还会压低拟合高度。

## 当前运行方式

先在旧识别窗口按 q，关闭旧编辑器，重新启动：

```powershell
# 在项目目录执行
.\.venv\Scripts\python.exe step_simulator.py
```

在“物块库”选择物块，保留勾选“扩展 12 类仿真预设”和“第二路 RGB”，点“自动取景”后启动。启动时也会自动检查全部 CAD 几何的测距及画面边缘；本截图场景从 240 mm 自动调整到约 290.4 mm。编辑器预览会提示物块是否超出深度范围。

若本机仍保留已修复的原始场景，可运行以下命令；新克隆的仓库请使用 README 中的六类示例命令：

```powershell
.\.venv\Scripts\python.exe app.py --scene simulations/screenshot_autofit_scene.json --config models/user_blocks/config_extended.json
```

## 修改内容

- 可选新增球体、正四/五/六棱锥、正五/六棱柱，合计 12 类；正多边形模板约束底面为规则多边形。原硬件配置仍可运行默认六类。
- 自动取景检查完整几何的光学深度和像素边界；只调整虚拟相机，不使用 CAD 文件名或尺寸作为识别答案。
- 第二路虚拟俯视 RGB 使用独立三角网格渲染及已知外参，通过现有背景分割、目标关联和投影轮廓评分融合。没有把第二路深度或模型所有者标签交给识别器。
- 小物块扩展预设按模拟噪声收紧表面/深度评分尺度，并采用 0.30 表面、0.25 深度、0.10 主轮廓、0.35 俯视轮廓权重。分类分差仍为 0.06，确认次数仍为 3，深度质量门限仍为 0.65。
- 对一致且有效的观测平均最近 7 次类别评分；类别改变、位置明显移动、质量差或姿态改变时清除历史。持续接近的候选仍输出 unknown。
- 圆锥/棱锥最高采样点仅作为顶点高度的下界，由表面与轮廓补充估计尖顶；周期全面搜索将上一轮可靠拟合作为额外候选保留，并与新起点一起按当前观测比较。
- unknown 输出新增 `decision_reason`，界面显示候选类别和原因，例如 `confirming_frames`、`ambiguous_classes`、`insufficient_depth`、`segmentation_or_model_fit`。
- 编辑器启动时保存独立的场景和 `.config.json` 参数快照，原全局配置与原 STEP 文件不被覆盖。算法面板保存的是本次参数快照。

## 验证与边界

- 50 项回归测试通过，涵盖原六类、配置/GUI、相机模拟、双视角投影、自动取景、评分平均和未知目标保护。
- 你目录中 A01–A12 逐个测试，每个 8 次观测，最终类别均正确；使用固定的虚拟视角、模拟噪声和扩展预设。
- 截图对应六物块完成 30 帧重放。前两帧确认阶段后，28/28 帧均检出 6 个目标且类别正确，包含多次周期全面搜索。结果见 `simulations/screenshot_weighted_final.jsonl`。
- 更长测试曾暴露周期搜索丢弃先前可靠解、类别边界抖动的问题；上述结果对应修复后的最终版本，不只取单张成功画面。

验证使用了用于调参的 CAD 和固定虚拟相机，不是独立泛化精度测试。第二路相机外参和同步在仿真中是理想的，真实相机需要另行标定和验证。薄壁、镂空、组合体等剩余模型仍未完整建模；能导入不代表能正确区分全部 50 类。相机更远、噪声更大、轮廓被遮挡时，保留 unknown 是正常的。

验证文件：`simulations/basic12_validation/recognition_validation.json`、`simulations/recognition_fix_summary.json`、`simulations/regression_after_recognition_fix.log`；预览为 `simulations/screenshot_recognition_fixed.png`。
