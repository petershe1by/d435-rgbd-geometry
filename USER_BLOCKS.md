# 本机 STEP 物块库与历史验证

仓库不分发原始用户模型或包含绝对路径的本机索引。下文记录一组 50 个 STEP 的本地导入与验证；新电脑请先运行 `prepare_step_library.py` 导入自己的模型目录，再使用“物块库”。

50 个 STEP 均可读取、三角化和生成虚拟 RGB-D，原始模型保留在原目录；场景、清单与测试结果写入本项目。模型多数约 40 mm，A06 正四面体的桌面法向高度约 32.66 mm。

## 直接打开

```powershell
.\.venv\Scripts\python.exe step_simulator.py
```

默认加载 A01 正方体、A02 长方体、A03 圆柱、A05 圆锥、A06 正四面体、A10 三棱柱，使用小物块扩展配置。物块库现在支持 A01–A12 的 12 种基础几何模型，含球体及规则四/五/六棱形。近距离识别与误分类的修复及最新验证见 [识别修复说明](RECOGNITION_FIX.md)。

点工具栏“物块库”，可以浏览全部 50 个模型及外接尺寸。单选或 Ctrl/Shift 多选，点“载入所选物块到场景”，每次最多 6 个；再点“更新预览”和“启动仿真识别”。列表中的支持类别是参考说明，不是识别结果。

直接打开六物块的识别窗口：

```powershell
.\.venv\Scripts\python.exe app.py --scene models/user_blocks/basic_six.json --config models/user_blocks/config_extended.json
```

## 生成文件

| 文件 | 作用 |
| --- | --- |
| `models/user_blocks/library.json` | 50 个模型的路径、网格尺寸、面数、单物体场景和参考类别 |
| `models/user_blocks/basic_six.json` | 六类基础物块组合场景 |
| `models/user_blocks/config_40mm.json` | 小物块专用仿真算法参数，独立于全局 config.json |
| `models/user_blocks/config_extended.json` | 默认 12 类仿真配置，配合第二路 RGB 与多帧评分 |
| `models/user_blocks/single/` | 每个物块一个独立场景 |
| `models/user_blocks/group_*.json` | A–E 分组场景，每个场景最多 6 个物块 |
| `models/user_blocks/recognition_validation.json` | 原全局参数下，50 模型各 3 帧的初步识别结果 |
| `models/user_blocks/basic_six_validation.json` | 小物块参数下，六类组合的 8 帧验证及尺寸误差 |
| `simulations/user_blocks_basic_six_40mm.jsonl` | 组合测试的逐帧输出 |
| `simulations/user_blocks_recognition.png` | 用你的实际 STEP 生成的识别预览 |

模型分组：A 基础实心 12 个，B 薄壁 4 个，C 组合 7 个，D 扩展几何 7 个，E 镂空 20 个。所有颜色都是指定的仿真颜色，没有读取模型材质。

## 验证结果和边界

原全局离桌分割阈值为 8 mm。对于高度只有约 33–40 mm 的小物块，该阈值会丢弃较多底部观测，正四面体在原参数下未稳定确认。专用配置把该阈值改为 3 mm，把拟合表面噪声尺度设为 1.5 mm，以匹配场景的 0.8 mm 模拟深度噪声。没有降低分类分差、确认次数或深度质量门限；原全局配置保持不变。

六物块组合测试共 8 帧，从第 3 帧起六类均正确确认，共 36 次物体观测；长宽高分量平均绝对误差约 0.66 mm，最大约 2.1 mm。真值按空间位置匹配，取 STEP 三角网格的同定义外接尺寸。参数在这批模型上调整、视角固定、噪声简化，因此不是独立泛化结果，也不是 D435 实拍精度承诺。

上述 8 帧误差及 `recognition_validation.json` 中 50 模型各 3 帧的记录来自上一版六类识别器。该版存在五/六棱柱被映射为圆柱等情况。现在可选启用 12 类，A01–A12 的逐个仿真验证通过；球体、五/六棱形已加入模型。其余 38 个薄壁/组合/扩展/镂空模型仍未完整建模，**不能声称可靠区分全部 50 类或实心、薄壁和镂空。**

面板“网格外接尺寸”是模型参考值；识别窗口/JSON 中的尺寸仍由可见 RGB-D 拟合。未支持类别的候选尺寸或误确认尺寸不可当作可靠测量。

## 刷新物块库

新增 STEP 后可重新生成索引与场景。命令读取外部模型、更新项目内生成文件；已有 `config_40mm.json` 会保留。

```powershell
.\.venv\Scripts\python.exe prepare_step_library.py 'D:\MyModels'
```

新电脑上需保留源模型路径，或重新执行导入命令建立本机库。仿真通用说明见 [STEP_SIMULATION.md](STEP_SIMULATION.md)。
