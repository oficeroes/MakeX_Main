# 纠察文档 — mecanum_drive.py 问题记录

> 记录四轮麦轮小车代码从首次编写到稳定运行过程中遇到的全部问题、根因及修复方法。
> 如之后遇到类似症状，按序号排查。

---

## 1. 整车完全无反应，显示屏无任何信息

**症状**：烧录后 LED 点阵屏不亮，摇杆、按键全部无效，小车死机。

**根因**：`from mbuild.smartservo import smartservo_class` 导入失败。
小车硬件不一定配有智能舵机，`smartservo_class` 模块在无舵机的板子上不存在，
导致整个 Python 脚本在 import 阶段抛出 `ImportError`，主循环永远无法启动。

**修复**：
```python
# 改前（脆弱）
from mbuild.smartservo import smartservo_class

# 改后（容错）
try:
    from mbuild.smartservo import smartservo_class
    _HAS_SERVO = True
except ImportError:
    _HAS_SERVO = False
    smartservo_class = None
```
并且所有对 `__servo_1`、`__servo_2` 的调用必须包裹在 `if _HAS_SERVO: try: ... except: pass` 中。

**教训**：任何"可能不存在"的硬件模块，导入和调用都必须容错。
对照三轮车代码（`mecanum_forward.py`）——它从来不 import smartservo。

---

## 2. 按 + 键启动自动程序无任何反应

**症状**：遥控模式正常，但按手柄 + 键不触发 auto_mode。

**根因**：`AUTO_SEQUENCE[0]` 是一个 5 元素标记元组（如 `("encoder_cal", 2.0, 0, 50, 0)`），
而 + 键启动代码强制按 4 元素解包：

```python
# 崩溃代码
dur0, Vx0, Vy0, w0 = AUTO_SEQUENCE[0]   # ValueError: too many values to unpack
```

此异常被 Novapi 静默吞掉，表现为按键无反应。

**修复**：解包前先检查 `isinstance(first[0], str)`，标记元组走单独分支。

**教训**：AUTO_SEQUENCE 含混合元组格式时，所有读取 `AUTO_SEQUENCE[n]` 的地方
（+ 键启动、步骤切换打印）都必须做类型判断。

---

## 3. 自动程序方向与遥控不一致（倒着走）

**症状**：遥控时推摇杆前进方向正确，但 + 键启动自动程序后小车后退。

**根因**：遥控模式在输出前应用了 `INVERT_VY / INVERT_VX / INVERT_OMEGA`（见代码
第 14 节），但自动模式直接使用 AUTO_SEQUENCE 中的原始值，未经过方向校准。

**修复**：在 auto_mode 的标准运动元组处理中，解开 `step_Vx, step_Vy, step_omega` 后
立即应用 INVERT 标志：
```python
if INVERT_VX: step_Vx = -step_Vx
if INVERT_VY: step_Vy = -step_Vy
if INVERT_OMEGA: step_omega = -step_omega
```
同样逻辑也要加到 `encoder_cal` 等标记元组的驱动分支。

**教训**：方向校准是一种"执行端"逻辑，无论遥控还是自动都应该经过同一道处理。

---

## 4. 小车自动程序起步打滑 / 抖动

**症状**：auto_mode 中每步一开始就输出满功率（如 50%），轮子打滑空转，
车身偏移，走不准。

**根因**：功率从 0 瞬间跳变到目标值，静摩擦→动摩擦突变导致打滑。

**修复**：加入加速度渐变（S 曲线），见下方第 6 节。

---

## 5. 摇杆不动但其他按键正常 / 误入调试模式

**症状**：正常遥控时摇杆完全无法操控底盘电机，但 D-pad、N1/N4 等按键功能正常。

**根因**：误按了 ≡（菜单键）进入了调试模式。
调试模式下整个正常遥控流程被 `continue` 跳过，只有调试模式的十字键和 N 键有效。

**判断方法**：看 LED 是否显示 `Test` 而非 `Main`。`Main` = 正常模式，`Test` = 调试模式。

**修复**：按 ≡ 键退出调试模式，LED 恢复显示 `Main`。
代码加固：进入调试模式时重置所有边沿变量（`last_* = False`），
防止退出调试后残留按键状态触发误操作。

---

## 6. LED 屏 / 控制台显示乱码或空白

**症状**：点阵屏或串口打印出现乱码、空白、或完全不更新。

**根因**：
- LED 点阵屏只能显示 ASCII 字符，传中文到 `__led.show()` 会乱码
- 某些 Novapi 控制台编码不支持 UTF-8 中文

**修复**：
- 所有 `__led.show()` 参数改为纯 ASCII（如 `"Main"`, `"Test"`, `"M1"`, `"S50"`）
- 所有 `print()` 字符串改为英文（即使注释保留中文）
- 启动时必须有 `__led.show("Main")` + `novapi.reset_timer()`

**教训**：嵌入式设备输出一律用 ASCII。注释随便用中文因为不会上传执行。

---

## 7. 代码中用 `;` 分隔多语句 / 内联 if 导致兼容问题

**症状**：本地 Python 语法检查通过，但烧录后部分功能异常或静默失败。

**根因**：Novapi 平台的 Python 解释器可能不支持某些语法糖：
- `stmt1; stmt2`（分号分隔的多语句）
- `if cond: stmt`（条件与语句同行）

**修复**：每条语句独占一行，条件和语句体分行缩进。
```python
# 改前
__motor_M1.set_power(0); __motor_M2.set_power(0)
if INVERT_VX: _vx = -_vx

# 改后
__motor_M1.set_power(0)
__motor_M2.set_power(0)
if INVERT_VX:
    _vx = -_vx
```

---

## 8. 自动程序完成后执行机构未完全停止

**症状**：auto_mode 跑完后 M5 滚球电机、无刷电机、舵机仍在运行。

**根因**：旧代码只对 M1-M4 做了 `set_power(0)`，没有调用 `stop_all_motors()`。

**修复**：auto_mode 完成时统一调用 `stop_all_motors()`，该函数涵盖全部执行机构。

---

## 9. 标定测试走太远

**症状**：编码器标定（直走 2 秒）跑出去超过半米，场上没有足够空间。

**修复**：标定时长从 2 秒缩短为 1 秒，功率不变。距离约减半。

在 GUI 的 `on_encoder_calibration_export` 中改：
```python
seq = [("encoder_cal", 1.0, 0, 50, 0), STOP_BUFFER]  # 原来是 2.0
```

---

## 10. 编码器标定后按 N1-N4 LED 没有显示 / 显示错误

**症状**：编码器标定程序跑完后，按机器人 N1/N2/N3/N4 键，LED 屏不变，或者显示旧的电机状态而不是 `E####`。

**根因 A：`_enc_cal_start` NameError（已修复）**
旧代码中 `_enc_cal_start` 只在 `if elapsed < AUTO_STEP_DELAY * 1.5` 块里赋值，
若第一帧恰好错过时间窗口，则后面的 `_enc_cal_end - _enc_cal_start` 抛出 `NameError`，
被 `try/except NameError: pass` 静默吞掉，`_cal_delta_M1/2/3/4` 永远是 `None`，
所以 N1-N4 边沿检测里 `if _cal_delta_M1 is not None` 始终为假，不触发显示。

**修复**：`_enc_cal_start` 改为模块级列表 `[0, 0, 0, 0]`，在赋值时用切片 `[:]` 就地更新，
避免变量作用域问题。同时移除 `try/except NameError`。

**根因 B：LED 格式太长**
`"E%d" % delta` 若 delta 超过 4 位数（如 12345），LED 屏幕可能只显示前几位或乱码。
Novapi 8×16 点阵屏最多稳定显示 4 个字符（含小数点）。

**处理方法**：读取增量后，在 GUI 的「📥 录入标定结果」对话框中手动输入数值，而不依赖 LED 原始显示。若增量超过 9999，分批记录（两位数一组）。

**完整的标定结果读取流程**：
1. 按 + 键运行编码器标定（约 1 秒直走后停止）
2. 量距离 D（cm）
3. 按 N1 键 → LED 显示 M1 编码器增量（`E####`），记下数字
4. 按 N2/N3/N4 分别读 M2/M3/M4
5. 回到 GUI 右侧面板 → 点「📥 录入标定结果」→ 填入 D 和四个增量
6. GUI 自动推算 PPR 和 P50 直行速度，并更新输入框

---

## 快速排查清单

遇到问题按以下顺序检查：

- [ ] LED 是否亮？不亮 → 检查 import 是否容错（问题 1）
- [ ] LED 显示 `Main` 还是 `Test`？`Test` → 按 ≡ 退出调试（问题 5）
- [ ] 按 + 键有反应吗？没反应 → 检查 AUTO_SEQUENCE[0] 类型（问题 2）
- [ ] 自动程序方向对不对？倒着走 → 检查 INVERT 是否在 auto 中应用（问题 3）
- [ ] 起步打滑不走直线？→ 检查加速度渐变是否生效（问题 4）
- [ ] 打印/屏幕正常吗？乱码 → 检查 print 和 LED 是否含中文（问题 6）
- [ ] 摇杆正常吗？不动 → 是否误入 debug 模式（问题 5）
- [ ] 代码语法检查通过但烧录后异常 → 检查是否有 `;` 或内联 if（问题 7）

---

## 三轮代码参考（mecanum_forward.py）

三轮全向车的代码是稳定基准，结构清晰：
- 无 servo/smartservo 依赖
- auto_mode 简洁：只有 4 元组运动步骤，无标记元组
- 有 `face` 旋转用于正面切换
- 有 `__led.show("Main")` + `novapi.reset_timer()` 启动
- print 和 LED 全英文 ASCII
- 调试模式有标定子模式（编码器闭环走固定距离）

四轮车的新功能应参照三轮的代码风格，而非反向。
