# 三轮/四轮自动程序升级：闭环、行进自旋、位置 PID、UI 重构（2026-07-08）

> 面向后续排错的实战记录：**诉求 → 实现 → 待验证/排错要点**。
> 机器人当时不在手上，所有改动均为**静态实现 + 语法/逻辑验证**，未经实车。
> 上实车前请逐项对照本文「排错指引」。

作者：Claude (Opus) + 用户联调
================================================

## 0. 背景与整体诉求

这一轮围绕**三轮车（omni3）自动程序**展开，兼顾四轮（mecanum_4w）。用户的赛场任务：
自动阶段从启动区出发 → 到中央资源区一侧 → 用升降臂夹取两个方块 → 分别放到两个兑换区。

关键约束：
- **三轮必须非常精准**，宁可牺牲速度。
- 四轮实测**四个行走电机功率不一致**（M3 时多时少）。
- 动作要能**沿路径同步执行**（边走边做），而不是「先做完动作再跑线」。

---

## 1. 双击画不成样条/直线（回归 bug 修复）

### 症状
样条/直线工具：单击落点、移动能看到预览线，但**双击后线消失**，再点变成新线——一条完整曲线都画不成。

### 根因
`field_view.py` 鼠标处理有两个问题：
1. 放点模式下每次左键都调 `begin_anchor_placement()`（**重置**锚点列表为 1 个点），`add_anchor()` 写好却从没被调用 → 锚点永远累加不上去。
2. 双击那一下的坐标从没被加进锚点 → `finish` 时不足 2 个点 → 直接丢弃。

### 修复
- `mousePressEvent`：首次点 = `begin_anchor_placement`，之后每次点 = `add_anchor`。
- `mouseDoubleClickEvent`：收尾前先 `add_anchor(双击点)`。
- `add_anchor` 加 <1cm 近距去重（双击的重复点不会产生零长段）。
- 放点模式下点击不被路径/预览线拦截（只让 `_AnchorHandle` 拦截，用于拖控制点）。

### 排错要点
- 画不成线 → 看 `FieldScene._anchor_pts` 是否随点击累加。
- 双击后终点丢失 → 确认 `mouseDoubleClickEvent` 里 `add_anchor` 在 `finish_anchor_placement` 之前。

---

## 2. 四轮曲线改编码器闭环（解决 M3 功率飘）

### 根因
四轮曲线原本走**开环时间驱动** `traj_v (dur, Vx, Vy, omega)`——没有编码器反馈纠偏，四个轮子各跑各的，所以 M3 时多时少。

### 实现
- 运动设置区新增复选框 **`chk_mec_closed_loop`「四轮编码器闭环（精准，牺牲速度）」**，默认**开**。
- `main_window._build_combined_sequence`：`use_continuous_traj` 仅在**关掉开关**时才成立；默认走 `build_encoder_sequence`（三轮同款闭环 builder），输出 `enc_move (ticks, vy_power, vx_power)`。
- 四轮固件本来就有 `enc_move` 闭环分支能接住 → **零固件改动**。
- 设置随 JSON 存读（`settings["mecanum_closed_loop"]`）。

### 排错要点
- 四轮还在飘 → 确认导出序列是 `enc_move` 不是 `traj_v`（关掉开关才回退开环）。
- 走得比以前慢是**预期**（闭环逐步到位换精度）。
- `enc_move` 靠 `ENCODER_TICKS_PER_CM` 换算，标定值不对会走错距离——见 [[omni3-distance-calibration]]。

---

## 3. `enc_moverot` 组合闭环 + 三轮「行进中自旋」

### 诉求
「换面」本质不存在：用户用**行进中自旋**（边走边转到某个朝向）+ 调对应舵机就实现了换面。所以**不做固件的 face_rotate 换面系统**。

### 固件：新增 `enc_moverot` 动作（两个固件各一个）
- 步格式：`('enc_moverot', ticks, vy_power, vx_power, omega_power, rot_ticks)`。
- 原理：**平移各轮目标角度 + 旋转各轮目标角度叠加**，一次 `move()` 让所有轮子同时闭环到位 → 边走边转且精准。
- 平移贡献沿用 `_enc_move_start` 算法，旋转贡献沿用 `_enc_rot_start`，完成检测复用 `_enc_move_tick`。
- 纯新增，不动现有逻辑；dispatch 分支已接（omni3 用单引号，4w 用双引号）。

### GUI：`build_travel_spin_sequence`（kinematics.py）
- 路径按弧长切成 N 个微步，每步走一点 + 转总角的 1/N，输出 `enc_moverot`。
- 转角编码与现有 `enc_rot` 完全一致：`rot_ticks = deg * ticks_per_cm * 0.5`，符号由 omega 携带。
- **车身坐标补偿**：随车头累计转角，把场地系路径切向旋转回车身系（`bx, by` 那段）。
- 入口：**右键路段 → 设置行进中自旋角度**（+顺 / -逆），`PathItem.travel_spin_deg`，随 JSON 存读。
- **自旋精度滑块** `sp_spin_precision`（默认 8cm）：每微步弧长，越小越精准但步数越多。

### 排错要点
- **旋转方向反了** → `build_travel_spin_sequence` 里 `spin_sign` 一个符号翻转即可（已注释）。这是最可能需要实车调的点。
- **长曲线撑爆三轮字节码** → 自旋精度调大（减少微步数）。三轮字节码贴上限，见 [[omni3-memory-ceiling]]。
- **转不够/转过头** → 检查累计 `rot_ticks` 是否等于目标角（`sum(rot_ticks)/ticks_per_cm/0.5 == 目标度数`）。
- 平移与旋转**不同步到位** → 固件 `_enc_moverot_start` 里 rpm 按各轮位移比例缩放，若某轮明显拖后腿看这里。

---

## 4. 通用编码电机位置 PID（`motor_pos`，M1-M5，可调 PID）

### 诉求
编码电机块要能「跑到目标值就停」，带**可调 PID**，去掉固定档位改成**自己填目标角度**。M1-M5 都要（M6 是舵机）。用于加爪升降臂精确定位。

### 固件：新增 `motor_pos` 动作（两个固件各一个）
- 步格式：`('motor_pos', motor_id, target_deg, kp, ki, kd)`。
- 目标为**绝对编码角度**（`get_value("angle")`），PID 闭环到位即停。
- **每帧后台 tick**：自动主循环开头调 `motor_pos_tick_all()` → 与路径**同步执行**（模型 A：边走边定位）。
- dispatch **即发即走**：启动 PID 后立即前进下一步，PID 在后台驱动。
- ⚠️ **四轮固件无 `import math`**，四轮版减速段用**线性公式**（omni3 用 `math.cos`，4w 不能照抄）。

### GUI：编码电机块新增位置模式
- `mode` 下拉：`continuous`（原开环持续转，向后兼容）/ `position`（位置闭环）。
- position 模式字段：目标角度 + Kp/Ki/Kd（用户自调，默认 0.35/0.002/0.05）。
- 导出：position → `motor_pos`，continuous → `motor`（原样）。
- 所有字段随 JSON 存读。

### 排错要点
- **电机不动/乱转** → 确认 `motor_pos_tick_all()` 在自动循环每帧被调（否则 PID 永不更新）。
- **到不了位/超调** → 调 Kp/Ki/Kd。Kp 太大振荡，Ki 太大超调。
- **目标角度语义** = 绝对编码角度，不是相对。换算需实车标定（升降臂档位角度）。
- 四轮若报 `math not defined` → 确认四轮版 `motor_pos_tick` 没用到 `math.*`。

---

## 5. 三轮固件补全空的 `servo` 自动标签

### 根因
三轮自动程序里 `servo` 标签是**空的 `pass`**——自动阶段夹爪根本不动。四轮的 servo 分支有实现（`sv.move_to(angle, speed)`）。

### 实现
- 补全三轮 `servo` 分支，参照四轮：`S1/SV1 → __servo_1`，`S2/SV2 → __servo_2`，`move_to(angle, speed)`，等 `wait_ms`。

### ⚠️ 安全约定（三轮夹爪）
固件注释：**0°=张开零点，负角度=夹紧，正角度=撑爆报废**（机械极限 ±280°）。
- GUI 舵机块角度范围已从 `0..270` 改为 `-280..280`，加了安全提示。
- **默认值仍是 90（正数）**——三轮上填正数会撑爆！用户在三轮场景必须填**负角度夹紧**。

### 排错要点
- 自动程序夹爪不动 → 确认三轮 `servo` 分支不是 `pass`。
- 夹爪撑坏 → 三轮必须用**负角度**夹紧，别用默认 90。

---

## 6. UI 四项重构

1. **修按钮溢出**：绘制工具原来是第三工具栏 `tb3`，挤在一行被 Qt 折进「»」溢出菜单（要点空白才出来）。移除 tb3。
2. **左侧竖排工具面板**：新增 `_build_left_toolpanel()`，返回 QFrame，手绘/样条/直线常驻可见。中央 view 包进 `QHBoxLayout`（左面板 + view）。
3. **动作按钮重排**：7 个动作块按钮从挤成一行改成两组分行。
4. **锚点操作增强**：Backspace 撤销上一个锚点（`undo_last_anchor`），状态栏实时显示已放锚点数（`anchor_count_changed` 信号）。

### 排错要点
- **崩溃 `no attribute _build_left_toolpanel`** → `__init__` 调 `_build_left_toolpanel()` 但方法定义丢失。这是本轮**反复出现**的落盘问题（见第 8 节）。call site 和 def 必须同时存在。
- 绘制工具点不出来 → 确认没有残留 tb3（`grep tb3 main_window.py` 应为 0）。

---

## 7. 执行模型：只做「模型 A」（同步/融合）

用户明确：**只要模型 A**（动作与线同步启动，边走边做）。
- 要「走到某点停下做动作」→ 用户自己把线画到那点、动作挂**尾部**即可，不需要中点插入。
- 所以**不做**「动作沿路径中间任意位置插入」和「走到 X% 停下」这类模型 B 功能。
- `motor_pos` 的即发即走 + 每帧 tick，正是模型 A 的实现。

---

## 8. ⚠️ 工作流教训：CJK 文件的落盘陷阱

本轮**多次**出现：bash-python 编辑报告成功，但改动**没真正落盘**（或被后续编辑基于旧快照覆盖）。已确认踩坑的文件：`action_block_item.py`、`main_window.py`、`mecanum_drive.py`。

**根因**：这些文件 CJK 密集，且 bash 环境是 GBK 控制台——
- `Read` 工具对这些文件**会乱码、错行**，不能信其行号/内容。
- heredoc 里的 CJK 字面量在 GBK 下会损坏，导致 `str.replace` 的 anchor 匹配失败。
- 同一命令里读-改-写若中途异常被吞，会报成功但没写入。

**可靠做法**（本轮验证有效）：
1. 用 `python - <<'PY'` 读写，**anchor 用纯 ASCII**，CJK 用 `\uXXXX` 或 f-string 变量。
2. 每次编辑**立即重读校验**（`before/after` 字节数 + 关键子串计数 + `ast.parse`）。
3. 大改用**偏移切片**（`data[:s] + new + data[e:]`）而非 `str.replace`。
4. 文件是 CRLF 还是 LF 要先确认（`repr` 看 `\r\n`）。
5. 收尾**全量审计**：grep 每处改动的关键标识是否都在。

见 [[curve-drawing-and-head-anchor]] 同类教训。

---

## 9. 上实车前的验证清单

| 项 | 怎么验 | 最可能要调的地方 |
|---|---|---|
| 双击画线 | 画样条/直线，双击收尾 | 已修，应可用 |
| 四轮闭环 | 画曲线导出，看是 enc_move | 慢是预期；标定值 |
| 行进自旋方向 | 画直线设 90° 自旋 | `spin_sign` 符号（大概率要翻） |
| 行进自旋精度 | 长曲线看是否撑爆 | 自旋精度滑块调大 |
| motor_pos 定位 | 升降臂设目标角度 | Kp/Ki/Kd + 目标角标定 |
| 三轮夹爪 | 自动程序调 servo | **必须负角度**，别用默认 90 |
| 左侧工具面板 | 开程序看左侧 | 已修崩溃 |

---

## 10. 改动文件清单

**GUI（trajectory_planner/）**
- `field_view.py` — 双击修复、锚点放置状态机、Backspace 撤销、行进自旋回调、左面板容器
- `path_item.py` — `travel_spin_deg` 属性 + 序列化、右键设自旋菜单、锚点手柄
- `action_block_item.py` — 编码电机位置模式（mode/target/PID）、servo 角度范围
- `kinematics.py` — `build_travel_spin_sequence` 微步 builder
- `main_window.py` — 四轮闭环开关、自旋精度滑块、左侧工具面板、导出分支路由
- `curves.py` — （前一轮）Catmull-Rom 样条 + 直线
- `file_io.py` — 曲线/自旋字段序列化

**固件**
- `mecanum_forward.py`（三轮）— `enc_moverot`、`motor_pos`、补全 `servo`、每帧 tick
- `mecanum_drive.py`（四轮）— `enc_moverot`、`motor_pos`（无 math 版）、每帧 tick
