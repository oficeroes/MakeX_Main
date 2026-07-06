# MakeX_Main — 系统文档

> 面向下一个 AI agent 或接手开发者的完整说明。

## 项目结构

```
MakeX_Main/
├── mecanum_forward.py          # 三轮全向机器人遥控程序（Novapi 平台）
├── mecanum_X_forward.py        # X 型麦克纳姆（4 轮）遥控程序（Novapi 平台）
├── trajectory_planner/         # 轨迹绘制 GUI 工具（PyQt5）
│   ├── __init__.py
│   ├── main.py                 # 入口：python -m trajectory_planner.main
│   ├── main_window.py          # 主窗口控制
│   ├── field_view.py           # 画布（QGraphicsScene/View，单位=cm）
│   ├── path_item.py            # 轨迹图形项
│   ├── obstacle_item.py        # 可拖拽障碍物
│   ├── smoothing.py            # Chaikin 平滑 + 等弧长重采样
│   ├── kinematics.py           # 路径 → AUTO_SEQUENCE
│   ├── file_io.py              # JSON 存读 + 自增文件名
│   ├── exporter.py             # 写回机器人源文件
│   └── config.py               # 所有常量 + ChassisProfile 注册表
├── trajectories/               # 保存的轨迹 JSON（自动创建）
│   └── trajectory_001.json…
└── docs/                       # 本文档目录
    ├── README.md               # 本文件
    └── EXTENDING_FOR_NEW_CHASSIS.md
```

---

## 快速上手

### 安装依赖
```bash
pip install PyQt5
```

### 启动 GUI
```bash
cd D:\UserData\Desktop\MakeX_Main
python -m trajectory_planner.main
```

### 基本流程
1. 在右侧面板选择底盘（三轮全向 / X 型麦克纳姆）
2. 在画布上按住左键画轨迹
3. 调整运动参数（功率、反转、漂移补偿）
4. 点击「导出到机器人」写入对应 .py 文件
5. 烧录到机器人，按 + 键触发自动程序

---

## 核心概念

### 坐标系

| 方向 | GUI 场景 | 机器人 |
|------|----------|--------|
| +X | 右 | 右横移 (+Vx) |
| +Y | 上（前方） | 前进 (+Vy) |
| omega | 逆时针 → GUI 取反 | 顺时针为正 |

FieldView 用 `scale(1, -1)` 翻转屏幕 Y 轴，使屏幕"上"对应场景 +Y。  
场景单位直接等于厘米，`mapToScene()` 不需要手动换算。

### AUTO_SEQUENCE 格式

```python
AUTO_SEQUENCE = [
    (duration_sec, Vx, Vy, omega),
    ...
    (0.1, 0, 0, 0),   # 末尾停止缓冲
]
```

- `Vx / Vy / omega`：电机功率百分比（-100 ~ 100），不是 cm/s
- 末尾 `(0.1, 0, 0, 0)` 是必要的停止缓冲，exporter 自动追加

### 编码器标定

当前主轨迹使用编码器闭环，距离精度主要由 `ENCODER_TICKS_PER_CM`
（GUI 里显示为“编码器比例 °/cm”）决定。

推荐流程：
1. 点击「编码器标定」导出 1 秒 P50 直走测试。
2. 烧录后按 `+` 运行，量实际距离 `D`。
3. 按 `N1`~`N4` 读取四轮编码器增量。
4. 回 GUI 点「录入标定结果」，填入 `D` 和四轮增量。

GUI 会自动计算：
- `编码器比例 = 四轮平均增量 / D`
- `P50 直行 = D cm/s`

单独的速度标定按钮已移除；仅测速度不能修正“画得大、跑得小”的比例问题。

### 步间速度插值（AUTO_RAMP_MS）

```python
AUTO_RAMP_MS = 100   # 每步前 100ms 内从上一步速度线性插值到当前步
```

避免瞬间跳速导致轮子打滑和机身震动。  
编码器标定导出时设为 0，避免插值影响测量。

### 漂移补偿

机器人横移时可能因轮子安装误差整体偏转。通过 `drift_left_omega` / `drift_right_omega` 叠加旋转来补偿：

- `drift_left_omega`：Vx < 0（GUI 向左）时附加的 omega（正值=顺时针）
- `drift_right_omega`：Vx > 0（GUI 向右）时附加的 omega
- 强度随 `|Vx|/auto_power` 线性缩放（斜移时按比例）
- **补偿在轴反转之前计算**，所以永远对应 GUI 里画的方向

### 轴反转

| 设置 | 效果 |
|------|------|
| `invert_x = True` | 所有 Vx 取反（左右接反） |
| `invert_y = True` | 所有 Vy 取反（前后接反） |
| 单轴翻转 | omega 同时取反（镜像变换） |
| 双轴翻转 | omega 不变（180° 旋转） |

---

## 模块快速索引

| 文件 | 核心职责 | 关键 API |
|------|----------|----------|
| `config.py` | 常量 + ChassisProfile 注册表 | `CHASSIS_PROFILES`, `get_profile(id)` |
| `smoothing.py` | Chaikin + 等弧长重采样 | `smooth_and_resample(raw, iter, step)` |
| `kinematics.py` | 路径 → AUTO_SEQUENCE | `build_sequence(points, mode, ...)` |
| `exporter.py` | 安全写回机器人源文件 | `write_auto_sequence(seq, robot_file, ...)` |
| `file_io.py` | JSON 存读 | `save_trajectory(payload)`, `load_trajectory(path)` |
| `field_view.py` | 画布 Scene/View | `FieldScene`, `FieldView` |
| `path_item.py` | 轨迹图形项 | `PathItem.set_points(raw, smooth)` |
| `obstacle_item.py` | 障碍物图形项 | `ObstacleItem(x,y,w,h)`, `to_dict()` |
| `main_window.py` | 主窗口逻辑 | `MainWindow` |
| `main.py` | 启动入口 | `main()` |

---

## 机器人文件说明

### mecanum_forward.py（三轮全向）

运动学（120° 对称布局，顺时针 omega 为正）：
```
M1 = -0.5*Vx - 0.866*Vy - omega
M2 = -0.5*Vx + 0.866*Vy - omega
M3 =      Vx             - omega
```

特有功能：
- `FACE_ROTATIONS` 矩阵 — R1/L1 切换正面方向（3 个对称正面）
- 手柄按键：R1=切下一面，L1=切上一面

### mecanum_X_forward.py（X 型麦克纳姆 4 轮）

运动学（X 型布局，顺时针 omega 为正）：
```
M1(FL) = Vy + Vx + omega
M2(FR) = Vy - Vx - omega
M3(BL) = Vy - Vx + omega
M4(BR) = Vy + Vx - omega
```

- 前后/左右对称，没有 face 概念，R1/L1 空置
- 任一轮超过 100 时所有轮等比缩放（`mecanum_kinematics` 内处理）

两个文件共用相同的：
- 主循环结构（auto_mode / debug_mode / 正常遥控）
- `AUTO_RAMP_MS` 步间插值系统
- `AUTO_SEQUENCE` 格式
- 调试模式（≡ 键进入，方向键切换电机，N1/N2 测试）
- N1 收球电机开关

---

## 测试与验证

```bash
# 语法检查全部脚本
python -c "
import ast, glob
for f in glob.glob('trajectory_planner/*.py') + ['mecanum_forward.py','mecanum_X_forward.py']:
    ast.parse(open(f,encoding='utf-8').read())
    print('OK', f)
"

# 运行 GUI
python -m trajectory_planner.main
```

验证流程：
1. GUI 启动，画一段轨迹，保存 → 检查 `trajectories/trajectory_001.json`
2. 选择底盘，导出 → 检查目标文件的 `AUTO_SEQUENCE` 块被替换
3. 切换到 X 型麦克纳姆，再次导出 → 确认写入了 `mecanum_X_forward.py`
4. 导入刚才的 JSON → 确认底盘下拉框自动恢复正确型号
