# 新增底盘类型指南

> 目标读者：下一个需要为新款机器人扩展轨迹规划器的 AI agent 或开发者。

本指南以**新增一款差速两轮底盘（diff2）**为例，说明全部必要步骤。
步骤总数：**3 步**，不需要修改 GUI 核心代码。

---

## 第 1 步：创建机器人源文件

新建 `diff2_forward.py`（参照 `mecanum_X_forward.py`），必须包含：

### 1a. AUTO_RAMP_MS 常量（exporter 会就地更新这行）

```python
AUTO_RAMP_MS = 100
```

### 1b. AUTO_SEQUENCE 块（exporter 会替换这段）

```python
AUTO_SEQUENCE = [
    (4.0,  0, 50,  0),   # 前进 4 秒
    (0.1,  0,  0,  0),   # 停止缓冲
]
```

格式要求：
- 锚行必须是 `AUTO_SEQUENCE = [` 开头（正则 `^\s*AUTO_SEQUENCE\s*=\s*\[`）
- `]` 单独一行结尾
- exporter 用括号计数定位结束行，行内注释、字符串均可正常处理

### 1c. 运动学函数（可选，但建议明确命名）

```python
def diff2_kinematics(Vx, Vy, omega):
    # 差速两轮：只能前进后退和自转，不能横移
    left  = Vy - omega
    right = Vy + omega
    max_abs = max(abs(left), abs(right))
    if max_abs > 100:
        scale = 100.0 / max_abs
        left *= scale; right *= scale
    return left, right
```

### 1d. 主循环里使用 AUTO_SEQUENCE

```python
auto_mode = False
auto_step = 0
auto_step_start = 0.0
auto_prev_Vx = 0.0
auto_prev_Vy = 0.0
auto_prev_omega = 0.0

# 在主循环 auto_mode 分支里：
if AUTO_RAMP_MS > 0 and elapsed * 1000.0 < AUTO_RAMP_MS:
    alpha = elapsed * 1000.0 / AUTO_RAMP_MS
    Vx_out  = auto_prev_Vx  + (step_Vx  - auto_prev_Vx)  * alpha
    Vy_out  = auto_prev_Vy  + (step_Vy  - auto_prev_Vy)  * alpha
    w_out   = auto_prev_omega + (step_omega - auto_prev_omega) * alpha
else:
    Vx_out, Vy_out, w_out = step_Vx, step_Vy, step_omega

left, right = diff2_kinematics(Vx_out, Vy_out, w_out)
```

---

## 第 2 步：在 config.py 注册

打开 `trajectory_planner/config.py`，在 `CHASSIS_PROFILES` 列表里追加：

```python
DIFF2_PROFILE = ChassisProfile(
    profile_id="diff2",
    display_name="差速两轮",
    file_name="diff2_forward.py",
    wheel_count=2,
    has_face_concept=False,          # True = 机器人有可切换的正面（R1/L1 键）
    description="左轮 + 右轮，纯前进/后退/自转。横移 Vx 在此底盘无效。",
)

CHASSIS_PROFILES = [OMNI3_PROFILE, MECANUM_X_PROFILE, DIFF2_PROFILE]  # 追加到末尾
```

**就这两处**，GUI 重启后下拉框会自动出现新底盘。

---

## 第 3 步：验证

```bash
# 语法检查
python -c "import ast; ast.parse(open('diff2_forward.py', encoding='utf-8').read()); print('OK')"
python -c "import ast; ast.parse(open('trajectory_planner/config.py', encoding='utf-8').read()); print('OK')"

# 启动 GUI，切换到差速两轮，导出轨迹
python -m trajectory_planner.main
```

检查清单：
- [ ] `diff2_forward.py` 语法检查通过
- [ ] GUI combo_chassis 出现"差速两轮"选项
- [ ] 切换后提示标签显示 `diff2_forward.py`
- [ ] 导出后 `diff2_forward.py` 的 `AUTO_SEQUENCE` 被正确替换
- [ ] `AUTO_RAMP_MS` 被写入正确数值
- [ ] 保存 JSON 再导入 → 底盘自动恢复为"差速两轮"

---

## ChassisProfile 字段说明

| 字段 | 类型 | 含义 |
|------|------|------|
| `profile_id` | str | 唯一 ID，写入 JSON，保持小写蛇形（如 `"diff2"`） |
| `display_name` | str | 下拉框显示名称 |
| `file_name` | str | 机器人源文件名（相对 PROJECT_ROOT） |
| `wheel_count` | int | 轮子数（仅用于显示，不影响逻辑） |
| `has_face_concept` | bool | True → 提示用户先检查正面方向（适用于三轮全向） |
| `description` | str | 控制面板里的底盘描述文字 |

`file_path` 属性自动返回 `PROJECT_ROOT / file_name` 的绝对路径。

---

## 注意事项

### Vx 对某些底盘无效
差速两轮不能横移（Vx 方向）。如果用户画了斜向轨迹，
kinematics 会生成含 Vx 的元组，但机器人侧 `diff2_kinematics` 中 Vx 项被忽略。
可以在 `diff2_kinematics` 里加断言或 print 警告，但 exporter 不会过滤。

**若需 GUI 侧过滤**：在 `main_window.on_export()` 里根据 profile 做预检查（可选扩展）。

### schema_version
每次 JSON 格式发生不兼容变化时，在 `config.py` 里递增 `SCHEMA_VERSION`。
当前版本：`4`（v4 新增 `chassis_profile_id`）。

### 旧 JSON 兼容
`_apply_payload` 里对所有字段都用 `.get(key, default)` 读取，
旧格式（无 `chassis_profile_id`）导入时会默认选 omni3，不会报错。

### 多底盘共用轨迹
AUTO_SEQUENCE 的 `(duration, Vx, Vy, omega)` 格式与底盘无关。
同一条轨迹切换底盘重新导出是完全合法的操作。

---

## 已注册底盘一览

| profile_id | display_name | 文件 | 轮数 | face 概念 |
|------------|--------------|------|------|-----------|
| `omni3` | 三轮全向（120° 对称）| `mecanum_forward.py` | 3 | ✓ |
| `mecanum_x` | X 型麦克纳姆（4 轮）| `mecanum_X_forward.py` | 4 | ✗ |
| `mecanum_drive` | 麦克纳姆竞赛车（含收球/滚球）| `mecanum_drive.py` | 4 | ✗ |
