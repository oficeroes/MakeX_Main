# 四轮麦克纳姆曲线路径建模调研

本目录只沉淀资料和重构方案，不修改机器人主流程。资料已经下载到
`sources/`，后续断网也可以回看。

## 当前结论

1. GUI 里画出的轨迹点应解释为“小车中心点轨迹”，不是车头边缘、轮子、外框或球机构轨迹。
2. 因为轨迹是中心点，所以场地边缘和障碍物天然存在“中心点禁入区”：
   - 四轮车宽 48 cm，左右至少留 24 cm。
   - 四轮车长 50 cm，前后至少留 25 cm。
   - 如果未来允许车头旋转，禁入区不能只用半长半宽，要按机器人外形做配置空间膨胀。
3. 当前曲线跑得“一格一格”的源头不是单一参数问题，而是模型架构问题：
   - GUI 把曲线离散成很多短线段。
   - 导出端把短线段变成多个 `enc_move`。
   - 机器人端每个 `enc_move` 都调用 Novapi `encoder_motor.move(position, rpm)` 的位置模式。
   - 位置模式到目标会减速并等待到位，即使中间没有 `enc_stop`，曲线也会变成“到点、停、下一点”。

## trajectory_015.json 诊断

`trajectory_015.json` 当前四轮参数：

- `encoder_ticks_per_cm = 12.287`
- `cm_per_second_at_power_50 = 81.0`
- `strafe_cm_per_second_at_power_50 = 30.0`
- `max_speed = 100.0 cm/s`
- `auto_power = 85`
- `smoothing_iterations = 6`
- `resample_step_cm = 1.0`
- 四轮车实际尺寸：长 `50 cm`，宽 `48 cm`

当前这条曲线长度约 94.68 cm。导出逻辑先生成 96 个 1 cm 左右的
`enc_move`，再合并成 6 段：

```text
('enc_move', 300, 84, 11)
('enc_move', 216, 75, 40)
('enc_move', 12, 58, 62)
('enc_move', 300, 46, 72)
('enc_move', 12, 27, 81)
('enc_move', 297, 5, 85)
('enc_stop', 0, 0, 0)
```

这说明 GUI 曲线已经被压成若干“方向台阶”。机器人端又用位置模式逐段追踪，
所以实车会明显停顿。

2026-07-07 已执行第一阶段止血修改：四轮麦克纳姆纯平移曲线不再导出
`enc_move`，改为连续速度采样 `traj_v`。

第二次修正：加入横移速度独立标定。前进 P50=81 cm/s，横移先按
P50=30 cm/s 建模。当前 `trajectory_015.json` 导出结果为 17 个
`traj_v` + 1 个 `traj_stop`，总时长约 `1.399s`。最后的横移段使用
接近 `Vx=85` 的功率并延长持续时间，解决“前进够、横移很少”的问题。

## 标准建模方向

### 1. 中心轨迹 + 配置空间

轨迹规划阶段不应该直接在真实场地边界上画车身外框，而应把机器人当作一个
点在“配置空间”里运动：障碍物和边界按机器人半尺寸膨胀，剩下的区域才允许
中心点通过。

对当前不旋转的纯平移模式，先用简单版本即可：

```text
center_x ∈ [ROBOT_WIDTH_CM / 2, field_width - ROBOT_WIDTH_CM / 2]
center_y ∈ [ROBOT_LENGTH_CM / 2, field_height - ROBOT_LENGTH_CM / 2]
```

### 2. 曲线不是多个独立位置步

曲线应保存为连续参数路径 `p(s) = (x(s), y(s))`，其中 `s` 是弧长。
然后生成带速度约束的时间轨迹：

```text
state(t) = x, y, heading, vx, vy, omega
```

### 3. 麦克纳姆底盘用标准逆运动学

机器人每个控制周期根据期望底盘速度 `(vx, vy, omega)` 计算四个轮子的目标速度。
这一步是标准 mecanum kinematics，不应该把曲线拆成一个个相对位置目标。

### 4. 机器人端需要连续路径跟随器

推荐两级实现：

1. 第一版：开环时间轨迹播放。按 20 ms 周期输出连续变化的 `(vx, vy, omega)`，
   编码器只用于标定比例和安全诊断。它能先解决“一格一格停顿”。
2. 第二版：闭环轨迹跟随。用四轮编码器和陀螺仪估计位姿，再用 holonomic
   follower 或 pure pursuit 修正误差。

## 需要修改的位置

第一阶段已修改这些文件：

- `trajectory_planner/kinematics.py`
  - 新增连续轨迹生成器，不再把四轮曲线主路径导出成一串 `enc_move`。
  - 保留 `enc_move` 给直线、标定、短距离精确动作。
- `trajectory_planner/exporter.py`
  - 支持新的 `traj_v` 序列格式。
- `mecanum_drive.py`
  - 新增连续轨迹执行器：按每个 `traj_v` 的时长持续输出四轮功率。
  - 保留旧 `enc_move` 兼容标定。
- `trajectory_planner/field_view.py`
  - 显示小车中心点可行区域。
- `trajectory_planner/config.py`
  - 明确四轮车外形尺寸、中心禁入区、安全余量。
