# 拟重构方案

此文件是待确认方案，不代表已经开始修改核心代码。

## 阶段 A：先止血，解决“一格一格”（已执行）

目标：曲线能连续跑，不再每段位置模式停顿。

1. 已新增导出格式：

```python
('traj_v', duration_s, vx_power, vy_power, omega_power)
...
('traj_stop', 0, 0, 0)
```

2. GUI 仍按当前曲线平滑结果生成中心线，但不再对四轮曲线输出 `enc_move`。
3. 机器人端新增 `traj_v` 执行器，持续 set_power，不进入位置模式。
4. 保留 `enc_move` 给编码器标定和直线测试。

风险：开环播放会受地面、轮胎、负载影响，精度不如闭环，但能最快验证数学方向。

## 阶段 B：引入标准轨迹模型

目标：把曲线从“点列”升级成“路径 + 时间”。

1. `PathModel`
   - 输入 raw/smoothed points。
   - 输出弧长参数化路径 `p(s)`。
   - 提供切线、曲率、总长度。
2. `TrajectoryModel`
   - 输入 `PathModel` 和最大速度/加速度。
   - 输出定时状态 `state(t)`。
3. `MecanumModel`
   - 输入底盘速度 `(vx, vy, omega)`。
   - 输出四轮速度或功率比例。

## 阶段 C：加入中心禁入区

目标：用户画线时就知道哪里车中心不能去。

1. 场地边界内缩：

```text
left/right margin = ROBOT_WIDTH_CM / 2 + safety_margin
front/back margin = ROBOT_LENGTH_CM / 2 + safety_margin
```

2. 障碍物按机器人半尺寸膨胀。
3. GUI 用半透明区域提示“中心不可进入”。
4. 导出前检查路径是否穿越禁入区。

## 阶段 D：闭环跟随

目标：真正解决误差来源，而不只是播放速度。

1. 机器人端维护 `pose = x, y, heading`。
2. 用四轮编码器增量 + 陀螺仪更新 odometry。
3. 每 20 ms 找轨迹目标点。
4. 用 holonomic PID / pure pursuit 计算修正速度。
5. 输出连续轮速。

硬件前提：最好能稳定读陀螺仪 yaw；如果没有 yaw，旋转和横移误差会难以长期闭环。
