# 机器人误差补偿方法

> **来源**：ResearchGate 论文、ROS2_Control、WPILib、CEVA MotionEngine Scout、Robotics StackExchange

---

## 1. 误差来源分类

### 1.1 系统性误差（可校准）

| 误差源               | 原因                         | 影响                      |
| -------------------- | ---------------------------- | ------------------------- |
| 轮径不准确           | 测量误差、轮胎磨损、气压变化 | 直线距离累积误差（~1-5%） |
| 轮距/轴距偏差        | 实际装配与设计值不符         | 旋转角度累积误差          |
| 编码器分辨率         | 编码器精度有限               | 量化误差                  |
| 电机非线性           | 不同电机响应特性不同         | 各轮速度不匹配            |
| 齿轮间隙（Backlash） | 传动系统间隙                 | 换向时短暂失控            |

### 1.2 非系统性误差（不可完全消除）

| 误差源                 | 原因               | 影响                 |
| ---------------------- | ------------------ | -------------------- |
| 轮子打滑（Wheel Slip） | 加速过猛、地面光滑 | 里程计漂移、方向偏移 |
| 地面不平               | 颠簸、斜坡         | 轮子离地空转         |
| 外力干扰               | 碰撞、推挤         | 位置突变             |
| 温度变化               | 轮胎膨胀、润滑变化 | 参数漂移             |

---

## 2. 系统性误差的校准方法

### 2.1 UMBmark 方法（双向正方形路径测试）

最经典的差分/全向机器人校准方法（Borenstein & Feng, 1996）：

1. 让机器人沿 4m × 4m 正方形路径行驶（顺时针和逆时针各 5 次）
2. 测量终点与起点的位置偏差
3. 根据偏差反算轮径修正系数和轮距修正系数

对于全向/麦克纳姆机器人，改编方法：

- 前进 N 米，记录编码器计数 → 校准轮径
- 原地旋转 N 圈，记录编码器计数 → 校准几何参数 L

### 2.2 本项目中的编码器校准

三轮车已有标定模式：

```python
CAL_DEG_PER_CM = 1000 / 58   # 编码 1000° = 直走 58 cm
CAL_TARGET_CM = 20           # 目标行走距离
CAL_ANGLE = int(CAL_TARGET_CM * CAL_DEG_PER_CM)  # 换算为编码角度
```

建议扩展到四轮麦克纳姆：

- **前进校准**：四轮同时等速前进，记录编码器增量 vs 实际位移
- **横移校准**：四轮横移，同样记录（可能与前进行程不同，因为辊子滑动）
- **旋转校准**：原地旋转，记录编码器增量 vs 实际转角

### 2.3 编码器分辨率利用

```python
# 读取编码器角度（高精度）
angle = motor.get_value("angle")  # 返回编码器累计角度（度）

# 位移换算
wheel_circumference = math.pi * wheel_diameter  # 轮子周长
distance = (angle / 360.0) * wheel_circumference  # 行驶距离
```

---

## 3. 电机同步补偿

### 3.1 Cross-Coupled Control（交叉耦合控制）

**核心思想**：不仅要每个电机跟踪自己的目标，还要让所有电机的**进度保持一致**。

```python
# 伪代码
progress_M1 = abs(M1_angle - M1_target) / abs(M1_target)  # 0→1
progress_M2 = abs(M2_angle - M2_target) / abs(M2_target)

error = progress_M1 - progress_M2  # M1 超前则 >0

M1_speed = base_speed - KP_sync * error  # 减速 M1
M2_speed = base_speed + KP_sync * error  # 加速 M2
```

**效果**：消除各轮进度不一致导致的路径弯曲，在长距离直行时效果显著。

### 3.2 本项目三轮车的 S 曲线 + PID 实现

```python
# S 曲线加速（sin 加速 + cos 减速）
if progress < CAL_RAMP_UP:
    factor = 0.15 + 0.85 * math.sin(progress / CAL_RAMP_UP * math.pi / 2.0)
elif progress < 1.0 - CAL_RAMP_DOWN:
    factor = 1.0
else:
    p_dec = (progress - (1.0 - CAL_RAMP_DOWN)) / CAL_RAMP_DOWN
    factor = 0.15 + 0.85 * math.cos(p_dec * math.pi / 2.0)

# 左右 PID 补偿
error = progress_M1 - progress_M2
correction = CAL_PID_KP * error * CAL_MOVE_SPEED
M1_speed = base_speed - correction
M2_speed = base_speed + correction
```

**优点**：同时解决启动冲击（S 曲线）和左右不同步（PID）。

### 3.3 死区补偿

电机在低功率时有"死区"——功率太小不足以克服静摩擦。补偿方法：

```python
# 最低速度兜底
if abs(power) < MIN_POWER and abs(power) > 0:
    power = MIN_POWER * sign(power)  # 强制给最小有效功率
```

---

## 4. 传感器融合

### 4.1 IMU + 编码器融合

| 传感器        | 优势                                 | 劣势                   |
| ------------- | ------------------------------------ | ---------------------- |
| 编码器        | 高频（100-1000Hz）、低延迟、直接测量 | 累积漂移、打滑时失效   |
| IMU（陀螺仪） | 直接测量角速度/加速度、不受打滑影响  | 零偏漂移（积分后发散） |

**互补融合**：

- 编码器数据补偿 IMU 的长期漂移
- IMU 数据补偿编码器的打滑误差
- 使用互补滤波器或 Kalman 滤波器融合

### 4.2 扩展 Kalman 滤波（EKF）

$$\begin{aligned} \text{预测步} &: \hat{x}_{k|k-1} = f(\hat{x}_{k-1}, u_k) \\ \text{更新步} &: \hat{x}_k = \hat{x}_{k|k-1} + K_k(z_k - h(\hat{x}_{k|k-1})) \end{aligned}$$

- 状态向量：$[x, y, \theta, v_x, v_y, \omega]$
- 控制输入：编码器测得的各轮转速
- 观测值：IMU 角速度 + 加速度

### 4.3 实用简化方案（低算力平台）

对于 Novapi 等性能有限的平台，使用**互补滤波器**代替 EKF：

```python
# 互补滤波：陀螺仪高频准但漂移，编码器低频准
alpha = 0.98  # 陀螺仪权重
angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * encoder_angle
```

---

## 5. 轮子打滑检测与处理

### 5.1 打滑检测方法

1. **加速度异常检测**：编码器推算的加速度突然超过物理极限 → 打滑
2. **四轮速度不一致**：运动学上不可能的轮速组合 → 某轮打滑
3. **IMU 与实际转速不匹配**：陀螺仪 yaw rate 与编码器推算的 yaw rate 差异大 → 打滑

### 5.2 打滑处理策略

```python
# 方法 1: 降功率
if detected_slip:
    power *= 0.5  # 立即减半

# 方法 2: 信任 IMU（用于里程计）
if abs(gyro_rate - encoder_rate) > THRESHOLD:
    # 打滑中，轮式里程计不可信，使用 IMU 推算
    use_imu_for_odometry = True

# 方法 3: S 曲线加速（预防打滑）
# 让加速度始终在物理极限以内
```

---

## 6. 建议的实施优先级

| 优先级 | 方法                     | 难度 | 效果               |
| ------ | ------------------------ | ---- | ------------------ |
| ⭐⭐⭐ | 向量等比缩放（防硬截断） | 低   | 防止方向畸变       |
| ⭐⭐⭐ | 轮径/轮距精确校准        | 低   | 大幅提高里程计精度 |
| ⭐⭐⭐ | S 曲线加速               | 低   | 防止起步打滑       |
| ⭐⭐   | Cross-Coupled PID 同步   | 中   | 长距离直行不再弯曲 |
| ⭐⭐   | 编码器最低功率兜底       | 低   | 消除低速抖动       |
| ⭐     | IMU 互补滤波             | 中   | 需额外硬件         |
| ⭐     | EKF 多传感器融合         | 高   | 需较强算力         |
