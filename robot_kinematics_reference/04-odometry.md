# 里程计与航位推算（Odometry & Dead Reckoning）

> **来源**：WPILib `MecanumDriveOdometry`、ROS2_Control、Robotics StackExchange

---

## 1. 基本概念

### 1.1 Odometry vs Dead Reckoning

| 术语                           | 定义                                        |
| ------------------------------ | ------------------------------------------- |
| **Odometry（里程计）**         | 仅使用轮式编码器推算位置和姿态              |
| **Dead Reckoning（航位推算）** | 使用编码器 + 航向传感器（如陀螺仪）推算位置 |

### 1.2 工作原理

```
编码器读数 → 各轮转动角度 → 前向运动学 → 机体位移
→ 坐标变换至世界系 → 积分更新位姿
```

---

## 2. 麦克纳姆轮里程计

### 2.1 WPILib 实现（最权威）

WPILib 的前向运动学使用 Moore-Penrose 伪逆：

```java
// 每周期（通常 20ms）读取一次编码器增量
// 计算各轮位移（米）：
//   distance = (angle_delta / 360) * wheel_circumference

// 前向运动学 → 机体位移
Twist2d twist = kinematics.toTwist2d(wheelDeltas);

// 更新全局位姿
pose = pose.exp(twist);  // SE(2) 指数映射
```

### 2.2 简化实现（适用于本项目）

```python
# 每周期读取编码器角度变化
d_angle_1 = motor_M1.get_value("angle") - prev_angle_1
d_angle_2 = motor_M2.get_value("angle") - prev_angle_2
d_angle_3 = motor_M3.get_value("angle") - prev_angle_3
d_angle_4 = motor_M4.get_value("angle") - prev_angle_4

# 换算为轮子转动弧度（编码器单位：度）
wheel_angle_1 = math.radians(d_angle_1)  # 注意反接取反
...

# 前向运动学（使用简化公式）
L = lx + ly  # 几何参数
r = wheel_radius  # 轮子半径

dx_body = (wheel_angle_1 + wheel_angle_2 + wheel_angle_3 + wheel_angle_4) * r / 4
dy_body = (-wheel_angle_1 + wheel_angle_2 + wheel_angle_3 - wheel_angle_4) * r / 4
dtheta  = (-wheel_angle_1 + wheel_angle_2 - wheel_angle_3 + wheel_angle_4) * r / (4 * L)

# 坐标变换（机体 → 世界）
dx_world = dx_body * math.cos(theta) - dy_body * math.sin(theta)
dy_world = dx_body * math.sin(theta) + dy_body * math.cos(theta)

# 更新位姿
x += dx_world
y += dy_world
theta += dtheta
```

### 2.3 三轮全向里程计

对于三轮 120° 布局，使用前向运动学伪逆：

$$\begin{bmatrix} \Delta x \\ \Delta y \\ \Delta\theta \end{bmatrix} = r \cdot A^\dagger_{3\times3} \begin{bmatrix} \Delta\theta_1 \\ \Delta\theta_2 \\ \Delta\theta_3 \end{bmatrix}$$

三轮系统的优势：恰好确定（3 方程 3 未知数），不需要伪逆。

---

## 3. 坐标变换

### 3.1 机体 → 世界坐标变换

$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_{world} = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}_{body}$$

### 3.2 离散积分（Euler 法）

```python
x += vx_body * cos(theta) * dt - vy_body * sin(theta) * dt
y += vx_body * sin(theta) * dt + vy_body * cos(theta) * dt
theta += omega * dt
```

### 3.3 更高精度的积分方法

**Runge-Kutta 4 阶**（RK4）可减少积分误差：

```python
def rk4_step(x, y, theta, vx, vy, omega, dt):
    # 4 个子步，加权平均
    k1_vx = vx * cos(theta) - vy * sin(theta)
    ...
    # 通常不需要，Euler 法在 50Hz 足够
```

> ⚠️ 在 50Hz 以上的更新频率下，Euler 法的积分误差通常小于轮子打滑带来的误差。

---

## 4. IMU 增强里程计

### 4.1 为什么需要 IMU

- 轮式里程计的 $\theta$（航向角）在打滑时会发散
- IMU 陀螺仪直接测量 $\omega$，不受打滑影响
- 但 IMU 有零偏漂移（积分 1 分钟后可能漂移数度）

### 4.2 互补滤波融合

```python
# 融合陀螺仪和编码器的航向角估计
alpha = 0.98  # 陀螺仪权重（实验调参）

# 陀螺仪角速度积分
gyro_theta = prev_theta + gyro_rate * dt

# 编码器角速度
encoder_theta = prev_theta + encoder_omega * dt

# 互补融合
theta = alpha * gyro_theta + (1 - alpha) * encoder_theta
```

### 4.3 何时信任哪个传感器

| 场景       | 信任编码器     | 信任 IMU       |
| ---------- | -------------- | -------------- |
| 匀速直行   | ✅             | -              |
| 急加速     | ❌（可能打滑） | ✅             |
| 被推挤     | ❌（被动滑动） | ✅             |
| 长时间静止 | ✅             | ❌（零偏漂移） |

---

## 5. 里程计漂移管理

### 5.1 漂移特征

- 轮式里程计：距离误差 ~1-5%，角度误差 ~0.5-2°/m
- 纯里程计走 10m 后，位置误差可达 ~50cm
- 航向角误差是位置误差的主要来源

### 5.2 减轻漂移的方法

1. **精确校准**：轮径、轮距的准确测量（见 `03-error-compensation.md`）
2. **IMU 融合**：用陀螺仪稳定航向角
3. **外部校正**：已知位置标记（如 AprilTag、红外信标）
4. **零速更新（ZUPT）**：静止时重置速度积分
5. **高频更新**：>50Hz 的里程计更新率减少积分误差

---

## 6. WPILib 里程计类的完整架构

```
MecanumDriveOdometry
├── MecanumDriveKinematics  (运动学模型)
│   ├── toWheelSpeeds()      (逆运动学：底盘→轮速)
│   ├── toChassisSpeeds()    (前向运动学：轮速→底盘)
│   └── toTwist2d()          (轮位移→机体位移)
├── Rotation2d               (航向角，来自陀螺仪)
└── Pose2d                   (位姿累积)
    └── update(Rotation2d, MecanumDriveWheelPositions)
        → 返回最新 Pose2d
```

### 关键设计模式

- **解耦运动学与里程计**：`Kinematics` 只做速度/位移换算，`Odometry` 负责积分和位姿管理
- **陀螺仪航向角**：里程计直接使用陀螺仪角度（而非编码器反算的航向），避免打滑误差
- **SE(2) 指数映射**：`Pose2d.exp(Twist2d)` 用于位姿更新，数学上比 Euler 积分更准确
