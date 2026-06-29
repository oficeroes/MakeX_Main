# 麦克纳姆轮四轮运动学（权威参考）

> **来源**：WPILib `MecanumDriveKinematics.java`、ROS2_Control `mobile_robot_kinematics.rst`、Ecam Eurobot 教程、Modern Robotics Ch.13

---

## 1. 坐标约定

### WPILib (FIRST Robotics) 坐标系 — NWU (North-West-Up)

- **X 轴**：机器人前进方向（North）
- **Y 轴**：机器人左侧方向（West）
- **Z 轴**：向上（Up）
- **ω_z 正方向**：逆时针（CCW，从上方俯视）

### ROS REP-103 坐标系

- **X 轴**：机器人前进方向
- **Y 轴**：机器人左侧方向
- **ω_z 正方向**：逆时针

### 本项目 MakeX 坐标系

- **X 轴**：机器人右侧方向
- **Y 轴**：机器人前进方向
- **ω 正方向**：顺时针（CW，从上方俯视）

> ⚠️ 不同坐标系之间转换只需交换 Vx/Vy 和取反 ω。

---

## 2. 轮子布局定义

```
         前进方向 (+Y MakeX / +X WPILib)
              ↑
       M1 ←──┼──→ M2      (左前 FL, 右前 FR)
              │
       M3 ←──┼──→ M4      (左后 RL, 右后 RR)

轮子位置（相对于机器人几何中心）:
  FL: ( x=+lx/2,  y=+ly/2 )    ← 前方左侧
  FR: ( x=+lx/2,  y=-ly/2 )    ← 前方右侧
  RL: ( x=-lx/2,  y=+ly/2 )    ← 后方左侧
  RR: ( x=-lx/2,  y=-ly/2 )    ← 后方右侧
```

### X 型 vs O 型

| 特性             | X 型               | O 型               |
| ---------------- | ------------------ | ------------------ |
| 辊子方向（俯视） | 指向中心（形成 X） | 指向外侧           |
| 横移效率         | 高（力在横向对齐） | 较低               |
| 旋转效率         | 稍低               | 高（力在切向对齐） |
| 适用场景         | 需要频繁横移       | 需要频繁旋转       |

**本项目使用 X 型。**

---

## 3. 逆运动学（Inverse Kinematics）

### 3.1 WPILib 源码中的实现（最权威）

来自 `MecanumDriveKinematics.java` 第 294-300 行：

```java
// 逆运动学矩阵（每行对应一个轮子）
// 行格式：[vx系数, vy系数, ω系数]
m_inverseKinematics.setRow(0, 0, 1, -1, -(fl.getX() + fl.getY()));  // FL
m_inverseKinematics.setRow(1, 0, 1,  1,  (fr.getX() - fr.getY()));  // FR
m_inverseKinematics.setRow(2, 0, 1,  1,  (rl.getX() - rl.getY()));  // RL
m_inverseKinematics.setRow(3, 0, 1, -1, -(rr.getX() + rr.getY()));  // RR
```

**展开为标准方程**（WPILib 坐标系：vx=前进, vy=左移, ω=CCW）：

$$\begin{bmatrix} \omega_{FL} \\ \omega_{FR} \\ \omega_{RL} \\ \omega_{RR} \end{bmatrix} = \begin{bmatrix} 1 & -1 & -(x_{FL}+y_{FL}) \\ 1 & 1 & (x_{FR}-y_{FR}) \\ 1 & 1 & (x_{RL}-y_{RL}) \\ 1 & -1 & -(x_{RR}+y_{RR}) \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

### 3.2 对称布局简化形式

当四轮关于中心对称时（$x_{FL}=l_x/2$, $y_{FL}=l_y/2$ 等）：

$$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix} = \frac{1}{r} \begin{bmatrix} 1 & -1 & -L \\ 1 & 1 & L \\ 1 & 1 & -L \\ 1 & -1 & L \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

其中 $L = (l_x + l_y) / 2$，$r$ 为轮子半径，$v_x$ 为前进速度。

### 3.3 Ecam Eurobot 形式（vx=右移, vy=前进, ωz=CCW）

$$\begin{cases} \omega_{fl} = \frac{1}{r}[v_x - v_y - (l_x+l_y)\omega_z] \\ \omega_{fr} = \frac{1}{r}[v_x + v_y + (l_x+l_y)\omega_z] \\ \omega_{rl} = \frac{1}{r}[v_x + v_y - (l_x+l_y)\omega_z] \\ \omega_{rr} = \frac{1}{r}[v_x - v_y + (l_x+l_y)\omega_z] \end{cases}$$

### 3.4 本项目使用的形式（Vx=右, Vy=前, ω=CW）

经坐标变换（$Vx = v_y^{WPILib}$, $Vy = v_x^{WPILib}$, $\omega = -\omega_z^{WPILib}$）：

$$\begin{bmatrix} M_1 \\ M_2 \\ M_3 \\ M_4 \end{bmatrix}_{raw} = \begin{bmatrix} 1 & 1 & 1 \\ -1 & 1 & -1 \\ -1 & 1 & 1 \\ 1 & 1 & -1 \end{bmatrix} \begin{bmatrix} V_x \\ V_y \\ \omega \end{bmatrix}$$

然后对反接电机（M2, M4）取反。

---

## 4. 前向运动学（Forward Kinematics）

### 4.1 WPILib 方法：Moore-Penrose 伪逆

WPILib 使用伪逆（因为 4 个方程 3 个未知数是超定的）：

```java
m_forwardKinematics = m_inverseKinematics.pseudoInverse();
```

对于对称布局，伪逆结果为：

$$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \frac{r}{4} \begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & 1 & 1 & -1 \\ -\frac{1}{L} & \frac{1}{L} & -\frac{1}{L} & \frac{1}{L} \end{bmatrix} \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix}$$

其中 $L = l_x + l_y$。

### 4.2 里程计中的应用

里程计通过编码器读取每轮转动角度 $\Delta\theta_i$，结合前向运动学推算位移：

$$\begin{bmatrix} \Delta x \\ \Delta y \\ \Delta\theta \end{bmatrix} = \frac{r}{4} \begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & 1 & 1 & -1 \\ -\frac{1}{L} & \frac{1}{L} & -\frac{1}{L} & \frac{1}{L} \end{bmatrix} \begin{bmatrix} \Delta\theta_1 \\ \Delta\theta_2 \\ \Delta\theta_3 \\ \Delta\theta_4 \end{bmatrix}$$

---

## 5. 速度饱和处理（Desaturation）

### 5.1 WPILib 方法

```java
// MecanumDriveWheelVelocities.desaturate(double maxSpeed)
// 当任一轮速超过最大值时，等比例缩放所有轮速
```

### 5.2 向量等比缩放（本项目方法）

```python
max_abs = max(abs(M1), abs(M2), abs(M3), abs(M4))
if max_abs > 100:
    scale = 100.0 / max_abs
    M1 *= scale; M2 *= scale; M3 *= scale; M4 *= scale
```

✅ **为什么不用硬截断**：硬截断会改变速度向量方向，导致机器人运动方向偏移。等比缩放保持方向不变，仅降低整体速度。

---

## 6. 反接电机的处理

当电机物理反接时（正功率 → 轮子反转），需要在运动学输出端取反：

```
M2_out = -M2_raw
M4_out = -M4_raw
```

**验证方法**：让四个轮子都正转，机器人应前进（而非旋转或横移）。

---

## 7. 关键注意事项

1. **坐标系一致性**：确保 Vx/Vy/ω 的符号约定在整个代码中统一
2. **轮子直径**：里程计精度直接依赖轮子直径的准确测量
3. **几何参数 L**：$(l_x + l_y)$ 应在实际机器人上精确测量（单位：米或编码器单位）
4. **伪逆 vs 解析解**：WPILib 使用伪逆处理非对称布局；对称布局使用解析解更高效
5. **死区**：摇杆输入应有死区（建议 5-10%），防止零漂
