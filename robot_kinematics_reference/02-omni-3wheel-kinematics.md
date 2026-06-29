# 三轮全向运动学（权威参考）

> **来源**：ROS2_Control `mobile_robot_kinematics.rst`、Modern Robotics Ch.13.2、ResearchGate 论文

---

## 1. ROS2_Control 通用 N 轮全向公式

这是 ROS2 生态中的**最标准**实现，适用于 3 轮及以上任意数量的全向轮。

### 1.1 坐标约定（ROS REP-103）

```
        x_b (前进方向)
         ↑
    W1 ←─┼─→ W2
         │
        W3     (n=3 的情况)
```

- $x_b, y_b$：机器人机体坐标系
- $v_{b,x}$：前进速度
- $v_{b,y}$：左移速度（ROS 中 Y 轴指向左）
- $\omega_{b,z}$：角速度（CCW 为正）
- $R$：机器人半径（中心到轮子距离）
- $\gamma$：第一个轮子的角度偏移（相对于 $x_b$ 轴）
- $\theta = 2\pi / n$：相邻轮子间角度
- $r$：轮子半径

### 1.2 通用逆运动学矩阵

对于 $n$ 个全向轮：

$$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \vdots \\ \omega_n \end{bmatrix} = \frac{1}{r} \begin{bmatrix} \sin(\gamma) & -\cos(\gamma) & -R \\ \sin(\theta+\gamma) & -\cos(\theta+\gamma) & -R \\ \sin(2\theta+\gamma) & -\cos(2\theta+\gamma) & -R \\ \vdots & \vdots & \vdots \\ \sin((n-1)\theta+\gamma) & -\cos((n-1)\theta+\gamma) & -R \end{bmatrix} \begin{bmatrix} v_{b,x} \\ v_{b,y} \\ \omega_{b,z} \end{bmatrix}$$

### 1.3 单个轮子的代数形式

$$\omega_i = \frac{1}{r}\left[\sin((i-1)\theta+\gamma) \cdot v_{b,x} - \cos((i-1)\theta+\gamma) \cdot v_{b,y} - R \cdot \omega_{b,z}\right]$$

### 1.4 前向运动学（伪逆）

$$\begin{bmatrix} v_{b,x} \\ v_{b,y} \\ \omega_{b,z} \end{bmatrix} = r \cdot A^\dagger \begin{bmatrix} \omega_1 \\ \omega_2 \\ \vdots \\ \omega_n \end{bmatrix}$$

其中 $A^\dagger$ 是逆运动学矩阵 $A$ 的 Moore-Penrose 伪逆。

---

## 2. 三轮 120° 对称布局（本项目）

### 2.1 布局参数

```
         前进方向 (+Vy MakeX)
              ↑
       M1 ←──┼──→ M2      (前左 M1, 前右 M2)
              │
              │
             M3            (尾部 M3)
```

- $\gamma = 90°$（第一轮 M1 在 y 轴正上方）
- $\theta = 120° = 2\pi/3$
- 轮子角度：M1=90°, M2=210°（即-150°）, M3=330°（即-30°）

### 2.2 简化方程（本项目坐标系：Vx=右, Vy=前, ω=CW）

经过 ROS → MakeX 坐标变换（vx→-Vy, vy→Vx, ωz→-ω）：

$$\begin{bmatrix} M_1 \\ M_2 \\ M_3 \end{bmatrix} = \begin{bmatrix} -\frac{1}{2} & -\frac{\sqrt{3}}{2} & -1 \\ -\frac{1}{2} & \frac{\sqrt{3}}{2} & -1 \\ 1 & 0 & -1 \end{bmatrix} \begin{bmatrix} V_x \\ V_y \\ \omega \end{bmatrix}$$

即：

$$\begin{aligned} M_1 &= -0.5 V_x - 0.866 V_y - \omega \\ M_2 &= -0.5 V_x + 0.866 V_y - \omega \\ M_3 &= 1.0 V_x - \omega \end{aligned}$$

### 2.3 向量缩放

同麦克纳姆轮：当任一 |M_i| > 100 时，等比例缩放。

---

## 3. 正面切换（坐标系旋转）

三轮全向机器人有 3 个可选的"正面"（三角形三条边），切换正面需要将摇杆速度向量旋转。

### 3.1 旋转矩阵

$$\begin{bmatrix} V_x' \\ V_y' \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} V_x \\ V_y \end{bmatrix}$$

三个正面对应的旋转角：

- Face 0（M1-M2 正面）：0°
- Face 1（M2-M3 正面）：+120°
- Face 2（M3-M1 正面）：-120°

---

## 4. WPILib 对三轮全向的处理

WPILib 没有专门的 3 轮全向类，但通用公式在 `Kinematics` 基类中通过配置轮子位置实现。如果使用 WPILib，可以将 3 个全向轮作为 3 个 Swerve Module（每个仅有驱动无转向）来建模。

---

## 5. 与麦克纳姆轮的对比

| 特性     | 三轮全向                 | 四轮麦克纳姆                 |
| -------- | ------------------------ | ---------------------------- |
| 轮子数   | 3                        | 4                            |
| 超定性   | 恰好确定（3方程3未知数） | 超定（4方程3未知数，需伪逆） |
| 稳定性   | 3 点支撑，可能翻倒       | 4 点支撑，更稳定             |
| 横移效率 | 高                       | 高（但有力损失在辊子上）     |
| 力分布   | 不对称                   | 对称                         |
| 适合场景 | 轻型、竞速               | 重型、需要更多牵引力         |
