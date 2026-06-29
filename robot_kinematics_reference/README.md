# 机器人运动学权威参考知识库

> **用途**：供 AI 和人类开发者快速查阅最标准的全向轮/麦克纳姆轮运动学方程、误差补偿方法和里程计实现。
> **最后更新**：2026-06-29
> **来源**：ROS2_Control、WPILib (FIRST Robotics)、Modern Robotics (Lynch & Park)、Ecam Eurobot、多篇学术论文

---

## 目录

| 文件                                                           | 内容                                                              |
| -------------------------------------------------------------- | ----------------------------------------------------------------- |
| [01-mecanum-kinematics.md](./01-mecanum-kinematics.md)         | 麦克纳姆轮四轮运动学（X型/O型），正逆运动学，WPILib/ROS2 标准实现 |
| [02-omni-3wheel-kinematics.md](./02-omni-3wheel-kinematics.md) | 三轮全向 120° 对称布局运动学，通用 N 轮公式                       |
| [03-error-compensation.md](./03-error-compensation.md)         | 误差来源分析、编码器校准、PID同步补偿、IMU融合                    |
| [04-odometry.md](./04-odometry.md)                             | 里程计（Dead Reckoning）、前向运动学积分、多传感器融合            |
| [05-sources.md](./05-sources.md)                               | 所有参考来源的超链接和简要说明                                    |

---

## 核心公式速查

### 麦克纳姆轮（X 型布局，四轮）

**逆运动学**（底盘速度 → 轮速）：

$$\begin{bmatrix} \omega_{FL} \\ \omega_{FR} \\ \omega_{RL} \\ \omega_{RR} \end{bmatrix} = \frac{1}{r} \begin{bmatrix} 1 & -1 & -(l_x+l_y) \\ 1 & 1 & (l_x+l_y) \\ 1 & 1 & -(l_x+l_y) \\ 1 & -1 & (l_x+l_y) \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

**前向运动学**（轮速 → 底盘速度），使用 Moore-Penrose 伪逆：

$$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \frac{r}{4} \begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & 1 & 1 & -1 \\ -\frac{1}{l_x+l_y} & \frac{1}{l_x+l_y} & -\frac{1}{l_x+l_y} & \frac{1}{l_x+l_y} \end{bmatrix} \begin{bmatrix} \omega_{FL} \\ \omega_{FR} \\ \omega_{RL} \\ \omega_{RR} \end{bmatrix}$$

### 三轮全向（120° 对称布局）

**逆运动学**（ROS2_Control 标准，γ=0 即第一轮在 x 轴）：

$$\omega_i = \frac{1}{r}\left[\sin(\theta_i) \cdot v_x - \cos(\theta_i) \cdot v_y - R \cdot \omega_z\right]$$

$$\theta_i = (i-1) \cdot \frac{2\pi}{3}, \quad i = 1,2,3$$
