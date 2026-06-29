# 参考来源

> 以下所有来源均为 2026 年 6 月可访问的最新版本。

---

## 权威开源项目

### WPILib (FIRST Robotics Competition)

- **MecanumDriveKinematics 源码**：https://github.wpilib.org/allwpilib/docs/2027/java/src-html/org/wpilib/math/kinematics/MecanumDriveKinematics.html
  - 最广泛使用的竞赛机器人运动学库
  - 使用 Moore-Penrose 伪逆处理超定系统
  - 支持可变旋转中心
- **Mecanum Drive Kinematics 文档**：https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html
  - 完整的 API 使用教程
  - 包括场地定向驱动（Field-Oriented Drive）

### ROS2_Control

- **Wheeled Mobile Robot Kinematics**：https://control.ros.org/jazzy/doc/ros2_controllers/doc/mobile_robot_kinematics.html
  - ROS2 生态的工业标准运动学实现
  - 通用 N 轮全向公式（支持任意数量全向轮）
  - 包括差速、阿克曼、Swerve 等多种底盘类型

### Modern Robotics (Lynch & Park)

- **Chapter 13.2 Omnidirectional Wheeled Mobile Robots**：https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/
  - 机器人学权威教材
  - 视频讲解麦克纳姆轮和全向轮的运动学推导
- **Chapter 13.4 Odometry**：https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-4-odometry/
  - 里程计的数学基础

---

## 实用教程

### Ecam Eurobot

- **Mecanum Wheels Tutorial**：https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
  - 包含完整的正逆运动学示例
  - 有具体数值计算案例
  - 基于论文：_Kinematic Model of a Four Mecanum Wheeled Mobile Robot_ (IJCA)

---

## 学术论文

### 运动学建模

1. **A General Inverse Kinematic Formulation and Control Schemes for Omnidirectional Mobile Robots**
   - 来源：Engineering Letters
   - 链接：https://www.engineeringletters.com/issues_v29/issue_4/EL_29_4_06.pdf
   - 提供通用的全向/麦克纳姆轮运动学建模

2. **Kinematic Model of a Four Mecanum Wheeled Mobile Robot**
   - 来源：International Journal of Computer Applications (IJCA)
   - 链接：https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

3. **Dynamics of a Four-Wheeled Mobile Robot with Mecanum Wheels**
   - 来源：DB Thüringen
   - 链接：https://www.db-thueringen.de/servlets/MCRFileNodeServlet/dbt_derivate_00049732/1521-4001_99_2019_12_e201900173.pdf
   - 包括动力学和非完整约束分析

### 误差补偿

4. **Reduction of Odometry Errors in Over-constrained Mobile Robots**
   - 来源：ResearchGate
   - 链接：https://www.researchgate.net/publication/2834717_Reduction_of_Odometry_Errors_in_Over-constrained_Mobile_Robots
   - 提出 Cross-Coupled Control 方法

5. **Learning Wheel Odometry and IMU Errors for Localization**
   - 来源：ResearchGate
   - 链接：https://www.researchgate.net/publication/335139228_Learning_Wheel_Odometry_and_IMU_Errors_for_Localization
   - 使用 EKF 融合编码器和 IMU

---

## 社区资源

### Robotics StackExchange

- **Odometry vs Dead-reckoning**：https://robotics.stackexchange.com/questions/7287/odometry-vs-dead-reckoning
- **How to do odometry for 4 mecanum wheeled robot?**：https://robotics.stackexchange.com/questions/21239/how-to-do-odometry-for-4-mecanum-wheeled-robot

### CEVA MotionEngine Scout

- **IMU-Enhanced Wheel Odometry**：https://daischsensor.com/imu-enhanced-wheel-odometry-robotics-navigation-technology
  - 商业级传感器融合方案参考
  - 编码器 + IMU + 光流传感器的三合一融合

### Multi-Sensor Fusion Paper

- **Multi-sensor fusion based wheeled robot research on indoor localization**
  - 来源：ScienceDirect
  - 链接：https://www.sciencedirect.com/science/article/pii/S2590123024005231
  - 编码器 + IMU + LiDAR 的完整融合方案

### Chegg (教学资源)

- **Three Wheeled Omnidirectional Robot Inverse Kinematics**：https://www.chegg.com/homework-help/questions-and-answers/3-three-wheeled-omnidirectional-robot-shown-v-v-90-figure-x-y-represent-robot-s-position-x-q45117560
  - 三轮全向标准坐标系下的一般形式

---

## 本项目内的参考实现

| 文件                 | 说明                                                         |
| -------------------- | ------------------------------------------------------------ |
| `mecanum_forward.py` | 三轮全向机器人代码（含 S 曲线加速 + PID 同步补偿的标定模式） |
| `mecanum_drive.py`   | 四轮麦克纳姆机器人代码（含基础运动学 + 向量缩放）            |

---

## 推荐阅读顺序（供 AI 学习）

1. **ROS2_Control 页面**（了解通用 N 轮公式）→ 15 分钟
2. **WPILib MecanumDriveKinematics 源码**（看 setInverseKinematics 方法）→ 5 分钟
3. **Ecam Eurobot 教程**（看具体数值案例）→ 10 分钟
4. **Modern Robotics Ch.13 视频**（理解推导过程）→ 20 分钟
5. **ResearchGate 误差补偿论文**（了解 Cross-Coupled Control）→ 15 分钟
