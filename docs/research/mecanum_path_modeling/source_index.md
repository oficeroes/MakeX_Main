# 资料索引

已下载到 `sources/` 的资料：

## 麦克纳姆运动学 / 里程计

- `wpilib_mecanum_kinematics.html`
  - 来源：https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html
  - 用途：标准 mecanum inverse kinematics，把底盘速度转换为四轮速度。
- `wpilib_mecanum_odometry.html`
  - 来源：https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-odometry.html
  - 用途：用轮编码器和陀螺仪更新机器人位姿。
- `wpilib_chassis_speeds.html`
  - 来源：https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
  - 用途：解释机器人坐标系底盘速度 `(vx, vy, omega)`。
- `modern_robotics_omni_feedback.html`
  - 来源：https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-2-of-2/
  - 用途：全向轮移动机器人反馈控制示例。

## 轨迹生成 / 曲线跟随

- `wpilib_trajectory_generation.html`
  - 来源：https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
  - 用途：路径点生成带速度和加速度约束的轨迹。
- `wpilib_trajectory_constraints.html`
  - 来源：https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/constraints.html
  - 用途：最大速度、最大加速度等约束思想。
- `roadrunner_parametric_paths.html`
  - 来源：https://rr.brott.dev/docs/v0-5/tour/parametric-paths/
  - 用途：FTC 常用的参数路径模型。
- `roadrunner_trajectories.html`
  - 来源：https://rr.brott.dev/docs/v0-5/tour/trajectories/
  - 用途：轨迹 = 路径 + 时间参数化。
- `roadrunner_path_following.html`
  - 来源：https://rr.brott.dev/docs/v1-0/guides/path-following/
  - 用途：路径跟随器的整体结构。
- `roadrunner_tuning.html`
  - 来源：https://rr.brott.dev/docs/v1-0/tuning/
  - 用途：实际车上调参流程参考。

## Pure Pursuit / 速度约束

- `nav2_regulated_pure_pursuit.html`
  - 来源：https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
  - 用途：工业 ROS 导航里的 pure pursuit 变体和速度约束思路。
- `regulated_pure_pursuit_arxiv.html`
  - 来源：https://ar5iv.labs.arxiv.org/html/2305.20026
  - 用途：Regulated Pure Pursuit 论文 HTML 版。

## 中心点禁入区 / 配置空间

- `modern_robotics_cspace_obstacles.html`
  - 来源：https://modernrobotics.northwestern.edu/nu-gm-book-resource/10-2-c-space-obstacles/
  - 用途：配置空间障碍物理论。
- `nav2_inflation_layer.html`
  - 来源：https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
  - 用途：导航中按机器人安全半径膨胀障碍物。
- `cmu_configuration_space.pdf`
  - 来源：https://www.cs.cmu.edu/~motionplanning/lecture/Chap3-Config-Space_howie.pdf
  - 用途：CMU 运动规划配置空间讲义。

