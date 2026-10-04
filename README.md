# MakeX 机器人控制与路径规划

本仓库保存三轮全向及四轮麦克纳姆底盘的 Novapi 控制程序、两套桌面规划工具，以及调试记录。现有仓库由莫欣睿维护和整理，开发过程中使用了 AI 辅助工具。项目按实际硬件和赛场任务逐步迭代；仓库中的源码与文档可供阅读，但不能据此认定所有功能都完成了实车验证。

## 从哪里开始

| 内容 | 入口 | 说明 |
| --- | --- | --- |
| 三轮全向底盘程序 | [`mecanum_forward.py`](mecanum_forward.py) | Novapi 机器人端控制程序，含遥控、自动动作与执行机构逻辑。 |
| 四轮麦克纳姆底盘程序 | [`mecanum_drive.py`](mecanum_drive.py) | Novapi 机器人端控制程序，含四轮运动分配、遥控及自动动作。 |
| 轨迹规划工具 | [`trajectory_planner/`](trajectory_planner/) | PyQt5 桌面界面，可绘制、保存和导出轨迹。 |
| 步骤规划工具 | [`step_planner/`](step_planner/) | PyQt5 桌面界面，以步骤组织机器人动作。 |
| 系统说明 | [`docs/README.md`](docs/README.md) | 坐标系、参数标定、模块索引、操作与检查流程。 |
| 硬件结构与调试 | [`docs/`](docs/) | 物理结构、算法解释、故障排查与分阶段调试记录。 |

## 在电脑上查看规划工具

规划工具需要 Python 和 PyQt5。在仓库根目录运行：

```powershell
python -m pip install PyQt5
python -m trajectory_planner.main
```

步骤规划器的入口为：

```powershell
python -m step_planner.main
```

下图为从当前源码在 Windows 上启动的轨迹规划器界面；它只证明桌面界面可启动，不代表轨迹已在实车完成验证。

![轨迹规划器桌面界面](docs/images/trajectory-planner.png)

机器人端程序依赖 Novapi、mBuild 设备和对应硬件连接，普通电脑上不能直接运行。电机端口、底盘尺寸、编码器比例等参数与具体硬件有关；在真实设备上使用前，需按 [`docs/README.md`](docs/README.md) 和 [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) 核对接线、标定并逐项测试。不要直接将仓库内的赛场参数套用到其他机器人。

## 目前的证据与限制

- 仓库保存了控制程序、规划器源码、轨迹数据与测试文件，可检查实现过程。
- [`docs/lessons-learned/`](docs/lessons-learned/) 分开记录了实车调试问题和静态检查结果；其中明确写为“未经实车”的功能，仍以待验证处理。
- 仓库已收录当前源码的规划器桌面界面截图，尚无完整实车演示视频。实物照片如需与代码建立对应关系，应补充具体机器人型号、程序版本和测试日期。
- 这是持续迭代中的项目。历史备份、测试样本和赛场文件保留作过程记录，不宜把文件数量解读为已完成的功能数量。

## 文档

建议先读 [`docs/README.md`](docs/README.md)，再根据需要查看 [`docs/mecanum_drive_算法详解.md`](docs/mecanum_drive_算法详解.md)、[`docs/EXTENDING_FOR_NEW_CHASSIS.md`](docs/EXTENDING_FOR_NEW_CHASSIS.md) 和 [`docs/lessons-learned/`](docs/lessons-learned/)。
