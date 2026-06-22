# 第 1 课：开发环境搭建与 Hello World

> **学习目标：** 搭建开发环境，编写并运行你的第一个 Novapi 程序，理解代码上传和执行的流程。

---

## 1. 硬件准备清单

在开始编程之前，确保你手上有以下硬件：

| 物品                          | 数量 | 用途             |
| ----------------------------- | ---- | ---------------- |
| **Novapi 主控板**             | 1    | 核心控制器       |
| **USB 数据线**                | 1    | 连接电脑与主控板 |
| **电脑**（Windows/Mac/Linux） | 1    | 编写和上传代码   |

> 💡 第一课不需要外接任何模块，我们只用板载功能来验证环境。

---

## 2. 开发环境搭建（三选一）

### 方式 A：mBlock 5 图形化 IDE（推荐新手）

mBlock 5 是 Makeblock 官方编程软件，同时支持积木拖拽和 Python 代码编辑。

**步骤：**

1. 访问 [https://www.mblock.cc/](https://www.mblock.cc/) 下载 mBlock 5
2. 安装并打开软件
3. 用 USB 线连接 Novapi 主控板到电脑
4. 点击软件中的 **"添加设备"**，选择你的设备
5. 确保顶栏模式切换为 **"上传模式"**
6. 点击右上角 **切换代码模式** → 选择 **"Python"**

```
┌─────────────────────────────────────────────┐
│  mBlock 5 界面关键区域                         │
│                                               │
│  [上传模式] ←── 务必切换到上传模式              │
│  [Python  ▼] ←── 切换到 Python 代码模式        │
│                                               │
│  ┌─────────────────────────────────┐          │
│  │                                 │          │
│  │   在这里编写 Python 代码          │          │
│  │                                 │          │
│  └─────────────────────────────────┘          │
│                                               │
│  [ 上传到设备 ] ←── 点击上传                    │
└─────────────────────────────────────────────┘
```

---

### 方式 B：firefly_upload 命令行（进阶）

适合习惯命令行的开发者，支持批量上传和自定义类库。

**第一步：安装依赖**

```bash
pip install pyserial
pip install progressbar2
```

**第二步：下载上传脚本**

```bash
git clone https://github.com/YanMinge/firefly_upload.git
cd firefly_upload
```

**第三步：确定串口号**

- **Windows：** 在设备管理器中查看 `端口 (COM 和 LPT)`，通常为 `COM3` ~ `COM8`
- **Mac：** 在终端运行 `ls /dev/tty.*`，通常为 `/dev/tty.usbserial-xxxx`
- **Linux：** 在终端运行 `ls /dev/ttyUSB*`，通常为 `/dev/ttyUSB0`

**第四步：上传代码**

```bash
python firefly_upload.py -p COM5 -i main.py -o /flash/main.py
#                       ↑串口号      ↑本地文件      ↑板子上的路径
```

> ⚠️ **重要：** 文件名必须是 `main.py`（或 `main.mpy`）并放在 `/flash/` 目录下，主控板上电后才会自动执行。

---

### 方式 C：mpy-cross 编译 + 手动上传（高级）

将 Python 编译为 `.mpy` 二进制文件以保护源码和提高加载速度。

```bash
# 1. 进入 mpy-cross 目录并编译工具
cd micropython/mpy-cross
make

# 2. 编译你的 Python 文件
./mpy-cross main.py
# 生成 main.mpy

# 3. 用 firefly_upload 上传 .mpy 文件
python firefly_upload.py -p COM5 -i main.mpy -o /flash/main.mpy
```

---

## 3. 你的第一个程序：Hello World

### 3.1 在 mBlock 5 中编写

将以下代码粘贴到 mBlock 5 的代码编辑区：

```python
# 我的第一个 Novapi 程序
import novapi
import time

print("Hello, Novapi!")
print("系统启动完成！")

counter = 0

while True:
    counter = counter + 1
    print("运行第", counter, "秒")
    time.sleep(1)
```

点击 **"上传到设备"** 按钮，等待上传完成后打开 **串口监视器** 查看输出。

### 3.2 作为 main.py 文件上传

如果用命令行方式，将上述代码保存为 `main.py`，然后上传：

```bash
python firefly_upload.py -p [你的串口号] -i main.py -o /flash/main.py
```

上传完成后，按一下主控板的 **复位键（Reset）**，程序开始运行。通过串口工具（如 PuTTY、串口助手）连接可以看到 `print` 输出。

---

## 4. 理解代码执行流程

Novapi 上电后的启动流程：

```
上电 / 复位
    │
    ▼
加载 MicroPython 固件
    │
    ▼
查找 /flash/main.py（或 main.mpy）
    │
    ├── 找到 → 自动执行 main.py
    │            │
    │            ▼
    │      import novapi（加载板载驱动）
    │      import time   （加载时间库）
    │            │
    │            ▼
    │      执行顶层代码（不在函数中的代码）
    │            │
    │            ▼
    │      while True: 主循环（永不退出）
    │
    └── 未找到 → 进入 REPL 交互模式（等待用户输入命令）
```

> 💡 **核心要点：** MicroPython 会从上到下执行 `main.py` 中的每一行代码。`while True:` 是一个无限循环，机器人程序通常在这个循环中不断读取传感器并控制电机。

---

## 5. 验证板载资源

用以下程序验证板载陀螺仪是否正常工作：

```python
import novapi
import time

print("=== Novapi 板载传感器测试 ===")

# 测试计时器
novapi.reset_timer()
print("计时器已复位")

while True:
    # 读取姿态角
    pitch = novapi.get_pitch()
    roll = novapi.get_roll()

    # 读取系统时间
    t = novapi.timer()

    # 格式化输出
    print("时间: %.1fs | 俯仰: %.1f° | 翻滚: %.1f°" % (t, pitch, roll))

    time.sleep(0.5)
```

上传后，拿起主控板旋转，观察串口输出中的角度变化。

---

## 6. 常见问题排查

| 问题                            | 可能原因                    | 解决方法                                   |
| ------------------------------- | --------------------------- | ------------------------------------------ |
| 上传失败、没有找到设备          | 串口驱动未安装              | Windows 安装 CH340/CP210x 驱动             |
| 上传后程序不运行                | 文件名不是 `main.py`        | 确保文件名为 `main.py` 且在 `/flash/` 路径 |
| 打开串口提示端口占用            | mBlock 或其他软件占用了串口 | 关闭其他软件再试                           |
| 代码报错 `ImportError`          | 模块名拼写错误              | 检查 `import` 语句拼写                     |
| `ValueError: invalid .mpy file` | 固件版本太旧                | 更新主控板固件（随 mBlock 最新版发布）     |

---

## 7. 学习检查

完成本课后，你应该能：

- ✅ 用至少一种方式将代码上传到 Novapi
- ✅ 写出包含 `import` + `while True` 的基本程序
- ✅ 理解 `main.py` 的执行机制
- ✅ 用串口监视器查看 print 输出

> 📖 **下一课：** [02-Novapi程序结构与编程范式](./02-程序结构与编程范式.md)
