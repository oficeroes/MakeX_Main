import os
import time
import pyautogui
import pyperclip

# ==================== 配置区域 ====================
# 你想要复制的目标文件名
TARGET_FILE = "mecanum_forward.py"

# 给系统反应的缓冲时间（秒）
DELAY_TIME = 1.0
# ==================================================

def run_automation():
    print("🚀 自动化脚本已启动...")

    # 1. 检查当前目录下是否存在该文件
    if not os.path.exists(TARGET_FILE):
        print(f"❌ 错误：在当前文件夹下找不到 {TARGET_FILE} 文件！")
        print("请确保该文件和本脚本在同一个文件夹内。")
        return

    # 2. 读取 mecanum_forward.py 的内容并写入剪贴板
    print(f"📄 正在读取 {TARGET_FILE} 的内容...")
    with open(TARGET_FILE, "r", encoding="utf-8") as f:
        file_content = f.read()
    
    pyperclip.copy(file_content)
    print("✅ 代码已成功复制到剪贴板！")

    # 3. 倒计时，留出时间让你手动把鼠标点进 mBlock 软件
    print("\n⚠️ 请注意：")
    print(f"请在 {int(DELAY_TIME * 5)} 秒内，切换到 mBlock 软件，并点击进入它的【代码输入框】中！")
    
    for i in range(5, 0, -1):
        print(f"倒计时 {i}...")
        time.sleep(DELAY_TIME)

    # 4. 模拟键盘粘贴操作
    print("🤖 正在尝试粘贴代码到 mBlock...")
    
    # 模拟按下 Ctrl+A（全选旧代码，防止叠加），然后 Ctrl+V（粘贴新代码）
    # 如果你是 Mac 电脑，请把 'ctrl' 改为 'command'
    pyautogui.hotkey('ctrl', 'a')
    time.sleep(0.2)  # 微弱延迟确保全选成功
    pyautogui.hotkey('ctrl', 'v')
    
    print("🎉 粘贴完成！请检查 mBlock 软件。")

if __name__ == "__main__":
    run_automation()