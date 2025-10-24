import serial
import threading
import time

# --- 配置区 ---
SERIAL_PORT = 'COM6'  # 修改为你的串口号
BAUD_RATE = 9600      # 修改为与你小车端一致的波特率

# --- 串口初始化 ---
ser = None
try:
    print(f"正在尝试连接到端口 {SERIAL_PORT}，波特率 {BAUD_RATE}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ 连接成功！")
    print("现在你可以输入命令发送给小车（例如：'w' 前进，'s' 后退，'a' 左转，'d' 右转）")
    print("按 Ctrl+C 可退出程序。\n")

except serial.SerialException as e:
    print(f"❌ 连接失败: {e}")
    exit(1)

# --- 接收线程函数 ---
def receive_data():
    """持续接收并打印从小车发来的数据"""
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"\n📡 小车回复: {data}")
            time.sleep(0.01)
        except Exception as e:
            print(f"接收错误: {e}")
            break

# --- 启动接收线程 ---
recv_thread = threading.Thread(target=receive_data, daemon=True)
recv_thread.start()

# --- 主线程用于发送指令 ---
try:
    while True:
        cmd = input("请输入要发送的命令（或输入 exit 退出）：").strip()
        if cmd.lower() == "exit":
            break
        if cmd:
            ser.write((cmd + "\n").encode('utf-8'))  # 发送带换行的命令
            print(f"➡️ 已发送: {cmd}")
except KeyboardInterrupt:
    print("\n程序被用户终止。")

finally:
    if ser and ser.is_open:
        ser.close()
        print("🔌 串口连接已关闭。")
