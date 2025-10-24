import serial
import time
import re
import math
import matplotlib.pyplot as plt
from typing import List, Tuple
from robot1 import LidarRobot

# --- 配置区 ---
# !!! 关键：将这里替换成你在设备管理器中找到的COM端口号 !!!
SERIAL_PORT = 'COM6'  # 示例: 'COM3' 或 'COM4'

# !!! 关键：这里的波特率必须和小车端代码中设置的完全一致 !!!
BAUD_RATE = 9600      # 常见的波特率有 9600, 115200 等


# --- 数据存储区 ---
# 【新增】按照传感器类型，预先创建4个列表，共12个变量，用于存储最新数据
# 初始值设为0.0
lidar_data = [0.0, 0.0, 0.0]  # 分别存储 Lidar 的 [角度A, 距离D, 质量Q]
angles_data = [0.0, 0.0, 0.0] # 分别存储 angles 的三个值
accel_data = [0.0, 0.0, 0.0]  # 分别存储 accel 的 [x, y, z]
gyro_data = [0.0, 0.0, 0.0]   # 分别存储 gyro 的 [x, y, z]

# 【保留】用一个列表来存储地图上的所有点 (x, y)
# 这些点会不断累积，用于观察建图稳定性
map_points: List[Tuple[float, float]] = []


# --- 初始化绘图窗口 ---
plt.ion() # 开启交互模式
fig, ax = plt.subplots(figsize=(10, 10))

def update_plot():
    """根据最新数据重绘整个地图"""
    ax.clear() # 清除旧的图像
    
    # 绘制机器人位置（固定在中心）
    ax.plot(0, 0, 'ro', markersize=8, label='Robot (Stationary)')
    
    # 绘制所有累积的地图点
    if map_points:
        x_coords, y_coords = zip(*map_points)
        ax.scatter(x_coords, y_coords, s=2, c='blue', label='Lidar Points')

    # 重新设置固定的坐标轴范围和标签
    ax.set_xlim(-6000, 6000)
    ax.set_ylim(-6000, 6000)
    ax.set_title("Stationary SLAM (Mapping)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    
    # 暂停一小段时间，让图像有时间刷新
    plt.pause(0.001)

def parse_bracket_data(data_line: str) -> List[float]:
    """一个辅助函数，用于解析包含方括号的行，例如 '\"angles\":[237.3,1.7,8.5],'"""
    try:
        # 找到方括号内的内容
        content = re.search(r'\[(.*?)\]', data_line)
        if content:
            # 去除所有空白，并按逗号分割
            values_str = content.group(1).replace(" ", "").split(',')
            # 转换为浮点数列表
            return [float(v) for v in values_str]
    except (ValueError, IndexError):
        # 如果解析或转换失败，返回空列表
        print(f"警告：解析数据行失败: {data_line}")
        return []
    return []

# --- 主程序 ---
ser = None
try:
    print(f"正在尝试连接到端口 {SERIAL_PORT}，波特率 {BAUD_RATE}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("连接成功！")
    print("\n--- 运行说明 ---")
    print("机器人固定在原点 (0,0) 不会移动。")
    print("程序会持续接收多种传感器数据并更新内部变量。")
    print("仅当接收到Lidar数据时，才会进行SLAM（建图）并更新绘图。")
    print("关闭绘图窗口即可结束程序。")
    print("------------------\n")
    ser.write('0'.encode('utf-8'))  # 发送初始命令启动数据传输
    
    
    while plt.fignum_exists(fig.number): # 当窗口存在时循环
        if ser.in_waiting > 0:
            received_data = ser.readline().decode('utf-8').strip()
            
            # 标志位，只有当Lidar数据更新时才重绘地图
            lidar_updated = False

            # --- 【核心修改】数据解析路由 ---
            # 根据收到的数据行格式，更新对应的变量列表
            
            if received_data.startswith('Lidar:'):
                match_A = re.search(r'A=([\d.]+)', received_data)
                match_D = re.search(r'D=([\d.]+)', received_data)
                match_Q = re.search(r'Q=([\d.]+)', received_data)
                if match_A and match_D and match_Q:
                    # 更新Lidar数据变量
                    lidar_data[0] = float(match_A.group(1)) # 角度 A
                    lidar_data[1] = float(match_D.group(1)) # 距离 D
                    lidar_data[2] = float(match_Q.group(1)) # 质量 Q
                    # 只有当距离有效时，才认为需要更新地图
                    if lidar_data[1] > 10.0:
                        lidar_updated = True

            elif '"angles":' in received_data:
                parsed = parse_bracket_data(received_data)
                if len(parsed) == 3:
                    angles_data = parsed # 更新angles数据

            elif '"accel":' in received_data:
                parsed = parse_bracket_data(received_data)
                if len(parsed) == 3:
                    accel_data = parsed # 更新accel数据
            
            elif '"gyro":' in received_data:
                parsed = parse_bracket_data(received_data)
                if len(parsed) == 3:
                    gyro_data = parsed # 更新gyro数据

            # --- SLAM建图逻辑 ---
            # 如果收到有效的Lidar数据，则执行建图并更新绘图
            if lidar_updated:
                # 从Lidar数据列表中获取角度和距离
                angle_deg = lidar_data[0]
                distance_mm = lidar_data[1]
                
                # 【核心SLAM逻辑】在静止状态下，SLAM简化为Mapping
                # 将Lidar的极坐标转换为笛卡尔坐标，机器人固定在原点(0,0)
                angle_rad = math.radians(angle_deg)
                x = distance_mm * math.cos(angle_rad)
                y = distance_mm * math.sin(angle_rad)
                
                # 将新计算出的点添加到地图点列表中（不断累积）
                map_points.append((x, y))
                
                # 更新绘图
                update_plot()
        else:
            # 短暂暂停，避免CPU空转
            time.sleep(0.01)

except serial.SerialException as e:
    print(f"串口错误: {e}")
    print(f"请检查端口 {SERIAL_PORT} 是否正确，或者小车是否已连接。")
except Exception as e:
    print(f"发生未知错误: {e}")
finally:
    if ser and ser.is_open:
        ser.close()
        print("串口连接已关闭。")
    
    print("程序结束。")
    plt.ioff()
    plt.show() # 保持最终图像显示