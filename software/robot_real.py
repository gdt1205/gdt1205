import serial
import time
import re
import math
# import numpy as np
from typing import List, Tuple, Optional
from maze_real import Maze

GRID_LEN = 0.2
THETA_ADD = 90
# STEP = 0.1

class LidarRobot:
    """
    一个集成了蓝牙通信和雷达扫描功能的机器人控制类。
    """

    def __init__(self, port: str, baud_rate: int, maze:Maze):
        """
        初始化机器人。

        参数:
        - port (str): 蓝牙串口号 (例如 'COM3')。
        - baud_rate (int): 波特率 (例如 9600)。
        """
        # --- 蓝牙连接属性 ---
        self.port = port
        self.baud_rate = baud_rate
        self.connection: Optional[serial.Serial] = None

        # --- 机器人状态属性 (示例) ---
        sx, sy = maze.start
        self.x = sx
        self.y = sy
        self.theta = 0.0  # 机器人的朝向（弧度）
        # 【新增】存储初始世界坐标，作为里程计的基准
        self.start_x, self.start_y = sx, sy

        # --- 地图数据 ---
        # 存储机器人的路径
        self.path=[(self.x,self.y)]
        self.hits: List[Tuple[float, float]] = []  # 存储障碍物点的世界坐标
        # self.scanned: Set[Tuple[int, int]] = set() # 存储已扫描的自由区域的网格坐标
        self.rays: List[dict] = [] # 用于调试或可视化的射线信息

    # ===============================================================
    #  方法一：蓝牙连接管理
    # ===============================================================
    def connect_bluetooth(self) -> bool:
        """
        建立到蓝牙模块的串口连接。
        """
        print(f"正在尝试连接到端口 {self.port} (波特率: {self.baud_rate})...")
        try:
            self.connection = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2) # 等待连接稳定
            print("蓝牙连接成功！")
            return True
        except serial.SerialException as e:
            print(f"蓝牙连接失败: {e}")
            self.connection = None
            return False

    def disconnect_bluetooth(self):
        """
        断开蓝牙连接。
        """
        if self.connection and self.connection.is_open:
            self.connection.close()
            print("蓝牙连接已断开。")


    def send_command(self, command: str):
        """
        通过蓝牙向小车发送一个字符指令。
        """
        if not (self.connection and self.connection.is_open):
            print(f"错误：蓝牙未连接，无法发送指令 '{command}'。")
            return
        
        try:
            # 将字符串（如 '0'）编码为字节并发送
            self.connection.write(command.encode('utf-8'))
            print(f"指令已发送: '{command}'")
            # 发送指令后可以短暂延时，给下位机反应时间
            time.sleep(0.1) 
        except Exception as e:
            print(f"发送指令 '{command}' 失败: {e}")

    # ===============================================================
    #  方法二：从蓝牙接收一整帧雷达数据
    # ===============================================================
    def    _receive_frame(self, TASK: str) -> Tuple[List[Tuple[float, float]], List[float]]:
        """
        【核心修改】根据任务类型，选择性地接收数据。
        - TASK='explore': 接收里程计和雷达数据。
        - TASK='return': 只接收里程计数据。
        """
        if not (self.connection and self.connection.is_open):
            print("错误：蓝牙未连接，无法接收数据。")
            return [], []

        raw_lidar_data: List[Tuple[float, float]] = []
        odom_data = [0.0, 0.0, 0.0]

        # --- 阶段一：等待并接收里程计数据帧 (所有任务都需要) ---
        print(f"任务 '{TASK}': 等待 START_ODOM 指令...")  
        start_odom_found = False
        while not start_odom_found:
            if self.connection.in_waiting > 0:
                try:
                    line = self.connection.readline().decode('utf-8').strip()
                    if 'START_ODOM' in line:
                        start_odom_found = True
                        print("START_ODOM 接收到，开始接收里程计数据...")
                except UnicodeDecodeError:
                    pass
            else:
                time.sleep(0.01)

        odom_received_in_frame = False
        while True:
            if self.connection.in_waiting > 0:
                try:
                    line = self.connection.readline().decode('utf-8').strip()

                    if 'END_ODOM' in line:
                        if not odom_received_in_frame:
                            print("警告：在START_ODOM和END_ODOM之间未找到有效的Odom数据行。")
                        print("END_ODOM 接收到，里程计数据帧接收完成。")
                        break

                    if line.startswith('Odom:'):
                        match_X = re.search(r'X=([-\d.]+)', line)
                        match_Y = re.search(r'Y=([-\d.]+)', line)
                        match_Yaw = re.search(r'Yaw=([-\d.]+)', line)
                        if match_X and match_Y and match_Yaw:
                            offset_x_m = (float(match_X.group(1)) / 100.0)/0.35
                            offset_y_m = (float(match_Y.group(1)) / 100.0)/0.35 # 加旋转矩阵，输入offsetx offsety 小车偏航角（绝对坐标系下）

                            robot_yaw_world_degrees =self.theta
                            robot_yaw_world_rad = math.radians(robot_yaw_world_degrees)
                            offset_x_world_m = offset_x_m * math.cos(robot_yaw_world_rad) - offset_y_m * math.sin(robot_yaw_world_rad)
                            offset_y_world_m = offset_x_m * math.sin(robot_yaw_world_rad) + offset_y_m * math.cos(robot_yaw_world_rad)
                            
                            absolute_x = self.start_x + offset_x_world_m
                            absolute_y = self.start_y + offset_y_world_m
                            self.x = absolute_x
                            self.y = absolute_y 
                            
                            absolute_yaw = float(match_Yaw.group(1))
                            self.theta = absolute_yaw + THETA_ADD  # self.theta是小车偏离绝对y的角度，正方向逆时针
                            if self.theta >=360:
                                self.theta -=360

                            odom_data = [absolute_x, absolute_y, absolute_yaw]
                            odom_received_in_frame = True
                            print(f"里程计数据接收成功: World Pose=[X:{absolute_x:.2f}, Y:{absolute_y:.2f}, Yaw:{absolute_yaw:.2f}]")
                except UnicodeDecodeError:
                    pass
            else:
                time.sleep(0.001)

        # --- 阶段二：等待并接收雷达扫描帧 (仅在'explore'任务中执行) ---
        if TASK == 'explore':
            print("任务 'explore': 等待 START_SCAN 指令...")
            start_found = False
            while not start_found:
                if self.connection.in_waiting > 0:
                    try:
                        line = self.connection.readline().decode('utf-8').strip()
                        if 'START_SCAN' in line:
                            start_found = True
                            print("START_SCAN 接收到，开始累积Lidar数据...")
                    except UnicodeDecodeError:
                        pass
                else:
                    time.sleep(0.01)

            while True:
                if self.connection.in_waiting > 0:
                    try:
                        line = self.connection.readline().decode('utf-8').strip()
                        if 'END_SCAN' in line:
                            print(f"END_SCAN 接收到，完成一帧数据接收，共 {len(raw_lidar_data)} 个点。")
                            break
                        
                        if line.startswith('Lidar:'):
                            match_A = re.search(r'A=([\d.]+)', line)
                            match_D = re.search(r'D=([\d.]+)', line)
                            if match_A and match_D:
                                angle = float(match_A.group(1))
                                distance_cm = float(match_D.group(1))
                                distance_m = distance_cm / 100.0
                                raw_lidar_data.append((angle, distance_m))
                    except UnicodeDecodeError:
                        pass
                else:
                    time.sleep(0.001)
        
        else: # 如果任务不是 'explore'
             print(f"任务 '{TASK}': 跳过雷达数据接收。")


        return raw_lidar_data, odom_data

    # ===============================================================
    #  方法三：执行扫描并更新地图
    # ===============================================================
    def scan(self, TASK='explore') -> Tuple[List[Tuple[float, float]], List[float]]:
        """
        执行一次完整的雷达扫描：
        1. 调用内部方法从蓝牙接收一整帧数据。
        2. 遍历该帧中的每一个数据点，更新地图（障碍物、已扫描区域）。
        """
        # 1. 从蓝牙接收一整帧（一整圈）的雷达数据
        #    hits_frame 的内容是 [(angle1, dist1), (angle2, dist2), ...]
        hits_frame, odom_data = self._receive_frame(TASK)

        if not hits_frame:
            print("警告：本次扫描未接收到任何雷达数据。")
            return hits_frame, odom_data

        # 2. 【关键】遍历这一帧中的每一个数据点
        for angle_deg, distance in hits_frame:
            # 跳过无效的读数
            if distance <= 0:
                continue

            # 转换角度：度 -> 弧度
            # angle_rad_relative = math.radians(angle_deg)
            
            # 计算雷达射线在世界坐标系中的绝对角度
            # (机器人自身朝向 + 雷达相对于机器人的角度)
            temp = self.theta - angle_deg  #temp是相对绝对y，正方向逆 # angle_deg是相对于机器人前进方向，顺时针为正方向  angle_deg基于小车的雷达偏角，self.theta产生于小车，所以即使不在同一坐标系下也可以运算
            if temp >=360:
                temp = temp - 360
            absolute_angle = temp + 90 # absolute_angle是相对于绝对x，正方向逆时针
            absolute_angle = math.radians(absolute_angle)
       
            # 计算障碍物点在世界坐标系中的位置 (hits)
            wx_hit = self.x + (distance/0.35) * math.cos(absolute_angle)
            wy_hit = self.y + (distance/0.35) * math.sin(absolute_angle)
            self.hits.append((wx_hit, wy_hit))

            # (可选) 存储射线信息用于绘图
            self.rays.append({
                'start': (self.x, self.y),
                'angle': absolute_angle,
                'end': (wx_hit, wy_hit),
                'distance': distance
            })

        return self.hits, odom_data   

    # 机器人坐标转换函数
    def world_to_grid(self, x, y): 
        return int(math.floor(x/GRID_LEN)), int(math.floor(y/GRID_LEN))

    def grid_to_world(self, gx, gy): 
        return gx*GRID_LEN+GRID_LEN/2, gy*GRID_LEN+GRID_LEN/2

