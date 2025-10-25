# -*- coding: utf-8 -*-
import numpy as np
import serial
import time
import re
import json
from typing import List, Tuple, Optional, Dict
import math
import sys
class BluetoothCommunicator:
    """
    一个专门用于通过单个蓝牙串口进行双向数据通信和解析的类。
    """
    # <<< 修改点: __init__ 现在只接收一个 port 参数
    def __init__(self, port: str, baud_rate: int):
        """
        初始化通信类。
        :param port: 用于发送和接收的串口号 (例如 "COM6")。
        :param baud_rate: 串口波特率。
        """
        self.port = port
        self.baud_rate = baud_rate
        # <<< 修改点: 两个连接变量合并成一个
        self.connection: Optional[serial.Serial] = None

    def connect(self) -> bool:
        """建立到蓝牙模块的串口连接。"""
        print(f"正在尝试连接端口 {self.port} (波特率: {self.baud_rate})...")
        try:
            # <<< 修改点: 只创建一个 serial.Serial 实例用于双向通信
            self.connection = serial.Serial(
                self.port, 
                self.baud_rate, 
                timeout=1,
                write_timeout=1
            )
            # 等待一小段时间确保连接稳定
            time.sleep(2) 
            print("蓝牙端口连接成功！")
            return True
        except serial.SerialException as e:
            print(f"蓝牙端口 {self.port} 连接失败: {e}")
            self.connection = None
            return False

    def disconnect(self):
        """断开蓝牙连接。"""
        # <<< 修改点: 只关闭一个连接
        if self.connection and self.connection.is_open:
            self.connection.close()
            print("蓝牙端口已断开。")

    def send_command(self, command: str) -> bool:
        """通过串口向硬件发送指令。"""
        # <<< 修改点: 检查 self.connection
        if not self.connection or not self.connection.is_open:
            print("错误: 蓝牙端口未连接，无法发送指令。")
            return False
        try:
            print(f"--- 通过 {self.port} 发送指令: '{command}' ---")
            # 很多硬件设备是基于换行符来判断一条指令结束的，这里可以加上
            full_command = command + '\n' 
            self.connection.write(full_command.encode('utf-8'))
            return True
        except serial.SerialTimeoutException:
            print(f"发送指令 '{command}' 超时！请检查连接。")
            return False
        except serial.SerialException as e:
            print(f"发送指令失败: {e}")
            return False

    def t1_clc_reset(self):
        # <<< 修改点: 只检查一个连接
        if not self.connection or not self.connection.is_open:
            print("错误: 蓝牙端口未连接，无法开始测试。")
            return

        print("\n--- 步骤 1: 发送 '0' 指令以启动数据传输 ---")
        if not self.send_command('0'):
            print("启动指令发送失败，测试中止。")
            return
        
        # --- 关键改进 ---
        # 1. 清空输入缓冲区，丢弃可能存在的旧数据或指令本身的回显
        self.connection.reset_input_buffer()
        print("接收缓冲区已清空，准备接收数据...")
        # 2. 给予硬件一个短暂的响应时间
        time.sleep(0.1)

    def t2_receive_odom(self) :
        print("\n--- 步骤 2: 等待并接收里程计数据 ---")
        odom_data = self._receive_odom_frame()
        
        if odom_data:
            print("\n>>> 成功接收到里程计数据 <<<")
            print(json.dumps(odom_data, indent=2))
            return odom_data
        else:
            print("\n>>> 接收里程计数据失败或超时，测试中止。 <<<")
            return None
    def t3_receive_lidar(self) :
        print("\n--- 步骤 3: 等待并接收雷达扫描数据 ---")
        lidar_data = self._receive_lidar_frame()

        if lidar_data:
            print(f"\n>>> 成功接收到 {len(lidar_data)} 个雷达数据点 <<<")
            print(lidar_data)
            return lidar_data
        else:
            print("\n>>> 接收雷达数据失败或超时。 <<<")
            return None



    def _receive_odom_frame(self) -> Optional[List[Dict[str, float]]]:
        """等待并读取一个完整的里程计数据帧（带有5秒超时）。"""
        if not self.connection or not self.connection.is_open:
            return None

        try:
            start_time = time.time()
            # 1. 循环等待起始标志
            while time.time() - start_time < 5.0:
                line = self.connection.readline().decode('utf-8', errors='ignore').strip()
                if 'START_ODOM' in line:
                    print(f"接收到标志: {line}")
                    break # 找到标志，跳出循环
                # 如果没找到，循环会继续，直到超时
            else: # 如果循环是因超时而结束
                print("等待 'START_ODOM' 超时。")
                return None

            # 2. 读取数据行
            data_line = self.connection.readline().decode('utf-8', errors='ignore').strip()
            print(f"接收到数据: {data_line}")
            
            if not data_line.startswith("Odom:"):
                print(f"错误: 期望收到'odom:'开头的数据行，但实际收到: {data_line}")
                return None
            
            json_str = data_line.split("Odom:", 1)[1]
            try:
                data = json.loads(json_str)
            except json.JSONDecodeError:
                print(f"错误: 无法解析里程计的 JSON 数据: {json_str}")
                return None

            # 3. 读取结束标志
            end_line = self.connection.readline().decode('utf-8', errors='ignore').strip()
            print(f"接收到标志: {end_line}")
            if 'END_ODOM' not in end_line:
                print(f"错误: 期望收到 'END_ODOM' 标志，但实际收到: {end_line}")
                return None

            return [data] # 以列表形式返回
        except serial.SerialException as e:
            print(f"串口通信错误: {e}")
            return None

    def _receive_lidar_frame(self) -> Optional[List[Tuple[float, float]]]:
        """等待并读取一个完整的雷达扫描数据帧（带有5秒超时）。"""
        if not self.connection or not self.connection.is_open:
            return None
            
        lidar_data_list: List[Tuple[float, float]] = []
        try:
            start_time = time.time()
            # 1. 循环等待起始标志
            while time.time() - start_time < 5.0:
                line = self.connection.readline().decode('utf-8', errors='ignore').strip()
                if 'START_SCAN' in line:
                    print(f"接收到标志: {line}")
                    break
            else:
                print("等待 'START_SCAN' 超时。")
                return None
            
            # 2. 循环接收数据点，直到遇到结束标志或超时
            while time.time() - start_time < 50.0: # 雷达数据可能较多，给更长的总时间
                line = self.connection.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                
                print(f"接收到标志: {line}")

                if 'END_SCAN' in line:
                    return lidar_data_list

                if line.startswith('Lidar:'):
                    match = re.search(r'A=([-\d.]+),\s*D=([-\d.]+)', line)
                    if match:
                        angle = float(match.group(1))
                        distance = float(match.group(2))
                        lidar_data_list.append((angle, distance))
            
            print("错误: 等待 'END_SCAN' 标志超时。")
            return None # 如果超时了也没收到结束标志
        except serial.SerialException as e:
            print(f"串口通信错误: {e}")
            return None
THETA_ADD = 0  # 根据需要调整偏航角的补偿值
START_X = 0.0  # 初始X坐标
START_Y = 0.0  # 初始Y坐标
def lichengji_calculate(odom_data):
    # 这里实现里程计数据的处理逻辑
    #首先jmin里程计数据解析，直接抄过来
    if odom_data.startswith('odom:'):
        match_X = re.search(r'X=([-\d.]+)', odom_data)
        match_Y = re.search(r'Y=([-\d.]+)', odom_data)
        match_Yaw = re.search(r'Yaw=([-\d.]+)', odom_data)
        if match_X and match_Y and match_Yaw:
            offset_x_m = (float(match_X.group(1)) / 100.0)/0.35
            offset_y_m = (float(match_Y.group(1)) / 100.0)/0.35 # 加旋转矩阵，输入offsetx offsety 小车偏航角（绝对坐标系下）

            robot_yaw_world_degrees =  THETA_ADD    ##这里存疑。
            robot_yaw_world_rad = math.radians(robot_yaw_world_degrees)
            offset_x_world_m = offset_x_m * math.cos(robot_yaw_world_rad) - offset_y_m * math.sin(robot_yaw_world_rad)
            offset_y_world_m = offset_x_m * math.sin(robot_yaw_world_rad) + offset_y_m * math.cos(robot_yaw_world_rad)

            now_x = START_X + offset_x_world_m
            now_y = START_Y + offset_y_world_m

            
            absolute_yaw = float(match_Yaw.group(1))
            now_theta = absolute_yaw + THETA_ADD
            if now_theta >= 360:
                now_theta -= 360

            processed_odom_data = [now_x, now_y, now_theta]
            print(f"里程计数据解算成功: World Pose=[X:{now_x:.2f}, Y:{now_y:.2f}, Yaw:{now_theta:.2f}]")
            return processed_odom_data


def leida_calculate(lidar_data, odom_data):

    if not lidar_data:
        print("警告：本次扫描未接收到任何雷达数据。")
        return lidar_data

    '''now_car_x = odom_data[0]
    now_car_y = odom_data[1]
    now_car_theta = odom_data[2]'''
    # hits: List[Tuple[float, float]] = []  # 存储障碍物点的世界坐标
    xiaochehits: List[Tuple[float, float]] = []  # 存储障碍物点的世界坐标
    # 2. 【关键】遍历这一帧中的每一个数据点
    for angle_deg, distance in lidar_data:
        # 跳过无效的读数
        if distance <= 0:
            continue

        # 转换角度：度 -> 弧度
        # angle_rad_relative = math.radians(angle_deg)
        
        # 计算雷达射线在世界坐标系中的绝对角度
        # (机器人自身朝向 + 雷达相对于机器人的角度)
        '''temp = now_car_theta - angle_deg
        if temp >=360:
            temp = temp - 360
        absolute_angle = temp + 90
        absolute_angle = math.radians(absolute_angle)'''
    
        # 计算障碍物点在世界坐标系中的位置 (hits)
        # wx_hit = now_lidar_x + (distance/0.35) * math.cos(absolute_angle)
        # wy_hit = now_lidar_y + (distance/0.35) * math.sin(absolute_angle)
        # hits.append((wx_hit, wy_hit))
        distance = distance/0.35

        angle_ld = math.radians(90-angle_deg) # angle_deg是相对于小车的雷达偏角，顺时针为正方向 angle_ld是相对于小车x，正方向逆时针
        xiaoche_hit_x =  distance * math.cos(angle_ld)
        xiaoche_hit_y =  distance * math.sin(angle_ld)
        xiaochehits.append((xiaoche_hit_x, xiaoche_hit_y))
        # 更新已扫描的自由区域（scanned）
        '''for d in np.arange(0, distance, 0.4): # 步长0.4可调
            wx = now_lidar_x + d * math.cos(absolute_angle)
            wy = now_lidar_y + d * math.sin(absolute_angle)
            gx, gy = self.world_to_grid(wx, wy)
            self.scanned.add((gx, gy))'''

        # (可选) 存储射线信息用于绘图
        # self.rays.append({
        #     'start': (self.x, self.y),
        #     'angle': absolute_angle,
        #     'end': (wx_hit, wy_hit),
        #     'distance': distance
        # })

        qianfang=0
        houfang=0
        zuofang=0
        youfang=0
        #这里开始检测xiaochehits和是否正确
        for xiaoche_x, xiaoche_y in xiaochehits:

            # 检查某个区域内有多少个障碍物点
            if -1.0 <= xiaoche_x <= 1.0 and -1.0 <= xiaoche_y <= 1.0:#区域前
                qianfang += 1
            elif -1.0 <= xiaoche_x <= 1.0 and 1.0 < xiaoche_y <= 2.0:#区域后
                houfang += 1
            elif -2.0 <= xiaoche_x <= -1.0 and -1.0 <= xiaoche_y <= 1.0:#区域左
                zuofang += 1
            elif 1.0 < xiaoche_x <= 2.0 and -1.0 <= xiaoche_y <= 1.0:#区域右
                youfang += 1
        print(f"前方障碍物点数量: {qianfang}, 后方障碍物点数量: {houfang}, 左方障碍物点数量: {zuofang}, 右方障碍物点数量: {youfang}")

        if qianfang >= 5:#阈值可调
            print("前方检测到较多障碍物点，可能存在障碍物！")
        if houfang >= 5:
            print("后方检测到较多障碍物点，可能存在障碍物！")
        if zuofang >= 5:
            print("左方检测到较多障碍物点，可能存在障碍物！")
        if youfang >= 5:
            print("右方检测到较多障碍物点，可能存在障碍物！")
        return qianfang,houfang,zuofang,youfang



def plot_data(lichengji_data, zhangai_data, save_path: Optional[str]=None):
    """
    绘制小车与障碍物点。
    :param lichengji_data: [now_x, now_y, now_theta]  （世界坐标，theta 单位：度，逆时针为正，参考说明见下）
    :param zhangai_data: [(x1, y1), (x2, y2), ...]
    :param save_path: 可选，保存图片的路径（例如 'map.png'）。不提供则显示窗口。
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.transforms import Affine2D
    import numpy as np

    if not lichengji_data:
        print("plot_data: 缺少里程计位姿，无法绘图。")
        return

    now_x, now_y, now_theta = lichengji_data  # theta 单位为度

    # 车辆尺寸（世界坐标单位），可根据实际调节
    ROBOT_LENGTH = 0.6
    ROBOT_WIDTH = 0.4

    fig, ax = plt.subplots(figsize=(6,6))

    # 绘制障碍物点
    if zhangai_data:
        xs, ys = zip(*zhangai_data) if len(zhangai_data) > 0 else ([], [])
        ax.scatter(xs, ys, c='red', s=10, label='障碍物')

    # 绘制小车：创建以 (0,0) 为中心的矩形，然后用仿射变换平移到 (now_x, now_y) 并旋转
    # 说明：代码中把 now_theta 视为**相对于 Y 轴**的角度（逆时针为正），
    # 若 now_theta 是相对于 X 轴，请把下面 rotate_deg 的偏移去掉。
    rect = Rectangle((-ROBOT_LENGTH/2, -ROBOT_WIDTH/2), ROBOT_LENGTH, ROBOT_WIDTH,
                     facecolor='C0', alpha=0.5, edgecolor='k', zorder=5)
    # 将角度从“相对于 Y 轴”转换为“相对于 X 轴”的旋转：matplotlib 的 rotate_deg 以 X 轴为基准，逆时针为正
    rect_angle_for_matplotlib = now_theta - 90
    transform = Affine2D().rotate_deg(rect_angle_for_matplotlib).translate(now_x, now_y) + ax.transData
    rect.set_transform(transform)
    ax.add_patch(rect)

    # 标注车头方向（箭头）
    head_len = max(ROBOT_LENGTH, ROBOT_WIDTH) * 0.8
    head_angle_rad = np.deg2rad(now_theta - 90)  # 同上转换
    hx = now_x + head_len * np.cos(head_angle_rad)
    hy = now_y + head_len * np.sin(head_angle_rad)
    ax.annotate('', xy=(hx, hy), xytext=(now_x, now_y),
                arrowprops=dict(arrowstyle="->", color='k', lw=1.5))

    # 格式化坐标轴
    ax.set_aspect('equal', adjustable='datalim')
    # 自动设置显示范围，保留一点边距
    all_x = [now_x] + (list(xs) if zhangai_data else [])
    all_y = [now_y] + (list(ys) if zhangai_data else [])
    if all_x and all_y:
        xmin, xmax = min(all_x), max(all_x)
        ymin, ymax = min(all_y), max(all_y)
        dx = max(1.0, (xmax - xmin) * 0.6 + 0.5)
        dy = max(1.0, (ymax - ymin) * 0.6 + 0.5)
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        ax.set_xlim(cx - dx/2, cx + dx/2)
        ax.set_ylim(cy - dy/2, cy + dy/2)

    ax.grid(True, linestyle='--', alpha=0.4)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('里程计与雷达点云（世界坐标）')
    ax.legend(loc='upper right')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"plot_data: 图像已保存到 {save_path}")
        plt.close(fig)
    else:
        plt.show()



# ==============================================================================
#  主程序入口
# ==============================================================================
if __name__ == "__main__":
    # --- 请在这里修改您的蓝牙串口号和波特率 ---
    # <<< 修改点: 只需要一个端口号
    BLUETOOTH_PORT = "COM6" 
    BAUD_RATE = 9600
    # -----------------------------------------

    # <<< 修改点: 使用新的类名和参数
    communicator = BluetoothCommunicator(
        port=BLUETOOTH_PORT, 
        baud_rate=BAUD_RATE
    )
    dongzuo = ('1','2','3','4')#动作序列，1表示前进，2表示左转，3表示右转，4表示掉头


    lichengji_data = None
    zhangai_data = None

    if communicator.connect():
        try:

            communicator.t1_clc_reset()#必要步骤，reset

            #主逻辑在这里
            #一共有4步，分别是前进建图，左转前进建图，右转前进建图，掉头前进建图。前进结束
            for action in dongzuo:
                communicator.send_command(action)
                #step1:上电之后直接发送1指令，让小车动起来
                print(f"当前动作为 = {action}，小车开始运动")
                if action != '1':
                    time.sleep(2)  # 转向动作等待2秒
                    communicator.send_command('1')  # 转向后继续前进
                #step2:接收里程计数据,确认小车停止
                #这里应该收到小车的xy以及角度信息
                odomdata = communicator.t2_receive_odom()
                #解算数据到变量里面
                lichengji_data = lichengji_calculate(odomdata)
                #step3:接收雷达数据
                #这里应该收到雷达的点云数据
                lidardata = communicator.t3_receive_lidar()
                #解算数据到变量里面
                zhangai_data = leida_calculate(lidardata,lichengji_data)


                #如果数据接收有误，这里可以print具体数据进行检查
                # print(f"当前里程计数据为 = {lichengji_data}")
                # print(f"当前雷达数据点数量为 = {len(zhangai_data) if zhangai_data else 0}")
                # print(f"当前雷达数据为 = {zhangai_data if zhangai_data else []}")


                #step4 : 根据里程计和雷达数据进行绘图
                if lichengji_data and zhangai_data:
                    print("\n>>> 准备进行数据绘图处理 <<<")
                    # 这里可以调用绘图函数，传入 lichengji_data(格式为[now_x, now_y, now_theta]) 和 zhangai_data（格式为[(x1, y1), (x2, y2), ...]）
                    #直接调用ai绘图
                    #思路为，新建一个图片，Y轴为正方向，画一个长方形代表小车，小车的坐标由nowx和nowy确定，nowtheta代表矩形偏离Y轴的角度（逆时针为正）。再根据zhangai_data的x和y坐标画出障碍物点
                    plot_data(lichengji_data, zhangai_data)
                    print(f">>> 数据绘图处理完成，当前step为{action} <<<")

                else:
                    print("\n>>> 数据接收不完整，直接退出函数 <<<")
                    sys.exit(1)
                    



        except KeyboardInterrupt:
            print("\n检测到 [Ctrl+C]，正在退出程序...")
        except Exception as e:
            print(f"\n程序发生意外错误: {e}")
        finally:
            communicator.disconnect()
            print("程序已退出。")
