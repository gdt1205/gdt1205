import serial
import time
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from typing import List, Tuple, Optional, Set

class StaticMapper:
    """
    一个用于静止建图的可运行类。
    它使用提供的雷达接收函数，在机器人静止（假定在 0,0,0）的情况下，
    循环接收雷达数据，将其转换为世界坐标，并实时绘制点云地图。
    """
    
    # 车辆尺寸（世界坐标单位）
    ROBOT_LENGTH = 0.6
    ROBOT_WIDTH = 0.4

    def __init__(self, port: str, baudrate: int):
        """
        初始化建图器。
        
        :param port: 串口号 (例如 'COM3' 或 '/dev/ttyUSB0')
        :param baudrate: 波特率 (例如 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.connection: Optional[serial.Serial] = None
        
        # 存储世界坐标系下的 (x, y) 点
        self.map_points: Set[Tuple[float, float]] = set()
        
        # 假设机器人静止在原点，朝向 X 轴正方向
        # 姿态: (x, y, theta_degrees)
        self.robot_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0) 
        
        # 绘图对象
        self.fig = None
        self.ax = None
        self.scatter_plot = None # 用于更新点云
        self.robot_patch = None  # 用于显示机器人

    def connect(self) -> bool:
        """连接到串口。"""
        try:
            print(f"正在连接到 {self.port} (波特率: {self.baudrate})...")
            # timeout=1 使得 readline() 在 1 秒后超时
            self.connection = serial.Serial(self.port, self.baudrate, timeout=1)
            print("连接成功。")
            return True
        except serial.SerialException as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开串口连接。"""
        if self.connection and self.connection.is_open:
            self.connection.close()
            print("连接已断开。")

    # -----------------------------------------------------------------
    # 这是您提供的函数，已集成到类中
    # -----------------------------------------------------------------
    def _receive_lidar_frame(self) -> Optional[List[Tuple[float, float]]]:
        """等待并读取一个完整的雷达扫描数据帧（带有5秒超时）。"""
        if not self.connection or not self.connection.is_open:
            print("错误: 串口未连接。")
            return None
            
        lidar_data_list: List[Tuple[float, float]] = []
        try:
            start_time = time.time()
            
            # 1. 循环等待起始标志
            #    将总的起始超时时间减少到 5 秒，避免卡住
            while time.time() - start_time < 10: 
                line_bytes = self.connection.readline()
                if not line_bytes:
                    continue # 超时，读到空字节串，继续尝试
                    
                line = line_bytes.decode('utf-8', errors='ignore').strip()
                if 'START_SCAN' in line:
                    # print(f"接收到标志: {line}") # 调试时取消注释
                    break
            else:
                print("等待 'START_SCAN' 超时。")
                return None
            
            # 2. 循环接收数据点，直到遇到结束标志或超时
            #    从找到 START_SCAN 开始，再给 5 秒时间接收完整一帧
            scan_start_time = time.time()
            while time.time() - scan_start_time < 50: 
                line_bytes = self.connection.readline()
                if not line_bytes:
                    continue # 超时

                line = line_bytes.decode('utf-8', errors='ignore').strip()
                if not line: continue
                
                # print(f"接收到数据: {line}") # 调试时取消注释

                if 'END_SCAN' in line:
                    # print("接收到 'END_SCAN'。") # 调试时取消注释
                    return lidar_data_list

                if line.startswith('Lidar:'):
                    match = re.search(r'A=([-\d.]+),\s*D=([-\d.]+)', line)
                    if match:
                        try:
                            angle = float(match.group(1))
                            distance = float(match.group(2))
                            # 过滤掉无效的距离数据
                            if distance > 0.01: 
                                lidar_data_list.append((angle, distance))
                        except ValueError:
                            print(f"无法解析 Lidar 数据行: {line}")
            
            print("错误: 等待 'END_SCAN' 标志超时。")
            return None # 如果超时了也没收到结束标志
            
        except serial.SerialException as e:
            print(f"串口通信错误: {e}")
            self.disconnect() # 发生严重错误时断开连接
            return None
        except Exception as e:
            print(f"处理数据时发生意外错误: {e}")
            return None

    def _transform_lidar_to_world(self, lidar_data: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        将雷达坐标（角度，距离）转换为世界坐标（x, y）。
        
        :param lidar_data: 来自 _receive_lidar_frame 的 (angle, distance) 列表。
        :return: (x, y) 坐标列表。
        """
        world_points = []
        rx, ry, rtheta_deg = self.robot_pose

        for angle_deg, distance in lidar_data:
            # 假设：
            # 1. 机器人的 0 度 (rtheta_deg) 指向世界坐标的 X 轴正方向。
            # 2. 雷达的 0 度 (angle_deg) 指向机器人的前方。
            # 因此，一个点在世界坐标系中的总角度是 (rtheta_deg + angle_deg)。
            
            world_angle_deg = rtheta_deg + angle_deg
            world_angle_rad = np.deg2rad(world_angle_deg)
            
            # 极坐标到笛卡尔坐标的转换
            # (相对于机器人中心的偏移量)
            ox = distance * np.cos(world_angle_rad)
            oy = distance * np.sin(world_angle_rad)
            
            # 加上机器人的世界坐标
            wx = rx + ox
            wy = ry + oy
            
            world_points.append((wx, wy))
        return world_points

    def _init_plot(self):
        """初始化实时绘图窗口。"""
        print("初始化绘图窗口...")
        plt.ion() # 开启交互模式
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        
        # 1. 绘制障碍物点（先画一个空的）
        self.scatter_plot = self.ax.scatter(
            [], [], c='red', s=5, label='障碍物点云'
        )

        # 2. 绘制小车（固定在原点）
        rx, ry, rtheta_deg = self.robot_pose
        
        rect = Rectangle((-self.ROBOT_LENGTH/2, -self.ROBOT_WIDTH/2), 
                         self.ROBOT_LENGTH, self.ROBOT_WIDTH,
                         facecolor='C0', alpha=0.5, edgecolor='k', zorder=5,
                         label='机器人 (静止)')
        
        # 使用标准X轴约定：rotate_deg(0) 表示朝向X轴正向
        transform = Affine2D().rotate_deg(rtheta_deg).translate(rx, ry) + self.ax.transData
        rect.set_transform(transform)
        self.ax.add_patch(rect)

        # 3. 标注车头方向（箭头）
        head_len = max(self.ROBOT_LENGTH, self.ROBOT_WIDTH) * 0.8
        head_angle_rad = np.deg2rad(rtheta_deg) # 0 度
        hx = rx + head_len * np.cos(head_angle_rad)
        hy = ry + head_len * np.sin(head_angle_rad)
        self.ax.annotate('', xy=(hx, hy), xytext=(rx, ry),
                         arrowprops=dict(arrowstyle="->", color='k', lw=1.5))

        # 4. 格式化坐标轴
        self.ax.set_aspect('equal', adjustable='datalim')
        self.ax.grid(True, linestyle='--', alpha=0.4)
        self.ax.set_xlabel('X (米)')
        self.ax.set_ylabel('Y (米)')
        self.ax.set_title('静止建图 - 实时点云')
        self.ax.legend(loc='upper right')
        
        # 设置初始视图范围
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        
        plt.show(block=False) # 非阻塞显示
        self.fig.canvas.flush_events()

    def _update_plot(self):
        """更新绘图窗口的数据。"""
        if not self.map_points:
            return # 没有点则不更新

        # 1. 高效更新散点图数据
        #    np.c_ 是一个快速将 x 和 y 列表组合成 (N, 2) 数组的方法
        points_array = np.array(list(self.map_points))
        self.scatter_plot.set_offsets(points_array)

        # 2. 自动调整坐标轴范围
        self.ax.relim() # 重新计算数据范围
        self.ax.autoscale_view() # 自动缩放视图

        # 3. 刷新画布
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self, num_scans: int = 20):
        """
        运行建图的主循环。
        
        :param num_scans: 期望收集的雷达扫描帧数。
        """
        if not self.connect():
            return
            
        self._init_plot()
        
        scans_received = 0
        try:
            while scans_received < num_scans:
                print(f"正在等待第 {scans_received + 1}/{num_scans} 帧雷达数据...")
                
                # 1. 接收一帧数据
                lidar_frame = self._receive_lidar_frame()
                
                if lidar_frame:
                    scans_received += 1
                    print(f"接收到 {len(lidar_frame)} 个数据点。")
                    
                    # 2. 坐标转换
                    world_points = self._transform_lidar_to_world(lidar_frame)
                    
                    # 3. 聚合数据
                    self.map_points.update(world_points)
                    
                    # 4. 更新绘图
                    self._update_plot()
                else:
                    print("未收到有效数据帧，重试...")
                    # 如果连接丢失，_receive_lidar_frame 会打印错误
                    if not self.connection or not self.connection.is_open:
                        print("检测到连接丢失，正在停止...")
                        break
                
                # 短暂休眠，让绘图窗口响应
                plt.pause(0.01)

        except KeyboardInterrupt:
            print("\n检测到用户中断 (Ctrl+C)，正在停止...")
        except Exception as e:
            print(f"主循环发生未知错误: {e}")
        finally:
            self.disconnect()
            print(f"建图完成。总共收集到 {len(self.map_points)} 个唯一的地图点。")
            plt.ioff() # 关闭交互模式
            plt.show() # 阻塞显示最终的地图

# -----------------------------------------------------------------
# 运行入口
# -----------------------------------------------------------------
if __name__ == "__main__":
    
    # !!! --- 修改为你实际的串口号和波特率 --- !!!
    SERIAL_PORT = 'COM5'  # Linux 示例
    # SERIAL_PORT = 'COM3'      # Windows 示例
    SERIAL_BAUDRATE = 9600      # 假设波特率为 115200
    
    # 要收集的总帧数
    TOTAL_SCANS = 30
    
    print("--- 启动静止建图程序 ---")
    print(f"端口: {SERIAL_PORT}, 波特率: {SERIAL_BAUDRATE}")
    print(f"将收集 {TOTAL_SCANS} 帧雷达数据。")
    print("按 Ctrl+C 提前停止。")
    print("-" * 30)
    
    # 实例化并运行
    try:
        mapper = StaticMapper(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)
        mapper.run(num_scans=TOTAL_SCANS)
    except Exception as e:
        print(f"启动失败: {e}")
        print("请确保已安装 pyserial, matplotlib, numpy, 并且串口号正确。")