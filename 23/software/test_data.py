# -*- coding: utf-8 -*-
import serial
import time
import re
import json
from typing import List, Tuple, Optional, Dict

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

    def request_and_receive_data(self):
        """
        执行完整的数据请求和接收流程。
        """
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

        print("\n--- 步骤 2: 等待并接收里程计数据 ---")
        odom_data = self._receive_odom_frame()
        
        if odom_data:
            print("\n>>> 成功接收到里程计数据 <<<")
            print(json.dumps(odom_data, indent=2))
        else:
            print("\n>>> 接收里程计数据失败或超时，测试中止。 <<<")
            return

        print("\n--- 步骤 3: 等待并接收雷达扫描数据 ---")
        lidar_data = self._receive_lidar_frame()

        if lidar_data:
            print(f"\n>>> 成功接收到 {len(lidar_data)} 个雷达数据点 <<<")
            print(lidar_data)
        else:
            print("\n>>> 接收雷达数据失败或超时。 <<<")
        
        print("\n--- 数据接收流程完成 ---")

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
            
            if not data_line.startswith("odom:"):
                print(f"错误: 期望收到'odom:'开头的数据行，但实际收到: {data_line}")
                return None
            
            json_str = data_line.split("odom:", 1)[1]
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

    if communicator.connect():
        try:
            communicator.request_and_receive_data()
        except KeyboardInterrupt:
            print("\n检测到 [Ctrl+C]，正在退出程序...")
        except Exception as e:
            print(f"\n程序发生意外错误: {e}")
        finally:
            communicator.disconnect()
            print("程序已退出。")
