import serial
import time
import re

def read_car_sensor_data():
    """
    通过蓝牙读取小车的雷达数据和里程计数据
    
    返回值:
        dict: 包含雷达数据和里程计数据的字典，格式如下:
            {
                'radar': {
                    '角度': float,
                    '离障碍物距离': float,
                    '质量值': int
                },
                'odometer': {
                    '坐标': (float, float),  # (x, y)坐标
                    '小车偏航角': float
                }
            }
    """
    # 初始化返回数据结构
    result = {
        'radar': None,
        'odometer': None
    }
    
    # 蓝牙串口配置 - 根据实际情况修改端口和波特率
    # Windows通常为'COMx'，Linux/macOS通常为'/dev/ttyUSBx'或'/dev/ttyACMx'
    bluetooth_port = '/dev/tty.HC-05'  # 蓝牙模块端口，需根据实际情况修改
    baud_rate = 9600                  # 波特率，需与小车通信协议一致
    
    # 尝试连接蓝牙设备
    try:
        # 初始化串口连接
        ser = serial.Serial(
            port=bluetooth_port,
            baudrate=baud_rate,
            timeout=1,        # 读取超时时间
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        # 确保串口已打开
        if not ser.is_open:
            ser.open()
        
        # 等待设备初始化
        time.sleep(2)
        
        # 读取数据直到获取到完整的一行雷达数据和一行里程计数据
        while result['radar'] is None or result['odometer'] is None:
            # 读取一行数据
            line = ser.readline().decode('utf-8').strip()
            
            if not line:
                continue  # 跳过空行
            
            # 判断是雷达数据还是里程计数据并解析
            # 假设雷达数据以'R:'开头，里程计数据以'O:'开头
            # 实际格式需根据小车的通信协议进行调整
            if line.startswith('R:'):
                # 解析雷达数据，格式假设为: R:角度,距离,质量值
                # 例如: R:30.5,120.3,85
                radar_data = line[2:].split(',')
                if len(radar_data) == 3:
                    try:
                        result['radar'] = {
                            '角度': float(radar_data[0]),
                            '离障碍物距离': float(radar_data[1]),
                            '质量值': int(radar_data[2])
                        }
                    except ValueError:
                        print(f"雷达数据格式错误: {line}")
            
            elif line.startswith('O:'):
                # 解析里程计数据，格式假设为: O:x坐标,y坐标,偏航角
                # 例如: O:150.2,300.5,15.3
                odom_data = line[2:].split(',')
                if len(odom_data) == 3:
                    try:
                        result['odometer'] = {
                            '坐标': (float(odom_data[0]), float(odom_data[1])),
                            '小车偏航角': float(odom_data[2])
                        }
                    except ValueError:
                        print(f"里程计数据格式错误: {line}")
        
        # 关闭串口连接
        ser.close()
        return result
        
    except serial.SerialException as e:
        print(f"串口通信错误: {e}")
        return None
    except Exception as e:
        print(f"发生错误: {e}")
        return None

# 测试函数
if __name__ == "__main__":
    # 读取传感器数据
    sensor_data = read_car_sensor_data()
    
    if sensor_data:
        print("雷达数据:", sensor_data['radar'])
        print("里程计数据:", sensor_data['odometer'])
    else:
        print("未能获取传感器数据")
