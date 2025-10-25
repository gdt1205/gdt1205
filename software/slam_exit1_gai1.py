from maze1 import Maze
from robot1 import LidarRobot
import math
import matplotlib.pyplot as plt
from astar1 import AStar
from matplotlib.patches import Circle
import time
import numpy as np
from test_data_dongzuo2_gai1 import leida_calculate

# --- 这些常量保持不变 ---
# CLOSE = 0.15
FREE = 0
BLK = 1
UNK = 9
# LIDAR_RANGE = 6.0
PATH_COLOR = 'blue'
PORT_NUM = 'COM5'
BAUD_RATE = 9600
ZHANGAI_DISTANCE = 0.5

class SLAM:
    def __init__(self, file='input2.json'):
        # 假设 Maze 类能够读取 'start' 和 'exit'
        self.maze=Maze(file)
        self.robot=LidarRobot(port=PORT_NUM, baud_rate=BAUD_RATE, maze=Maze)
        self.occ = {}
        self.planner=AStar(self.occ)

        # --- 绘图部分保持不变 ---
        self.fig, (self.axL, self.axR) = plt.subplots(1, 2, figsize=(10,5))
        self.fig.suptitle('Dynamic Navigation', fontsize=18, fontweight='bold')
        self._init_axes()
        plt.ion()

    # ==================================================================
    # 以下是您原始代码中无需修改的部分 (绘图和辅助函数)
    # ==================================================================
    def _init_axes(self):
        for ax in (self.axL, self.axR):
            ax.set_xlim(self.maze.min_x,self.maze.max_x)
            ax.set_ylim(self.maze.min_y,self.maze.max_y)
            ax.set_aspect('equal')
            ax.grid(alpha=0.3)

    def draw_robot(self, ax):
        ax.plot(self.robot.x, self.robot.y, 'o', color='blue',
                markersize=10, markeredgewidth=2, markeredgecolor='black')
        '''circle = Circle((self.robot.x, self.robot.y), LIDAR_RANGE,
                        fill=False, color='red', linestyle='--', alpha=0.3)
        ax.add_patch(circle)'''

    def slam_draw(self, plan = None):
        self.axL.cla()
        self.axL.set_title('Robot View', fontsize=14)
        self.axR.cla()
        self.axR.set_title('SLAM Map', fontsize=14)
        self._init_axes()

        for a,b in self.maze.segments:
            self.axL.plot([a[0],b[0]],[a[1],b[1]],'k')

        if len(self.robot.path)>1:
            xs,ys=zip(*self.robot.path)
            self.axL.plot(xs, ys, PATH_COLOR)
        
        self.draw_robot(self.axL)

        for (gx, gy), state in self.occ.items():
            wx, wy = self.robot.grid_to_world(gx, gy)
            color = 'gray' # 默认为 UNK
            if state == FREE: color = 'white'
            elif state == UNK: color = 'gray'
            else: color = 'black'
            self.axR.scatter(wx, wy, color = color, s = 5, alpha = 0.7, edgecolors='none')

        if plan:
            px, py=zip(*plan)
            self.axL.plot(px, py, 'red', lw = 2)
            self.axR.plot(px, py, 'red', lw = 2, alpha=0.5)
        
        # 绘制起点和终点以供参考
        if hasattr(self.maze, 'start') and hasattr(self.maze, 'exit'):
             self.axR.plot(self.maze.start[0], self.maze.start[1], 'go', markersize=10, label='Start')
             self.axR.plot(self.maze.exit[0], self.maze.exit[1], 'rx', markersize=10, mew=2, label='Exit')

        plt.pause(0.01)

    def update_occ(self):
        lidar_data, odom_data = self.robot.scan()
        qianfang,houfang,zuofang,youfang = leida_calculate(lidar_data=lidar_data, odom_data=odom_data)

        if qianfang == 1 :
            #根据小车的朝向，把前方路径标记为黑色
            theta_rad = math.radians(self.robot.theta + 90) #小车朝向转换为相对于绝对x的角度，正方向逆时针
            centerx = self.robot.x + ZHANGAI_DISTANCE * math.cos(theta_rad) # zhangai_distance是到障碍矩形框的距离
            centery = self.robot.y + ZHANGAI_DISTANCE * math.sin(theta_rad)
            #以中心点为基准，构建一个矩形区域 #继续确认
            if 0 <= theta_rad <= (math.pi/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width 
            if (math.pi/2) <= theta_rad <= math.pi:
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if math.pi <= theta_rad <= (math.pi*3/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if (math.pi*3/2) <= theta_rad <= (2*math.pi):
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width        
            #把前方路径标记为黑色
            for x in range(int(juxing_xmin), int(juxing_xmax) + 1):
                for y in range(int(juxing_ymin), int(juxing_ymax) + 1):
                    self.occ[self.robot.world_to_grid(x, y)] = BLK

        if houfang == 1 :
            #根据小车的朝向，把前方路径标记为黑色
            theta_rad = math.radians(self.robot.theta + 90) #小车朝向转换为相对于绝对x的角度，正方向逆时针
            centerx = self.robot.x + ZHANGAI_DISTANCE * math.cos(theta_rad) # zhangai_distance是到障碍矩形框的距离
            centery = self.robot.y + ZHANGAI_DISTANCE * math.sin(theta_rad)
            #以中心点为基准，构建一个矩形区域 #继续确认
            if 0 <= theta_rad <= (math.pi/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width 
            if (math.pi/2) <= theta_rad <= math.pi:
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if math.pi <= theta_rad <= (math.pi*3/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if (math.pi*3/2) <= theta_rad <= (2*math.pi):
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width        
            #把前方路径标记为黑色
            for x in range(int(juxing_xmin), int(juxing_xmax) + 1):
                for y in range(int(juxing_ymin), int(juxing_ymax) + 1):
                    self.occ[self.robot.world_to_grid(x, y)] = BLK

        if zuofang == 1 :
            #根据小车的朝向，把前方路径标记为黑色
            theta_rad = math.radians(self.robot.theta + 90) #小车朝向转换为相对于绝对x的角度，正方向逆时针
            centerx = self.robot.x + ZHANGAI_DISTANCE * math.cos(theta_rad) # zhangai_distance是到障碍矩形框的距离
            centery = self.robot.y + ZHANGAI_DISTANCE * math.sin(theta_rad)
            #以中心点为基准，构建一个矩形区域 #继续确认
            if 0 <= theta_rad <= (math.pi/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width 
            if (math.pi/2) <= theta_rad <= math.pi:
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if math.pi <= theta_rad <= (math.pi*3/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if (math.pi*3/2) <= theta_rad <= (2*math.pi):
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width        
            #把前方路径标记为黑色
            for x in range(int(juxing_xmin), int(juxing_xmax) + 1):
                for y in range(int(juxing_ymin), int(juxing_ymax) + 1):
                    self.occ[self.robot.world_to_grid(x, y)] = BLK

        if youfang == 1 :
            #根据小车的朝向，把前方路径标记为黑色
            theta_rad = math.radians(self.robot.theta + 90) #小车朝向转换为相对于绝对x的角度，正方向逆时针
            centerx = self.robot.x + ZHANGAI_DISTANCE * math.cos(theta_rad) # zhangai_distance是到障碍矩形框的距离
            centery = self.robot.y + ZHANGAI_DISTANCE * math.sin(theta_rad)
            #以中心点为基准，构建一个矩形区域 #继续确认
            if 0 <= theta_rad <= (math.pi/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width 
            if (math.pi/2) <= theta_rad <= math.pi:
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if math.pi <= theta_rad <= (math.pi*3/2):
                    juxing_length = 1*math.cos(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.sin(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width
            if (math.pi*3/2) <= theta_rad <= (2*math.pi):
                    juxing_length = 1*math.sin(theta_rad)  #矩形半宽度
                    juxing_width = 0.5*math.cos(theta_rad)  #矩形半长度#这里存疑
                    juxing_xmin = centerx - juxing_length 
                    juxing_xmax = centerx + juxing_length 
                    juxing_ymin = centery - juxing_width 
                    juxing_ymax = centery + juxing_width        
            #把前方路径标记为黑色
            for x in range(int(juxing_xmin), int(juxing_xmax) + 1):
                for y in range(int(juxing_ymin), int(juxing_ymax) + 1):
                    self.occ[self.robot.world_to_grid(x, y)] = BLK
        #后方同理
        # for wx, wy in hits:
        #     self.occ[self.robot.world_to_grid(wx, wy)] = BLK
        for g in self.robot.scanned:
            if self.occ.get(g) != BLK:
                self.occ[g] = FREE

    def is_safe_point(self, grid_point, safety_margin=2):
        """检查网格点是否远离障碍物"""
        gx, gy = grid_point
        
        # 检查周围网格是否有障碍物
        for dx in range(-safety_margin, safety_margin ):
            for dy in range(-safety_margin, safety_margin ):
                neighbor = (gx + dx, gy + dy)
                if self.occ.get(neighbor) == BLK:
                    return False
        return True
    
    # ==================================================================
    # 从这里开始是新的核心算法和主流程
    # ==================================================================

    def navigate_to_goal(self, goal_grid, max_steps=5000):
        """
        使用基于局部探测和贪心策略的算法导航到目标点。
        该算法模仿explore的局部搜索，但总是选择最朝向目标点的方向。
        """
        visited = set()  # 防止重复探索，栅格
        stack_exp = []       # 期望路径，栅格
        stack_real = []      # 真实移动路径，栅格

        start_grid = self.robot.world_to_grid(self.robot.x, self.robot.y)
        stack_exp.append(start_grid)
        stack_real.append(start_grid)
        visited.add(start_grid)

        step = 0
        while stack_exp and step < max_steps:
            # 0. 获取当前位置并移动机器人实体到该点
            current_grid_exp = stack_exp[-1]
            current_grid_real = stack_real[-1]

            distance_to_exp_target = self.planner.cheb(current_grid_real, current_grid_exp)
            if distance_to_exp_target > 5:
                print(f"探索目标 {current_grid_exp}  (距离: {distance_to_exp_target})，计算移动方向...")
                
                # 将栅格坐标转换为世界坐标以计算精确角度
                start_w = self.robot.grid_to_world(*current_grid_real)
                target_w = self.robot.grid_to_world(*current_grid_exp)
                
                # 计算从当前真实位置指向探索目标的向量的世界角度
                target_angle_world = math.atan2(target_w[1] - start_w[1], target_w[0] - start_w[0])

                # 计算目标方向与机器人当前朝向 (self.robot.theta) 的角度差
                theta_1 = self.robot.theta - 270
                if theta_1 < -180:
                    theta_1 += 360
                theta_1_rad = math.radians(theta_1)
                angle_diff = target_angle_world - theta_1_rad
                
                # 将角度差标准化到 [-pi, pi] 范围，以便于比较
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                # 根据角度差判断方向并调用 self.robot 的方法发送指令
                if -math.pi / 4 <= angle_diff <= math.pi / 4:
                    print(f"方向判断：前方 (角度差: {math.degrees(angle_diff):.1f}°)。发送前进指令。")
                    self.robot.send_command('1') # 前进
                elif math.pi / 4 < angle_diff < 3 * math.pi / 4:
                    print(f"方向判断：左方 (角度差: {math.degrees(angle_diff):.1f}°)。发送左转、前进指令。")
                    self.robot.send_command('2') # 左转
                    time.sleep(1.5)
                    self.robot.send_command('1') # 前进
                elif -3 * math.pi / 4 < angle_diff < -math.pi / 4:
                    print(f"方向判断：右方 (角度差: {math.degrees(angle_diff):.1f}°)。发送右转、前进指令。")
                    self.robot.send_command('3') # 右转
                    time.sleep(1.5)
                    self.robot.send_command('1') # 前进
                else:
                    print(f"方向判断：后方 (角度差: {math.degrees(angle_diff):.1f}°)。发送后退、前进指令。")
                    self.robot.send_command('4') # 后退
                    time.sleep(1.5)
                    self.robot.send_command('1') # 前进
                    # 判断小车坐标和角度是否符合预期（加纠错指令）
            else:
                print(f"探索目标 {current_grid_exp} 已在附近，无需发送移动指令。")
            
            hits, _ = self.robot.scan(self.maze)
            self.update_occ(hits)
            stack_real.append(self.robot.world_to_grid(self.robot.x, self.robot.y))
            current_grid_real = stack_real[-1]

            # 1. 检查是否已到达终点
            if self.planner.cheb(current_grid_real, goal_grid) <= 5:
                print("成功到达目标点！")
                self.slam_draw(plan=[self.robot.grid_to_world(*g) for g in stack_real])
                return True, stack_exp

            # 3. 寻找所有有效的邻居候选点（逻辑源自explore）
            neighbors = [] # 栅格
            step_size = 5  # 探索步长
            for dx, dy in [(-step_size, 0), (step_size, 0), (0, -step_size), (0, step_size)]:
                ng = (current_grid_real[0] + dx, current_grid_real[1] + dy)
                if ng in visited: continue
                if self.occ.get(ng, UNK) == BLK: continue

                # --- 2. 核心修改：检查从当前点到邻居的整条路径 ---
                is_path_valid = True
                # 获取步进方向（-1, 0, 或 1）
                step_x = int(np.sign(dx))
                step_y = int(np.sign(dy))

                # 逐一检查路径上的每一个格子
                # 范围从1到step_size，包含起点和终点之间的所有格子
                for i in range(1, step_size + 1):
                    check_grid = (current_grid_real[0] + i * step_x, current_grid_real[1] + i * step_y)
                    
                    # 【新增】将检查点从栅格坐标转换为世界坐标，以用于边界检查
                    check_world_x, check_world_y = self.robot.grid_to_world(*check_grid)

                    # 【核心】执行双重检查：
                    # 1. is_safe_point: 检查是否撞到已知障碍物
                    # 2. is_inside_maze: 检查是否超出物理边界
                    if not self.is_safe_point(check_grid, safety_margin=1) or \
                       not self.maze.is_inside_maze(check_world_x, check_world_y, margin=0.2):
                        
                        # 只要任一检查失败，整条路径就视为无效
                        is_path_valid = False
                        print(f"  -> 路径到 {ng} 在 {check_grid} 处无效 (安全或边界问题)。")
                        break # 无需再检查此路径的其余部分
                
                # --- 3. 只有整条路径都安全的邻居，才能被选中 ---
                if is_path_valid:
                    print(f"  -> 找到路径安全的邻居: {ng}")
                    neighbors.append(ng)                

            # 4. 贪心决策：选择离终点最近的邻居
            if neighbors:
                # 按到最终目标的距离对所有有效邻居进行排序
                neighbors.sort(key=lambda n: self.planner.cheb(n, goal_grid))
                best_neighbor = neighbors[0]   
                step += 1
                stack_exp.append(best_neighbor)       
                visited.add(best_neighbor)

            else:
                # 6. 如果没有找到有效邻居，则回溯
                print(f"在 {current_grid_real} 处遇到死胡同，正在回溯...")
                stack_exp.pop()

            # 7. 实时可视化
            plan_path_world = [self.robot.grid_to_world(*g) for g in stack_exp]
            self.slam_draw(plan=plan_path_world)
        
        if not stack_exp:
             print("探索了所有可达区域，仍无法找到目标点。")
        else:
             print("超过最大移动步数，任务失败。")
             
        return False
    
    def return_to_start(self, return_path_grid: list) -> bool:
        """
        控制机器人沿着规划好的路径点，通过发送底层指令返回起点。

        Args:
            return_path_grid (list): 由A*规划出的从终点到起点的栅格路径点列表。

        Returns:
            bool: 如果返航过程正常完成，返回True，否则返回False。
        """
        if not return_path_grid or len(return_path_grid) <= 1:
            print("返航失败：提供的返航路径无效。")
            return False

        print("开始执行返航程序...")
        return_path_world = [self.robot.grid_to_world(*g) for g in return_path_grid]

        # 沿着返航路径的每一个目标点移动
        for next_target_grid in return_path_grid[1:]:
            print(f"\n--- 返航新目标点: {next_target_grid} ---")
            
            # 1. 在当前位置进行扫描，以获取最精确的里程计数据
            _, _ = self.robot.scan(self.maze, TASK='return')

            # 4. 【核心决策逻辑】
            #    当前真实栅格位置
            current_real_grid = self.robot.world_to_grid(self.robot.x, self.robot.y)
            
            #    计算当前真实位置与下一个路径点的距离
            distance_to_target = self.planner.cheb(current_real_grid, next_target_grid)

            if distance_to_target > 5:
                print(f"距离目标 {next_target_grid} 较远 (距离: {distance_to_target})，开始移动...")
                
                # 计算方向并发送指令 (逻辑与探索时完全一致)
                start_w = (self.robot.x, self.robot.y)
                target_w = self.robot.grid_to_world(*next_target_grid)
                target_angle_world = math.atan2(target_w[1] - start_w[1], target_w[0] - start_w[0])
                angle_diff = target_angle_world - self.robot.theta
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                if -math.pi / 4 <= angle_diff <= math.pi / 4:
                    self.robot.send_command('1') # 前进
                    time.sleep(3.0) # 假设前进一格需要1秒
                elif math.pi / 4 < angle_diff < 3 * math.pi / 4:
                    self.robot.send_command('2') # 左转
                    time.sleep(1.5) # 假设左转90度需要1.5秒
                    self.robot.send_command('1') # 前进
                    time.sleep(1.0)
                elif -3 * math.pi / 4 < angle_diff < -math.pi / 4:
                    self.robot.send_command('3') # 右转
                    time.sleep(1.5)
                    self.robot.send_command('1') # 前进
                    time.sleep(1.0)
                else:
                    self.robot.send_command('4') # 后退(掉头)
                    time.sleep(1.5)
                    self.robot.send_command('1') # 前进
                    time.sleep(1.0)

            else:
                print(f"距离目标 {next_target_grid} 小于等于5，已到达，无需移动。")
            
            # 5. 实时可视化返航过程
            self.slam_draw(plan=return_path_world)

        print("返航路径的所有节点都已处理完毕。")
        return True

    def main(self):
        """
        任务主流程：先去终点，再返回起点。
        """
        print("任务开始...")
        
        # 确保maze对象已成功加载起点和终点
        if not hasattr(self.maze, 'start') or not hasattr(self.maze, 'exit'):
            print("错误: Maze对象未从JSON文件加载'start'或'exit'坐标。")
            return
            
        exit_grid = self.robot.world_to_grid(*self.maze.exit)

        # 阶段一: 从起点导航到终点
        print(f"阶段一: 从起点 {self.maze.start} 前往终点 {self.maze.exit}...")
        self.robot.send_command('0')
        success_to_exit, stack = self.navigate_to_goal(exit_grid)

        if not success_to_exit:
            print("未能到达终点，任务提前结束。")
            self.show_final_map_and_wait()
            return

        print("已成功到达终点！准备返航...")
        self.slam_draw() # 显示到达终点的状态
        # plt.pause(2)     # 暂停2秒，方便观察

        # 阶段二: 从终点导航回起点
        print(f"阶段二: 从终点 {self.maze.exit} 返回起点 {self.maze.start}...")
        success_to_start = False

        success_to_start = self.return_to_start(stack[::-1]) # 反转路径以返航

        # ======================= 任务总结 =======================
        if success_to_start:
            print("任务完成，已成功返回起点！")
        else:
            print("成功到达终点，但未能完成返航。")

        self.show_final_map_and_wait()

    def show_final_map_and_wait(self):
        """任务结束时调用，用于冻结最终图像并等待用户关闭。"""
        print("任务结束。请关闭图形窗口以退出程序。")
        plt.ioff()
        self.slam_draw()
        plt.show()


if __name__=='__main__':
    # 确保你的JSON文件中有 "start": [x, y] 和 "exit": [x, y] 字段
    SLAM('input1.json').main()