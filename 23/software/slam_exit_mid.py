from maze_mid import Maze
from robot_mid import Robot
import math
import matplotlib.pyplot as plt
from astar_real import AStar
from matplotlib.patches import Circle
import numpy as np
import matplotlib.patches as patches

# --- 这些常量保持不变 ---
CLOSE = 0.15
FREE = 0
BLK = 1
UNK = 9
LIDAR_RANGE = 6.0
PATH_COLOR = 'blue'
GRID_LEN = 0.2
THETA_ADD = 90

class SLAM:
    def __init__(self, file='input2.json'):
        # 假设 Maze 类能够读取 'start' 和 'exit'
        self.maze=Maze(file)
        self.robot=Robot(self.maze)
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
        circle = Circle((self.robot.x, self.robot.y), LIDAR_RANGE,
                        fill=False, color='red', linestyle='--', alpha=0.3)
        ax.add_patch(circle)

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
            elif state == BLK: color = 'black'
            bottom_left_x = wx - GRID_LEN
            bottom_left_y = wy - GRID_LEN
                
                # 2. 创建一个矩形 (patch)
            rect = patches.Rectangle(
                (bottom_left_x, bottom_left_y), # (x, y) 左下角坐标
                GRID_LEN,  # 矩形宽度
                GRID_LEN,  # 矩形高度
                facecolor=color, # 使用你计算的颜色
                edgecolor='none',# 保持你原来的设置
                alpha=0.7       # 保持你原来的 alpha
            )
                
                # 3. 将矩形添加到坐标轴
            self.axR.add_patch(rect)

        if plan:
            px, py=zip(*plan)
            self.axL.plot(px, py, 'red', lw = 2)
            self.axR.plot(px, py, 'red', lw = 2, alpha=0.5)
        
        # 绘制起点和终点以供参考
        if hasattr(self.maze, 'start') and hasattr(self.maze, 'exit'):
             self.axR.plot(self.maze.start[0], self.maze.start[1], 'go', markersize=10, label='Start')
             self.axR.plot(self.maze.exit[0], self.maze.exit[1], 'rx', markersize=10, mew=2, label='Exit')

        plt.pause(0.01)

    def update_occ(self, hits):
        for wx, wy in hits:
            self.occ[self.robot.world_to_grid(wx, wy)] = BLK
        for g in self.robot.scanned:
            if self.occ.get(g) != BLK:
                self.occ[g] = FREE

    def is_safe_point(self, grid_point, safety_margin):
        """检查网格点是否远离障碍物"""
        gx, gy = grid_point
        
        # 检查周围网格是否有障碍物
        for dx in range(-safety_margin, safety_margin):
            for dy in range(-safety_margin, safety_margin):
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
        visited = set()  # 存放所有被设为过目标的栅格，防止重复探索
        stack = []       # 用于记录路径历史和实现回溯

        start_grid = self.robot.world_to_grid(self.robot.x, self.robot.y)
        stack.append(start_grid)
        visited.add(start_grid)

        step = 0
        while stack and step < max_steps:
            # 0. 获取当前位置并移动机器人实体到该点
            current_grid = stack[-1]
            self.robot.x, self.robot.y = self.robot.grid_to_world(*current_grid)

            # 1. 检查是否已到达目标
            if self.planner.cheb(current_grid, goal_grid) <= 1:
                print("成功到达目标点！")
                self.slam_draw(plan=[self.robot.grid_to_world(*g) for g in stack])
                return True, stack

            # 2. 感知环境并更新地图
            hits = self.robot.scan(self.maze)
            self.update_occ(hits)

            # 3. 寻找所有有效的邻居候选点（逻辑源自explore）
            neighbors = []
            step_size = 10  # 探索步长
            for dx, dy in [(-step_size, 0), (step_size, 0), (0, -step_size), (0, step_size)]:
                ng = (current_grid[0] + dx, current_grid[1] + dy)
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
                    check_grid = (current_grid[0] + i * step_x, current_grid[1] + i * step_y)
                    
                    # 【新增】将检查点从栅格坐标转换为世界坐标，以用于边界检查
                    check_world_x, check_world_y = self.robot.grid_to_world(*check_grid)

                    # 【核心】执行双重检查：
                    # 1. is_safe_point: 检查是否撞到已知障碍物
                    # 2. is_inside_maze: 检查是否超出物理边界
                    if not self.is_safe_point(check_grid, safety_margin=1) or \
                       not self.maze.is_inside_maze(check_world_x, check_world_y, margin=0.1):
                        
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
                        
                # 移动成功，更新状态和地图
                step += 1
                stack.append(best_neighbor)       
                visited.add(best_neighbor)

            else:
                # 6. 如果没有找到有效邻居，则回溯
                print(f"在 {current_grid} 处遇到死胡同，正在回溯...")
                stack.pop()

            # 7. 实时可视化
            plan_path_world = [self.robot.grid_to_world(*g) for g in stack]
            self.slam_draw(plan=plan_path_world)
        
        if not stack:
             print("探索了所有可达区域，仍无法找到目标点。")
        else:
             print("超过最大移动步数，任务失败。")
             
        return False

    def main(self):
        """
        任务主流程：先去终点，再返回起点。
        """
        print("任务开始...")
        
        # 确保maze对象已成功加载起点和终点
        if not hasattr(self.maze, 'start') or not hasattr(self.maze, 'exit'):
            print("错误: Maze对象未从JSON文件加载'start'或'exit'坐标。")
            return
            
        start_grid = self.robot.world_to_grid(*self.maze.start)
        exit_grid = self.robot.world_to_grid(*self.maze.exit)

        # 阶段一: 从起点导航到终点
        print(f"阶段一: 从起点 {self.maze.start} 前往终点 {self.maze.exit}...")
        success_to_exit, stack1 = self.navigate_to_goal(exit_grid)

        if not success_to_exit:
            print("未能到达终点，任务提前结束。")
            self.show_final_map_and_wait()
            return

        print("已成功到达终点！准备返航...")
        self.slam_draw() # 显示到达终点的状态
        # plt.pause(2)     # 暂停2秒，方便观察

        # 阶段二: 从终点导航回起点
        print(f"阶段二: 从终点 {self.maze.exit} 返回起点 {self.maze.start}...")
        return_path_grid = stack1[::-1]
        
        # 返航路径检查
        success_to_start = False
        if not return_path_grid or len(return_path_grid) <= 1:
            print("成功到达终点，但返航路径无效。")
        else:
            print("已生成返航路径，开始移动...")
            return_path_world = [self.robot.grid_to_world(*g) for g in return_path_grid]
            print(return_path_world)

            for target_point in return_path_world:
                self.robot.x, self.robot.y = target_point
                self.slam_draw(plan=return_path_world)
            success_to_start = True


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