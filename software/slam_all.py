from maze import Maze
from robot import Robot
import math
import matplotlib.pyplot as plt
from astar import AStar
from matplotlib.patches import Circle

CLOSE = 0.15
FREE = 0
BLK = 1
UNK = 9

LIDAR_RANGE = 6.0  
PATH_COLOR = 'blue'

class SLAM:
    def __init__(self, file='input2.json'):
        self.maze=Maze(file)
        self.robot=Robot(self.maze)
        self.occ = {}
        self.planner=AStar(self.occ)

        #整体构图
        self.fig, (self.axL, self.axR) = plt.subplots(1, 2, figsize=(10,5))

        # 整体图标题
        self.fig.suptitle('GUI', fontsize=18, fontweight='bold')

        self._init_axes()
        plt.ion()

    #初始化坐标轴
    def _init_axes(self):
        for ax in (self.axL, self.axR):
           ax.set_xlim(self.maze.min_x,self.maze.max_x)
           ax.set_ylim(self.maze.min_y,self.maze.max_y)
           ax.set_aspect('equal')
           ax.grid(alpha=0.3)

    # 机器人本体：蓝色圆形，标记机器人的位置
    def draw_robot(self, ax):
        ax.plot(self.robot.x, self.robot.y, 'o', color='blue', 
            markersize=10, markeredgewidth=2, markeredgecolor='black')

        # 机器人朝向的箭头：红色箭头，表示机器人的朝向
        arrow_length = 0.6  # 箭头的长度
        dx = arrow_length * math.cos(self.robot.theta)  # 计算箭头的 x 方向分量
        dy = arrow_length * math.sin(self.robot.theta)  # 计算箭头的 y 方向分量
        ax.arrow(self.robot.x, self.robot.y, dx, dy,head_width=0.3, head_length=0.2, 
             fc='red', ec='black', linewidth=2)  # 绘制箭头

        # 机器人雷达范围：以红色虚线圆圈表示
        circle = Circle((self.robot.x, self.robot.y), LIDAR_RANGE, 
                    fill=False, color='red', linestyle='--', alpha=0.3)
        ax.add_patch(circle)  # 将圆添加到图中


    # 绘制
    # plan: 规划的路径
    def slam_draw(self, plan = None):

        self.axL.cla()
        # 为左侧子图添加标题
        self.axL.set_title('Laser scan', fontsize=14)

        self.axR.cla()
        # 为右侧子图添加标题
        self.axR.set_title('SLAM', fontsize=14)

        self._init_axes()

        # 左图绘制
        # 迷宫绘制
        for a,b in self.maze.segments:
            self.axL.plot([a[0],b[0]],[a[1],b[1]],'k')
        # 机器人及路线绘制
        if len(self.robot.path)>1:
            xs,ys=zip(*self.robot.path)

            #机器人路径绘制
            self.axL.plot(xs, ys, PATH_COLOR)

            # 在左侧图中绘制机器人（包括本体、朝向箭头、雷达范围）
            self.draw_robot(self.axL)
        

        # 右图绘制
        for (gx, gy), state in self.occ.items():
            wx, wy = self.robot.grid_to_world(gx, gy)

            # 判断网格颜色
            if state == FREE:
                color = 'white'
            elif state == BLK:
                color = 'black'
            elif state == UNK:
                color = 'gray'

            self.axR.scatter(wx, wy, color = color, s = 5, alpha = 0.7)  # 这里 s=30 是圆点的大小，可以根据需要调整

        if plan:
            px, py=zip(*plan)
            self.axL.plot(px, py, 'red', lw = 2)
            #self.axR.plot(px, py, 'red', lw = 2)
        plt.pause(0.001)

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

    def explore(self, max_iter=3500):
        """使用DFS策略完全探索迷宫，每次移动用A*规划，遇到死路自动回溯"""
        visited = set() # 已探索栅格
        stack = [] # DFS栈，机器人探索过的路径点（栅格）
        start = self.robot.world_to_grid(self.robot.x, self.robot.y)
        stack.append(start)
        visited.add(start)
        stuck = 0 # 用来干嘛的？
        step = 0

        while stack and step < max_iter:
            # 1、感知环境并更新地图
            current = stack[-1]
            self.robot.x, self.robot.y = self.robot.grid_to_world(*current)
            hits = self.robot.scan(self.maze)
            self.update_occ(hits)

            # 2、找下一个目标点（步长可调）
            neighbors = [] # 未来可达邻居（栅格）
            step_size = 5
            for dx, dy in [(-step_size,0),(step_size,0),(0,-step_size),(0,step_size)]:
                ng = (current[0]+dx, current[1]+dy)
                # 是否已访问
                if ng in visited:
                    continue

                # 是否为障碍物
                if self.occ.get(ng, UNK) == BLK:
                    continue
                
                # 周围是否安全（避障）
                if not self.is_safe_point(ng, safety_margin=1):
                    continue
                    
                # 是否在迷宫内    
                wx, wy = self.robot.grid_to_world(*ng)
                if not self.maze.is_inside_maze(wx, wy, margin=0.1):
                    continue

                # 通过检查的加入邻居
                neighbors.append(ng)

            # 3、规划路径并移动
            # 若找到邻居
            if neighbors:
                next_cell = neighbors[0] # 这个选择有点随意？

                # 基于已知地图A*规划路径
                self.planner.occ = self.occ
                path = self.planner.plan(current, next_cell)

                # 若A*无法规划路径
                if not path:
                    # 检查起点和终点之间是否有直接路径
                    start_w = self.robot.grid_to_world(*current)
                    end_w = self.robot.grid_to_world(*next_cell)
                    
                    # 检查直线路径上是否有障碍物
                    dx = end_w[0] - start_w[0]
                    dy = end_w[1] - start_w[1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    steps = int(dist / 0.1)  # 每0.1单位检查一次
                    path_clear = True
                    for i in range(steps):
                        t = i / steps
                        check_x = start_w[0] + t * dx
                        check_y = start_w[1] + t * dy
                        
                        # 检查点是否在迷宫内且无障碍
                        if not self.maze.is_inside_maze(check_x, check_y, margin=0.1) or \
                           self.maze.is_hit_wall(check_x, check_y):
                            path_clear = False
                            break
                    
                    if path_clear:
                        # 如果直线路径可行,创建一个简单路径
                        path = [current, next_cell]
                    else:
                        # 如果确实无法到达,才标记为障碍物（这个不一定是障碍物啊？）
                        self.occ[next_cell] = BLK
                        stuck += 1
                
                # 机器人按照path执行移动
                plan_path = [self.robot.grid_to_world(*g) for g in path]
                self.slam_draw(plan=plan_path)  # 在左图绘制红色路径
                for g in path[1:]:
                    wx, wy = self.robot.grid_to_world(*g)

                    # 若在移动中撞到障碍物，则标记该栅格为障碍物并停止移动（这里停止后干嘛呢？）
                    if not self.robot.is_move_robot(wx, wy, self.maze):
                        self.occ[g] = BLK
                        stuck += 1
                        break

                    self.robot.x, self.robot.y = wx, wy
                    hits = self.robot.scan(self.maze)
                    self.update_occ(hits)
                    visited.add(g)
                    step += 1
                    
                    # 每20步刷新一次绘图（帧率）
                    if step % 20 == 0:
                        self.slam_draw()

                # 成功到达下一个栅格
                stack.append(next_cell)
                visited.add(next_cell)

            # 如果没有邻居    
            else:
                # 死路，回溯
                stack.pop() # 这里pop出几个路径点呢？
                if stack:
                    # 以A*回溯路径到新的栈顶
                    back_path = self.planner.plan(current, stack[-1])
                    if back_path and len(back_path) > 1:
                        plan_path = [self.robot.grid_to_world(*g) for g in back_path]
                        self.slam_draw(plan=plan_path)

                        # 为什么这里还要在回去的时候扫图？
                        for g in back_path[1:]:
                            wx, wy = self.robot.grid_to_world(*g)
                            if not self.robot.is_move_robot(wx, wy, self.maze):
                                break
                            self.robot.x, self.robot.y = wx, wy
                            hits = self.robot.scan(self.maze)
                            self.update_occ(hits)
                            step += 1
                            if step % 20 == 0:
                                self.slam_draw()
                
                # 如果已经回到起点且没有邻居可走，直接结束
                if len(stack) == 0:
                    print("探索完毕！")
                    break

            # 为什么这里还要更新帧率？
            if step % 50 == 0:
                self.slam_draw()

        # 为什么这里还要绘图？
        self.slam_draw()
        print("探索结束")


    # 更新占用网格
    def update_occ(self, hits):
        for wx, wy in hits:
            self.occ[self.robot.world_to_grid(wx, wy)] = BLK
        for g in self.robot.scanned:
            if self.occ.get(g) != BLK: 
                self.occ[g] = FREE
    
    def find_exit(self):
        if not self.occ:
            return None

        # 获取迷宫的边界范围
        xs = [g[0] for g in self.occ]
        ys = [g[1] for g in self.occ]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        candidate = []
        
        for gx in range(xmin, xmax + 1):
            gy = ymax   # 上边界
            if self.occ.get((gx, gy)) != FREE:
                continue
            if self.occ.get((gx, gy + 1), UNK) != UNK:  # 检查上方是否是未知区域
                continue
            # 检查是否有障碍物
            cx, cy = self.robot.grid_to_world(gx, gy)
            wx, wy = cx, cy + 0.2
            if self.maze.is_hit_wall(wx, wy):
                continue
            candidate.append((gx, gy))
        for gy in range(ymin, ymax + 1):
            gx = xmax   # y边界
            if self.occ.get((gx, gy)) != FREE:
                continue
            if self.occ.get((gx+1, gy), UNK) != UNK:  # 检查右方是否是未知区域
                continue
            # 检查是否有障碍物
            cx, cy = self.robot.grid_to_world(gx, gy)
            wx, wy = cx +0.2, cy 
            if self.maze.is_hit_wall(wx, wy):
                continue
            candidate.append((gx, gy))
        if not candidate:
            return None

        # 返回距离起点最近的出口
        sx, sy = self.robot.world_to_grid(*self.maze.start)
        return min(candidate, key=lambda c: self.planner.cheb((sx, sy), c))


    def main(self):
        #探索迷宫
        self.explore()

        #找出口
        exit = self.find_exit()

        if not exit: 
            print("无出口")
            plt.ioff()
            plt.show()
            return
        
        exit_w = self.robot.grid_to_world(*exit)
        print("出口:", exit_w)
        self.planner.occ = self.occ

        # 打印最短路径，并画出
        start = self.robot.world_to_grid(*self.maze.start)
        path = self.planner.plan(start, exit)
        print(path)

        shortest_path = []
        for g in path:
            shortest_path.append(self.robot.grid_to_world(*g))
        self.slam_draw(plan = shortest_path)

        # 绘制终点
        exit_w = self.robot.grid_to_world(*exit)
        plt.scatter(*exit_w, color='red', s = 50) 

        # 沿着最短路径移动到终点
        for g in path[1:]:
            wx, wy = self.robot.grid_to_world(*g)
            if not self.robot.is_move_robot(wx, wy, self.maze):
                print("遇到障碍，无法到达终点！")
                break
            self.robot.x, self.robot.y = wx, wy
            hits = self.robot.scan(self.maze)
            self.update_occ(hits)
            self.slam_draw(plan=shortest_path)

        plt.ioff()
        plt.show()

        
if __name__=='__main__':
    SLAM('input1.json').main()

