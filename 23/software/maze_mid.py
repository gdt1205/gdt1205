import json
import math

LIDAR_RANGE = 6
SAMPLE_STEP = 0.3

class Maze:
    def __init__(self, f):
        # 读取JSON文件并解析数据
        with open(f, 'r') as f:
            maze_data = json.load(f)

        self.segments1 = maze_data["segments"]
        
        # 解析迷宫的所有线段，并将起点和终点存储为元组
        self.segments = [(tuple(s['start']), tuple(s['end'])) for s in maze_data['segments']]
        
        # 存储迷宫的起点和终点
        self.start = tuple(maze_data['start_point_g'])
        self.exit = tuple(maze_data['end_point_g'])
        
        # 提取所有线段的x和y坐标，计算迷宫的边界并加上一些缓冲区（绘图使用）
        xs = []
        ys = []
        for s in self.segments:
          for p in s:
            xs.append(p[0])  # p[0] 是 x 坐标
            ys.append(p[1])  # p[1] 是 y 坐标
        self.min_x = min(xs) - 0.2
        self.max_x = max(xs) + 0.2
        self.min_y = min(ys) - 0.2
        self.max_y = max(ys) + 0.2

    """检查点(x, y)是否在迷宫的墙壁内"""
    def is_hit_wall(self, x, y, tolerance=0.11):
        # 遍历迷宫中的每个墙壁线段，判断点是否碰到墙壁
        for seg_start, seg_end in self.segments:
            if self.cal_point_to_segment((x, y), seg_start, seg_end) < tolerance:
                return True
        return False

    """检查点(x, y)是否在迷宫的有效区域内，考虑一定的边缘缓冲"""
    def is_inside_maze(self, x, y, margin=0.05):
        # 判断点是否在迷宫的有效区域内，并加上一定的缓冲区
        if self.min_x + margin <= x <= self.max_x - margin and self.min_y + margin <= y <= self.max_y - margin:
            return True
        return False

    """检查从(x1, y1)到(x2, y2)的直线是否没有碰到任何障碍物(这条直线是指小车如果按它这么走有没有可能撞墙)"""
    def is_line_hit(self, x1, y1, x2, y2):
        # 计算从x1, y1到x2, y2的直线步数
        samples = int(max(abs(x2 - x1), abs(y2 - y1)) / SAMPLE_STEP) + 1
        
        # 遍历直线的每个采样点，检查每个点是否不在墙壁内并且在有效区域内
        for i in range(1, samples):
            t = i / samples
            xx = x1 + t * (x2 - x1)
            yy = y1 + t * (y2 - y1)
            
            # 如果点不在有效区域内或点位于墙壁内，则返回False
            if not self.is_inside_maze(xx, yy) or self.is_hit_wall(xx, yy):
                return False
        
        # 如果所有的检查都通过，说明路径没有障碍物
        return True

    """从点(x, y)发射一条射线，检测其与迷宫墙壁的交点"""
    def ray_intersec(self, x, y, angle):
        # 计算射线的终点坐标
        ex = x + LIDAR_RANGE * math.cos(angle)
        ey = y + LIDAR_RANGE * math.sin(angle)
        
        best_intersection = None  # 用于存储最佳交点
        min_distance = LIDAR_RANGE  # 用于存储最近交点的距离
        
        # 遍历每个墙壁线段，检查射线是否与其相交
        for seg_start, seg_end in self.segments:
            # 获取射线与当前线段的交点
            intersection = self.cal_wall_intersection((x, y), (ex, ey), seg_start, seg_end)
            
            if intersection:
                # 计算交点到射线起点的距离
                distance = math.hypot(intersection[0] - x, intersection[1] - y)
                
                # 如果交点距离更近且符合要求，则更新最佳交点
                if 0.01 < distance < min_distance:
                    best_intersection = intersection
                    min_distance = distance
        
        # 如果找到交点，则返回交点的坐标和距离，否则返回射线的终点
        if best_intersection:
            return best_intersection, min_distance
        else:
            return (ex, ey), min_distance


    """计算点到线段的距离"""
    def cal_point_to_segment(self, p, seg_x, seg_y):
        px, py = p
        x1, y1 = seg_x
        x2, y2 = seg_y
        
        # 计算从点p到线段起点a的向量 (vx, vy)
        vx, vy = px - x1, py - y1
        
        # 计算线段的方向向量(wx, wy)
        wx, wy = x2 - x1, y2 - y1
        
        # 如果线段长度为零，直接返回点p到线段起点的距离
        if wx * wx + wy * wy == 0:
            return math.hypot(vx, vy)
        
        # 计算点p到线段的投影比例t，并确保其限制在0到1之间
        t = max(0, min(1, (vx * wx + vy * wy) / (wx * wx + wy * wy)))
        
        # 根据投影比例t计算投影点的坐标
        sx, sf = x1 + t * wx, y1 + t * wy
        
        # 返回点p到投影点的最短距离
        return math.hypot(px - sx, py - sf)
    
    """计算两线段交点"""
    def cal_wall_intersection(self, a1, a2, b1, b2):

        def det(a, b, c, d):
            return a * d - b * c
        
        x1, y1 = a1
        x2, y2 = a2
        x3, y3 = b1
        x4, y4 = b2

        # 计算分母
        denominator = det(x1 - x2, y1 - y2, x3 - x4, y3 - y4)
        if abs(denominator) < 1e-10:
            return None  # 平行或重合

        # 计算交点（通过解线性方程组）
        px = det(det(x1, y1, x2, y2), x1 - x2, det(x3, y3, x4, y4), x3 - x4) / denominator
        py = det(det(x1, y1, x2, y2), y1 - y2, det(x3, y3, x4, y4), y3 - y4) / denominator

        # 判断交点是否在线段范围内
        if (
            min(x1, x2) - 1e-8 <= px <= max(x1, x2) + 1e-8 and
            min(y1, y2) - 1e-8 <= py <= max(y1, y2) + 1e-8 and
            min(x3, x4) - 1e-8 <= px <= max(x3, x4) + 1e-8 and
            min(y3, y4) - 1e-8 <= py <= max(y3, y4) + 1e-8
        ):
            return (px, py)
        else:
            return None
