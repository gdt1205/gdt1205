from maze_mid import Maze
import math, random
import numpy as np

GRID_LEN = 0.2
LIDAR_RANGE = 6
LIDAR_DEG_SEG = 5
STEP = 0.1

class Robot:
    def __init__(self,maze:Maze):
        sx, sy = maze.start
        self.x = sx
        self.y = sy

        # 存储机器人的路径
        self.path=[(self.x,self.y)]
        # 障碍物世界坐标集
        self.hits: list[tuple[float, float]] = []
        # 存储已经被扫描的网格坐标
        self.scanned: set[tuple[int, int]] = set()
        self.rays: list[dict] = [] # 用来存储每条激光雷达射线的数据

    # 障碍物世界坐标和已扫描网格坐标
    def scan(self,maze:Maze):
        for degree in range(0, 360, LIDAR_DEG_SEG):
            angle = math.radians(degree)

            hit_point, distance = maze.ray_intersec(self.x, self.y, angle)
            if distance < LIDAR_RANGE: 
                self.hits.append(hit_point)

            # 这里有可能可以删除
            if (hit_point[0] - 2.5) <0.1 and (hit_point[0] - 2.5) >-0.1:
                self.hits.append(hit_point)

            for d in np.arange(0, distance, 0.4):
                wx = self.x+d * math.cos(angle)
                wy = self.y+d * math.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                self.scanned.add((gx,gy))

            # 存储射线的数据，包含起始点 (self.x, self.y), 角度 angle, 终点 hit_point 和距离 distance
            self.rays.append({
                'start': (self.x, self.y),  # 起点
                'angle': angle,               # 角度
                'end': hit_point,                  # 终点
                'distance': distance               # 距离
            })

        return self.hits
    
    # 机器人坐标转换函数
    def world_to_grid(self, x, y): 
        return int(math.floor(x/GRID_LEN)), int(math.floor(y/GRID_LEN))

    def grid_to_world(self, gx, gy): 
        return gx*GRID_LEN+GRID_LEN/2, gy*GRID_LEN+GRID_LEN/2
