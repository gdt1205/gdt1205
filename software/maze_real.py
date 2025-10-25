class Maze:
    def __init__(self):
        
        # 存储迷宫的起点和终点
        self.start = ['start_point_g']
        self.exit = ['end_point_g']

    """检查点(x, y)是否在迷宫的有效区域内，考虑一定的边缘缓冲"""
    def is_inside_maze(self, x, y, margin=0.05):
        # 判断点是否在迷宫的有效区域内，并加上一定的缓冲区
        if 0 + margin <= x <= 8 - margin and 0 + margin <= y <= 8 - margin:
            return True
        return False
