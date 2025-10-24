import heapq

FREE = 0
BLK = 1
UNK = 9


class AStar:
    def __init__(self, occ):
        self.occ = occ # occ字典表示网格占用情况

    # 切比雪夫距离
    # 该距离度量在网格上移动时的最小步数
    def cheb(self, a, b): 
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))
    
    # 邻居节点生成器
    def neighbor(self, point):
        x, y = point

        # 生成4个方向的邻居节点
        for dx, dy, w in [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1)]:
            neighbor = (x + dx , y + dy)
            if self.occ.get(neighbor, UNK) == FREE: 
                yield neighbor, w

    # A*算法规划路径
    def plan(self, s, g):  
        if s == g: 
            return [s]
        
        pq = [(self.cheb(s,g),s)] # 优先队列，存储 (估计成本, 当前点)
        pre_points = {} # 记录每个点的前驱节点
        cost={s : 0} # 记录到达每个点的实际成本
        closed = set() # 已访问的点集合

        while pq:
            _, least_cost_point = heapq.heappop(pq)

            if least_cost_point == g: 
                return self.back_path(pre_points, least_cost_point)
            
            if least_cost_point in closed: 
                continue
            closed.add(least_cost_point)

            for neighbor, w in self.neighbor(least_cost_point):
                new_cost = cost[least_cost_point] + w
                if new_cost < cost.get(neighbor, 1e9):
                    cost[neighbor] = new_cost
                    pre_points[neighbor] = least_cost_point
                    heapq.heappush(pq, (new_cost + self.cheb(neighbor,g), neighbor))
        return []

    # 递归构建路径
    def back_path(self, pre_points, point):
        # 初始化路径 p，从目标点开始
        p = [point]

        while point in pre_points: 
            point = pre_points[point]
            p.append(point)
             
        return p[::-1]