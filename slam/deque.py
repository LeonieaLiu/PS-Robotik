import yaml
import maze_grid as maze
import numpy as np
import rospy
from motor.msg import MotorPWM
import math
from std_msgs.msg import Float64MultiArray

class TFDataSubscriber:
    def __init__(self):
        self.id= None
        self.distance = 0
        self.relative_degree = None
        self.sub = rospy.Subscriber('apritag_data', Float64MultiArray, self.callback)
        
        

    def callback(self, msg):
        self.id=(msg.data[0])
        self.distance = (msg.data[1])
        self.degree = (msg.data[2])
    
        rospy.loginfo(f"Received tf_data: position={self.position}, rotation_matrix={self.rotation_matrix}")#打印接收到的tf_data：位置和旋转矩阵
        
    def get_position(self):
        return self.position

    def get_rotation_matrix(self):
        return self.rotation_matrix
    def get_id(self):
        try:
            return self.id
        except:
            rospy.loginfo("No id received")

#初始化地图和记录路径
import numpy as np
import matplotlib.pyplot as plt#导入Matplotlib库，用于绘图

class Maze:
    def __init__(self, size, walls):
        self.size = size
        self.grid = np.ones((size, size)) * 255  # 初始化空白区域
        self.walls = walls
        self.path = []
        self._initialize_walls()#调用私有方法

    def _initialize_walls(self):
        for wall in self.walls:#遍历墙壁列表
            x, z = wall#获得墙壁坐标
            self.grid[z, x] = 0  # 0 表示墙壁

    def add_to_path(self, x, z):
        self.path.append((x, z))
    
    def plot(self):#绘制迷宫和路径
        plt.imshow(self.grid, cmap='gray')
        path_x, path_z,= zip(*self.path)#将路径拆分为x和z坐标
        plt.plot(path_x, path_z, marker='o', color='r')#绘制路径，使用红色圆圈标记
        plt.show()

from collections import deque

def dfs_with_deque(start, grid):#使用双端队列实现深度优先搜索
    rows, cols = len(grid), len(grid[0])#获取行数和列数
    dq = deque([start])  # 初始双端队列，将起始点加入队列
    visited = set()  # 用于记录已访问过的点（记录到哪里？）
    visited.add(start)

    # 定义方向：上、下、左、右
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while dq:
        # a. 弹出坐标
        z, x = dq.popleft()
        print(f"Visiting: ({z}, {x})")

        # b. 走到坐标（w），在这个示例中只是标记为访问过
        grid[z][x] = 'w'  # 这里 'w' 只是表示已访问过，具体可根据需求调整

        # c. 观察四周板（h）
        for direction in directions:
            new_z, new_x = z + direction[0], x + direction[1]

            # d. 把新坐标塞进队列
            if 0 <= new_z < rows and 0 <= new_x < cols and (new_z, new_x) not in visited:#如果新坐标在网格内且未被访问过
                dq.appendleft((new_z, new_x))  # 新坐标放到队列前列
                visited.add((new_z, new_x))

        # e. 重复a-d
        print(f"Deque: {list(dq)}")


# 起始点
start = (0, 0)

dfs_with_deque(start, grid)

# 输出访问后的网格
for row in grid:
    print(' '.join(map(str, row)))
