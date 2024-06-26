#y向下，x向右，z向前
#从YAML文件中读取墙壁信息
#终端输入pip install opencv-python apriltag
import yaml
import maze_grid as maze
import numpy as np
import rospy
from motor.msg import MotorPWM
import math
from std_msgs.msg import Float64MultiArray

#def load_maze_from_yaml(file_path):#读取给定的yaml文件
# with open(file_path, 'r') as file:
    #    maze_data = yaml.safe_load(file)
    #walls = maze_data['walls']
    #return walls
    
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
    
#不需要is_located函数，因为我们将使用DFS算法来搜索迷宫并找到JetBot的位置    


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

#在迷宫中移动和扫描（DFS）
import cv2
import apriltag
from typing import Union
from functools import lru_cache
from dataclasses import dataclass

@dataclass(eq=False)
class Node:
    x: int
    z: int
    parent: "Node" = None

    def __sub__(self, other) -> int:#计算节点与坐标的曼哈顿距离
        if isinstance(other, Node):#判断other是否为Node
            return abs(self.x - other.x) + abs(self.z - other.z)#返回两个节点之间的曼哈顿距
        elif isinstance(other, (tuple, list)):#判断other是否为元组或列表
            return abs(self.x - other[0]) + abs(self.z - other[1])#返回节点与坐标之间的曼哈顿距离
        raise ValueError("other必须为坐标或Node")
    
    def __add__(self, other: Union[tuple, list]) -> "Node":#生成新节点
        x = self.x + other[0]#计算新节点的x坐标
        z = self.z + other[1]#计算新节点的z坐标
        return Node(x, z, self)
        
    def __eq__(self, other):#坐标x,z比较 -> node in close_list
        if isinstance(other, Node):
            return self.x == other.x and self.z == other.z#判断两个节点的x和z坐标是否相等
        elif isinstance(other, (tuple, list)):
            return self.x == other[0] and self.z == other[1]#判断节点的x和z坐标是否与给定坐标相等
        return False
    
    def __hash__(self) -> int:#使可变对象可hash, 能放入set中
        return hash((self.x, self.z))

class DFS:
    def __init__(self, maze, start_pos=(0, 0),move_stop=1, move_direction=4):
        self.maze = maze
        self.start = Node(*start_pos)
        self.close_set = set()#已经访问过的节点
        self.open_list = [self.start]#待访问的节点
        self.move_step= move_stop
        self.move_direction = move_direction
        self.detector = apriltag.Detector()
        self.cap = cv2.VideoCapture(0)

    def reset(self):
        self.close_set = set()
        self.open_list = [self.start]

    def _move(self):
        @lru_cache(maxsize=3)#缓存最近3次调用的结果
        def _move():#返回移动的方向
            move = [
                (0, self.move_step),  # 前
                (0, -self.move_step), # 后
                (-self.move_step, 0), # 左
                (self.move_step, 0),  # 右
                ]
            return move
        return _move()

    def _update_open_list(self, curr: Node):#更新open_list
        for add in self._move():#遍历移动方向
            next_ = curr + add
            if next_ in self.close_set or next_ in self.open_list:
                continue
            if self.maze.grid[next_.z, next_.x] == 0:  # 墙壁
                continue
            self.open_list.append(next_)
    #不要了
    
    def search(self):
        while self.open_list:
            curr = self.open_list.pop()#从open_list中弹出一个节点
            self._update_open_list(curr)#更新open_list
            self.close_set.add(curr)#将当前节点添加到close_set中   
            self.maze.add_to_path(curr.x, curr.z)#将当前节点添加到路径中
            self.detect_apriltag()#调用 detect_apriltag 方法，检测当前位置是否有 AprilTag 标签。
            if curr == self.start:#如果当前节点是起始节点，继续循环
                break
    #改成deque
    
    def detect_apriltag(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#将图像转换为灰度图像
        result = self.detector.detect(gray)#调用 detect 方法，检测图像中的 AprilTag 标签。
        for tag in result:#遍历检测到的 AprilTag 标签
            print(f"Detected AprilTag ID: {tag.tag_id} at ({tag.center[0]}, {tag.center[1]})")#打印检测到的AprilTag的ID和中心坐标
        cv2.imshow('Frame', frame)#显示图像
        cv2.waitKey(1)
        
    def rotate_and_scan(self, rotations=4):#控制 JetBot 旋转并扫描周围环境中的 AprilTag 标签
        for _ in range(rotations):
            ret, frame = self.cap.read()
            if not ret:
            break
        tags = self.detect_apriltag(frame)#调用 detect_apriltag 方法，在当前帧中检测 AprilTag 标签。
        for tag in tags:
            print(f"Detected AprilTag ID: {tag.tag_id} at ({tag.center[0]}, {tag.center[1]})")
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):#按下键盘上的“q”键，退出循环
            break
        # Simulate rotation
        print("Rotating 90 degrees")#打印旋转90度
    cv2.destroyAllWindows()

    
    def run(self):
        # 原地转圈扫描
        print("Scanning at the starting point (0,0)")
        self.rotate_and_scan()

        # 向前走一步到 (1,0)
        path = self.search()
        for node in path:
            print(f"Moving to: ({node.x}, {node.z})")
            if (node.x, node.z) == (1, 0):
                print("Reached (1,0), scanning left and right")
                ret, frame = self.cap.read()
                if not ret:#如果没有读取到帧，退出循环
                    break
                tags = self.detect_apriltag(frame)
                for tag in tags:
                    print(f"Detected AprilTag ID: {tag.tag_id} at ({tag.center[0]}, {tag.center[1]})")
                # Simulate scanning left
                print("Scanning left")
                self.rotate_and_scan(rotations=1)
                # Simulate scanning right
                print("Scanning right")
                self.rotate_and_scan(rotations=1)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cap.release()
        cv2.destroyAllWindows()
#不要了

#从Yaml文件中加载墙壁信息，初始化迷宫并运行DFS
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Maze Exploration and Mapping')#创建解析器
    parser.add_argument('yaml_file', type=str, help='Path to the YAML file containing maze walls')#添加参数
    args = parser.parse_args()#解析参数

    walls = load_maze_from_yaml(args.yaml_file)#加载迷宫墙壁信息
    maze_size = 4  # 迷宫的大小是4x4
    maze = Maze(maze_size, walls)

    start_pos = (0, 0)  # 起始点
    dfs = DFS(maze, start_pos)
    dfs.run()

