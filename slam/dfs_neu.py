#y向下，x向右，z向前
#从YAML文件中读取墙壁信息
import yaml

def load_maze_from_yaml(file_path):#读取给定的yaml文件
    with open(file_path, 'r') as file:
        maze_data = yaml.safe_load(file)
    walls = maze_data['walls']
    return walls

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
        if isinstance(other, Node):
            return abs(self.x - other.x) + abs(self.z - other.z)
        elif isinstance(other, (tuple, list)):
            return abs(self.x - other[0]) + abs(self.z - other[1])
        raise ValueError("other必须为坐标或Node")
    
    def __add__(self, other: Union[tuple, list]) -> "Node":#生成新节点
        x = self.x + other[0]
        z = self.z + other[1]
        return Node(x, z, self)
        
    def __eq__(self, other):#坐标x,z比较 -> node in close_list
        if isinstance(other, Node):
            return self.x == other.x and self.z == other.z
        elif isinstance(other, (tuple, list)):
            return self.x == other[0] and self.z == other[1]
        return False
    
    def __hash__(self) -> int:#使可变对象可hash, 能放入set中
        return hash((self.x, self.z))

class DFS:
    def __init__(self, maze, start_pos=(0, 0),move_stop=1, move_direction=4):
        self.maze = maze
        self.start = Node(*start_pos)
        self.close_set = set()
        self.open_list = [self.start]
        self.move_step= move_stop
        self.move_direction = move_direction
        self.detector = apriltag.Detector()
        self.cap = cv2.VideoCapture(0)

    def reset(self):
        self.close_set = set()
        self.open_list = [self.start]

    def _move(self):
        @lru_cache(maxsize=3)
        def _move():
            move = [
                (0, self.move_step),  # 前
                (0, -self.move_step), # 后
                (-self.move_step, 0), # 左
                (self.move_step, 0),  # 右
                ]
            return move
        return _move()

    def _update_open_list(self, curr: Node):
        for add in self._move():
            next_ = curr + add
            if next_ in self.close_set or next_ in self.open_list:
                continue
            if self.maze.grid[next_.y, next_.x] == 0:  # 墙壁
                continue
            self.open_list.append(next_)
    
    def search(self):
        while self.open_list:
            curr = self.open_list.pop()
            self._update_open_list(curr)
            self.close_set.add(curr)
            self.maze.add_to_path(curr.x, curr.y)
            self.detect_apriltag()
            if curr == self.start:
                break
    
    def detect_apriltag(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = self.detector.detect(gray)
        for tag in result:
            print(f"Detected AprilTag ID: {tag.tag_id} at ({tag.center[0]}, {tag.center[1]})")#打印检测到的AprilTag的ID和中心坐标
        cv2.imshow('Frame', frame)
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
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # Simulate rotation
        print("Rotating 90 degrees")
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
                if not ret:
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

