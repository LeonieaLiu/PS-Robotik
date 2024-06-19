import yaml
import numpy as np
import heapq
import math

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data


def maze_wall(tags):
    # 初始化网格为 1 (可通行区域)
    wall_set = set()
    max_x = 0
    max_y = 0
    #处理每个 tag，将其位置转换为网格坐标，并标记为 'X' (墙)
    for tag in tags:
        x = math.floor(tag['x'] /0.25)
        y = math.floor(tag['y'] /0.25)
        qw = tag['qw']
        # qx = tag['qx']
        # qy = tag['qy']
        # qz = tag['qz']

        if abs(qw) == 0.5:
            x=x-0.5
            max_y = max(max_y, y)
        else:
            y=y-0.5
            max_x = max(max_x, x)
        coord = (x, y)
        if coord not in wall_set:
            wall_set.add(coord)
    grid = np.ones((max_x, max_y))
    return wall_set, grid


class Node:
    def __init__(self, position, parent=None):
        self.position = position  # 节点的坐标
        self.parent = parent  # 父节点
        self.g = 0  # 从起点到此节点的实际代价
        self.h = 0  # 此节点到终点的估计代价
        self.f = 0  # 评估函数 f = g + h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __repr__(self):
        return f"Node({self.position})"

def heuristic(node1, node2):
    # 使用曼哈顿距离作为启发式函数
    return ((abs(node1.position[0] - node2.position[0]))**2 + (abs(node1.position[1] - node2.position[1]))**2)**(0.5)
    #return abs(node1.position[0] - node2.position[0])+abs(node1.position[1] - node2.position[1])

def get_neighbors(node, wall_set):
    neighbors = []
    movements = [(0, 0.5), (0, -0.5), (0.5, 0), (-0.5, 0)]  # 4个方向的移动

    for move in movements:
        neighbor_pos = (node.position[0] + 2 * move[0], node.position[1] + 2 * move[1])
        wall_detection = (node.position[0] + move[0], node.position[1] + move[1])
        # 检查邻居节点是否在网格范围内并且不是障碍物
        if wall_detection not in wall_set:
            neighbors.append(Node(neighbor_pos, node))

    return neighbors

###A*算法
def a_star(wall_set, start, end):
    start_node = Node(start)
    end_node = Node(end)

    open_list = []
    closed_list = set()
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        # 如果到达终点，重建路径
        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        # 获取相邻节点
        neighbors = get_neighbors(current_node, wall_set)
        for neighbor in neighbors:
            if neighbor.position in closed_list:
                continue

            # 计算邻居节点的代价
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor, end_node)
            neighbor.f = neighbor.g + neighbor.h

            # 如果邻居节点在开放列表中，并且新的路径代价更大，跳过
            if any(open_node for open_node in open_list if neighbor == open_node and neighbor.g > open_node.g):
                continue

            heapq.heappush(open_list, neighbor)

    return None  # 如果未找到路径

def path_to_movements(path):
    movement_list = np.ones((len(path)-1,3))
    for i  in range(len(path) - 1):
        movement_list[i][0] = path[i + 1][0] - path[i][0]
        movement_list[i][1] = path[i + 1][1] - path[i][1]
        if movement_list[i][0] < 0:
            movement_list[i][2] = 180
        else:
            movement_list[i][2] = 90 * movement_list[i][1]
    return movement_list

# def motor_plan(movement_list):
#     motor_plan = []
#     height = movement_list.shape[0]
#     for i in range(height-1):
#         if movement_list[i+1][3] - movement_list[i][3] == 0:
#
#         else 0 < movement_list[i + 1][3] - movement_list[i][3] <= 180:
#             turn
#         e
#     return  y
def main():
    file_path = '/Users/MADAO/PycharmProjects/pythonProject/project/tags.yaml'
    tags_data = load_yaml_file(file_path)
    tags = tags_data['tag_bundles'][0]['layout']
    walls, grid = maze_wall(tags)
    start=(0,0)
    end=(0,1)
    path=a_star(walls, start, end)
    movement_ = path_to_movements(path)
    # y = motor_plan(movement_)
    print("path:",path)



if __name__ == "__main__":
    main()
