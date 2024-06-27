import rospy
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
import pathtomove as pth
import maze_grid as mg


class TreeNode:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.children = []

    def add_child(self, child_pos):
        child_node = TreeNode(self, child_pos)
        self.children.append(child_node)
        return child_node

    def find_in_tree(self, position):
        if self.position == position:
            return True
        for child in self.children:
            if child.find_in_tree(position):
                return True
        return False


class Maze_detector:
    def __init__(self):
        self.tag_id, self.tag_distance, self.tag_degree = 0, 0, 0
        self.x, self.y = 0, 0
        self.i, self.j = 0, 0
        self.position = (100, 100)
        self.direction = 0
        self.root = TreeNode((0, 0))
        self.current_node = self.root
        self.history = []
        self.stack = [self.root]
        self.walls = []
        self.sub = rospy.Subscriber('apriltag_data', Float64MultiArray, self.tag_info)
        self.sub = rospy.Subscriber('maze_data', Float64MultiArray, self.maze_info)
        self.rate = rospy.Rate(5)

    def tag_info(self, msg):
        self.tag_id = msg.data[0]
        self.tag_distance = msg.data[1]
        self.tag_degree = msg.data[2]

    def maze_info(self, msg):
        self.direction = msg.data[0]
        self.x = msg.data[1]
        self.y = msg.data[2]
        self.i = math.floor(self.x / 0.25)
        self.j = math.floor(self.y / 0.25)
        self.position = (self.i, self.j)

    def get_coordinate(self):
        return self.position

    def wall_init(self):
        wall_set, wall_set_id, grid = mg.maze_wall()
        max_x, max_y = grid.shape
        for item in wall_set_id:
            if item[0] == -0.5 or item[1] == -0.5 or item[0] == max_x + 0.5 or item[1] == max_y + 0.5:
                self.walls.append(item)

    def wall_detection(self):
        wall_set, wall_set_id, grid = mg.maze_wall()
        id = pth.TFDataSubscriber.get_id()
        for item in wall_set:
            if item[2] == id:
                if not item in self.walls:
                    self.walls.append(item)

    def get_walls(self):
        return self.walls

    def rotate(self):
        rotate = []
        current_position = self.get_coordinate()
        current_direction = self.direction
        i, j = current_position[0], current_position[1]
        N, S, E, W = 0, -180, -90, 90
        if -10 < self.direction < 10:  # 朝北 只检测东西 并归位
            rotate = [E, W]
        elif -100 < self.direction < -80:  # 朝东
            rotate = [S, N]
        elif 80 < self.direction < 100:  # 朝西
            rotate = [N, S]
        elif -180 < self.direction < -175 or 175 < self.direction < 180:  # 朝南
            rotate = [W, E]
        if j == 0:
            rotate.remove(S)
        if j == 3:
            rotate.remove(N)
        if i == 0:
            rotate.remove(W)
        if i == 3:
            rotate.remove(E)
        for direction in rotate:
            self.rotate_to(direction)
            self.wall_detection()
            self.generate_new_position()
        self.rotate_to(current_direction)

    def generate_new_position(self):
        position = self.get_coordinate()
        i, j = position[0], position[1]
        self.add_to_file()
        new_position = []
        if np.abs(self.tag_distance) > 0.25 and np.abs(self.tag_degree) < 15:
            if -10 < self.direction < 10:
                new_position.append((i, j + 1))
            elif -100 < self.direction < -80:
                new_position.append((i + 1, j))
            elif 80 < self.direction < 100:
                new_position.append((i - 1, j))
            elif -180 < self.direction < -175 or 175 < self.direction < 180:
                new_position.append((i, j - 1))
            for pos in new_position:
                if not self.root.find_in_tree(pos):
                    new_node = self.current_node.add_child(pos)
                    self.stack.append(new_node)

    def move_to_new_postion(self):
        if self.stack:
            current_position = ()
            current_position = self.get_coordinate()
            next_position = self.stack.pop()
            move(next_position)
            self.history.append(current_position)
            # next_node = self.stack.pop()  # 弹出栈顶元素
            # self.move(self.current_node.position, next_node.position)
            # self.current_node = next_node  # 更新当前节点
            # self.history.append(self.current_node.position)

    def rotate_to(self, target):
        publisher = pth.demand_publisher()
        current_direction = self.direction
        orient_diff = target - current_direction
        while np.abs(orient_diff) >= 3:
            if 3 < orient_diff <= 180 or -360 <= orient_diff < -180:
                publisher.forward_turn_left_fast()
                rospy.loginfo("Turning left.")
            if -180 <= orient_diff < -3 or 180 < orient_diff <= 360:
                publisher.forward_turn_right_fast()
                rospy.loginfo("Turning left.")
                orientation_1 = self.direction
                orient_diff = target - orientation_1
                print(orient_diff)

    def add_to_file(self):
        return 2

    def run_detect(self):
        self.wall_init()
        if len(self.history) == 16:
            print('finish')
        while len(self.history) != 16:
            self.rotate()
            self.move_to_new_postion()


def move(target):
    subscriber = Maze_detector()
    publisher = pth.demand_publisher()
    current_position = subscriber.get_coordinate()
    walls = subscriber.get_walls()
    path = mg.a_star(walls, current_position, target)
    movements = mg.path_to_movements(path)
    pth.motor_motion(movements, path, publisher, subscriber)


if __name__ == '__main__':
    TreeNode((0, 0), None)
    jetbot = Maze_detector()
    jetbot.run_detect()
    rospy.spin()
