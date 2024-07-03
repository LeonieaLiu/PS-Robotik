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
        child_node = TreeNode(child_pos, self)
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
        self.tag_id, self.tag_distance, self.tag_degree = 0,0,0
        self.x, self.y = 100, 100
        self.i, self.j = 100, 100
        self.position = (100, 100)
        self.direction = 361
        self.root = None
        self.current_node = None
        self.history = []
        self.stack = []
        self.walls = []
        self.wall_set_id = []
        self.apriltag_list = []
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
#        if not self.history:
#            self.root = TreeNode(self.position)
#            self.current_node = self.root

    def get_coordinate(self):
        return self.position

    def wall_init(self, tags):
        wall_set, self.wall_set_id, grid = mg.maze_wall(tags)
        max_x, max_y = grid.shape
      #  print(self.wall_set_id)
        for item in self.wall_set_id:
            if item[0] == -0.5 or item[1] == -0.5 or item[0] == max_x + 0.5 or item[1] == max_y + 0.5:
                self.walls.append(item)
   #     print(self.walls)

    def wall_detection(self):
        id = int(self.tag_id)
        for item in self.wall_set_id:
            if item[2] == id:
                if not item in self.walls:
                    self.walls.append(item)
#                    print(self.walls)

    def get_walls(self):
#        print(self.walls)
        return self.walls

    def rotate(self):
        current_direction = self.direction
        while current_direction == 361:
            current_direction = self.direction
            print("direction not detected yet")
            rospy.sleep(1)
        current_position = self.get_coordinate()
        i, j = current_position[0], current_position[1]
        while i >= 10 or j >= 10:
            print("Still looking for legal posiion.")
            current_position = self.get_coordinate()
            i, j = current_position[0], current_position[1]
        #self.root = TreeNode(current_position)
        #print('root is :',self.root.position)
        #self.current_node = self.root
        #self.stack = [self.root]
        N, S, E, W = 0, -180, -90, 90
        rotate = [N, S, E, W]
#        print("self.direction:",self.direction)
        if -10 < self.direction < 10:  # 朝北 只检测东西 并归位
#            rotate.remove(N)
            rotate.remove(S)
        elif -100 < self.direction < -80:  # 朝东
 #           rotate.remove(E)
            rotate.remove(W)
        elif 80 < self.direction < 100:  # 朝西
            rotate.remove(E)
  #          rotate.remove(W)
        elif -180 < self.direction < -175 or 175 < self.direction < 180:  # 朝南
            rotate.remove(N)
   #         rotate.remove(S)
        if j == 0 and S in rotate:
            rotate.remove(S)
        elif j == 3 and N in rotate:
            rotate.remove(N)
        if i == 0 and W in rotate:
            rotate.remove(W)
        elif i == 3 and E in rotate:
            rotate.remove(E)
        #print("jetbot will rotate to:", rotate)
        current_position = self.get_coordinate()
        while current_position == None:
            current_position = self.get_coordinate() 
        if current_position == self.root.position: # 根结点判断
            if -10 < current_direction < 10:  # 当前朝北
                rotate.append(S)  # 添加南
            elif -100 < current_direction < -80:  # 当前朝东
                rotate.append(W)  # 添加西
            elif 80 < current_direction < 100:  # 当前朝西
                rotate.append(E)  # 添加东
            elif -180 < current_direction < -175 or 175 < current_direction < 180:  # 当前朝南
                rotate.append(N)  # 添加北
            #print("root node will rotate to:",rotate)
        for direction in rotate:
            print('to rotate direction',direction)
            self.rotate_to(direction)
            self.wall_detection()
            self.generate_new_position()
        #self.rotate_to(current_direction)

       # print("already back to original direction")

    def generate_new_position(self):
 #       print("generating")
        publisher = pth.demand_publisher()
        position = self.get_coordinate()
        while position[0] == 100 or position[1] == 100:
            print("Still looking for available target.")
            position = self.get_coordinate()
        i, j = position[0], position[1]
        new_position = []
        self.apriltag_list.append(self.tag_id)            
  #      print(self.tag_id,self.tag_distance)
        if np.abs(self.tag_distance) > 0.25 and np.abs(self.tag_degree) < 25:
            #publisher.stop()
            print("detected apriltag:",self.tag_id,"distance:",self.tag_distance,\
                  "relative angle:",self.tag_degree)
            print('self.direction:',self.direction)
            if -35 < self.direction < 35:
                new_position.append((i, j + 1))
            elif -125 < self.direction < -55:
                new_position.append((i + 1, j))
            elif 55 < self.direction < 125:
                new_position.append((i - 1, j))
            elif -180 < self.direction < -145 or 145 < self.direction < 180:
                new_position.append((i, j - 1))
#            print('new position:',new_position)
            for pos in new_position:
                if pos not in self.history:
                    new_node = self.current_node.add_child(pos)
                    print("new added position:",new_node.position)

    def move_to_new_position(self):
        self.current_node = self.stack[-1]
        self.rotate()
        print(self.stack)
#        self.history.append(self.root.position)
        while self.stack:
            all_children_visited = True
            for child in self.current_node.children:
                if child.position not in self.history:
                    self.history.append(child.position)
                    self.stack.append(child)
                    print("move to", child.position)
                    self.move(child.position)
                    print("current position:",child.position)
                    self.rotate()
                   # print('historical list:', self.history)
                    self.current_node = self.stack[-1]
                    all_children_visited = False
                    break
            if all_children_visited:
                self.stack.pop()
                print("already pop")
                if self.stack:
                    self.current_node = self.stack[-1]

    def rotate_to(self, target):
        publisher = pth.demand_publisher()
        current_direction = self.direction
        orient_diff = target - current_direction
        while np.abs(orient_diff) >= 5:
            if 5 < orient_diff <= 180 or -360 <= orient_diff < -180:
                publisher.stay_turn_left()
                orientation_1 = self.direction
                #print("Turning left. Current direction:", orientation_1, "Target:", target)
                orient_diff = target - orientation_1
#                print(orient_diff)
            if -180 <= orient_diff < -5 or 180 < orient_diff <= 360:
                publisher.stay_turn_right()
                orientation_1 = self.direction
                #print("Turning left. Current direction:", orientation_1, "Target:", target)
                orient_diff = target - orientation_1
#                print(orient_diff)
        publisher.stop()
    #def add_to_file(self):

    def run_detect(self, tags):
        rospy.loginfo("Starting maze detection")
        self.wall_init(tags)
        if not self.history:
            initial_position = self.get_coordinate()  # 获取当前坐标，假设已在适当位置初始化
            print("1")
            self.root = TreeNode(initial_position)
            self.current_node = self.root
            self.stack.append(self.root)  # 将根节点加入栈中以开始DFS
            self.history.append(self.root.position)  # 记录历史位置
            print("root",self.root)
        while self.stack:
            self.move_to_new_position()
            if len(self.history) >= 16:  # 假设当历史位置数量达到16时完成任务
                rospy.loginfo("Finished maze detection")
                break
        rospy.loginfo("Maze detection process is now complete")

    def move(self, target):
        subscriber = pth.TFDataSubscriber()
        publisher = pth.demand_publisher()
        current_position = self.get_coordinate()
        walls_new = self.walls
        print(walls_new)
        path = mg.a_star(walls_new, current_position, target)
        print(path)
        movements = mg.path_to_movements(path)
        print(movements)
        pth.init_orientation(movements, publisher, subscriber)
        pth.motor_motion(movements, path, publisher, subscriber)


if __name__ == '__main__':
    rospy.init_node('maze_detector')
    TreeNode((0,0),None)
    jetbot = Maze_detector()
    tags_data= mg.load_yaml_file('/home/jetson/workspace/catkin_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml')
    tags = tags_data['tag_bundles'][0]['layout']
    jetbot.run_detect(tags)
    rospy.spin()

