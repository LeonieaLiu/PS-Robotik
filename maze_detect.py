import rospy
import time
import re
from collections import deque
import numpy as np
import math
import tf
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray

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
         self.tag_id, self.tag_distance, self.tag_degree=0, 0, 0
         self.x,self.y,self.direction=0, 0, 0
         self.i, self.j=0, 0
         self.position = 0
         self.direction = 0
         self.root=TreeNode((0, 0))
         self.current_node = self.root
         self.history = []
         self.stack = [self.root]
         self.sub = rospy.Subscriber('apriltag_data', Float64MultiArray, self.tag_info)
         self.sub = rospy.Subscriber('maze_data', Float64MultiArray, self.maze_info )
         self.rate = rospy.Rate(5)

     def tag_info(self, msg):
         self.tag_id= msg.data[0]
         self.tag_distance= msg.data[1]
         self.tag_degree= msg.data[2]
     def maze_info(self,msg):
         self.direction=msg.data[0]
         self.x = msg.data[1]
         self.y = msg.data[2]
         self.i = math.floor(self.x / 0.25)
         self.j = math.floor(self.y / 0.25)
         self.position = (self.i, self.j)

     def get_coordinate(self):
         return self.position

     def rotate(self):
         rotate=[]
         current_position = self.get_coordinate()
         i, j = current_position[0], current_position[1]
         N, S, E, W= 0, -180, -90, 90
         if -10 < self.direction < 10: #朝北 只检测东西 并归位
             rotate = [E, W]
         elif -100 < self.direction < -80:#朝东
             rotate = [S, N]
         elif 80 < self.direction < 100:#朝西
             rotate = [N, S]
         elif -180 < self.direction < -175 or 175 < self.direction < 180:#朝南
             rotate = [W, E ]
         if j==0:
             rotate.remove(S)
         if j==3:
             rotate.remove(N)
         if i==0:
             rotate.remove(W)
         if i==3:
             rotate.remove(E)
         for direction in rotate:
             self.rotate_to(direction)
         self.rotate_to(current_position)




     def generate_new_position(self):
         i, j=self.get_coordinate()
         self.add_to_file()
         new_position = []
         if np.abs(self.tag_distance) > 0.25 and np.abs(self.tag_degree)<15 :
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
             self.move(current_position, next_position)
             self.history.append(current_position)
             #next_node = self.stack.pop()  # 弹出栈顶元素
             #self.move(self.current_node.position, next_node.position)
             #self.current_node = next_node  # 更新当前节点
             #self.history.append(self.current_node.position)

     def add_to_file(self):
     def move(self, current, target):
     def rotate_to(self,direction):





