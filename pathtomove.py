import maze_grid as maze
import numpy as np
import rospy
from motor.msg import MotorPWM
import math
from std_msgs.msg import Float64MultiArray


class demand_publisher:
    def __init__(self):
        self.pub_pwmcmd = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=1)
        self.msg_pwmcmd = MotorPWM()
        self.rate = rospy.Rate(10)

    def stop(self):
        self.msg_pwmcmd.pwm_left = 0.0
        self.msg_pwmcmd.pwm_right = 0.0
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

    def forward(self):
        self.msg_pwmcmd.pwm_left = -0.26
        self.msg_pwmcmd.pwm_right = -0.26
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_slow(self):
        self.msg_pwmcmd.pwm_left = -0.18
        self.msg_pwmcmd.pwm_right = -0.18
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def stay_turn_left(self):
        self.msg_pwmcmd.pwm_left = 0.19
        self.msg_pwmcmd.pwm_right = -0.19
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

    def stay_turn_right(self):
        self.msg_pwmcmd.pwm_left = -0.19
        self.msg_pwmcmd.pwm_right = 0.19
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
    
    def forward_turn_left_fast(self):
        self.msg_pwmcmd.pwm_left = 0.0
        self.msg_pwmcmd.pwm_right = -0.2
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_turn_right_fast(self):
        self.msg_pwmcmd.pwm_left = -0.22
        self.msg_pwmcmd.pwm_right = 0.0
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_turn_left_slow(self):
        self.msg_pwmcmd.pwm_left = 0.0
        self.msg_pwmcmd.pwm_right = -0.19

        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_turn_right_slow(self):
        self.msg_pwmcmd.pwm_left = -0.19
        self.msg_pwmcmd.pwm_right = 0.0
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()


class TFDataSubscriber:
    def __init__(self):
        self.position_x = 100
        self.position_y = 100
        self.distance = 100
        self.orientation = 361
        self.id = -1
        self.rotation_matrix = None
        self.sub = rospy.Subscriber('maze_data', Float64MultiArray, self.callback_maze)
        self.sub = rospy.Subscriber('apriltag_data', Float64MultiArray, self.callback_apriltag)

    def callback_maze(self, msg):
        self.rotation_matrix = msg.data[0]
        self.position_x = msg.data[1]
        self.position_y = msg.data[2]
        #rospy.loginfo(f"Received tf_data: position={self.position}, rotation_matrix={self.rotation_matrix}")

    def callback_apriltag(self, msg):
        self.distance = msg.data[1]
        self.orientation = msg.data[2]
        tag_id = msg.data[0]
        # 转换为整数
        self.id = int(tag_id)
        #rospy.loginfo(f"Received apriltag_data: distance={self.distance}, orientation={self.orientation}, id = {self.id}")
        
    def get_position_x(self):
        return self.position_x

    def get_position_y(self):
        return self.position_y

    def get_rotation_matrix(self):
        return self.rotation_matrix

    def get_distance(self):
        return self.distance

    def get_orientation(self):
        return self.orientation

    def get_id(self):
        return self.id


def pose_calib(publisher, subscriber, movements, i):
    orientation = subscriber.get_rotation_matrix()
    print(orientation)
    orient_diff = orientation - movements[i][2]
    while not -3 <= orient_diff <= 3 or orient_diff >= 357:
        if -180 < orient_diff < -3 or 180 <= orient_diff:
            publisher.forward_turn_left_slow()
#            rospy.loginfo(f"Calibrating left.")
        if 3 < orient_diff <180 or orient_diff <= -180:
            publisher.forward_turn_right_slow()
#            rospy.loginfo(f"Calibrating right.")
            publisher.stop()
        orientation = subscriber.get_rotation_matrix()
#        print(orientation)
        orient_diff = orientation - movements[i][2]
#    rospy.loginfo(f"Calibration finished.")


def findself(subscriber):
    selfx = subscriber.get_position_x()
    selfy = subscriber.get_position_y()    
    while selfx == 100 or selfy == 100:
        rospy.loginfo(f"Still looking for position.")
        selfx = subscriber.get_position_x()
        selfy = subscriber.get_position_y()
    i = math.floor(selfx/0.25)
    j = math.floor(selfy/0.25)
    return selfx, selfy, (i, j)


def init_orientation(movement_list, publisher, subscriber):
    current_orientation = 361
    current_orientation = subscriber.get_rotation_matrix()
    while current_orientation ==361 or current_orientation == None:
        rospy.loginfo("Looking for orientation.")
        current_orientation = subscriber.get_rotation_matrix()
        rospy.sleep(1)
        print(current_orientation)
    first_orientation = movement_list[0][2]
    angle = current_orientation - first_orientation
    while np.abs(angle) >= 5:
        if -180 < angle < -5 or 180 <= angle:
            publisher.stay_turn_left()
        if 5 < angle <180 or angle <= -180:
            publisher.stay_turn_right()
        current_orientation = subscriber.get_rotation_matrix()
 #       print(current_orientation)
        angle = current_orientation - first_orientation
    print('Initialized.')


def motor_motion(movement_list, path, publisher, subscriber):
    rospy.loginfo(f"Start motion.")
    search_tuple = None
   # num=0
    old_i = -1
    while not rospy.is_shutdown():
        #print(num)
        #num+=1
        while search_tuple == None:
            rospy.loginfo(f"Looking for position.")
            pos_x, pos_y, search_tuple = findself(subscriber)
        rospy.loginfo(f"x:{pos_x} y:{pos_y} Position: {search_tuple}")
        i = path.index(search_tuple)
        print("i=", i)
        print(len(path))
        while not i == len(path):
            pos_x, pos_y, search_tuple = findself(subscriber)
            i = path.index(search_tuple)
            print("i=", i)
            # Transport to final block
            if i == len(path) - 1:
                target_x = (path[i][0] + 0.5) * 0.25
                target_y = (path[i][1] + 0.5) * 0.25
                while np.sqrt((target_x - pos_x) ** 2 + (target_y - pos_y) ** 2) >= 0.045:
                    publisher.forward()
                    pos_x, pos_y, search_tuple = findself(subscriber)
                publisher.stop()
                rospy.loginfo("Goal reached.")
                break
            # Forward
            if i < 1 or movement_list[i][2] - movement_list[i-1][2] == 0 and i != len(path) - 1:
                publisher.forward()
                rospy.sleep(0.1)
                pose_calib(publisher, subscriber, movement_list, i)
            # Turning
            if movement_list[i][2] - movement_list[i-1][2] != 0 and i >= 1 and i != len(path) - 1:
                orientation_1 = subscriber.get_rotation_matrix()
                if old_i == i:
                    target_x = (path[i+1][0] + 0.5) * 0.25
                    target_y = (path[i+1][1] + 0.5) * 0.25
                else:
                    target_x = (path[i][0] + 0.5) * 0.25
                    target_y = (path[i][1] + 0.5) * 0.25

                while np.sqrt((target_x - pos_x) ** 2 + (target_y - pos_y) ** 2) >= 0.02:
                    publisher.forward_slow()
                    rospy.sleep(0.1)
                    publisher.stop()
                    pos_x, pos_y, search_tuple_1 = findself(subscriber)
                    print(np.sqrt((target_x - pos_x) ** 2 + (target_y - pos_y) ** 2))
                old_i = i    
                #print("old_i=", old_i)

                orient_diff = movement_list[i][2]-orientation_1
                while np.abs(orient_diff) >= 3:
                    if 3 < orient_diff <= 180 or -360 <= orient_diff < -180:
                        publisher.forward_turn_left_fast()
  #                      rospy.loginfo("Turning left.")
                    if -180 <= orient_diff < -3 or 180 < orient_diff <= 360:
                        publisher.forward_turn_right_fast()
                    orientation_1 = subscriber.get_rotation_matrix()
                    orient_diff = movement_list[i][2] - orientation_1
  #                  print(orient_diff)
                pose_calib(publisher, subscriber, movement_list, i)
                pos_x_2, pos_y_2, search_tuple_2 = findself(subscriber)
                i_2 = path.index(search_tuple_2)
                while i_2 == i:
                    publisher.forward()
                    pos_x_2, pos_y_2, search_tuple_2 = findself(subscriber)
                    i_2 = path.index(search_tuple_2)


        publisher.stop()
        break



def main():
    rospy.init_node('motor_pwm_publisher', anonymous=True)
    tf_subscriber = TFDataSubscriber()
    demand = demand_publisher()
    rospy.sleep(2)
    tags_data= maze.load_yaml_file('/home/jetson/workspace/catkin_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml')
    tags = tags_data['tag_bundles'][0]['layout']
    walls, walls_id, grid = maze.maze_wall(tags)
    init_x, init_y, startpoint = findself(tf_subscriber)
    print(startpoint)
    end = (3, 3)
    path = maze.a_star(walls, startpoint, end)
    movements = maze.path_to_movements(path)
    print(movements)#
#    pose_calib(demand, tf_subscriber)
    init_orientation(movements, demand, tf_subscriber)
    motor_motion(movements, path, demand, tf_subscriber)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
