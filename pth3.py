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
        self.msg_pwmcmd.pwm_left = -0.19
        self.msg_pwmcmd.pwm_right = -0.19
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def stay_turn_left_slow(self):
        self.msg_pwmcmd.pwm_left = 0.2
        self.msg_pwmcmd.pwm_right = -0.2
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

    def stay_turn_right_slow(self):
        self.msg_pwmcmd.pwm_left = -0.2
        self.msg_pwmcmd.pwm_right = 0.2
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def stay_turn_left_fast(self):
        self.msg_pwmcmd.pwm_left = 0.22
        self.msg_pwmcmd.pwm_right = -0.22
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

    def stay_turn_right_fast(self):
        self.msg_pwmcmd.pwm_left = -0.22
        self.msg_pwmcmd.pwm_right = 0.22
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
    
    def forward_turn_left_fast(self):
        self.msg_pwmcmd.pwm_left = 0.0
        self.msg_pwmcmd.pwm_right = -0.25
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_turn_right_fast(self):
        self.msg_pwmcmd.pwm_left = -0.25
        self.msg_pwmcmd.pwm_right = 0.0
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_turn_left_slow(self):
        self.msg_pwmcmd.pwm_left = 0.0
        self.msg_pwmcmd.pwm_right = -0.21

        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()
        
    def forward_turn_right_slow(self):
        self.msg_pwmcmd.pwm_left = -0.21
        self.msg_pwmcmd.pwm_right = 0.0
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()


class TFDataSubscriber:
    def __init__(self):
        self.position_x = 100
        self.position_y = 100
        self.distance = 100
        self.orientation = 361
        self.imu_degree = 361
        self.id = -1
        self.rotation_matrix = None
        self.sub = rospy.Subscriber('maze_data', Float64MultiArray, self.callback_maze)
        self.sub = rospy.Subscriber('apriltag_data', Float64MultiArray, self.callback_apriltag)
        self.sub = rospy.Subscriber('Imu_data', Float64MultiArray, self.callback_imu)

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
        
    def callback_imu(self, msg):
        self.imu_degree = msg.data[0]
    
    def get_imu_degree(self):
        return self.imu_degree
        
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
#    print(orientation)
    orient_diff = orientation - movements[i][2]
    while not -3 <= orient_diff <= 3 or orient_diff >= 357:
        if -180 <= orient_diff <= -3 or 180 <= orient_diff:
            publisher.forward_turn_left_slow()
#            rospy.loginfo(f"Calibrating left.")
            publisher.stop()
        if 3 <= orient_diff < 180 or orient_diff <= -180:
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
    current_imu = 361
    while current_orientation == 361 or current_orientation == None or current_imu == 361:
        rospy.loginfo("Looking for orientation.")
        current_orientation = subscriber.get_rotation_matrix()
        current_imu = subscriber.get_imu_degree()
        rospy.sleep(1)
#        print(current_orientation)
    first_orientation = movement_list[0][2]
    angle = first_orientation - current_orientation
    target_imu = current_imu + angle
    if target_imu > 180:
        target_imu = target_imu - 360
    elif target_imu < -180:
        target_imu = target_imu + 360

    imu_diff = target_imu - current_imu
    while np.abs(imu_diff) >= 8:
        if -180 < imu_diff < -8 or 180 <= imu_diff:
            publisher.stay_turn_right_slow()
#            rospy.loginfo(f"turning left.")
#            print("target:", target_imu, "current:", current_imu)
#            publisher.stop()
        if 8 < imu_diff < 180 or imu_diff <= -180:
            publisher.stay_turn_left_slow()
#            rospy.loginfo(f"turning right.")
#            print("target:", target_imu, "current:", current_imu)
#            publisher.stop()
#        current_orientation = subscriber.get_rotation_matrix()
        current_imu = subscriber.get_imu_degree()
        imu_diff = target_imu - current_imu
#       print(current_orientation)
    print("already reached angle range")
    publisher.stop()
    current_orientation = subscriber.get_rotation_matrix()
    orient_diff = first_orientation - current_orientation
    while np.abs(orient_diff) >= 4:
        if 4 < orient_diff <= 180 or -360 <= orient_diff < -180:
            publisher.stay_turn_left_fast()
            publisher.stop()
            orientation_1 = subscriber.get_rotation_matrix()
#            print("Turning left. Current direction:", orientation_1, "Target:", target)
            orient_diff = first_orientation - orientation_1
#            print(orient_diff)
        if -180 <= orient_diff < -4 or 180 < orient_diff <= 360:
            publisher.stay_turn_right_fast()
            publisher.stop()
            orientation_1 = subscriber.get_rotation_matrix()
#            print("Turning left. Current direction:", orientation_1, "Target:", target)
            orient_diff = first_orientation - orientation_1
#            print(orient_diff)
    publisher.stop()
    print('Initialized.')
    

def centralization(orientation, x, y):
    center_x = x
    center_y = y
    if -40 < orientation < 40:
        center_x = x
        center_y = y - 0.04
    elif -130 < orientation < -50:
        center_x = x - 0.04
        center_y = y
    elif 50 < orientation < 130:
        center_x = x + 0.04
        center_y = y
    elif -180 <= orientation < -140 or 140 < orientation <= 180:
        center_x = x
        center_y = y + 0.04
    else:
        print("no legal orientation")
    return center_x, center_y


# def distance_mono_edge(wall_list, apriltagid, x, y, publisher):
#     dis_mono_wall = 1000
#     if dis_mono_wall == 1000:
#         print("apriltag_id=", apriltagid)
#         for item in wall_list:
#             if item[2] == apriltagid:
#                 if not isinstance(item[0], int):
#                     wall_x = (item[0] + 0.5) * 0.25
#                     dis_mono_wall = np.abs(wall_x - x)
#                 if not isinstance(item[1], int):
#                     wall_y = (item[1] + 0.5) * 0.25
#                     dis_mono_wall = np.abs(wall_y - y)
#
#     if dis_mono_wall == 1000:
#         if apriltagid % 2 != 0:
#             apriltagid = apriltagid - 1
#             print("new_id=",apriltagid)
#         else: apriltagid += 1
#
#         if item[2] == apriltagid:
#             if not isinstance(item[0], int):
#                 wall_x = (item[0] + 0.5) * 0.25
#                 dis_mono_wall = np.abs(wall_x - x)
#             if not isinstance(item[1], int):
#                 wall_y = (item[1] + 0.5) * 0.25
#                 dis_mono_wall = np.abs(wall_y - y)
#         print(dis_mono_wall)
#
#     dis_mono_edge = dis_mono_wall - math.floor(dis_mono_wall / 0.25) * 0.25
#     return dis_mono_edge


def motor_motion(movement_list, path, publisher, subscriber, true_path):
    rospy.loginfo(f"Start motion.")
    search_tuple = None
# num=0
    old_i = -1
    while not rospy.is_shutdown():
        while search_tuple == None:
            rospy.loginfo(f"Looking for position.")
            pos_x, pos_y, search_tuple = findself(subscriber)
        rospy.loginfo(f"x:{pos_x} y:{pos_y} Position: {search_tuple}")
        i = path.index(search_tuple)
        print("i=", i)
#        print(len(path))

        while not i == len(path):
            pos_x, pos_y, search_tuple = findself(subscriber)
            i = path.index(search_tuple)
#            print("posx=", pos_x, "posy=", pos_y, "i=", i, "length=", len(path))
            centered_x, centered_y = centralization(pos_x, pos_y, i)
            true_path.append((centered_x, centered_y))
            # Transport to final block
            if i == len(path) - 1:
                orientation_1 = subscriber.get_rotation_matrix()
                centered_x_1, centered_y_1 = centralization(orientation_1, pos_x, pos_y)
                if old_i == i:
                    target_x = (path[i+1][0] + 0.5) * 0.25
                    target_y = (path[i+1][1] + 0.5) * 0.25
                else:
                    target_x = (path[i][0] + 0.5) * 0.25
                    target_y = (path[i][1] + 0.5) * 0.25

                while not (np.sqrt((target_x - centered_x_1) ** 2 + (target_y - centered_y_1) ** 2) <= 0.03):
                    publisher.forward_slow()
                    pos_x, pos_y, search_tuple_1 = findself(subscriber)
                    centered_x_1, centered_y_1 = centralization(orientation_1, pos_x, pos_y)
                    if np.abs(orientation_1) < 20 or np.abs(orientation_1) > 160:
                        if np.abs(target_y - centered_y_1) < 0.025:
                            break
                    elif 70 < np.abs(orientation_1) < 110:
                        if np.abs(target_x - centered_x_1) < 0.025:
                            break
                true_path.append((centered_x_1, centered_y_1))
                publisher.stop()
                rospy.loginfo("Goal reached.")
                break

            # Forward
            if i < 1 or movement_list[i][2] - movement_list[i-1][2] == 0 and i != len(path) - 1:
                publisher.forward_slow()
                rospy.sleep(0.1)
                pose_calib(publisher, subscriber, movement_list, i)

            # Turning
            if movement_list[i][2] - movement_list[i-1][2] != 0 and i >= 1 and i != len(path) - 1:
                orientation_2 = subscriber.get_rotation_matrix()
                centered_x_2, centered_y_2 = centralization(orientation_2, pos_x, pos_y)
                if old_i == i:
                    target_x = (path[i+1][0] + 0.5) * 0.25
                    target_y = (path[i+1][1] + 0.5) * 0.25
                else:
                    target_x = (path[i][0] + 0.5) * 0.25
                    target_y = (path[i][1] + 0.5) * 0.25

                while not (np.sqrt((target_x - centered_x_2) ** 2 + (target_y - centered_y_2) ** 2) <= 0.03):
                    publisher.forward_slow()
                    pos_x, pos_y, search_tuple_1 = findself(subscriber)
                    centered_x_2, centered_y_2 = centralization(orientation_2, pos_x, pos_y)
                    if np.abs(orientation_2) < 20 or np.abs(orientation_2) > 160:
                        if np.abs(target_y - centered_y_2) < 0.025:
                            break
                    elif 70 < np.abs(orientation_2) < 110:
                        if np.abs(target_x - centered_x_2) < 0.025:
                            break
                true_path.append((centered_x_2, centered_y_2))
                publisher.stop()
                old_i = i

                orient_diff = movement_list[i][2]-orientation_2
                current_imu = subscriber.get_imu_degree()
                target_imu = current_imu + orient_diff
                imu_diff = target_imu - current_imu
                if target_imu > 180:
                    target_imu = target_imu - 360
                elif target_imu < -180:
                    target_imu = target_imu + 360

                while np.abs(imu_diff) >= 8:
                    if 8 < imu_diff <= 180 or imu_diff < -180:
                        publisher.stay_turn_left_slow()
                    if -180 <= imu_diff < -8 or 180 < imu_diff:
                        publisher.stay_turn_right_slow()
                    current_imu = subscriber.get_imu_degree()
                    imu_diff = target_imu - current_imu

                orientation_3 = subscriber.get_rotation_matrix()
                orient_diff = movement_list[i][2] - orientation_3
                while np.abs(orient_diff) >= 4:
                    if 4 < orient_diff <= 180 or -360 <= orient_diff < -180:
                        publisher.stay_turn_left_fast()
                        publisher.stop()
                        orientation_4 = subscriber.get_rotation_matrix()
                        #            print("Turning left. Current direction:", orientation_1, "Target:", target)
                        orient_diff = movement_list[i][2] - orientation_4
                    if -180 <= orient_diff < -4 or 180 < orient_diff <= 360:
                        publisher.stay_turn_right_fast()
                        publisher.stop()
                        orientation_4 = subscriber.get_rotation_matrix()
                        #            print("Turning left. Current direction:", orientation_1, "Target:", target)
                        orient_diff = movement_list[i][2] - orientation_4
                publisher.stop()

                pos_x_2, pos_y_2, search_tuple_2 = findself(subscriber)
                i_2 = path.index(search_tuple_2)
                while i_2 == i:
                    publisher.forward_slow()
                    pos_x_2, pos_y_2, search_tuple_2 = findself(subscriber)
                    i_2 = path.index(search_tuple_2)
                    # rospy.sleep(0.1)
                    # pose_calib(publisher, subscriber, movement_list, i_2)

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
    end = (2, 3)
    path = maze.a_star(walls, startpoint, end)
    movements = maze.path_to_movements(path)
    print(movements)#
#    pose_calib(demand, tf_subscriber)
    init_orientation(movements, demand, tf_subscriber)
    motor_motion(walls_id, movements, path, demand, tf_subscriber)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
