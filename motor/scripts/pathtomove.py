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
        self.rate = rospy.Rate(5)

    def stop(self):
        pwm_msg = MotorPWM()
        self.msg_pwmcmd.pwm_left = 0.0
        self.msg_pwmcmd.pwm_right = 0.0
        self.pub.publish(pwm_msg)
        self.rate.sleep()
    def forward(self):
        pwm_msg = MotorPWM()
        self.msg_pwmcmd.pwm_left = 0.25
        self.msg_pwmcmd.pwm_right = 0.25
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

    def turn_left(self):
        pwm_msg = MotorPWM()
        self.msg_pwmcmd.pwm_left = -0.25
        self.msg_pwmcmd.pwm_right = 0.25
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

    def turn_right(self):
        pwm_msg = MotorPWM()
        self.msg_pwmcmd.pwm_left = 0.25
        self.msg_pwmcmd.pwm_right = -0.25
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        self.rate.sleep()

class TFDataSubscriber:
    def __init__(self):
        self.position = (0, 0)
        self.rotation_matrix = None
        self.sub = rospy.Subscriber('/tf_data', Float64MultiArray, self.callback)

    def callback(self, msg):
        self.position = (msg.data[0], msg.data[1])
        self.rotation_matrix = msg.data[2]
        rospy.loginfo(f"Received tf_data: position={self.position}, rotation_matrix={self.rotation_matrix}")
        
    def get_position(self):
        return self.position

    def get_rotation_matrix(self):
        return self.rotation_matrix


def is_located(goal, tf_subscriber):
    length = 0.07
    #use the center of the car
    current_position = list(tf_subscriber.get_position())
    current_orientation = tf_subscriber.get_rotation_matrix()

#    if current_orientation is None:
#        rospy.logwarn("current_orientation is None")
#    else:
#        rospy.loginfo(f"current_orientation: {current_orientation}")
    
    current_position[0] = current_position[0] - length * np.cos(current_orientation)
    current_position[1] = current_position[1] - length * np.sin(current_orientation)
    if np.abs(current_position[0]-0.25 * goal[0]+0.125) >= 0.025 or np.abs(current_position[1]-goal[1]*0.25 + 0.125)>=0.025:
        return 'False'
    else:
        return 'True'


def is_orientation(movement_list, tf_subscriber):
    current_position = list(tf_subscriber.get_position())
    current_orientation = tf_subscriber.get_rotation_matrix()

    current_orientation = movement_list[0][2]
    if np.abs(current_orientation - init_orientation)>=3:
        return 'False'
    else:
        return 'True'


def to_startpoint(movement_list, start, demand_publisher, tf_subscriber):
    while not rospy.is_shutdown():
        while is_located(start, tf_subscriber) == 'False':

            length = 0.07
            current_position = list(tf_subscriber.get_position())
            current_orientation = tf_subscriber.get_rotation_matrix()
            current_position[0] = current_position[0] - length * np.cos(current_orientation)
            current_position[1] = current_position[1] - length * np.sin(current_orientation)
            delta_x = start[0] - current_position[0]
            delta_y = start[1] - current_position[1]
            rotation_angle = np.arctan(delta_x/delta_y)
            angle=np.degrees(rotation_angle)
            if 3 <= angle <= 180:
                print('1')
                demand_publisher.turn_left()
                demand_publisher.forward()

            if -180 <= angle <= -3:
                print('11')
                demand_publisher.turn_right()
                demand_publisher.forward()
               
            if -3 < angle < 3:
                print('111')
                demand_publisher.forward()

        while is_orientation(movement_list, tf_subscriber) == 'False':
            print('2')
            current_position_2 = list(tf_subscriber.get_position())
            current_orientation_2 = tf_subscriber.get_rotation_matrix()
            angle = current_orientation_2 - init_orientation
            if 0 <= angle <= 180:
                demand_publisher.turn_left()
            else:
                demand_publisher.turn_right()


def motor_motion(movement_list, path, demand_publisher, tf_subscriber):
    length = 0.07

    while not rospy.is_shutdown():
        current_position = list(tf_subscriber.get_position())
        current_orientation = tf_subscriber.get_rotation_matrix()
        current_position[0] = current_position[0] - length * np.cos(current_orientation)
        current_position[1] = current_position[1] - length * np.sin(current_orientation)
        x = math.floor(current_position[0]/0.25)
        y = math.floor(current_position[1]/0.25)
        search_tuple = (x, y)
        if search_tuple not in path:
            rospy.loginfo("You have run out of the path!")
            break
        else:
            i = path.index(search_tuple)
            if i == len(path) - 1:
                rospy.loginfo("Goal reached.")
                break
            if movement_list[i+1][2] - movement_list[i][2] == 0 and i != len(path) - 1:
                demand_publisher.forward()
                continue
            if movement_list[i+1][2] - movement_list[i][2] != 0 and i != len(path) - 1:
                demand_publisher.stop()
                position_1 = list(tf_subscriber.get_position())
                orientation_1 = data['rotation_matrix']
                orient_diff = orientation_1 - movement_list[i+1][2]
                while np.abs(orient_diff) >= 2:
                    if 2 < orient_diff <= 180:
                        demand_publisher.turn_left()
                    else:
                        demand_publisher.turn_right()
                    position_1 = list(tf_subscriber.get_position())
                    orientation_1 = tf_subscriber.get_rotation_matrix()
                    orient_diff = orientation_1 - movement_list[i + 1][2]
    demand_publisher.stop()


def main():
    rospy.init_node('motor_pwm_publisher', anonymous=True)
    tf_subscriber = TFDataSubscriber()
    demand = demand_publisher()
    rospy.sleep(2)
    tags_data= maze.load_yaml_file('/home/jetson/workspace/catkin_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml')
    tags = tags_data['tag_bundles'][0]['layout']
    walls, grid = maze.maze_wall(tags)
    start = (0, 0)
    end = (0, 1)
    path = maze.a_star(walls, start, end)
    movements = maze.path_to_movements(path)
    to_startpoint(movements, start, demand, tf_subscriber)
    motor_motion(movements, path, demand, tf_subscriber)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
