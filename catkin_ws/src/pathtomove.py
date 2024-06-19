import catkin_ws.src.maze_grid as maze
import time
import rospy
from motor.msg import MotorPWM



def path_to_motor(movement_list):
    height = movement_list.shape[0]
    for i in range(height - 1):
        if movement_list[i + 1][3] - movement_list[i][3] == 0:
            pwm_msg = MotorPWM()
            pwm_msg.pwm_left = 0.1
            pwm_msg.pwm_right = 0.1

            pub.publish(pwm_msg)

        if 0 < movement_list[i + 1][3] - movement_list[i][3] <= 180:
            pwm_msg = MotorPWM()
            pwm_msg.pwm_left = 0
            pwm_msg.pwm_right = 0.1

            pub.publish(pwm_msg)

        if 180 < movement_list[i + 1][3] - movement_list[i][3] < 360:
            pwm_msg = MotorPWM()
            pwm_msg.pwm_left = 0.1
            pwm_msg.pwm_right = 0

            pub.publish(pwm_msg)

        if -180 <= movement_list[i + 1][3] - movement_list[i][3] < 0:
            pwm_msg = MotorPWM()
            pwm_msg.pwm_left = 0.1
            pwm_msg.pwm_right = 0

            pub.publish(pwm_msg)

        if 180 < movement_list[i + 1][3] - movement_list[i][3] < 360:
            pwm_msg = MotorPWM()
            pwm_msg.pwm_left = 0
            pwm_msg.pwm_right = 0.1

            pub.publish(pwm_msg)

    time.sleep(1)
    return None

def main():
    yamlfile = maze.load_yaml_file('/Users/MADAO/PycharmProjects/pythonProject/project/tags.yaml')
    tags = maze.tags_data['tag_bundles'][0]['layout']
    walls, grid = maze.maze_wall(tags)
    start = (0, 0)
    end = (0, 1)
    path = maze.a_star(walls, start, end)
    movements = maze.path_to_movements(path)
    path_to_motor(movements)

if __name__ == "__main__":
    main()