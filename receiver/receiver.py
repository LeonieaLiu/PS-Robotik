#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64MultiArray


def angle_callback(msg):
    rospy.loginfo("Received angle: {}".format(msg.data))


def matrix_callback(msg):
    rospy.loginfo("Received matrix: {}".format(msg.data))

if __name__ == '__main__':
    rospy.init_node('receiver')
    angle_sub = rospy.Subscriber('/jetbot_angle', Float32, angle_callback, queue_size=1)
    matrix_sub = rospy.Subscriber('/adjusted_matrix', Float64MultiArray, matrix_callback, queue_size=1)
    rospy.spin()
    