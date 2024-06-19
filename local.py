#!/usr/bin/env python
import rospy
import time
import mavros

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import signal
import sys
import math
import numpy as np

# 全局变量
global r2d
global d2r
r2d = 180/np.pi
d2r = np.pi/180

def signal_handler(signal, frame):
    print('pressed ctrl + c!!!')
    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# 设置电机控制器
motor_driver = Adafruit_MotorHAT(i2c_bus=1)
motor_left_ID = 1
motor_right_ID = 2
motor_left = motor_driver.getMotor(motor_left_ID)
motor_right = motor_driver.getMotor(motor_right_ID)

def set_speed(motor_ID, value): # 设置电机速度
    max_pwm = 115.0
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
    if motor_ID == 1:
        motor = motor_left
    elif motor_ID == 2:
        motor = motor_right
    else:
        rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
        return
    motor.setSpeed(speed)
    if value > 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)

def all_stop(): # 停止所有电机
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)
    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)

def pause(): # 暂停电机
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)

class jetbot():
    def __init__(self):
        rospy.init_node('control_jetbot_imu', anonymous=True)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.local_val_callback, queue_size=1)
        rospy.Subscriber('/jet_cmd', String, self.desired_val_callback, queue_size=1)
        rospy.Subscriber('/missin_fin', String, self.isfin_callback, queue_size=1)

        self.slowly = 0
        self.chk = 0 
        self.d_speed = 0
        self.d_degree = 0
        self.desired_in = 0
        self.yaw_compensate = 0
        self.c_yaw_raw = 0

    def local_val_callback(self, msg):
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (c_roll, c_pitch, self.c_yaw_raw) = euler_from_quaternion(orientation_list)
        self.c_yaw = self.c_yaw_raw - self.yaw_compensate
        if self.c_yaw > np.pi:
            self.c_yaw -= 2*np.pi
        elif self.c_yaw < -np.pi:
            self.c_yaw += 2*np.pi
        print(rospy.Time.now(), self.c_yaw*r2d)

    def desired_val_callback(self, msg):
        self.desired_in = 1
        Data = msg.data
        d_degree, d_speed = Data.split(',')
        self.d_degree = float(d_degree) * d2r
        self.d_speed = float(d_speed)
        print("Desired_deg : %f, Desired_lin_vel : %f" %(self.d_degree, self.d_speed))

    def isfin_callback(self, msg):
        self.chk = int(msg.data)
        if self.chk == 1:
            all_stop()
            print("stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    def set_lin_velocity(self, lin_vel):
        print('i am here and vel : %f'%lin_vel)
        set_speed(motor_left_ID, lin_vel)
        set_speed(motor_right_ID, lin_vel)

    def set_ang_velocity(self):
        print('i am ang')
        error = (self.d_degree - self.c_yaw)
        if error > 180:
            error -= 360
        if error < -180:
            error += 360
        error = error * 0.7  # P增益
        print(error * d2r)
        if error > 0:  # 逆时针
            set_speed(motor_left_ID, -error * d2r)
            set_speed(motor_right_ID, error * d2r)
        elif error < 0:  # 顺时针
            set_speed(motor_left_ID, -error * d2r)
            set_speed(motor_right_ID, error * d2r)

rbt = jetbot()
time.sleep(2)

if __name__ == '__main__':
    r = rospy.Rate(10)
    while 1:
        if rbt.desired_in == 1:
            while abs(rbt.d_degree - rbt.c_yaw * r2d) > 23:
                rbt.set_ang_velocity()
                r.sleep()
            rbt.yaw_compensate = rbt.c_yaw
            rbt.set_lin_velocity(rbt.d_speed)
            r.sleep()
