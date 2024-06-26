#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from Adafruit_BNO055 import BNO055

def imu_publisher():
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    bno = BNO055.BNO055(serial_port='/dev/serial0')

    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055!')

    while not rospy.is_shutdown():
        imu_msg = Imu()
        quat = bno.read_quaternion()
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]
        imu_msg.orientation.w = quat[0]

        rospy.loginfo(imu_msg)
        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
