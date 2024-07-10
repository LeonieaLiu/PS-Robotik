import rospy
from std_msgs.msg import Float64MultiArray
from motor.msg import MotorPWM
import pth2 as pth


def test_sleep(publisher):
    publisher.stay_turn_left_fast()
    rospy.sleep(0.5)
    publisher.stop()
    rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('test_motor', anonymous=True)
    demand = pth.demand_publisher()
    test_sleep(demand)