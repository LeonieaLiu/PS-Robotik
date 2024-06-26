#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera_publisher():
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
    rospy.init_node('camera_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            rospy.loginfo('Publishing image')
            pub.publish(img_msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
