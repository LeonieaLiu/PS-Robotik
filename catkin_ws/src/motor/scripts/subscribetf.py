import rospy
import time
import re
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray


def quaternion_to_matrix(q):
    """Convert a quaternion into a 4x4 transformation matrix."""
    r = R.from_quat(q)
    R_matrix = r.as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_matrix
    return T


def inverse_transformation_matrix(T):
    """Compute the inverse of a 4x4 transformation matrix."""
    R_inv = T[:3, :3].T
    t_inv = -R_inv @ T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv



class TFListener:
    def __init__(self):
        rospy.loginfo("Initializing TFListener")
        self.pub = rospy.Publisher('/tf_data', Float64MultiArray, queue_size=1)
        self.sub = rospy.Subscriber('/tf', TFMessage, self.Orientation_Location, queue_size=1)
        self.rate = rospy.Rate(5)


    def Orientation_Location(self, msg):
        rospy.loginfo("Received TFMessage")
        global tf_data
        for transform in msg.transforms:
            if transform.child_frame_id == 'maze':

                rospy.loginfo("Processing world frame transform")
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.translation = {
                    'x': translation.x,
                    'y': translation.y,
                    'z': translation.z
                }
                x = self.translation['x']
                y = self.translation['y']
                self.rotation = {
                    'qx': rotation.x,
                    'qy': rotation.y,
                    'qz': rotation.z,
                    'qw': rotation.w
                }
                qx = self.rotation['qx']
                qy = self.rotation['qy']
                qz = self.rotation['qz']
                qw = self.rotation['qw']
                self.q = [qx, qy, qz, qw]
                self.Rotationmatrix = quaternion_to_matrix(self.q)
                self.Inverse = inverse_transformation_matrix(self.Rotationmatrix)
                R_a = np.array([[0, -1, 0,0], [0, 0, -1,0], [1, 0, 0,0],[0, 0, 0,1]])
                Rotation_mono_to_frame = np.dot(self.Inverse, R_a)
                #计算jetbot z轴和迷宫中心x轴夹角
                rotation_angle=-np.arcsin(self.Rotationmatrix[2,0])
                theta_degree=np.degrees(rotation_angle)
                # print(Rotation_mono_to_frame)
                rospy.loginfo(f"Publishing tf_data: position=({x}, {y}), rotation_matrix={theta_degree}")
                msg = Float64MultiArray()
                msg.data = [x, y, theta_degree]
                self.pub.publish(msg)
                #print("Position: ", tf_data['position'])
                #print("Rotation Matrix: ", tf_data['rotation_matrix'])
                self.rate.sleep()


                
def get_tf_data():
    global tf_data
    """返回最新的 tf 数据"""
    return tf_data
            
            
if __name__ == '__main__':
    rospy.init_node('tf_listener_node')
    rospy.loginfo("Starting TF Listener Node")
    listener = TFListener()

    rospy.spin()
