import rospy
import time
import re
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped

class Maze_Listener:
    def __init__(self):
        rospy.loginfo("Initializing Maze_Listener")
        self.listener= tf.TransformListener()
        self.pub_maze = rospy.Publisher('maze_data', Float64MultiArray, queue_size=1)
        self.sub = rospy.Subscriber('/tf', TFMessage, self.process_maze, queue_size=1)
        self.rate = rospy.Rate(5)
        self.jetbot_world_position = None
    def process_maze(self, msg):
        # 更新Jetbot在世界坐标系中的位置
        self.update_position()

        # 处理maze变换的逻辑
        for transform in msg.transforms:
            if transform.child_frame_id == 'maze':
                self.process_data(transform)
    def update_position(self):
        try:
            # 等待直到变换可用
            self.listener.waitForTransform('/maze', '/mono_cam', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform('/maze', '/mono_cam', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF transform not available")
            return
            # 定义Jetbot坐标系原点在Jetbot坐标系中的位置
        jetbot_origin = PointStamped()
        jetbot_origin.header.frame_id = "mono_cam"
        jetbot_origin.header.stamp = rospy.Time(0)
        jetbot_origin.point.x = 0.0
        jetbot_origin.point.y = 0.0
        jetbot_origin.point.z = 0.0

        # 使用变换将Jetbot坐标系原点转换到世界坐标系
        world_point = self.listener.transformPoint("maze", jetbot_origin)
        self.jetbot_world_position = (world_point.point.x, world_point.point.y, world_point.point.z)
        rospy.loginfo("Jetbot origin in world frame: (%.2f, %.2f, %.2f)" %
                      (world_point.point.x, world_point.point.y, world_point.point.z))

    def process_data(self,transform):
        rospy.loginfo("Processing Maze Message")
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x,y,z= translation.x,translation.y,translation.z
        qx,qy,qz,qw=rotation.x,rotation.y,rotation.z, rotation.w
        quat= R.from_quat([qx,qy,qz,qw])
        Rotationmatrix = quat.as_matrix()
        world_y=np.array([0, 1, 0])
        jetbot_z=np.array([0, 0, 1])
        jetbot_z_world=np.dot(jetbot_z, Rotationmatrix)
        #计算jetbot z轴和world y轴夹角
        cross_product=np.cross(jetbot_z_world, world_y)
        angle_rad = np.arctan2(np.linalg.norm(cross_product), np.dot(jetbot_z_world, world_y))
        if cross_product[2] > 0:
            angle_rad=-angle_rad
        theta_degree = np.degrees(angle_rad)
        maze_x=np.abs(np.sin(np.abs(theta_degree))*np.abs(z)-np.cos(np.abs(theta_degree))*np.abs(x))
        maze_y=np.abs(np.sin(np.abs(theta_degree))*np.abs(x)+np.cos(np.abs(theta_degree))*np.abs(z))
        rospy.loginfo(f"Publishing tf_data: position=({maze_x}, {maze_y}), degree={theta_degree}")
        maze_msg = Float64MultiArray()
        maze_msg.data = [theta_degree, self.jetbot_world_position[0], self.jetbot_world_position[1],\
                         self.jetbot_world_position[2]]
        self.pub_maze.publish(maze_msg)
                

class TagDataHandler:
    def __init__(self):
        rospy.loginfo("Initializing Apriltag_Listener")
        self.pub = rospy.Publisher('apriltag_data', Float64MultiArray, queue_size=1)
        self.sub = rospy.Subscriber('/tf', TFMessage, self.process_tag, queue_size=1)
        self.rate = rospy.Rate(5)

    def process_tag(self,msg):
        rospy.loginfo("Processing April_tag Message")
        for transform in msg.transforms:
            if transform.child_frame_id.startswith('tag_'):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                x, z = translation.x, translation.z
                qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
                quat = R.from_quat([qx, qy, qz, qw])
                Rotationmatrix = quat.as_matrix()
                jetbot_z=np.array([0, 0, 1])
                tag_z=np.array([0, 0, -1])
                jetbot_z_tag = np.dot(jetbot_z, Rotationmatrix)
                cross_product = np.cross(tag_z,jetbot_z_tag)
                angle_rad = np.arctan2(np.linalg.norm(cross_product), np.dot(jetbot_z_tag, tag_z))
                if cross_product[1] > 0:
                    angle_rad = -angle_rad
                # 从jetbot z轴到apriltag z轴方向 顺时针为正 与世界坐标系判断相同
                theta_degree = np.degrees(angle_rad)
                distance=np.sqrt(x**2+z**2)
                if abs(theta_degree) < 30:
                    rospy.loginfo(f"Processing tag data:id=({transform.child_frame_id}) position=({x},{z}),distance=({distance}) ,theta_degree=({theta_degree})")
                    number = transform.child_frame_id.replace('tag_','')
                    tag_id = float(number)
                    tag_msg=Float64MultiArray()
                    tag_msg.data = [tag_id,distance,theta_degree]
                    self.pub.publish(tag_msg)

if __name__ == '__main__':
    rospy.init_node('tf_listener_node')
    maze_listener=Maze_Listener()
    tag_listener=TagDataHandler()
    rospy.spin()
