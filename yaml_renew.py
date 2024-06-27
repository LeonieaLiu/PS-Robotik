#!/usr/bin/env python
import std_msgas.msg import Float64MultiArray
import rospy
import yaml # 用于处理YAML文件
import tf 
from tf2_msgs.msg import TFMessage
from apriltag_ros.msg import AprilTagDetectionArray # 用于接收AprilTag检测结果的消息类型

class AprilTagSaver:
    def __init__(self):
        self.original_yaml_file = '/home/jetson/workspace/catkin_ws/config/apriltag/tags.yaml'  # 原始YAML文件路径
        self.updated_yaml_file = '/Downloads/tags.yaml'  # 更新后的YAML文件路径
        self.tag_info = self.load_existing_tags()  # 加载已有的Tag信息

        rospy.init_node('apriltag_saver', anonymous=True)  # 初始化ROS节点
        rospy.Subscriber('apriltag_tag', AprilTagDetectionArray, self.tag_callback) 
        rospy.Subscriber('/tf',TFMessage , self.tag_callback)  # 订阅/apriltag_tag话题
        rospy.loginfo("AprilTagSaver node initialized and subscribed to /apriltag_tag")#打印日志消息


    def load_existing_tags(self): # 方法尝试从原始YAML文件加载已有的Tag信息，如果文件不存在，则返回一个空字典。
        try:
            with open(self.original_yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                if 'standalone_tags' not in data:
                    data['standalone_tags'] = []
                rospy.loginfo("Loaded existing tags from YAML file")
                return {tag['id']: tag for tag in data['standalone_tags']}
        except FileNotFoundError:
            rospy.logwarn("Original YAML file not found, starting with an empty tag list")
            return {}  # 如果文件不存在，则返回空字典
        
    def tf_callback(self, msg):
        rospy.loginfo('Received tf data')
        for transform in msg.transforms:#遍历msg.transforms中的每个变换，获取每个变换的frame_id和child_frame_id
            rospy.loginfo(f"TF frame_id: {transform.header.frame_id}, child_frame_id: {transform.child_frame_id}")# 打印日志消息
            
    def save_to_yaml(self):
        with open(self.updated_yaml_file, 'w') as file:#将更新后的self.tag_info字典保存到YAML文件中
            yaml.dump({'standalone_tags': list(self.tag_info.values())}, file)#将self.tag_info字典转换为列表，并将其保存到YAML文件中
        rospy.loginfo(f"Updated YAML file saved to {self.updated_yaml_file}")#打印日志消息
        
    def tag_callback(self, msg): # 接收到AprilTag检测结果时被调用
        rospy.loginfo('Received tag detection data')
        new_data_added = False
        for detection in msg.transforms: # 遍历msg.detections中的每个检测结果，获取每个AprilTag的ID和位置信息
            tag_id = detection.id[0] # 获取AprilTag的ID
            if tag_id not in self.tag_info: # 检查当前检测结果的ID是否已经存在于self.tag_info字典中
                tag_data = {
                    'id': tag_id,
                    'size': detection.size[0],
                    'x': detection.pose.pose.pose.position.x,
                    'y': detection.pose.pose.pose.position.y,
                    'z': detection.pose.pose.pose.position.z,
                    'qw': detection.pose.pose.pose.orientation.w,
                    'qx': detection.pose.pose.pose.orientation.x,
                    'qy': detection.pose.pose.pose.orientation.y,
                    'qz': detection.pose.pose.pose.orientation.z
                }
                self.tag_info[tag_id] = tag_data # 将tag_data字典添加到self.tag_info字典中
                new_data_added = True
                self.save_to_yaml() # 调用save_to_yaml方法，将更新后的self.tag_info字典保存到YAML文件中
                rospy.loginfo(f"New tag detected and saved: {tag_data}")
if __name__ == '__main__':
    try:
        AprilTagSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
