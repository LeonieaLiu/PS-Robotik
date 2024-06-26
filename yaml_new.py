#!/usr/bin/env python

import rospy
import yaml # 用于处理YAML文件
from apriltag_ros.msg import AprilTagDetectionArray # 用于接收AprilTag检测结果的消息类型

class AprilTagSaver:
    def __init__(self):
        self.original_yaml_file = '/path/to/original_tags.yaml'  # 原始YAML文件路径
        self.updated_yaml_file = '/path/to/updated_tags.yaml'  # 更新后的YAML文件路径
        self.tag_info = self.load_existing_tags()  # 加载已有的Tag信息

        rospy.init_node('apriltag_saver', anonymous=True)  # 初始化ROS节点
        rospy.Subscriber('apriltag_tag',Float64MultiArray , self.tag_callback)  # 订阅/tag_detections话题

    def load_existing_tags(self): # 方法尝试从原始YAML文件加载已有的Tag信息，如果文件不存在，则返回一个空字典。
        try:
            with open(self.original_yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                return {tag['id']: tag for tag in data['standalone_tags']}
        except FileNotFoundError:
            return {}  # 如果文件不存在，则返回空字典

    def tag_callback(self, msg): # 接收到AprilTag检测结果时被调用
        for detection in msg.detections: # 遍历msg.detections中的每个检测结果，获取每个AprilTag的ID
            tag_id = detection.id[0] # 获取当前检测结果的ID。detection.id是一个列表，包含一个元素，即AprilTag的ID。
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
                self.save_to_yaml() # 调用save_to_yaml方法，将更新后的self.tag_info字典保存到YAML文件中
                rospy.loginfo(f"New tag detected and saved: {tag_data}")

    def save_to_yaml(self):
        with open(self.updated_yaml_file, 'w') as file:
            yaml.dump({'standalone_tags': list(self.tag_info.values())}, file)

if __name__ == '__main__':
    try:
        AprilTagSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
