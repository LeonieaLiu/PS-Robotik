#!/usr/bin/env python

import rospy
import yaml
from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagSaver:
    def __init__(self):
        self.original_yaml_file = '/home/jetson/workspace/catkin_ws/config/apriltag/tags.yaml'  # 原始YAML文件路径
        self.updated_yaml_file = '/home/jetson/Downloads/updated_tags.yaml'  # 更新后的YAML文件路径
        self.tag_info = self.load_existing_tags()  # 加载已有的Tag信息

        rospy.init_node('apriltag_saver', anonymous=True)
        self.subscriber = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.tag_callback)

    def load_existing_tags(self):
        try:
            with open(self.original_yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                # 将列表转换为以ID为键的字典
                return {str(tag['id']): tag for tag in data.get('standalone_tags', [])}
        except FileNotFoundError:
            rospy.loginfo("Original YAML file not found, starting with an empty set of tags.")
            return {}

    def tag_callback(self, msg):
        new_tags = False
        for detection in msg.detections:
            tag_id = str(detection.id[0])
            if tag_id not in self.tag_info:
                rospy.logwarn(f"Tag ID {tag_id} not found in original YAML file.")
            else:
                # 提取的信息已在初始化时加载，直接使用
                self.save_to_yaml(tag_id)
                rospy.loginfo(f"Tag ID {tag_id} data added to new YAML file.")

    def save_to_yaml(self, tag_id):
        try:
            with open(self.updated_yaml_file, 'a') as file:  # 使用追加模式
                yaml.dump({tag_id: self.tag_info[tag_id]}, file)
            rospy.loginfo("Updated YAML file saved with new tag data.")
        except Exception as e:
            rospy.logerr(f"Failed to save YAML: {e}")

if __name__ == '__main__':
    try:
        ats = AprilTagSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AprilTag Saver node terminated.")
