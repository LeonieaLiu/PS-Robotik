import rospy
from std_msgs.msg import Float64MultiArray
from collections import deque
import os
import yaml

class TFDataSubscriber:
    def __init__(self, filename):
        self.id = None
        self.distance = 0
        self.relative_degree = None
        self.filename = filename
        self.visited_tags = self.load_existing_tags()
        self.sub = rospy.Subscriber('apritag_data', Float64MultiArray, self.callback)

    def callback(self, msg):
        self.id = str(int(msg.data[0]))  # 假设 id 是整数并转换为字符串
        self.distance = msg.data[1]
        self.degree = msg.data[2]

        rospy.loginfo(f"Received tf_data: id={self.id}, distance={self.distance}, degree={self.degree}")
        self.save_tag(self.id)

    def save_tag(self, tag):
        if tag not in self.visited_tags:
            self.visited_tags.add(tag)
            with open(self.filename, 'a') as file:
                file.write(tag + '\n')
            rospy.loginfo(f"Tag {tag} saved to {self.filename}")

    def load_existing_tags(self):#加载已保存的标签
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as file:
                return set(file.read().splitlines())
        return set()

def generate_map_from_tags(filename):#生成地图
    with open(filename, 'r') as file:
        tags = file.read().splitlines()
    # 生成地图的逻辑，这里简单打印标签列表
    print("Map generated with the following tags:")
    for tag in tags:
        print(tag)

def main():
    rospy.init_node('tf_data_subscriber_node')
    tags_file = "tags.yaml"  # 现有的tags信息文件
    output_file = TFDataSubscriber(tags_file)
    output_file.save_tag()

    
    # 保持节点运行
    rospy.spin()

    # 生成地图
    

if __name__ == "__main__":
    main()
