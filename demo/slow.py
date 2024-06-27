import rospy
from tf2_msgs.msg import TFMessage
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64MultiArray
from collections import deque
import os

# TFDataSubscriber class for subscribing to apritag_data
class TFDataSubscriber:
    def __init__(self):
        self.id = None
        self.distance = 0
        self.relative_degree = None
        self.sub = rospy.Subscriber('apritag_data', Float64MultiArray, self.callback)
        
    def callback(self, msg):
        self.id = int(msg.data[0])
        self.distance = msg.data[1]
        self.relative_degree = msg.data[2]
        rospy.loginfo(f"Received tf_data: id={self.id}, distance={self.distance}, degree={self.relative_degree}")#打印接收到的tf_data：位置和旋转矩阵
        
    def get_id(self):
        return self.id

# 初始化全局变量
coordinates_queue = deque()
observed_tags = set()
visited_coordinates = []
current_position = (0, 0)
tags_file_path = "observed_tags.txt"

def initialize_queue():
    global coordinates_queue
    coordinates_queue.append((0, 1))
    coordinates_queue.append((1, 0))

def april_tag_callback(data):
    global observed_tags

    for detection in data.detections:
        tag_id = detection.id[0]
        if tag_id not in observed_tags:
            observed_tags.add(tag_id)
            with open(tags_file_path, 'a') as f:
                f.write(f"{tag_id}\n")
            if len(observed_tags) >= 16:
                rospy.signal_shutdown("Collected 16 tags. Stopping.")

def move_and_rotate():
    global current_position

    tf_subscriber = TFDataSubscriber()

    while not rospy.is_shutdown() and len(observed_tags) < 16:
        if not coordinates_queue:
            break

        # 弹出坐标
        target = coordinates_queue.popleft()
        visited_coordinates.append(target)

        # 走到坐标
        current_position = target
        rospy.loginfo(f"Moved to {target}")

        # 观察四周并读取April Tag
        # 回调函数会自动处理

        # 获取新坐标并加入队列
        new_coords = [(target[0] + 1, target[1]), (target[0], target[1] + 1)]
        for coord in new_coords:
            if coord not in visited_coordinates and coord not in coordinates_queue:
                coordinates_queue.appendleft(coord)

        rospy.sleep(1)  # 模拟每次移动和旋转需要1秒

def main():
    rospy.init_node('robot_navigation', anonymous=True)

    if os.path.exists(tags_file_path):
        os.remove(tags_file_path)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, april_tag_callback)

    initialize_queue()
    move_and_rotate()

    rospy.spin()

if __name__ == "__main__":
    main()
