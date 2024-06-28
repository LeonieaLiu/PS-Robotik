import rospy
#from apriltag_data import AprilTagDetectionArray
from std_msgs.msg import Float64MultiArray
import os

#订阅/apriltag_tag话题，创建一个合集observed_tags 来存储已经检测到的AprilTag ID
class AprilTagRecorder:
    def __init__(self, output_file):
        self.output_file = '/Downloads/tags.yaml'
        self.observed_tags = set()# 用于存储已经观察到的Tag ID
        self.sub = rospy.Subscriber('apriltag_data', Float64MultiArray, self.callback)
        if os.path.exists(self.output_file):
            os.remove(self.output_file)

    def callback(self, msg):#处理接收到的数据，若检测到的AprilTag不存在于observed_tags中，则将其添加到observed_tags中，并将其ID写入到output_file中，将其写入并添加到observed_tags中
        #for detection in :#在AprilTagdetectionArray中存在detections字段，其中包含了检测到的AprilTag的信息
            self.tag_id = msg.data[0]
            #self.tag_distance = msg.data[1]
            #self.tag_degree = msg.data[2]
            self.id = int(tag_id)
            if tag_id not in self.observed_tags:
                self.observed_tags.add(tag_id)
                with open(self.output_file, 'a') as f:
                    f.write(f"{tag_id}\n")
                rospy.loginfo(f"Recorded new tag ID: {tag_id}")
            else:
                rospy.loginfo(f"Tag ID {tag_id} already recorded")
                
    def get_id(self):
        try:
            return self.id
        except:
            rospy.loginfo("No id received")
            
            
def main():
    rospy.init_node('april_tag_recorder', anonymous=True)
    recorder = AprilTagRecorder("observed_tags.txt")
    rospy.spin()

if __name__ == "__main__":
    main()
