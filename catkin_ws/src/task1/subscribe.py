import time
import yaml
import re
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from scipy.spatial.transform import Rotation as R

class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path == "/home/jetson/workspace/catkin_ws/src/task1/detector_record.txt":
            print("File has been modified")
            with open(event.src_path, 'r') as file:
                content = file.read()  # 读取整个文件内容
                self.extract_data(content)

    def extract_data(self, content):
        # 使用正则表达式匹配所有 ID 及其对应的位置和方向信息
        
        id_regex = re.compile(
            r"id: \[(\d+)\].*?position:\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+).*?"
            r"orientation:\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)\s*w:\s*([-\d.]+)",
            re.DOTALL
        )
        
        for match in id_regex.finditer(content):
            #摄像头读取的数据，即april_tag在jetbot坐标系里面的数据
            
            id_num, x, y, z, qx, qy, qz, qw = match.groups()
            id_num=int(id_num)
            t_ca=[x,y,z]
            q_ca=[qx,qy,qz,qw]
            #已知的april_tag在迷宫坐标系下的数据
            print(t_ca)
            tag_data=get_tag_coordinates_and_orientation('/home/jetson/workspace/catkin_ws/src/task1/tags.yaml',id_num)
            if tag_data is not None and len(tag_data) == 7:
                a_x, a_y, a_z, a_qw, a_qx, a_qy, a_qz = tag_data
                t_ma = [a_x, a_y, a_z]
                q_ma = [a_qx, a_qy, a_qz, a_qw]
                # 计算Jetbot相机相对于迷宫中心的位置和姿态
                t_mc, q_mc = transform_position(t_ma, q_ma, t_ca, q_ca)
                # 计算相机相对于迷宫中心的距离
                distance = np.linalg.norm(t_mc)
                print("distance:", distance)
            else:
                print(f"Error: Tag data for ID {id_num} is not valid")

def quaternion_to_matrix(q):
    """Convert a quaternion into a 4x4 transformation matrix."""
    r = R.from_quat(q)
    R_matrix = r.as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_matrix
    return T
def transformation_matrix(t, q):
    """Construct a 4x4 transformation matrix from translation and quaternion."""
    T = quaternion_to_matrix(q)
    T[:3, 3] = t
    return T
def inverse_transformation_matrix(T):
    """Compute the inverse of a 4x4 transformation matrix."""
    R_inv = T[:3, :3].T
    t_inv = -R_inv @ T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv
def transform_position(t_ma, q_ma, t_ca, q_ca):
    # Construct transformation matrices
    T_ma = transformation_matrix(t_ma, q_ma)
    T_ca = transformation_matrix(t_ca, q_ca)

    # Compute the inverse of T_ca to get T_ac
    T_ac = inverse_transformation_matrix(T_ca)

    # Compute the resulting transformation matrix T_mc
    T_mc = np.dot(T_ma, T_ac)

    # Extract translation and rotation (as quaternion) from T_mc
    t_mc = T_mc[:3, 3]
    r_mc = R.from_matrix(T_mc[:3, :3])
    q_mc = r_mc.as_quat()

    return t_mc, q_mc

def get_tag_coordinates_and_orientation(file_path, tag_id):
    """根据ID加载特定标签的坐标和方向数据"""
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        if 'tag_bundles' in data:
            for bundle in data['tag_bundles']:
                if bundle['name'] == 'maze':
                    for tag in bundle['layout']:
                        if tag['id'] == tag_id:
                            # 提取并返回坐标和四元数
                            return (tag['x'], tag['y'], tag['z'], tag['qw'], tag['qx'], tag['qy'], tag['qz'])
    return None


def start_watching():
    path = "/home/jetson/workspace/catkin_ws/src/task1/detector_record.txt" # 设置监控的目录
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)  # 这里可以调整睡眠时间以减少CPU使用或响应延迟
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

if __name__ == "__main__":
    start_watching()
