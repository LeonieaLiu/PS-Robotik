import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_yaml_file(file_path):
    """读取并返回 YAML 文件内容"""
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def extract_maze_info(maze_data):
    """解析 YAML 数据中的迷宫信息"""
    if 'tag_bundles' in maze_data:
        for bundle in maze_data['tag_bundles']:
            if bundle['name'] == 'maze':
                return bundle['layout']
    return None
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

# 文件路径替换为实际路径
def plot_maze(maze_layout):
    fig, ax = plt.subplots()
    for tag in maze_layout:
        # 将四元数转换为二维旋转角度（假设旋转仅在Z轴）
        angle = np.arctan2(2 * (tag['qw'] * tag['qz'] + tag['qx'] * tag['qy']), 1 - 2 * (tag['qz']**2 + tag['qy']**2))
        ax.quiver(tag['x'], tag['y'], np.cos(angle), np.sin(angle), scale=6, scale_units='inches')
        ax.text(tag['x'], tag['y'], f"ID:{tag['id']}", color='blue', fontsize=12)

    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_xlabel('X position (m)')
    ax.set_ylabel('Y position (m)')
    ax.set_title('Maze Visualization with AprilTags')
    plt.grid(True)
    plt.show()

# 文件路径替换为实际路径

	file_path = '/home/jetson/workspace/catkin_ws/src/task1/tags.yaml'
	maze_data = load_yaml_file(file_path)
	maze_layout = extract_maze_info(maze_data)
	tags_data = get_tag_coordinates_and_orientation(file_path,68)
	print(tags_data)  # 输出每个标签的ID及其数据)
if __name__=="__main__":
   test()

#if maze_layout:
    #plot_maze(maze_layout)
#else:
    #print("No maze information found in the YAML file.")
