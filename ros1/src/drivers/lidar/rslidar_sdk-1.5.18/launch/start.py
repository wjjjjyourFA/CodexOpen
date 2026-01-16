from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import random
import string

def random_suffix(n=6):
    """生成随机后缀，用于避免节点名冲突"""
    return ''.join(random.choices(string.ascii_lowercase + string.digits, k=n))

def generate_launch_description():

    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'

    config_file = '' # your config file path
    
    # 给节点名添加随机后缀，避免多次启动冲突
    node_name = f"rslidar_sdk_node_{random_suffix()}"

    return LaunchDescription([
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}], name=node_name),
        # Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
