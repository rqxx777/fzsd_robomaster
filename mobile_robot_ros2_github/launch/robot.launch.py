from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('mobile_robot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 创建节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0
        }],
        output='screen'
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    wheel_simulator = Node(
        package='mobile_robot',
        executable='wheel_simulator',
        name='wheel_simulator',
        output='screen'
    )
    
    teleop_keyboard = Node(
        package='mobile_robot',
        executable='teleop',
        name='teleop_keyboard',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'robot.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        wheel_simulator,
        teleop_keyboard,
        rviz_node
    ])
