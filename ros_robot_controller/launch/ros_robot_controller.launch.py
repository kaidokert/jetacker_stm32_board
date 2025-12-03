import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('ros_robot_controller')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(Create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
