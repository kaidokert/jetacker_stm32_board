from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

def generate_launch_description():
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value='imu_link')
    auto_enable_steering_servo_arg = DeclareLaunchArgument('auto_enable_steering_servo', default_value='false')
    steering_servo_id_arg = DeclareLaunchArgument('steering_servo_id', default_value='1')

    imu_frame = LaunchConfiguration('imu_frame')
    auto_enable_steering_servo = LaunchConfiguration('auto_enable_steering_servo')
    steering_servo_id = LaunchConfiguration('steering_servo_id')

    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
        parameters=[{
            'imu_frame': imu_frame,
            'auto_enable_steering_servo': ParameterValue(
                PythonExpression(["'", auto_enable_steering_servo, "' == 'true' or '", auto_enable_steering_servo, "' == 'True'"]),
                value_type=bool
            ),
            'steering_servo_id': steering_servo_id
        }]
    )

    return LaunchDescription([
        imu_frame_arg,
        auto_enable_steering_servo_arg,
        steering_servo_id_arg,
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(Create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
