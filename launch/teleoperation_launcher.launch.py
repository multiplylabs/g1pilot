from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1pilot',
            executable='joystick',
            name='joystick',
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='joy_mux',
            name='joy_mux',
            output='screen'
        ),

    ])
