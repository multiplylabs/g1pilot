from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            parameters=[
                {
                    'interface': 'eth0',
                    'use_robot': False,
                    'arm_controlled': 'both',
                    'enable_arm_ui': True,
                    'ik_use_waist': False,
                    'ik_alpha': 0.2,
                    'ik_max_dq_step': 0.05,
                    'arm_velocity_limit': 2.0,
                }
            ],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='create_map',
            name='create_map',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),
    ])
