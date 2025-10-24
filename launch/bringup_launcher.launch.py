from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    pkg1_share = FindPackageShare('g1pilot').find('g1pilot')

    livox_launcher = os.path.join(pkg1_share, 'launch', 'livox_launcher.launch.py')
    navigation_launcher = os.path.join(pkg1_share, 'launch', 'navigation_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg1_share, 'launch', 'robot_state_launcher.launch.py')
    teleoperation_launcher = os.path.join(pkg1_share, 'launch', 'teleoperation_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg1_share, 'launch', 'manipulation_launcher.launch.py')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleoperation_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher)
        ),
    ])