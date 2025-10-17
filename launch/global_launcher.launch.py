from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    pkg1_share = FindPackageShare('package1').find('package1')
    pkg2_share = FindPackageShare('package2').find('package2')

    launch_file1 = os.path.join(pkg1_share, 'launch', 'launchfile1.launch.py')
    launch_file2 = os.path.join(pkg2_share, 'launch', 'launchfile2.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file1)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file2)
        ),
    ])