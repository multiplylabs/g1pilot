from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os

package_name = "g1pilot"
urdf_file_name = "g1_29dof.urdf"
rviz_config_file_name = "g1_29dof.rviz"

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot = LaunchConfiguration("use_robot")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    interface = LaunchConfiguration("interface")
    sim_rate_hz = LaunchConfiguration("sim_rate_hz")

    urdf = os.path.join(
        get_package_share_directory(package_name), "description_files/urdf", urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument("use_robot", default_value="false",
                              description="Connect to real robot if true"),
        DeclareLaunchArgument("publish_joint_states", default_value="true",
                              description="Publish joint_states from node"),
        DeclareLaunchArgument("interface", default_value="eth0",
                              description="Network interface for Unitree SDK"),
        DeclareLaunchArgument("sim_rate_hz", default_value="50.0",
                              description="Simulation rate when use_robot=false"),

        Node(
            package='g1pilot',
            executable='robot_state',
            name='robot_state',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'sim_rate_hz': ParameterValue(sim_rate_hz, value_type=float),
                'publish_joint_states': ParameterValue(publish_joint_states, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_to_livox_tf',
            arguments=['0','0','0','0','0','3.14159265','mid360_link','livox_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_tf',
            arguments=['0','0','0','0','0','0','odom','world']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mrbeam_to_pelvis_tf',
            arguments=['0.0745','0.0','0.065','0','0.05236','0','waist_roll_link','mrbeam_link']
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "robot_description": robot_desc
            }],
            arguments=[urdf],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                os.path.join("/ros2_ws/src/g1pilot/config", rviz_config_file_name)
            ],
        ),
    ])
