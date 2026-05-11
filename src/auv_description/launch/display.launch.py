# ROS2 launch: view the AUV URDF in RViz (no Gazebo).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('auv_description')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'auv.urdf.xacro'])
    rviz_cfg   = PathJoinSubstitution([pkg_share, 'rviz', 'auv.rviz'])

    # ParameterValue(..., value_type=str) prevents launch_ros from trying to
    # parse the xacro-generated URDF as YAML.
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_path]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[robot_description,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'),
    ])
