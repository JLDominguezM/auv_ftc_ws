# Launch the T-S fuzzy controller + pseudo-inverse / QP allocator + reference generator.
# Expects Gazebo to already be running and the AUV to be spawned (see
# auv_gazebo/launch/gazebo.launch.py), with /auv/odom being published.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pick(context, *args, **kwargs):
    pkg_share = get_package_share_directory('auv_control')
    model = LaunchConfiguration('model').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    cfg_override = LaunchConfiguration('config').perform(context)
    if cfg_override:
        cfg = cfg_override
    elif model == 'rexrov':
        cfg = os.path.join(pkg_share, 'config', 'controller_rexrov.yaml')
    else:
        cfg = os.path.join(pkg_share, 'config', 'controller.yaml')
    print(f'[auv_control] model={model}  config={cfg}')

    return [
        Node(package='auv_control', executable='auv_controller_node',
             name='auv_controller', namespace='auv',
             parameters=[cfg, {'use_sim_time': use_sim_time}],
             output='screen'),
        # Node(package='auv_control', executable='reference_generator_node',
        #      name='reference_generator', namespace='auv',
        #      parameters=[cfg, {'use_sim_time': use_sim_time}],
        #      output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value='torpedo'),
        DeclareLaunchArgument('config',       default_value='',
            description='Override the YAML config path (empty = auto by model).'),
        OpaqueFunction(function=_pick),
    ])
