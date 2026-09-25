import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to pure_pursuit_controller parameter YAML file.',
    )

    sim_startup_delay_arg = DeclareLaunchArgument(
        'sim_startup_delay',
        default_value='5.0',
        description='Seconds to wait for Gazebo/the ros_gz bridge to come up before '
                     'starting the controller.',
    )

    prius_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('prius_bringup'), 'launch'),
            '/gz_sim.launch.py',
        ]),
    )

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    delayed_pure_pursuit_node = TimerAction(
        period=LaunchConfiguration('sim_startup_delay'),
        actions=[pure_pursuit_node],
    )

    return LaunchDescription([
        params_file_arg,
        sim_startup_delay_arg,
        prius_sim,
        delayed_pure_pursuit_node,
    ])
