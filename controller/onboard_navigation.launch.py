"""Headless Nav2 wrapper; tolerate slower on-board lifecycle startup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    defaults = {'map': None, 'params_file': None, 'use_sim_time': 'False',
                'autostart': 'True', 'use_composition': 'True'}
    declarations = [DeclareLaunchArgument(name) if default is None else
                    DeclareLaunchArgument(name, default_value=default)
                    for name, default in defaults.items()]
    source = Path(get_package_share_directory('nav2_bringup')) / 'launch/bringup_launch.py'
    return LaunchDescription(declarations + [
        SetParameter(name='bond_timeout', value=20.0),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
