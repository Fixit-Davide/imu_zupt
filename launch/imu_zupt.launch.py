from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='imu_zupt',
            executable='ImuZuptExecutable',
            name='imu_zupt',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[os.path.join(get_package_share_directory("imu_zupt"), 'params', 'params.yaml')],
        )
])
