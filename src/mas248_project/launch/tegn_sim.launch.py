"""Alt som trengs for a tegne i simulering (mock), i ett launch:

  1. mock-kontrollerne fra mas248_ur5e_moveit_config (ros2_control.launch.py)
  2. MoveIt + RViz (moveit.launch.py), startet litt etter kontrollerne
  3. pen_trace, som viser i RViz det pennen tegner

Kjor:   LIBGL_ALWAYS_SOFTWARE=1 ros2 launch mas248_project tegn_sim.launch.py
Deretter, i en annen terminal: image_to_targets ... og run_picture --execute.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_launch = Path(get_package_share_directory('mas248_ur5e_moveit_config')) / 'launch'

    def inkluder(navn):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit_launch / navn)))

    flate_z = DeclareLaunchArgument('flate_z', default_value='0.0',
                                    description='Tegneflatens hoyde i world (m) for pen_trace')
    return LaunchDescription([
        flate_z,
        inkluder('ros2_control.launch.py'),
        # MoveIt trenger /joint_states fra kontrollerne, som spawnes etter 3 s
        TimerAction(period=5.0, actions=[inkluder('moveit.launch.py')]),
        Node(package='mas248_project', executable='pen_trace', output='screen',
             parameters=[{'flate_z': LaunchConfiguration('flate_z')}]),
    ])
