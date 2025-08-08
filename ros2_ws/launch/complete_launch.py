from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                './launch/rosbridge_websocket_launch.xml'
            )
        ),
        Node(
            package='stations_monitor',
            executable='stations_monitor',
            name='stations_monitor',
            output='screen'
        ),
        Node(
            package='prompt_manager',
            executable='prompt_executor',
            name='prompt_executor',
            output='screen'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('webots_simulation'), 'launch', 'simulation_launch.py']),
        )
    ])
