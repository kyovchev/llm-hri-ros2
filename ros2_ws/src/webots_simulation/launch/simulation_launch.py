import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_simulation')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    robot_1_driver = WebotsController(
        robot_name='robot_1',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        output='screen'
    )

    robot_2_driver = WebotsController(
        robot_name='robot_2',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        output='screen'
    )

    robot_3_driver = WebotsController(
        robot_name='robot_3',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        output='screen'
    )

    return LaunchDescription([
        webots,
        robot_1_driver,
        robot_2_driver,
        robot_3_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])