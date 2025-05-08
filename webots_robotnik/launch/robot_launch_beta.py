import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def load_urdf(file_path):
    with open(file_path, 'r') as f:
        return f.read()

def generate_launch_description():
    package_dir = get_package_share_directory('webots_robotnik')
    robot_controller_path = os.path.join(package_dir, 'resource', 'rbrobout_controller.urdf')
    robot_description_path = os.path.join(package_dir, 'resource', 'rbrobout.urdf')
    
    robot_description_content = load_urdf(robot_description_path)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'Prueba_rbrobout_webots.wbt'),
        ros2_supervisor=True
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }],
    )

    my_robot_driver = WebotsController(
        robot_name='rbrobout',
        parameters=[
            {'robot_description': robot_controller_path},
            {'set_robot_state_publisher': True},
        ]
    )
    
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        robot_state_publisher,
        footprint_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
