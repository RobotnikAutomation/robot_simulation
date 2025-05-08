import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)

def load_urdf(file_path):
    with open(file_path, 'r') as f:
        return f.read()

def generate_launch_description():
    package_dir = get_package_share_directory('webots_robotnik')
    robot_controller_path = os.path.join(package_dir, 'resource', 'rbrobout_controller_avanzado.urdf')
    robot_description_path = os.path.join(package_dir, 'resource', 'rbrobout.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
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
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    my_robot_driver = WebotsController(
        robot_name='rbrobout',
        parameters=[
            {'robot_description': robot_controller_path,
            'set_robot_state_publisher': True},
        ]
    )
    
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    #diffdrive_controller_spawner = Node(
        #package='controller_manager',
        #output='screen',
        #prefix=controller_manager_prefix,
        #arguments=['diffdrive_controller'] + controller_manager_timeout,
    #)
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    
    robotnik_controller= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotnik_base_controller'] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        output='screen',
        emulate_tty=True,
    )
    
    ros_control_spawners = [robotnik_controller, joint_state_broadcaster]
    
    ros2_control_params = os.path.join(package_dir, 'resource', 'rbrobout_controller_params.yaml')
    
    rbrobout_driver = WebotsController(
        robot_name='rbrobout',
        parameters=[
            {'robot_description': robot_controller_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        respawn=True
    )
    
    waiting_node = WaitForControllerConnection(
        target_driver=rbrobout_driver,
        nodes_to_start=[joint_state_broadcaster]
    )

    
    init_robotnik_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg='Joint States spawned'),
                robotnik_controller
            ]
        )
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        #my_robot_driver,
        robot_state_publisher,
        rbrobout_driver,
        waiting_node,
        init_robotnik_controller,
        footprint_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
