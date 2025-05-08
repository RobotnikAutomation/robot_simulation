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

from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from launch.substitutions import TextSubstitution, PathJoinSubstitution, Command, PythonExpression


def load_urdf(file_path):
    with open(file_path, 'r') as f:
        return f.read()

def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)
    
    package_dir = get_package_share_directory('webots_robotnik')
    robot_controller_path = os.path.join(package_dir, 'resource', 'rbrobout_controller_avanzado.urdf')
    robot_description_path = os.path.join(package_dir, 'resource', 'rbrobout.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    mode = LaunchConfiguration('mode')
    
    robot_description_content = load_urdf(robot_description_path)
    
    
    arg = ExtendedArgument(
        name='namespace',
        description='Robot personal name',
        default_value='robot',
        use_env=True,
        environment='NAMESPACE',
    )
    add_to_launcher.add_arg(arg)
    namespace = LaunchConfiguration('namespace')

    arg = ExtendedArgument(
        name='robot',
        description='Robot model (rbvogui, rbkairos, rbtheron, rbsummit)',
        default_value='rbrobout',
        use_env=True,
        environment='ROBOT',
    )
    add_to_launcher.add_arg(arg)
    robot = LaunchConfiguration('robot')
    
    arg = ExtendedArgument(
        name='robot_model',
        description='Robot type variation (rbvogui, rbvogui_6w, rbvogui_ackermann)',
        default_value=robot,
        use_env=True,
        environment='ROBOT_MODEL',
    )
    add_to_launcher.add_arg(arg)
    robot_model = LaunchConfiguration('robot_model')


    arg = ExtendedArgument(
        name='x',
        description='x position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    x_pos = LaunchConfiguration('x')

    arg = ExtendedArgument(
        name='y',
        description='y position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    y_pos = LaunchConfiguration('y')

    arg = ExtendedArgument(
        name='z',
        description='z position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    z_pos = LaunchConfiguration('z')
    params = add_to_launcher.process_arg()
    
    
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )
    ld.add_action(robot_state_publisher)
    
    spawn_robot_service_call = ExecuteProcess(
    cmd=[
        'ros2', 'service', 'call', 
        '/Ros2Supervisor/spawn_node_from_string', 
        'webots_ros2_msgs/srv/SpawnNodeFromString', 
        [
            TextSubstitution(text='{data: "'), 
            robot, TextSubstitution(text=' { name \\"'), 
            namespace, TextSubstitution(text='\\" translation '), 
            x_pos, TextSubstitution(text=' '), 
            y_pos, TextSubstitution(text=' '), 
            z_pos, TextSubstitution(text=' }"}')
        ]
    ],
    output='screen'
    )
    ld.add_action(spawn_robot_service_call)

    my_robot_driver = WebotsController(
        robot_name='rbrobout',
        parameters=[
            {'robot_description': robot_controller_path,
            'set_robot_state_publisher': True},
        ]
    )
    #ld.add_action(my_robot_driver)
    
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    ld.add_action(footprint_publisher)
    
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    
    robotnik_controller= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotnik_base_controller'] + controller_manager_timeout,
        prefix=controller_manager_prefix,
        output='screen',
        emulate_tty=True,
    )
    
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster]
    
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2controlrbrobout.yml') #Va pero simplemente puedo seleccionar un yaml predeterminado
    
    
    ros2_control_params2= PathJoinSubstitution([   #No va porque no reconoce PathJoinSubstitution como string
     package_dir,
     'resource',
     TextSubstitution(text='ros2control'),
     LaunchConfiguration('robot'),
     TextSubstitution(text='.yml')
    ])
    
    ros2_control_params2 = [package_dir, '/resource/ros2control', robot, '.yml']  #No va porque no saca el valor de LaunchConfiguration(Robot)
    
    print(ros2_control_params2)
    
    
    rbrobout_driver = WebotsController(
        robot_name='rbrobout',
        parameters=[
            {'robot_description': robot_controller_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True,
             'update_rate': 100},
            ros2_control_params
        ],
        respawn=True
    )
    ld.add_action(rbrobout_driver)
    
    waiting_node = WaitForControllerConnection(
        target_driver=rbrobout_driver,
        nodes_to_start=ros_control_spawners
    )
    ld.add_action(waiting_node)

    return ld
