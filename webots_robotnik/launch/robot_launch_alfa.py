import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)

from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from launch.substitutions import TextSubstitution, PathJoinSubstitution, Command, PythonExpression
from webots_ros2_driver.utils import controller_protocol, controller_ip_address


def load_urdf(file_path):
    with open(file_path, 'r') as f:
        return f.read()

def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)
    
    package_dir = get_package_share_directory('webots_robotnik')
    robot_controller_path = os.path.join(package_dir, 'resource', 'rbrobout_controller_avanzado.urdf')

    #robot_description_path = os.path.join(package_dir, 'resource', 'rbrobout.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    mode = LaunchConfiguration('mode')
    
    #robot_description_content = load_urdf(robot_description_path)
    
    
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
            'robot_description': '<robot name=""><link name=""/></robot>',
        }],
        namespace=params['namespace'],
    )
    
    ld.add_action(robot_state_publisher)
    
    spawn_robot_service_call = ExecuteProcess(
    cmd=[
        'ros2', 'service', 'call', 
        '/Ros2Supervisor/spawn_node_from_string', 
        'webots_ros2_msgs/srv/SpawnNodeFromString', 
        [
            TextSubstitution(text='{data: "'), 
            robot, 
            TextSubstitution(text=' { name \\"'), namespace, TextSubstitution(text='\\" '), 
            TextSubstitution(text='robot_base_link \\"'), namespace, TextSubstitution(text='/base_link\\" '), 
            TextSubstitution(text='robot_rear_laser_base_link \\"'), namespace, TextSubstitution(text='/rear_laser_base_link\\" '),
            TextSubstitution(text='robot_rear_laser_link \\"'), namespace, TextSubstitution(text='/rear_laser_link\\" '),
            TextSubstitution(text='robot_front_laser_base_link \\"'), namespace, TextSubstitution(text='/front_laser_base_link\\" '),
            TextSubstitution(text='robot_front_laser_link \\"'), namespace, TextSubstitution(text='/front_laser_link\\" '),
            TextSubstitution(text='robot_rear_rgbd_camera_color_frame \\"'), namespace, TextSubstitution(text='/rear_rgbd_camera_color_frame\\" '),
            TextSubstitution(text='robot_rear_rgbd_camera_infra2_frame \\"'), namespace, TextSubstitution(text='/rear_rgbd_camera_infra2_frame\\" '),
            TextSubstitution(text='robot_rear_rgbd_camera_link \\"'), namespace, TextSubstitution(text='/rear_rgbd_camera_link\\" '),
            TextSubstitution(text='robot_rear_rgbd_camera_depth_frame \\"'), namespace, TextSubstitution(text='/rear_rgbd_camera_depth_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_color_frame \\"'), namespace, TextSubstitution(text='/front_rgbd_camera_color_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_infra2_frame \\"'), namespace, TextSubstitution(text='/front_rgbd_camera_infra2_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_depth_frame \\"'), namespace, TextSubstitution(text='/front_rgbd_camera_depth_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_link \\"'), namespace, TextSubstitution(text='/front_rgbd_camera_link\\" '),
            TextSubstitution(text='robot_vectornav_base_link \\"'), namespace, TextSubstitution(text='/vectornav_base_link\\" '),
            TextSubstitution(text='robot_back_right_wheel_link \\"'), namespace, TextSubstitution(text='/back_right_wheel_link\\" '),
            TextSubstitution(text='robot_front_right_wheel_link \\"'), namespace, TextSubstitution(text='/front_right_wheel_link\\" '),
            TextSubstitution(text='robot_back_left_wheel_link \\"'), namespace, TextSubstitution(text='/back_left_wheel_link\\" '),
            TextSubstitution(text='robot_front_left_wheel_link \\"'), namespace, TextSubstitution(text='/front_left_wheel_link\\" '),
            TextSubstitution(text='robot_front_laser_sensor \\"'), namespace, TextSubstitution(text='/front_laser_sensor\\" '),
            TextSubstitution(text='robot_rear_laser_sensor \\"'), namespace, TextSubstitution(text='/rear_laser_sensor\\" '),
            TextSubstitution(text='rear_camera_color \\"'), namespace, TextSubstitution(text='/rear_camera_color\\" '),
            TextSubstitution(text='rear_camera_depth \\"'), namespace, TextSubstitution(text='/rear_camera_depth\\" '),
            TextSubstitution(text='rear_camera_irred1 \\"'), namespace, TextSubstitution(text='/rear_camera_irred1\\" '),
            TextSubstitution(text='rear_camera_irred2 \\"'), namespace, TextSubstitution(text='/rear_camera_irred2\\" '),
            TextSubstitution(text='front_camera_color \\"'), namespace, TextSubstitution(text='/front_camera_color\\" '),
            TextSubstitution(text='front_camera_depth \\"'), namespace, TextSubstitution(text='/front_camera_depth\\" '),
            TextSubstitution(text='front_camera_irred1 \\"'), namespace, TextSubstitution(text='/front_camera_irred1\\" '),
            TextSubstitution(text='front_camera_irred2 \\"'), namespace, TextSubstitution(text='/front_camera_irred2\\" '),
            TextSubstitution(text='imu_inertial \\"'), namespace, TextSubstitution(text='/imu_inertial\\" '),
            TextSubstitution(text='imu_accelerometer \\"'), namespace, TextSubstitution(text='/imu_accelerometer\\" '),
            TextSubstitution(text='imu_gyro \\"'), namespace, TextSubstitution(text='/imu_gyro\\" '),
            TextSubstitution(text='imu_compass \\"'), namespace, TextSubstitution(text='/imu_compass\\" '),
            TextSubstitution(text='robot_vectornav_link \\"'), namespace, TextSubstitution(text='/vectornav_link\\" '),
            TextSubstitution(text='robot_top_structure_link \\"'), namespace, TextSubstitution(text='/top_structure_link\\" '),
            TextSubstitution(text='robot_gps_base_link \\"'), namespace, TextSubstitution(text='/gps_base_link\\" '),
            TextSubstitution(text='robot_top_3d_laser \\"'), namespace, TextSubstitution(text='/top_3d_laser\\" '),
            TextSubstitution(text='robot_top_3d_laserlink \\"'), namespace, TextSubstitution(text='/top_3d_laserlink\\" '),
            TextSubstitution(text='robot_top_3d_laserbase_link \\"'), namespace, TextSubstitution(text='/top_3d_laserbase_link\\" '),
            TextSubstitution(text='robot_antenna_base_link \\"'), namespace, TextSubstitution(text='/antenna_base_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_sensor \\"'), namespace, TextSubstitution(text='/top_ptz_camera_sensor\\" '),
            TextSubstitution(text='robot_top_ptz_camera_zoom_color_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_zoom_color_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_optical_frame_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_optical_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_frame_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_zoom_thermal_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_zoom_thermal_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_optical_thermal_frame_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_optical_thermal_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_thermal_frame_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_thermal_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_tilt_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_tilt_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_pan_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_pan_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_base_link \\"'), namespace, TextSubstitution(text='/top_ptz_camera_base_link\\" '),
            TextSubstitution(text='translation '), 
            x_pos, TextSubstitution(text=' '), 
            y_pos, TextSubstitution(text=' '), 
            z_pos, TextSubstitution(text=' }"}')
        ]
    ],
    output='screen'
    )
    ld.add_action(spawn_robot_service_call)


    
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', [namespace,'/base_link'], [namespace,'/base_footprint']],
        namespace=params['namespace'],
    )
    ld.add_action(footprint_publisher)
    
    worldtf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[x_pos, y_pos, z_pos, '0', '0', '0', 'world', [namespace,'/odom']],
        namespace=params['namespace'],
    )
    ld.add_action(worldtf_publisher)
    
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        namespace=params['namespace'],
    )
    
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        namespace=params['namespace'],
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
    
    
    ros2_control_params = [package_dir, '/resource/',robot,'/ros2control', robot, '.yml']
    
   
    
    node_name = ['webots_controller_',namespace]
    webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))
    protocol = controller_protocol()
    ip_address = controller_ip_address() if (protocol == 'tcp') else ''
    ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]
    port = '1234'
    

    
    ros_arguments = [
    f"-r", ["__ns:=/",namespace],
    f"-p", ["robot_description:=",package_dir,'/resource/',robot,'/',robot,'_controller_avanzado.urdf'],
    f"-p", "use_sim_time:=True",
    f"-p", "set_robot_state_publisher:=True",
    f"-p", "update_rate:=100"]
    
    
    rbrobout_driver = ExecuteProcess(
    cmd=[
    	webots_controller,
    	["--robot-name=",namespace],
    	['--protocol=', protocol],
    	*ip_address_option,
    	['--port=', port],
        'ros2',
        '--ros-args',
        *ros_arguments,
        '--params-file',
        ros2_control_params
    ],
    output='screen',
    name=node_name,
    respawn=True,
    # Set WEBOTS_HOME to package directory to load correct controller library
    additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
    )
    ld.add_action(rbrobout_driver)
    
    waiting_node = WaitForControllerConnection(
        target_driver=rbrobout_driver,
        nodes_to_start=joint_state_broadcaster
    )
    ld.add_action(waiting_node)
    
    init_diffdrive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg='Joint States spawned'),
                diffdrive_controller_spawner
            ]
        )
    )
    ld.add_action(init_diffdrive_controller)
    
    rviz2_config = [get_package_share_directory('webots_robotnik'),'/resource/', robot,'/rviz_config.rviz']
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        namespace=params['namespace'],
        arguments=['-d', rviz2_config]

    )
    ld.add_action(rviz2)

    return ld
