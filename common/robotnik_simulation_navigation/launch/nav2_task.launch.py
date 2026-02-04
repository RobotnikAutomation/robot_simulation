# Copyright (c) 2025, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():
    
    robot_id = LaunchConfiguration("robot_id")
    use_sim = LaunchConfiguration("use_sim")

    # Nav2 core node configurations

    controller_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/controller_server.yaml'
    ])

    planner_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/planner_server.yaml'
    ])

    # Nav2 auxiliary node configurations

    behavior_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/behavior_server.yaml'
    ])

    smoother_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/smoother_server.yaml'
    ])

    # Nav2 orchestration node configurations

    bt_navigator_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/bt_navigator.yaml'
    ])

    bt_navigator_pose_xml = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/behavior_trees/navigate_to_pose.xml'
    ])

    bt_navigator_poses_xml = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/behavior_trees/navigate_through_poses.xml'
    ])

    # Nav2 core nodes

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_config, {'use_sim_time': use_sim}],
        remappings=[
            ('cmd_vel', 'robotnik_base_control/cmd_vel'),
            ('odom', 'robotnik_base_control/odom'),
        ]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_config, {'use_sim_time': use_sim}],
    )

    # Nav2 auxiliary nodes

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_config, {'use_sim_time': use_sim}],
        remappings=[
            ('cmd_vel', 'robotnik_base_control/cmd_vel')
        ] 
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[smoother_config, {'use_sim_time': use_sim}],
    )


    # Nav2 orchestration nodes

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_navigator_config,
            {
                'use_sim_time': use_sim,
                'default_nav_to_pose_bt_xml': bt_navigator_pose_xml,
                'default_nav_through_poses_bt_xml': bt_navigator_poses_xml,
            }
        ],
        remappings=[
            ('odom', 'robotnik_base_control/odom'),
        ]
    )
   
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'smoother_server',
                    'bt_navigator'
                ]
            }
        ]
    )

    group = GroupAction([
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        lifecycle_manager_navigation
    ])

    return LaunchDescription([group])
