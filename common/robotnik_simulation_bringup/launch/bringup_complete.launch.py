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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.actions import TimerAction
from launch.substitutions import EqualsSubstitution, OrSubstitution

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot",
            description="Name for launch and config resources"
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="rbsummit",
            description="Set robot model"
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Enable simulation gui"
        ),
        DeclareLaunchArgument(
            "low_performance_simulation",
            default_value="true",
            description="Enable smooth simulation for low performance computers"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Enable rviz gui"
        ),
        DeclareLaunchArgument(
            "world_path",
            default_value=PathJoinSubstitution([
                #FindPackageShare('electrical_substation_world'), 'worlds/electrical_substation.world'
                FindPackageShare('robotnik_gazebo_ignition'), 'worlds/demo.world',
            ]),
            description="Path to the world file"
        ),
    ]

    robot_id = LaunchConfiguration("robot_id")
    robot_model = LaunchConfiguration("robot_model")
    use_gui = LaunchConfiguration("use_gui")
    low_performance_simulation = LaunchConfiguration("low_performance_simulation")
    world_path = LaunchConfiguration("world_path")
    use_rviz = LaunchConfiguration("use_rviz")

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robotnik_gazebo_ignition'), 'launch/spawn_world.launch.py'
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
            'gui': use_gui,
            'world_path': world_path
        }.items()
    )

    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_gazebo_ignition'), 'launch/spawn_robot.launch.py'
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
            'robot': robot_model,
            'low_performance_simulation': low_performance_simulation,
            'run_rviz': 'false'
        }.items()
    )

    # In spawn_robot.launch.py the add_laser("front") is defined for any robot model
    # This causes two publishers to /robot/front_laser/scan when laser filters are enabled
    # In rbsummit and rbwatcher is not an issue because front laser is not available
    # but in other robot models that have front laser, it causes conflict.
    laser_filters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_simulation_bringup'), 'launch/laser_filters.launch.py'
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
            'use_sim': 'true',
        }.items(),
        condition=IfCondition(
            OrSubstitution(
                EqualsSubstitution(robot_model, 'rbsummit'),
                EqualsSubstitution(robot_model, 'rbwatcher'),
            )
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_simulation_localization'), 'launch/localization.launch.py'
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
            'use_sim': 'true',
        }.items()
    )

    delayed_localization = TimerAction(
        period=10.0,
        actions=[localization]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_simulation_navigation'), 'launch/navigation.launch.py'
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
            'use_sim': 'true',
        }.items()
    )

    delayed_navigation = TimerAction(
        period=15.0,
        actions=[navigation]
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                 FindPackageShare('robotnik_simulation_bringup'), 'launch/rviz.launch.py'
            ])
        ),
        condition=IfCondition(use_rviz)
    )

    delayed_rviz = TimerAction(
        period=20.0,
        actions=[rviz]
    )

    group = GroupAction([
        gazebo_world,
        gazebo_robot,
        laser_filters,
        delayed_localization,
        delayed_navigation,
        delayed_rviz
    ])

    return LaunchDescription(declared_arguments + [group])