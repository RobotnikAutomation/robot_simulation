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
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():

    robot_id = LaunchConfiguration("robot_id")
    use_sim = LaunchConfiguration("use_sim")

    waypoint_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/waypoint_follower.yaml'
    ])

    route_config = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/route_server.yaml'
    ])

    route_graph_filepath = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_navigation'),
        'config/graph/demo_map_graph.geojson'
    ])

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[waypoint_config, {'use_sim_time': use_sim}]
    )

    # No full available in Jazzy
    route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[
            route_config,
            {
                'use_sim_time': use_sim,
                'graph_filepath': route_graph_filepath
            }
        ]
    )

    lifecycle_manager_mission_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mission_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim,
                'autostart': False,
                'node_names': [
                    'waypoint_follower',
                    #'route_server'
                ]
            }
        ]
    )

    group = GroupAction([
        waypoint_follower,
        #route_server,
        lifecycle_manager_mission_navigation,
    ])

    return LaunchDescription([group])
