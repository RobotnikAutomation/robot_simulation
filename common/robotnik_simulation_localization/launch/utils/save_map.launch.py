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
from launch.actions import OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from pathlib import Path
import shutil

def find_workspace_root(start_path: Path) -> Path:
    """
    Goes up directory by directory until it finds one that contains 'install'.
    """
    current = start_path.resolve()
    while current != current.parent:  # until reaching the root 
        if (current / "install").exists():
            return current
        current = current.parent
    raise FileNotFoundError("No 'install' folder found at any upper level.")

def move_map(context, maps_path, map_folder_name):
    """
    Moves the map folder to the src of the automatically detected workspace.
    """
    maps_path = context.perform_substitution(maps_path)
    map_folder_name = context.perform_substitution(map_folder_name)

    map_folder = Path(maps_path) / map_folder_name
    if not map_folder.exists():
        raise FileNotFoundError(f"The folder {map_folder} does not exist: {map_folder}")

    # Automatically detect the workspace root
    workspace_root = find_workspace_root(Path(__file__))
    workspace_src = workspace_root / "src" / "maps"

    dest_folder = workspace_src / map_folder_name

    if dest_folder.exists():
        raise FileExistsError(f"The destination folder already exists: {dest_folder}")

    shutil.move(str(map_folder), str(dest_folder))
    return []

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot",
            description="Name for launch and config resources"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "maps_path",
            default_value="/tmp",
            description="Path to the maps directory"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "map_name",
            default_value="demo_map",
            description="Name of the map to be saved"
        )
    )

    robot_id = LaunchConfiguration("robot_id")
    maps_path = LaunchConfiguration("maps_path")  
    map_name = LaunchConfiguration("map_name")

    # Map folder receives the same name as map_name
    map_folder_name = map_name

    create_map_folder = ExecuteProcess(
        cmd=['mkdir', '-p', [maps_path, '/', map_folder_name]],  # /tmp/demo_map
        shell=True
    )

    map_saver_cli = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        parameters=[],
        arguments=[
            '-f', [maps_path, '/', map_folder_name, '/', map_name], # /tmp/demo_map/demo_map.file
            '--fmt', 'pgm',        # Map format                  
            '-t', "robot_map"      # Map topic
        ]
    )

    move_map_action = OpaqueFunction(
        function=lambda context: move_map(context, maps_path, map_name)
    )

    move_after_save = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=map_saver_cli,
            on_exit=[move_map_action]
        )
    )

    group = GroupAction([
        PushRosNamespace(robot_id),
        create_map_folder,
        map_saver_cli,
        move_after_save
    ])

    return LaunchDescription(declared_arguments + [group])