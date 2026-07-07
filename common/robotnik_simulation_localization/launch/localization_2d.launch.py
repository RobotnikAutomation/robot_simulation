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

import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from robotnik_common.launch import ConfigFile, RewrittenYaml


_AMCL_OVERRIDE_KEYS = {
    "robot_model_type",
    "scan_topic",
    "alpha5",
    "laser_min_range",
    "laser_max_range",
}
_AMCL_REQUIRED_OVERRIDE_KEYS = {
    "robot_model_type",
    "scan_topic",
}
_AMCL_PARAMS_ROOT = "/**.amcl.ros__parameters"


def _resolve_amcl_base_config(context):
    return os.path.join(
        FindPackageShare("robotnik_simulation_localization").perform(context),
        "config",
        "amcl.yaml",
    )


def _resolve_amcl_override_config(context):
    robot = LaunchConfiguration("robot").perform(context)
    profile_config = os.path.join(
        FindPackageShare("robotnik_simulation_profiles").perform(context),
        robot,
        "localization",
        "amcl.yaml",
    )
    if not os.path.exists(profile_config):
        raise FileNotFoundError(
            f"AMCL override file for robot '{robot}' was not found at {profile_config}."
        )
    return profile_config


def _load_amcl_override_rewrites(override_config):
    with open(override_config, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}

    try:
        params = config["/**"]["amcl"]["ros__parameters"]
    except KeyError as exc:
        raise ValueError(
            f"AMCL override file '{override_config}' must define /**/amcl/ros__parameters."
        ) from exc

    invalid_keys = sorted(set(params) - _AMCL_OVERRIDE_KEYS)
    if invalid_keys:
        raise ValueError(
            f"AMCL override file '{override_config}' contains unsupported keys: "
            f"{', '.join(invalid_keys)}."
        )

    missing_keys = sorted(_AMCL_REQUIRED_OVERRIDE_KEYS - set(params))
    if missing_keys:
        raise ValueError(
            f"AMCL override file '{override_config}' is missing required keys: "
            f"{', '.join(missing_keys)}."
        )

    return {
        f"{_AMCL_PARAMS_ROOT}.{key}": str(value)
        for key, value in params.items()
    }


def _resolve_amcl_config(context):
    base_config = _resolve_amcl_base_config(context)
    override_config = _resolve_amcl_override_config(context)
    override_rewrites = _load_amcl_override_rewrites(override_config)
    rewritten_config = RewrittenYaml(
        source_file=base_config,
        param_rewrites=override_rewrites,
        convert_types=True,
    )
    return ConfigFile(rewritten_config)

def _launch_setup(context, *_args, **_kwargs):
    robot_id = LaunchConfiguration("robot_id")
    use_sim = LaunchConfiguration("use_sim")
    frame_prefix = LaunchConfiguration("frame_prefix")

    map_file = PathJoinSubstitution([
        FindPackageShare('robotnik_simulation_localization'),
        'maps/demo_map/demo_map.yaml'
    ])

    amcl_params = _resolve_amcl_config(context)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim,
                'yaml_filename': map_file,
                'frame_id': [frame_prefix, 'map']
            }
        ]
    )
            
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params,
            {
                'use_sim_time': use_sim,
            }
        ]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters = [
            {
                'use_sim_time': use_sim,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl'
                ]
            }
        ]
    )

    group = GroupAction([
        PushRosNamespace(robot_id),
        map_server,
        amcl,
        lifecycle_manager_localization,
    ])

    return [group]

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot",
            description="Name for launch and config resources"
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Enable simulation"
        ),
        DeclareLaunchArgument(
            "robot",
            default_value="rbwatcher",
            description="Robot profile used to resolve AMCL config"
        ),
        DeclareLaunchArgument(
            "frame_prefix",
            default_value=[LaunchConfiguration("robot_id"), "_"],
            description="Prefix for TF frames"
        ),
    ]

    group = GroupAction([
        OpaqueFunction(function=_launch_setup),
    ])

    return LaunchDescription(declared_arguments + [group])
