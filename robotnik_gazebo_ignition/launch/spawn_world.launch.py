# Copyright (c) 2022, Robotnik Automation S.L.L.
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
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)
    world = LaunchConfiguration('world')

    arg = ExtendedArgument(
        name='world',
        description='world in gazebo classic',
        default_value='demo.sdf.world',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='world_path',
        description='world path in gazebo classic',
        default_value=[FindPackageShare('robotnik_gazebo_ignition'), '/worlds/', world],
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    gazebo_ignition_launch_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory(
                                'ros_gz_sim'
                            ),
                            'launch'
                        ),
                        'gz_sim.launch.py')
                ),
                launch_arguments={                    
                    'gz_args':[
                        '-r ',
                        '-s ',
                        #'-v4 ', #verbose level
                        params['world_path']
                        # 'empty.sdf'
                    ], 
                    'on_exit_shutdown':'true'
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory(
                                'ros_gz_sim'
                            ),
                            'launch'
                        ),
                        'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args':[
                        '-g '
                    ],
                    'on_exit_shutdown':'true'
                }.items(),
            )
        ]
    )

    ld.add_action(gazebo_ignition_launch_group)

    return ld

