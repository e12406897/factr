# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    side = LaunchConfiguration('side')

    config_file = PythonExpression([
        "'franka_' + '",
        side,
        "' + '.yaml'"
    ])

    factr_teleop_franka = Node(
        package='factr_teleop',
        executable='factr_teleop_franka',
        name='factr_teleop_franka',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'config_file': config_file}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'side',
            description='Franka side: left or right'
        ),
        factr_teleop_franka,
    ])