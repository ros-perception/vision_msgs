# Copyright 2023 Georg Novotny
#
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    rviz_path = os.path.join(
        get_package_share_directory("vision_msgs_rviz_plugins"),
        "conf", "conf.rviz"
    )

    return LaunchDescription([
        Node(
            package="vision_msgs_rviz_plugins",
            executable="BoundingBox3D.py",
            name="boundingbox3d_test"
        ),
        Node(
            package="vision_msgs_rviz_plugins",
            executable="BoundingBox3DArray.py",
            name="boundingbox3darray_test"
        ),
        Node(
            package="vision_msgs_rviz_plugins",
            executable="Detection3D.py",
            name="detection3d_test"
        ),
        Node(
            package="vision_msgs_rviz_plugins",
            executable="Detection3DArray.py",
            name="detection3darray_test"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path]
        )

    ])
