#!/usr/bin/env python3
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


from math import pi, sin, cos
from numpy import array
from numpy.linalg import norm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D


def quaternion_about_axis(angle, axis):
    axis = array(axis)
    axis = axis / norm(axis)
    half_angle = angle / 2
    sine = sin(half_angle)
    w = cos(half_angle)
    x, y, z = axis * sine
    return x, y, z, w


class pub_detection3_d_array(Node):
    def __init__(self):
        super().__init__("pub_bounding_box_3_d_sample")
        self.__pub = self.create_publisher(
            BoundingBox3D, "bounding_box_3d", 10)
        self.__timer = self.create_timer(0.1, self.pub_sample)
        self.__counter = 0
        self.__header = Header()

    def pub_sample(self):
        while self.__pub.get_subscription_count() == 0:
            return
        self.__header.stamp = self.get_clock().now().to_msg()
        self.__header.frame_id = "map"

        # Reset counter and indices if counter is a multiple of 30
        if self.__counter % 10 == 0:
            self.__counter = 0

        # Create a single BoundingBox3D message
        bbox = BoundingBox3D()
        quat = quaternion_about_axis(
            (self.__counter % 100) * pi * 2 / 100.0, [0, 0, 1])
        bbox.center.orientation.x = quat[0]
        bbox.center.orientation.y = quat[1]
        bbox.center.orientation.z = quat[2]
        bbox.center.orientation.w = quat[3]
        # Set the center position to a fixed value
        bbox.center.position.x = -1.5
        bbox.center.position.y = 0.0
        bbox.size.x = (self.__counter % 10 + 1) * 0.1
        bbox.size.y = ((self.__counter + 1) % 5 + 1) * 0.1
        bbox.size.z = ((self.__counter + 2) % 10 + 1) * 0.1
        # Publish the BoundingBox3D message
        self.__pub.publish(bbox)
        self.__counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = pub_detection3_d_array()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
