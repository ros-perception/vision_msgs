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
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D
from vision_msgs.msg import BoundingBox3D
from vision_msgs.msg import ObjectHypothesisWithPose


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
        super().__init__("pub_detection3_d_array_sample")
        self.__pub = self.create_publisher(
            Detection3DArray, "detection3_d_array", 10)
        self.__timer = self.create_timer(0.1, self.pub_sample)
        self.__counter = 0
        self.__header = Header()
        self.__msg_def = {
            "score": [0.0, 1.0, 2.0, 3.0, 1.0, 20.0, 40.0],
            "obj_id": ["", "", "car", "cyclist", "tree", "house", "skateboard"]
        }

    def create_msg(self, bbox: BoundingBox3D, scores, obj_ids) -> Detection3D:
        msg = Detection3D()
        msg.header = self.__header
        msg.bbox = bbox
        for score, obj_id in zip(scores, obj_ids):
            obj = ObjectHypothesisWithPose()
            obj.hypothesis.score = score
            obj.hypothesis.class_id = obj_id
            msg.results.append(obj)
        return msg

    def pub_sample(self):
        while self.__pub.get_subscription_count() == 0:
            return
        self.__header.stamp = self.get_clock().now().to_msg()
        self.__header.frame_id = "map"
        msg = Detection3DArray()
        msg.header = self.__header
        for i in range(len(self.__msg_def["score"])):
            for j in range(len(self.__msg_def["score"])):
                bbox = BoundingBox3D()
                quat = quaternion_about_axis(
                    (self.__counter % 100) * pi * 2 / 100.0, [0, 0, 1])
                bbox.center.orientation.x = quat[0]
                bbox.center.orientation.y = quat[1]
                bbox.center.orientation.z = quat[2]
                bbox.center.orientation.w = quat[3]
                # Set the center position based on the indices of the loop
                bbox.center.position.x = 1.5 * (i + 1)
                bbox.center.position.y = 1.5 * (j + 1)
                bbox.size.x = (self.__counter % 10 + 1) * 0.1
                bbox.size.y = ((self.__counter + 1) % (5 * (i + 1)) + 1) * 0.1
                bbox.size.z = ((self.__counter + 2) % (10 * (i + 1)) + 1) * 0.1
                # Get the score and obj_id for the current indices
                if i == len(self.__msg_def["score"]) - 1:
                    score = random.choice(self.__msg_def["score"])
                    obj_id = random.choice(self.__msg_def["obj_id"])
                else:
                    score = self.__msg_def["score"][i]
                    obj_id = self.__msg_def["obj_id"][j]
                detection_msg = self.create_msg(
                    bbox=bbox, scores=[score], obj_ids=[obj_id]
                )
                msg.detections.append(detection_msg)

        self.__pub.publish(msg)
        self.__counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = pub_detection3_d_array()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
