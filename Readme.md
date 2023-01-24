# ROS Vision Messages

This package defines a set of [messages](vision_msgs) to unify computer
vision and object detection efforts in ROS and provides some visualization [plugins](vision_msgs_rviz_plugins) for RVIZ2.


```bash
cd ~/ros2_ws/src && git clone https://github.com/NovoG93/vision_msgs -b ros2
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-up-to vision_msgs_meta
```
