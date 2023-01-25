# vision_msgs_rviz_plugins

This package contains a RVIZ2 plugin to display (some) vision_msgs for ROS 2.

- [x] Detection3DArray
  - [x] Display ObjectHypothesisWithPose/score
  - [x] Change color based on ObjectHypothesisWithPose/id [car: orange, person: blue, cyclist: yellow, motorcycle: purple, other: grey]
  - [x] Visualization propperties
    - [x] Alpha
    - [x] Line or Box
    - [x] Linewidth
    - [x] Change color map based on provided yaml file
- [x] Detection3D
  - [x] Display ObjectHypothesisWithPose/score
  - [x] Change color based on ObjectHypothesisWithPose/id [car: orange, person: blue, cyclist: yellow, motorcycle: purple, other: grey]
  - [x] Visualization propperties
    - [x] Alpha
    - [x] Line or Box
    - [x] Linewidth
    - [x] Change color map based on provided yaml file
- [x] BoundingBox3D
    - [x] Alpha
    - [x] Line or Box    
        <span style="color:red">**Since no header in vision_msgs/BonudingBox visualizations uses rviz fixed frame for tf transformation**</span>
    - [x] Linewidth
- [x] BoundingBox3DArray
    - [x] Alpha
    - [x] Line or Box
    - [x] Linewidth

![Bounding Box Array](assets/BBoxArray.gif)

## Install and Testing

__Install:__
```bash
$ cd ros2_ws/src && git clone https://github.com/ros-perception/vision_msgs -b ros2
$ cd ros2_ws && rosdep install --from src --ignore-src -r -y \
  && colcon build --symlink-install --packages-up-to vision_msgs_rviz_plugins
```

__Testing:__
```bash
$ ros2 launch vision_msgs_rviz_plugins test_all.launch.py 
```