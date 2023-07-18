# vision_msgs_rviz_plugins

This package contains a RVIZ2 plugin to display vision_msgs for ROS 2.

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