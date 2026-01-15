^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vision_msgs_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.2.1 (2026-01-15)
------------------
* Updated with new API in RViz (`#113 <https://github.com/ros-perception/vision_msgs/issues/113>`_)
* Contributors: Alejandro Hernández Cordero

4.2.0 (2025-05-29)
------------------
* bumpign to 4.2.0 for kilted release
* Replace ament_target_dependencies with target_link_libraries (`#106 <https://github.com/ros-perception/vision_msgs/issues/106>`_)
* rclcpp::filesystem deprecation (`#107 <https://github.com/ros-perception/vision_msgs/issues/107>`_)
* Set topic description in bounding_box_3d to BoundingBox3D (`#102 <https://github.com/ros-perception/vision_msgs/issues/102>`_)
  Topic description was previously Detection3D and should be BoundingBox3D.
* Contributors: Alejandro Hernández Cordero, Spencer Hallyburton, Steve Macenski

4.1.1 (2024-01-23)
------------------
* bumping to 4.1.1 for release
* Clear stale markers (`#95 <https://github.com/ros-perception/vision_msgs/issues/95>`_)
* Validate BoundingBox3D message (`#85 <https://github.com/ros-perception/vision_msgs/issues/85>`_)
  * Validate BoundingBox3D message
  * correct small size comparison
  * add if/else brackets
  * fix formatting errors
  * fix formatting round 2
* Contributors: Jorge Miarnau, Marq Rasmussen, Steve Macenski

4.1.0 (2023-02-10)
------------------
* bump to 4.1.0 for release
* Fix for error: ‘rcpputils::fs’ has not been declared and QT MOC header (`#84 <https://github.com/ros-perception/vision_msgs/issues/84>`_)
* Update detection_3d_common.hpp (`#83 <https://github.com/ros-perception/vision_msgs/issues/83>`_)
  https://github.com/ros-perception/vision_msgs/issues/80 + added <iomanip> for setprecision() on x86_64
* Revert "std::basic_ostream<char>’ has no member named ‘str’ (`#81 <https://github.com/ros-perception/vision_msgs/issues/81>`_)" (`#82 <https://github.com/ros-perception/vision_msgs/issues/82>`_)
  This reverts commit 03fbe93ebd503e07a4c30ff7f08f4b3a61e088ce.
* std::basic_ostream<char>’ has no member named ‘str’ (`#81 <https://github.com/ros-perception/vision_msgs/issues/81>`_)
  Fix https://github.com/ros-perception/vision_msgs/issues/80
* Added rviz plugins (`#79 <https://github.com/ros-perception/vision_msgs/issues/79>`_)
  * Added rviz plugins
  Added meta package and moved initial vision_msgs package
  * Removed submodule & restructure
  * Update package.xml
  Removed qt5 depend
  * Update package.xml
  Added boost depend
  * Fixed most tests
  * Removed boost + colcon test fixes + change requests
  * Adapt copyright + changed for yaml_cpp_vendor
  * Update package.xml
  * removed tf_transformations dependency
* Contributors: Davide Aguiari, Georg No, Steve Macenski, stevemacenski

4.0.0 (2022-03-19)
------------------

3.0.1 (2021-07-20)
------------------

3.0.0 (2021-04-13)
------------------

2.0.0 (2020-08-11)
------------------

1.0.0 (2018-04-04)
------------------

0.0.1 (2017-11-14)
------------------
