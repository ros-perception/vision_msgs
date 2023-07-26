^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vision_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2020-08-11)
------------------
* Fix lint error for draconian header guard rule
* Rename create_aabb to use C++ extension
  This fixes linting errors which assume that .h means that a file
  is C (rather than C++).
* Add CONTRIBUTING.md
* Fix various linting issues
* Add gitignore
  Sync ros2 with master
* Update test for ros2
* add BoundingBox3DArray message (`#30 <https://github.com/Kukanani/vision_msgs/issues/30>`_)
  * add BoundingBoxArray message
* Make msg gen package deps more specific (`#24 <https://github.com/Kukanani/vision_msgs/issues/24>`_)
  Make message_generation and message_runtime use more specific depend tags
* Merge branch 'kinetic-devel'
* Removed "proposal" from readme (`#23 <https://github.com/Kukanani/vision_msgs/issues/23>`_)
* add tracking ID to the Detection Message (`#19 <https://github.com/Kukanani/vision_msgs/issues/19>`_)
  * add tracking ID to the Detection
  * modify comments
  * Change UUID messages to strings
  * Improve comment for tracking_id and fix whitespace
* Convert id to string (`#22 <https://github.com/Kukanani/vision_msgs/issues/22>`_)
* Specify that id is explicitly for object class
* Fix dependency of unit test. (`#14 <https://github.com/Kukanani/vision_msgs/issues/14>`_)
* 0.0.1
* Pre-release commit - setting up versioning and changelog
* Rolled BoundingRect into BoundingBox2D
  Added helper functions to make it easier to go from corner-size representation to
  center-size representation, plus associated tests.
* Added license
* Small fixes in message comments (`#10 <https://github.com/Kukanani/vision_msgs/issues/10>`_)
* Contributors: Adam Allevato, Leroy Rügemer, Martin Günther, Masaya Kataoka, Ronald Ensing, Shane Loretz, mistermult
* Switched to ROS2 for package definition files, create_aabb, etc.
* [ros2] use package format 3 (`#12 <https://github.com/Kukanani/vision_msgs/issues/12>`_)
* Contributors: Adam Allevato, Martin Günther, Mikael Arguedas, procopiostein
