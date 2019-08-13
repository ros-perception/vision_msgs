#pragma once

#include "vision_msgs/BoundingBox3D.h"

#include <QColor>
#include <tf2_eigen/tf2_eigen.h>

namespace Ogre
{
class Entity;
class Vector3;
class Quaternion;
class ManualObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace rviz
{
class MovableText;

class BoundingBoxVisual
{
public:
  BoundingBoxVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~BoundingBoxVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const vision_msgs::BoundingBox3D msg);

private:
  void addPose(Eigen::Vector3d& p);

  Ogre::SceneNode* object_node_{ nullptr };
  Ogre::SceneManager* scene_manager_{ nullptr };

  Ogre::ManualObject* manual_object_{ nullptr };

  QColor color_{ QColor(0, 255, 0) };
};
}  // namespace rviz