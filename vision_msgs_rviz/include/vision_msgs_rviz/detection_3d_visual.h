#pragma once

#include "vision_msgs/Detection3D.h"

#include <QColor>
#include <tf2_eigen/tf2_eigen.h>

#include <vision_msgs_rviz/bounding_box_visual.h>

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

class Detection3DVisual
{
public:
  Detection3DVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~Detection3DVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const vision_msgs::Detection3D msg);

  void setShowBox(bool b);
  void setShowHyp(const int num);
  void setCharacterHeight(const float size);
  void setShowPoints(const bool show);
  void setShowPropability(const bool show);
  void setShowId(const bool show);

protected:

  void updateLabel();

private:
  bool show_prob_{false};
  bool show_id_{false};
  int show_hyp_{0};
  float text_size_;
  vision_msgs::Detection3D last_msg_;

  Ogre::SceneManager* scene_manager_{ nullptr };
  Ogre::SceneNode* main_node_{ nullptr };
  Ogre::SceneNode* bb_node_{ nullptr };
  Ogre::SceneNode* text_node_{ nullptr };
  Ogre::SceneNode* points_node_{ nullptr };

  rviz::MovableText* text_{ nullptr };

  std::unique_ptr<BoundingBoxVisual> bbox_{nullptr};
};
}  // namespace rviz