#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreQuaternion.h>

#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <QColor>

#include <ros/ros.h>

#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/properties/parse_color.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>

#include "vision_msgs_rviz/bounding_box_visual.h"

namespace rviz
{
BoundingBoxVisual::BoundingBoxVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  object_node_ = parent_node->createChildSceneNode();
  manual_object_ = scene_manager->createManualObject();
  manual_object_->setDynamic(true);
  object_node_->attachObject(manual_object_);
}

BoundingBoxVisual::~BoundingBoxVisual()
{
  scene_manager_->destroySceneNode(object_node_);
}

void BoundingBoxVisual::addPose(Eigen::Vector3d& p)
{
  manual_object_->position(p[0], p[1], p[2]);
}

// ----------------------------------------------------------------------------
void BoundingBoxVisual::setMessage(const vision_msgs::BoundingBox3D msg)
{
  auto center = msg.center;
  auto size = msg.size;
  auto min_point = Eigen::Vector3d(-size.x / 2, -size.y / 2, -size.z / 2);
  auto max_point = Eigen::Vector3d(size.x / 2, size.y / 2, size.z / 2);

  Eigen::Isometry3d pose;
  
  tf::poseMsgToEigen(center, pose);

  // Extract 8 cuboid vertices
  Eigen::Vector3d p1(min_point[0], min_point[1], min_point[2]);
  Eigen::Vector3d p2(min_point[0], min_point[1], max_point[2]);
  Eigen::Vector3d p3(max_point[0], min_point[1], max_point[2]);
  Eigen::Vector3d p4(max_point[0], min_point[1], min_point[2]);
  Eigen::Vector3d p5(min_point[0], max_point[1], min_point[2]);
  Eigen::Vector3d p6(min_point[0], max_point[1], max_point[2]);
  Eigen::Vector3d p7(max_point[0], max_point[1], max_point[2]);
  Eigen::Vector3d p8(max_point[0], max_point[1], min_point[2]);

  p1 = pose * p1;
  p2 = pose * p2;
  p3 = pose * p3;
  p4 = pose * p4;
  p5 = pose * p5;
  p6 = pose * p6;
  p7 = pose * p7;
  p8 = pose * p8;

  manual_object_->clear();

  // TODO(lruegeme): check if msg changed

  Ogre::ColourValue color = rviz::qtToOgre(color_);
  color.a = 1;

  manual_object_->estimateVertexCount(18);
  manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  addPose(p1);
  manual_object_->colour(color);
  addPose(p2);
  manual_object_->colour(color);

  addPose(p1);
  manual_object_->colour(color);
  addPose(p4);
  manual_object_->colour(color);

  addPose(p1);
  manual_object_->colour(color);
  addPose(p5);
  manual_object_->colour(color);

  addPose(p5);
  manual_object_->colour(color);
  addPose(p6);
  manual_object_->colour(color);

  addPose(p5);
  manual_object_->colour(color);
  addPose(p8);
  manual_object_->colour(color);

  addPose(p4);
  manual_object_->colour(color);
  addPose(p3);
  manual_object_->colour(color);

  addPose(p2);
  manual_object_->colour(color);
  addPose(p6);
  manual_object_->colour(color);

  addPose(p7);
  manual_object_->colour(color);
  addPose(p8);
  manual_object_->colour(color);

  addPose(p7);
  manual_object_->colour(color);
  addPose(p3);
  manual_object_->colour(color);

  manual_object_->end();
}

}  // namespace rviz
