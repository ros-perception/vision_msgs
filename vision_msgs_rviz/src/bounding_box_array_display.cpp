#include "vision_msgs_rviz/bounding_box_array_display.h"

#include <QString>
#include <memory>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

namespace rviz
{
BoundingBoxArrayDisplay::BoundingBoxArrayDisplay()
{
}

void BoundingBoxArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void BoundingBoxArrayDisplay::reset()
{
  MFDClass::reset();
}

void BoundingBoxArrayDisplay::processMessage(const vision_msgs::BoundingBox3DArray::ConstPtr& msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    int count = 0;
    // scene_node to map

    for (auto bbox : msg->boxes)
    {
      std::shared_ptr<BoundingBoxVisual> box;
      if (++count > boxes_.size())
      {
        box = std::make_shared<BoundingBoxVisual>(context_->getSceneManager(), scene_node_);
        boxes_.push_back(box);
      }
      else
      {
        box = boxes_.at(count - 1);
      }
      box->setMessage(bbox);
    }

    for (int i = boxes_.size() - count; i > 0; i--)
    {
      boxes_.pop_back();
    }
  }  // lock_guard mutex_
}

}  // namespace rviz
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::BoundingBoxArrayDisplay, rviz::Display)