#pragma once

#ifndef Q_MOC_RUN
#include <vision_msgs/Detection3DArray.h>

#include "vision_msgs_rviz/detection_3d_visual.h"
#include "rviz/message_filter_display.h"
#endif

#include <mutex>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class BoolProperty;
class FloatProperty;
class IntProperty;

class Detection3DArrayDisplay : public rviz::MessageFilterDisplay<vision_msgs::Detection3DArray>
{
  Q_OBJECT
public:
  Detection3DArrayDisplay();
  ~Detection3DArrayDisplay() override = default;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void slotShowHypothesis();
  void slotShowBox();
  void slotShowPoints();
  void slotShowProb();
  void slotLabelSize();
  void slotShowTrackId();

private:
  void processMessage(const vision_msgs::Detection3DArray::ConstPtr& msg) override;

  vision_msgs::Detection3DArray::ConstPtr initMsg_;

  std::mutex mutex_;
  std::vector<std::shared_ptr<Detection3DVisual> > visuals_;

  rviz::IntProperty* showHypothesis_;
  rviz::BoolProperty* showId_;
  rviz::BoolProperty* showProb_;
  rviz::BoolProperty* showBox_;
  rviz::BoolProperty* showPoints_;
  rviz::FloatProperty* labelSize_;

};
}  // namespace rviz