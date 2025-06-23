// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISION_MSGS_RVIZ_PLUGINS__DETECTION_3D_HPP_
#define VISION_MSGS_RVIZ_PLUGINS__DETECTION_3D_HPP_

#include <QWidget>
#include <memory>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>


#include "vision_msgs_rviz_plugins/detection_3d_common.hpp"
#include "vision_msgs_rviz_plugins/visibility_control.hpp"

typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;

namespace rviz_plugins
{

class Detection3DDisplay
  : public Detection3DCommon<vision_msgs::msg::Detection3D>
{
  Q_OBJECT

public:
  using Marker = visualization_msgs::msg::Marker;
  using BoundingBox3D = vision_msgs::msg::BoundingBox3D;
  using Detection3D = vision_msgs::msg::Detection3D;

  DETECTION_3D_DISPLAY_HPP_PUBLIC
  Detection3DDisplay();
  DETECTION_3D_DISPLAY_HPP_PUBLIC
  ~Detection3DDisplay();
  DETECTION_3D_DISPLAY_HPP_PUBLIC
  void onInitialize() override;
  DETECTION_3D_DISPLAY_HPP_PUBLIC
  void load(const rviz_common::Config & config) override;
  DETECTION_3D_DISPLAY_HPP_PUBLIC
  void update(float wall_dt, float ros_dt) override;
  DETECTION_3D_DISPLAY_HPP_PUBLIC
  void reset() override;

private:
  // Convert boxes into markers, push them to the display queue
  void processMessage(Detection3D::ConstSharedPtr array) override;
  vision_msgs::msg::Detection3D::ConstSharedPtr latest_msg;

protected:
  bool only_edge_, show_score_;
  rviz_common::properties::BoolProperty * only_edge_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::IntProperty * lifetime_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::BoolProperty * show_score_property_;

protected Q_SLOTS:
  void updateEdge();
  void updateLineWidth();
  void updateLifetime();
  void updateAlpha();
  void updateShowScores();
  void updateColorConfigs();
};
}  // namespace rviz_plugins

#endif  // VISION_MSGS_RVIZ_PLUGINS__DETECTION_3D_HPP_
