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

#include <vision_msgs_rviz_plugins/detection_3d_array.hpp>

#include <memory>

namespace rviz_plugins
{

Detection3DArrayDisplay::Detection3DArrayDisplay()
{
  only_edge_property_ = new rviz_common::properties::BoolProperty(
    "Only Edge", false, "Display only edges of the boxes", this, SLOT(updateEdge()));
  line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 0.05, "Line width of edges", this, SLOT(updateLineWidth()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Transparency", this, SLOT(updateAlpha()));
  show_score_property_ = new rviz_common::properties::BoolProperty(
    "Show Score", false, "Display score next to bounding boxes", this, SLOT(updateShowScores()));
  string_property_ = new rviz_common::properties::StringProperty(
    "ConfigPath", "", "Path to yaml config for rgb color mappings", this,
    SLOT(updateColorConfigs()));
  lifetime_property_ = new rviz_common::properties::IntProperty(
    "Lifetime", 0, "Detection 3D array lifetime", this, SLOT(updateLifetime()));
}

Detection3DArrayDisplay::~Detection3DArrayDisplay()
{
  delete only_edge_property_;
  delete line_width_property_;
  delete alpha_property_;
  delete show_score_property_;
}

void Detection3DArrayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  m_marker_common->initialize(context_, scene_node_);

  topic_property_->setValue("detection3_d_array");
  topic_property_->setDescription("Detection3DArray topic to subscribe to.");

  line_width_property_->setMax(0.1);
  line_width_property_->setMin(0.01);
  line_width_property_->hide();

  alpha_property_->setMax(1.0);
  alpha_property_->setMin(0.1);

  line_width = line_width_property_->getFloat();
  alpha = alpha_property_->getFloat();
  lifetime = lifetime_property_->getInt();

  only_edge_ = only_edge_property_->getBool();
  show_score_ = show_score_property_->getBool();
}

void Detection3DArrayDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  m_marker_common->load(config);
}

void Detection3DArrayDisplay::processMessage(
  vision_msgs::msg::Detection3DArray::ConstSharedPtr msg)
{
  latest_msg = msg;
  if (!only_edge_) {
    showBoxes(msg, show_score_);
  } else {
    showEdges(msg, show_score_);
  }
}

void Detection3DArrayDisplay::update(float wall_dt, float ros_dt)
{
  m_marker_common->update(wall_dt, ros_dt);
}

void Detection3DArrayDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common->clearMarkers();
  edges_.clear();
}

void Detection3DArrayDisplay::updateEdge()
{
  only_edge_ = only_edge_property_->getBool();
  if (only_edge_) {
    line_width_property_->show();
  } else {
    line_width_property_->hide();
  }
  // Imediately apply attribute
  if (latest_msg) {
    if (only_edge_) {
      showEdges(latest_msg, show_score_);
    } else {
      showBoxes(latest_msg, show_score_);
    }
  }
}

void Detection3DArrayDisplay::updateLineWidth()
{
  line_width = line_width_property_->getFloat();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateLifetime()
{
  lifetime = lifetime_property_->getInt();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateAlpha()
{
  alpha = alpha_property_->getFloat();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateShowScores()
{
  show_score_ = show_score_property_->getBool();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateColorConfigs()
{
  this->updateColorConfig();
}

}  // namespace rviz_plugins

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Detection3DArrayDisplay, rviz_common::Display)
