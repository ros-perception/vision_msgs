#ifndef BOUNDING_BOX_3D_DISPLAY_HPP_
#define BOUNDING_BOX_3D_DISPLAY_HPP_

#include <memory>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <QWidget>

#include "vision_msgs_rviz_plugins/bounding_box_3d_common.hpp"
#include "vision_msgs_rviz_plugins/visibility_control.hpp"

typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;

namespace rviz_plugins
{

    class BoundingBox3DDisplay
        : public BoundingBox3DCommon<vision_msgs::msg::BoundingBox3D>
    {
        Q_OBJECT
    public:
        using Marker = visualization_msgs::msg::Marker;
        using BoundingBox3D = vision_msgs::msg::BoundingBox3D;

        BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        BoundingBox3DDisplay();
        BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        ~BoundingBox3DDisplay();
        BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        void onInitialize() override;
        BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        void load(const rviz_common::Config &config) override;
        BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        void update(float wall_dt, float ros_dt) override;
        BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        void reset() override;

    private:
        // Convert boxes into markers, push them to the display queue
        void processMessage(BoundingBox3D::ConstSharedPtr msg) override;
        BoundingBox3D::ConstSharedPtr latest_msg;

    protected:
        bool only_edge_;
        rviz_common::properties::BoolProperty *only_edge_property_;
        rviz_common::properties::FloatProperty *line_width_property_;
        rviz_common::properties::FloatProperty *alpha_property_;
        rviz_common::properties::ColorProperty *color_property_;
    protected Q_SLOTS:
        void updateEdge();
        void updateLineWidth();
        void updateAlpha();
        void updateColor();
    };
} // namespace rviz_plugins

#endif // BOUNDING_BOX_3D_DISPLAY_HPP_