#ifndef INITIAL_POSE3D_TOOL_HPP
#define INITIAL_POSE3D_TOOL_HPP

#include <ros/ros.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

namespace rviz_plugins {

class InitialPose3dTool : public rviz::PoseTool
{
    Q_OBJECT

        public:
            InitialPose3dTool();
            void onInitialize() override;

        protected:

            void onPoseSet(double x, double y, double theta) override;

        private Q_SLOTS:

            void updateTopic();

        private:

            ros::NodeHandle nh_;
            ros::Publisher pub_;
            rviz::StringProperty* topic_property_;
            rviz::BoolProperty* auto_height_property_;
            rviz::FloatProperty* height_property_;
            rviz::FloatProperty* roll_property_;
            rviz::FloatProperty* pitch_property_;
};

}

#endif
