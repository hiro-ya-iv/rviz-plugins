#ifndef INITIAL_POSE3D_TOOL_HPP
#define INITIAL_POSE3D_TOOL_HPP

#include <ros/ros.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
            void updateGround();

        private:

            bool pcdmap_available_;
            std::string pcdmap_frame_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap_;

            ros::NodeHandle nh_;
            ros::Publisher pub_;
            rviz::StringProperty* pose_topic_property_;
            rviz::FloatProperty* height_property_;
            rviz::FloatProperty* roll_property_;
            rviz::FloatProperty* pitch_property_;
            rviz::FloatProperty* cov_x_x_property_;
            rviz::FloatProperty* cov_x_y_property_;
            rviz::FloatProperty* cov_x_z_property_;
            rviz::FloatProperty* cov_x_roll_property_;
            rviz::FloatProperty* cov_x_pitch_property_;
            rviz::FloatProperty* cov_x_yaw_property_;
            rviz::FloatProperty* cov_y_y_property_;
            rviz::FloatProperty* cov_y_z_property_;
            rviz::FloatProperty* cov_y_roll_property_;
            rviz::FloatProperty* cov_y_pitch_property_;
            rviz::FloatProperty* cov_y_yaw_property_;
            rviz::FloatProperty* cov_z_z_property_;
            rviz::FloatProperty* cov_z_roll_property_;
            rviz::FloatProperty* cov_z_pitch_property_;
            rviz::FloatProperty* cov_z_yaw_property_;
            rviz::FloatProperty* cov_roll_roll_property_;
            rviz::FloatProperty* cov_roll_pitch_property_;
            rviz::FloatProperty* cov_roll_yaw_property_;
            rviz::FloatProperty* cov_pitch_pitch_property_;
            rviz::FloatProperty* cov_pitch_yaw_property_;
            rviz::FloatProperty* cov_yaw_yaw_property_;
            rviz::BoolProperty* auto_height_property_;
            rviz::RosTopicProperty* map_topic_property_;
};

}

#endif
