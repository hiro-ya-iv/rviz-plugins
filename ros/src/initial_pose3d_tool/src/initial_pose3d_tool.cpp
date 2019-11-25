#include "initial_pose3d_tool.hpp"

#include <ros/console.h>
#include <rviz/display_context.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

namespace {

double getGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap, const tf2::Vector3& point)
{
    constexpr double radius = 1.0 * 1.0;
    const double x = point.getX();
    const double y = point.getY();

    double height = INFINITY;
    for(const auto& p : pcdmap->points)
    {
        const double dx = x - p.x;
        const double dy = y - p.y;
        const double sd = (dx * dx) + (dy * dy);
        if(sd < radius)
        {
            height = std::min(height, static_cast<double>(p.z));
        }
    }
    return std::isfinite(height) ? height : point.getZ();
}

}

namespace rviz_plugins {

InitialPose3dTool::InitialPose3dTool()
{
    shortcut_key_ = '3';

    pcdmap_available_ = false;
    pcdmap_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pose_topic_property_ = new rviz::StringProperty("Pose Topic", "initialpose3d", "Topic name of 3d pose estimate.", getPropertyContainer(), SLOT(updateTopic()), this);
    height_property_ = new rviz::FloatProperty("Height [m]", 0.0, "Height for pose estimate.", getPropertyContainer());
    roll_property_ = new rviz::FloatProperty("Roll [rad]", 0.0, "Roll for pose estimate.", getPropertyContainer());
    pitch_property_ = new rviz::FloatProperty("Pitch [rad]", 0.0, "Pitch for pose estimate.", getPropertyContainer());

    cov_x_x_property_ = new rviz::FloatProperty("CovXX [m^2]", 0.0, "Covariance[0].", getPropertyContainer());
    cov_y_y_property_ = new rviz::FloatProperty("CovYY [m^2]", 0.0, "Covariance[7].", getPropertyContainer());
    cov_z_z_property_ = new rviz::FloatProperty("CovZZ [m^2]", 0.0, "Covariance[14].", getPropertyContainer());
    cov_roll_roll_property_ = new rviz::FloatProperty("CovRollRoll [m^2]", 0.0, "Covariance[21].", getPropertyContainer());
    cov_pitch_pitch_property_ = new rviz::FloatProperty("CovPitchPitch [m^2]", 0.0, "Covariance[28].", getPropertyContainer());
    cov_yaw_yaw_property_ = new rviz::FloatProperty("CovYawYaw [m^2]", 0.0, "Covariance[35].", getPropertyContainer());
    cov_x_y_property_ = new rviz::FloatProperty("CovXY [m^2]", 0.0, "Covariance[1].", getPropertyContainer());
    cov_x_z_property_ = new rviz::FloatProperty("CovXZ [m^2]", 0.0, "Covariance[2].", getPropertyContainer());
    cov_x_roll_property_ = new rviz::FloatProperty("CovXRoll [m^2]", 0.0, "Covariance[3].", getPropertyContainer());
    cov_x_pitch_property_ = new rviz::FloatProperty("CovXPitch [m^2]", 0.0, "Covariance[4].", getPropertyContainer());
    cov_x_yaw_property_ = new rviz::FloatProperty("CovXYaw [m^2]", 0.0, "Covariance[5].", getPropertyContainer());
    cov_y_z_property_ = new rviz::FloatProperty("CovYZ [m^2]", 0.0, "Covariance[8].", getPropertyContainer());
    cov_y_roll_property_ = new rviz::FloatProperty("CovYRoll [m^2]", 0.0, "Covariance[9].", getPropertyContainer());
    cov_y_pitch_property_ = new rviz::FloatProperty("CovYPitch [m^2]", 0.0, "Covariance[10].", getPropertyContainer());
    cov_y_yaw_property_ = new rviz::FloatProperty("CovYYaw [m^2]", 0.0, "Covariance[11].", getPropertyContainer());
    cov_z_roll_property_ = new rviz::FloatProperty("CovZRoll [m^2]", 0.0, "Covariance[15].", getPropertyContainer());
    cov_z_pitch_property_ = new rviz::FloatProperty("CovZPitch [m^2]", 0.0, "Covariance[16].", getPropertyContainer());
    cov_z_yaw_property_ = new rviz::FloatProperty("CovZYaw [m^2]", 0.0, "Covariance[17].", getPropertyContainer());
    cov_roll_pitch_property_ = new rviz::FloatProperty("CovRollPitch [m^2]", 0.0, "Covariance[22].", getPropertyContainer());
    cov_roll_yaw_property_ = new rviz::FloatProperty("CovRollYaw [m^2]", 0.0, "Covariance[23].", getPropertyContainer());
    cov_pitch_yaw_property_ = new rviz::FloatProperty("CovPitchYaw [m^2]", 0.0, "Covariance[29].", getPropertyContainer());

    auto_height_property_ = new rviz::BoolProperty("Auto Height", false, "Get height from point cloud map.", getPropertyContainer(), SLOT(updateGround()), this);
    map_topic_property_ = new rviz::RosTopicProperty("Map Topic", "/points_map", "Topic name of point cloud map", "sensor_msgs/PointCloud2", getPropertyContainer(), SLOT(updateGround()), this);
}

void InitialPose3dTool::onInitialize()
{
    PoseTool::onInitialize();
    updateTopic();
    updateGround();
    setName("3D Pose Estimate");
}

void InitialPose3dTool::updateTopic()
{
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_property_->getStdString(), 1);
}

void InitialPose3dTool::updateGround()
{
    pcdmap_available_ = false;
    if(auto_height_property_->getBool() == false)
    {
        return;
    }

    ROS_INFO_STREAM("Load point cloud map");
    const auto pcdmsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(map_topic_property_->getTopicStd(), ros::Duration(3.0));

    if(pcdmsg)
    {
        pcdmap_available_ = true;
        pcdmap_frame_ = pcdmsg->header.frame_id;
        pcl::fromROSMsg(*pcdmsg, *pcdmap_);
    }
    else
    {
        ROS_WARN_STREAM("failed to subscribe message: " + map_topic_property_->getTopicStd());
    }
}

void InitialPose3dTool::onPoseSet(double x, double y, double yaw)
{
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf2::Vector3 point(x, y, height_property_->getFloat());

    if(pcdmap_available_)
    {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        tf2::Transform transform;
        try
        {
            const auto stamped = tf_buffer.lookupTransform(pcdmap_frame_, fixed_frame, ros::Time(0), ros::Duration(1.0));
            tf2::fromMsg(stamped.transform, transform);
        }
        catch (tf2::TransformException& exception)
        {
            ROS_WARN_STREAM("failed to lookup transform: " << exception.what());
        }

        point = transform * point;
        point.setZ(getGroundHeight(pcdmap_, point));
        point = transform.inverse() * point;
    }

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = fixed_frame;
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = point.getX();
    msg.pose.pose.position.y = point.getY();
    msg.pose.pose.position.z = point.getZ();

    double roll = roll_property_->getFloat();
    double pitch = pitch_property_->getFloat();
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    msg.pose.pose.orientation = tf2::toMsg(quaternion);

    msg.pose.covariance[0] = cov_x_x_property_->getFloat();
    msg.pose.covariance[1] = cov_x_y_property_->getFloat();
    msg.pose.covariance[2] = cov_x_z_property_->getFloat();
    msg.pose.covariance[3] = cov_x_roll_property_->getFloat();
    msg.pose.covariance[4] = cov_x_pitch_property_->getFloat();
    msg.pose.covariance[5] = cov_x_yaw_property_->getFloat();
    msg.pose.covariance[6] = cov_x_y_property_->getFloat();
    msg.pose.covariance[7] = cov_y_y_property_->getFloat();
    msg.pose.covariance[8] = cov_y_z_property_->getFloat();
    msg.pose.covariance[9] = cov_y_roll_property_->getFloat();
    msg.pose.covariance[10] = cov_y_pitch_property_->getFloat();
    msg.pose.covariance[11] = cov_y_yaw_property_->getFloat();
    msg.pose.covariance[12] = cov_x_z_property_->getFloat();
    msg.pose.covariance[13] = cov_y_z_property_->getFloat();
    msg.pose.covariance[14] = cov_z_z_property_->getFloat();
    msg.pose.covariance[15] = cov_z_roll_property_->getFloat();
    msg.pose.covariance[16] = cov_z_pitch_property_->getFloat();
    msg.pose.covariance[17] = cov_z_yaw_property_->getFloat();
    msg.pose.covariance[18] = cov_x_roll_property_->getFloat();
    msg.pose.covariance[19] = cov_y_roll_property_->getFloat();
    msg.pose.covariance[20] = cov_z_roll_property_->getFloat();
    msg.pose.covariance[21] = cov_roll_roll_property_->getFloat();
    msg.pose.covariance[22] = cov_roll_pitch_property_->getFloat();
    msg.pose.covariance[23] = cov_roll_yaw_property_->getFloat();
    msg.pose.covariance[24] = cov_x_pitch_property_->getFloat();
    msg.pose.covariance[25] = cov_y_pitch_property_->getFloat();
    msg.pose.covariance[26] = cov_z_pitch_property_->getFloat();
    msg.pose.covariance[27] = cov_roll_pitch_property_->getFloat();
    msg.pose.covariance[28] = cov_pitch_pitch_property_->getFloat();
    msg.pose.covariance[29] = cov_pitch_yaw_property_->getFloat();
    msg.pose.covariance[30] = cov_x_yaw_property_->getFloat();
    msg.pose.covariance[31] = cov_y_yaw_property_->getFloat();
    msg.pose.covariance[32] = cov_z_yaw_property_->getFloat();
    msg.pose.covariance[33] = cov_roll_yaw_property_->getFloat();
    msg.pose.covariance[34] = cov_pitch_yaw_property_->getFloat();
    msg.pose.covariance[35] = cov_yaw_yaw_property_->getFloat();

    ROS_INFO("Setting pose3d: %.3f %.3f %.3f %.3f %.3f %.3f [frame=%s]", point.x(), point.y(), point.z(), roll, pitch, yaw, fixed_frame.c_str());

    pub_.publish(msg);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::InitialPose3dTool, rviz::Tool)
