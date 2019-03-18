#include "pose_history_display.hpp"

namespace rviz_plugins {

PoseHistory::PoseHistory()
{
    const char* topic_type = ros::message_traits::datatype<geometry_msgs::PoseStamped>();
    const char* topic_desc = "Name of topic to display";
    property_topic_ = new rviz::RosTopicProperty("Topic", "", topic_type, topic_desc, this, SLOT(updateTopic()));
    property_duration_ = new rviz::FloatProperty("Duration", 5.0, "", this);
    property_line_view_ = new rviz::BoolProperty("Line", true, "", this);
    property_line_width_ = new rviz::FloatProperty("Width", 0.1, "", property_line_view_);
    property_line_color_ = new rviz::ColorProperty("Color", Qt::white, "", property_line_view_);

    property_duration_->setMin(0.0);
    property_line_width_->setMin(0.0);
}

PoseHistory::~PoseHistory()
{
    // Properties are deleted by Qt
}

void PoseHistory::onInitialize()
{
    lines_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
}

void PoseHistory::onEnable()
{
    subscribe();
}

void PoseHistory::onDisable()
{
    unsubscribe();
}

void PoseHistory::update(float wall_dt, float ros_dt)
{
    if(!history_.empty())
    {
        updateHistory();
        updateLines();
    }
}

void PoseHistory::updateTopic()
{
    unsubscribe();
    subscribe();
}

void PoseHistory::subscribe()
{
    auto topic_name = property_topic_->getTopicStd();
    if(1 < topic_name.length())
    {
        sub_ = nh_.subscribe(topic_name, 10, &PoseHistory::onMessage, this);
    }
}

void PoseHistory::unsubscribe()
{
    sub_.shutdown();
}

void PoseHistory::onMessage(const geometry_msgs::PoseStamped& message)
{
    if(target_frame_ != message.header.frame_id)
    {
        history_.clear();
        target_frame_ = message.header.frame_id;
    }
    history_.emplace_back(message);
}

void PoseHistory::updateHistory()
{
    auto timeout = ros::Time::now() - ros::Duration(property_duration_->getFloat());
    while(!history_.empty())
    {
        if(timeout < history_.front().header.stamp)
        {
            break;
        }
        history_.pop_front();
    }
}

void PoseHistory::updateLines()
{
    lines_->clear();
    if(!property_line_view_->getBool())
    {
        return;
    }

    QColor color = property_line_color_->getColor();
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    auto frame_manager = context_->getFrameManager();
    if(!frame_manager->getTransform(target_frame_, ros::Time(0), position, orientation))
    {
        std::string error;
        frame_manager->transformHasProblems(target_frame_, ros::Time(0), error);
        setStatusStd(rviz::StatusProperty::Error, "Transform", error);
        return;
    }

    setStatusStd(rviz::StatusProperty::Ok, "Transform", "Transform OK");
    lines_->setMaxPointsPerLine(history_.size());
    lines_->setLineWidth(property_line_width_->getFloat());
    lines_->setPosition(position);
    lines_->setOrientation(orientation);
    lines_->setColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());

    for(const auto& message : history_)
    {
        Ogre::Vector3 point;
        point.x = message.pose.position.x;
        point.y = message.pose.position.y;
        point.z = message.pose.position.z;
        lines_->addPoint(point);
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseHistory, rviz::Display)
