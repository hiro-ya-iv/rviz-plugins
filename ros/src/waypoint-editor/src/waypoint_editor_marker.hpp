#ifndef WAYPOINT_EDITOR_MARKER_HPP
#define WAYPOINT_EDITOR_MARKER_HPP

#include "types/waypoint.hpp"
#include <ros/ros.h>
#include <string>

namespace rviz_plugins {

class WaypointEditorMarker
{
    public:

        WaypointEditorMarker();
        ~WaypointEditorMarker() = default;

        void publish(const Waypoints& waypoints, const std::string& frame);

    private:

        ros::NodeHandle nh_;
        ros::Publisher  pub_;
};

}

#endif
