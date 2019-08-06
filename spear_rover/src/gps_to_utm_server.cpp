#include "ros/ros.h"
#include "robot_localization/navsat_conversions.h"
#include "spear_rover/GpsToUtm.h"

#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"

#include <string>

bool gps_to_utm(spear_rover::GpsToUtm::Request &req,
                spear_rover::GpsToUtm::Response &res) {
    double utm_x, utm_y;
    std::string utm_zone;
    RobotLocalization::NavsatConversions::LLtoUTM(req.gps_fix.latitude, req.gps_fix.longitude, utm_y, utm_x, utm_zone);
    res.utm_coord.x = utm_x;
    res.utm_coord.y = utm_y;
    res.utm_coord.z = req.gps_fix.altitude;
    return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "gps_to_utm_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("gps_to_utm", gps_to_utm);
    ROS_INFO("GPS to UTM conversion service ready.");
    ros::spin();
    return 0;
}