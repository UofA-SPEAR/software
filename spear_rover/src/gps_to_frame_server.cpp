#include "ros/ros.h"
#include "robot_localization/navsat_conversions.h"
#include "spear_rover/GpsToFrame.h"

#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>


using namespace std::chrono_literals;


std::unique_ptr<tf::TransformListener> listener;


bool gps_to_frame(spear_rover::GpsToFrame::Request &req,
                spear_rover::GpsToFrame::Response &res) {
    ROS_INFO("Transforming point.");
    double utm_x, utm_y;
    std::string utm_zone;
    const double lat = req.gps_coord.x;
    const double lon = req.gps_coord.y;
    const double alt = req.gps_coord.z;
    RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone);

    const tf::Stamped<tf::Point> utm_point(
        {utm_x, utm_y, alt},
        ros::Time::now(),
        "utm"
    );
    tf::Stamped<tf::Point> map_point;
    while (true) {
        try {
            listener->transformPoint(req.frame.data, utm_point, map_point);
            break;
        }
        catch(tf::TransformException& err) {
            auto message = std::string("Could not transform utm -> map: ") + err.what();
            ROS_ERROR(message.c_str());
            std::this_thread::sleep_for(3s);
        }
    }
    res.coord.x = map_point.getX();
    res.coord.y = map_point.getY();
    res.coord.z = map_point.getZ();
    return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "gps_to_frame_server");
    ros::NodeHandle n;
    listener = std::make_unique<tf::TransformListener>(n);
    ros::ServiceServer service = n.advertiseService("gps_to_frame", gps_to_frame);
    ROS_INFO("GPS to UTM conversion service ready.");
    ros::spin();
    return 0;
}