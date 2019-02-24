#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>
#include <rover2/ArmAngles.h>

#include "arm_parameters.h"

//TODO: Separate this into a more standalone display application?
// http://docs.ros.org/melodic/api/librviz_tutorial/html/index.html

ros::NodeHandle* node;
ros::Subscriber arm_angles_sub;
ros::Publisher arm_visuals_pub;

void armAngleUpdateCallback(const rover2::ArmAngles::ConstPtr& msg){
    visualization_msgs::Marker marker;
    //TODO: Add markers for hand, so we can see the open/close state?

    visualization_msgs::MarkerArray fullArm;

    //Fill out marker fields
    // http://wiki.ros.org/rviz/DisplayTypes/Marker

    //Bicep Marker
    marker.header.frame_id = "shoulder_mount";
    marker.header.stamp = ros::Time();
    marker.ns = "bicep";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    
    fullArm.markers.push_back(marker);

    //Forearm Marker
    marker.header.frame_id = "bicep";
    marker.header.stamp = ros::Time();
    marker.ns = "forearm";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 1;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    
    fullArm.markers.push_back(marker);

    //Wrist Marker
    marker.header.frame_id = "shoulder_mount";
    marker.header.stamp = ros::Time();
    marker.ns = "wrist";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 2;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration();
    
    fullArm.markers.push_back(marker);



    arm_visuals_pub.publish(fullArm);
}

void setupTransforms(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 1.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "shoulder_mount"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_visual");
    node = new ros::NodeHandle;
    arm_angles_sub = node->subscribe("/arm/angles", 1000, armAngleUpdateCallback);
    arm_visuals_pub = node->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    //setupTransforms();
    ros::spin();
    return 0;
}
