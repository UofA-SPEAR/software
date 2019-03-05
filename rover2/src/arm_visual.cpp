#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <rover2/ArmAngles.h>

#include <tf2/LinearMath/Quaternion.h>

#include "arm_parameters.h"

//TODO: Separate this into a more standalone display application?
// http://docs.ros.org/melodic/api/librviz_tutorial/html/index.html

ros::NodeHandle* node;
ros::Subscriber arm_angles_sub;
ros::Publisher arm_visuals_pub;

void setupTransforms(float shoulderYaw, float shoulderPitch, float elbowPitch,
        float wristPitch){
    static tf2_ros::StaticTransformBroadcaster st_br;
    
    geometry_msgs::TransformStamped message;
    tf2::Quaternion quat;

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";
    message.child_frame_id = "shoulder_mount";
    message.transform.translation.x = 0;
    message.transform.translation.y = 0;
    message.transform.translation.z = 0;
    quat.setRPY(shoulderPitch, 0, shoulderYaw);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "shoulder_mount";
    message.child_frame_id = "elbow_joint";
    message.transform.translation.x = 0;
    message.transform.translation.y = 1;
    message.transform.translation.z = 0;
    quat.setRPY(elbowPitch-M_PI, 0, 0);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "elbow_joint";
    message.child_frame_id = "wrist_joint";
    message.transform.translation.x = 0;
    message.transform.translation.y = 1;
    message.transform.translation.z = 0;
    quat.setRPY(M_PI-elbowPitch-shoulderPitch+wristPitch, 0, 0);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "wrist_joint";
    message.child_frame_id = "fingertips";
    message.transform.translation.x = 0;
    message.transform.translation.y = 1;
    message.transform.translation.z = 0;
    quat.setRPY(0, 0, 0);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);
}

void putMarkers(){
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
    marker.header.frame_id = "elbow_joint";
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
    marker.header.frame_id = "wrist_joint";
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

void armAngleUpdateCallback(const rover2::ArmAngles::ConstPtr& msg){
    visualization_msgs::Marker marker;
    //TODO: Add markers for hand, so we can see the open/close state?

    setupTransforms(msg->joints[0].angle, msg->joints[1].angle, msg->joints[2].angle, msg->joints[4].angle);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "arm_visual");
    node = new ros::NodeHandle;
    arm_angles_sub = node->subscribe("/arm/angles", 1000, armAngleUpdateCallback);
    arm_visuals_pub = node->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, true); //bool at end makes this a Latched Publisher
    setupTransforms(0, M_PI/4.0f, M_PI/6, M_PI/16);
    ros::spin();

    return 0;
}
