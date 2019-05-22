#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <spear_msgs/ArmAngles.h>
#include <spear_msgs/arm_position.h>

#include <tf2/LinearMath/Quaternion.h>

#include "arm_parameters.h"

//TODO: Separate this into a more standalone display application?
// http://docs.ros.org/melodic/api/librviz_tutorial/html/index.html

ros::NodeHandle* node;
ros::Subscriber arm_angles_sub;
ros::Subscriber arm_coords_sub;
ros::Publisher arm_visuals_pub;

tf2_ros::Buffer tfBuf;
tf2_ros::TransformListener* listener;

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
    quat.setRPY(0, -shoulderPitch, shoulderYaw);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "shoulder_mount";
    message.child_frame_id = "elbow_joint";
    message.transform.translation.x = BICEP_LENGTH;
    message.transform.translation.y = 0;
    message.transform.translation.z = 0;
    quat.setRPY(0, M_PI-elbowPitch, 0);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "elbow_joint";
    message.child_frame_id = "wrist_joint";
    message.transform.translation.x = FOREARM_LENGTH;
    message.transform.translation.y = 0;
    message.transform.translation.z = 0;
    quat.setRPY(0, elbowPitch-M_PI+shoulderPitch-wristPitch, 0);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "wrist_joint";
    message.child_frame_id = "fingertips";
    message.transform.translation.x = WRIST_LENGTH;
    message.transform.translation.y = 0;
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

    visualization_msgs::Marker marker;

    geometry_msgs::TransformStamped transform;
    
    ROS_DEBUG("dsfg");


    //Fill out marker fields
    // http://wiki.ros.org/rviz/DisplayTypes/Marker

    try{
        transform = tfBuf.lookupTransform("map", "shoulder_mount", ros::Time(0), ros::Duration(3.0));
        ROS_INFO("hi there");
    }
    catch(tf::TransformException& e){
        ROS_ERROR("%s",e.what());
    }
    ROS_INFO("what");
    //Bicep Marker
    marker.header.frame_id = "shoulder_mount";
    marker.header.stamp = ros::Time();
    marker.ns = "bicep";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = BICEP_LENGTH;
    marker.scale.y = BICEP_WIDTH;
    marker.scale.z = BICEP_HEIGHT;
    marker.pose.position.x = 0;//(marker.scale.y/2.0);//(marker.scale.x/2.0);//+transform.transform.translation.x;/*+ shoulder transform Xpos*/;
    marker.pose.position.y = 0;//+transform.transform.translation.y;/*+ shoulder transform Ypos*/;
    marker.pose.position.z = 0;//(marker.scale.z/2.0);//+transform.transform.translation.z;/*+ shoulder transform Zpos*/;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    //marker.pose.orientation = transform.transform.rotation;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    
    fullArm.markers.push_back(marker);

    try{
        transform = tfBuf.lookupTransform("shoulder_mount", "elbow_joint", ros::Time(0));
    }
    catch(tf::TransformException& e){
        ROS_ERROR("%s",e.what());
    }
    //Forearm Marker
    marker.header.frame_id = "elbow_joint";
    marker.header.stamp = ros::Time();
    marker.ns = "forearm";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = FOREARM_LENGTH;
    marker.scale.y = FOREARM_WIDTH;
    marker.scale.z = FOREARM_HEIGHT;
    marker.pose.position.x = 0;//(marker.scale.y/2.0);//(marker.scale.x/2.0);//+transform.transform.translation.x;/*+ elbow transform Xpos*/;
    marker.pose.position.y = 0;//+transform.transform.translation.y;/*+ elbow transform Ypos*/;
    marker.pose.position.z = 0;//(marker.scale.z/2.0);//+transform.transform.translation.z;/*+ elbow transform Zpos*/;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    
    fullArm.markers.push_back(marker);

    try{
        transform = tfBuf.lookupTransform("elbow_joint", "wrist_joint", ros::Time(0));
    }
    catch(tf::TransformException& e){
        ROS_ERROR("%s",e.what());
    }
    //Wrist Marker
    marker.header.frame_id = "wrist_joint";
    marker.header.stamp = ros::Time();
    marker.ns = "wrist";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = WRIST_LENGTH;
    marker.scale.y = WRIST_WIDTH;
    marker.scale.z = WRIST_HEIGHT;
    marker.pose.position.x = 0;//(marker.scale.y/2.0);//(marker.scale.x/2.0);//+transform.transform.translation.x;/*+ wrist transform Xpos*/;
    marker.pose.position.y = 0;//+transform.transform.translation.y;/*+ wrist transform Ypos*/;
    marker.pose.position.z = 0;//(marker.scale.z/2.0);//+transform.transform.translation.z;/*+ wrist transform Zpos*/;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration();
    
    fullArm.markers.push_back(marker);



    arm_visuals_pub.publish(fullArm);
}

void armAngleUpdateCallback(const spear_msgs::ArmAngles::ConstPtr& msg){
    //TODO: Add markers for hand, so we can see the open/close state?

    setupTransforms(msg->joints[0].angle, msg->joints[1].angle, msg->joints[2].angle, msg->joints[4].angle);
    putMarkers();
}

void endEffectorUpdateCallback(const spear_msgs::arm_position::ConstPtr& msg){
    static tf2_ros::StaticTransformBroadcaster st_br;

    geometry_msgs::TransformStamped message;
    tf2::Quaternion quat;

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";
    message.child_frame_id = "target";
    message.transform.translation.x = (msg->x)*(BICEP_LENGTH+FOREARM_LENGTH+WRIST_LENGTH)/100.0;
    message.transform.translation.y = (msg->y)*(BICEP_LENGTH+FOREARM_LENGTH+WRIST_LENGTH)/100.0;
    message.transform.translation.z = (msg->z)*(BICEP_LENGTH+FOREARM_LENGTH+WRIST_LENGTH)/100.0;
    quat.setRPY(0, msg->flick, 0);
    message.transform.rotation.x = quat.x();
    message.transform.rotation.y = quat.y();
    message.transform.rotation.z = quat.z();
    message.transform.rotation.w = quat.w();
    
    st_br.sendTransform(message);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_visual");
    node = new ros::NodeHandle;
    listener = new tf2_ros::TransformListener(tfBuf);
    arm_angles_sub = node->subscribe("/arm/angles", 1000, armAngleUpdateCallback);
    arm_coords_sub = node->subscribe("/user_arm_controls", 1000, endEffectorUpdateCallback);
    arm_visuals_pub = node->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, true); //bool at end makes this a Latched Publisher
    setupTransforms(0, M_PI/4.0f, M_PI/6, M_PI/16);
    putMarkers();
    ros::spin();

    return 0;
}
