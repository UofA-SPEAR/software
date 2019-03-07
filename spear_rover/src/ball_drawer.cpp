#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <spear_msgs/BallCoords.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

ros::NodeHandle* node;
ros::Subscriber image_sub;
ros::Subscriber coords_sub;
ros::Publisher image_pub;

spear_msgs::BallCoords current_coords;

void coords_callback(const spear_msgs::BallCoords::ConstPtr& msg) {
    // literally just copy the coords to a static set
    current_coords = *msg;
}

void image_in_callback(const sensor_msgs::ImageConstPtr& msg) {
    Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;


    // Handle images, draw coords over images
    for (unsigned int i = 0; i < current_coords.balls.size(); i++) {
        CvPoint point;
        point.x = current_coords.balls[i].x_pos;
        point.y = current_coords.balls[i].y_pos;

        circle(image,
                point,
                current_coords.balls[i].radius,
                Scalar(0, 0, 255),
                2
              );
    }

    current_coords.balls.clear();

    image_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ball_drawer");
    node = new ros::NodeHandle;

    std::string image_sub_param = ros::this_node::getName();
    std::string coords_param = ros::this_node::getName();
    std::string image_pub_param = ros::this_node::getName();
    image_sub_param.append("/image_sub_topic");
    coords_param.append("/coords_topic");
    image_pub_param.append("/image_pub_topic");

    std::string image_sub_topic;
    std::string coords_topic;
    std::string image_pub_topic;

    node->param<std::string>(image_sub_param, image_sub_topic,
            "/camera_1/image_raw");
    node->param<std::string>(coords_param, coords_topic, "/ball_coords");
    node->param<std::string>(image_pub_param, image_pub_topic,
            "/ball_drawer/image");

    image_sub = node->subscribe(image_sub_topic, 1, image_in_callback);
    coords_sub = node->subscribe(coords_topic, 1, coords_callback);
    image_pub = node->advertise<sensor_msgs::Image>(image_pub_topic, 1);

    ros::spin();

    return 0;

}
