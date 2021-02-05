#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <spear_msgs/BallCoords.h>
#include <stdio.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;

ros::NodeHandle* node;
ros::Subscriber image_sub;
ros::Publisher coords_pub;

/** @brief Callback function for recieving raw frames from camera
 */
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

/** @brief Function to find the yellow circles within an image.
 * Assumes image is in BGR format.
 */
std::vector<Vec3f> getCircles(Mat image);

int main(int argc, char** argv) {
  ros::init(argc, argv, "ball_detector");
  node = new ros::NodeHandle;

  std::string image_param = ros::this_node::getName();
  image_param.append("/image_topic");

  std::string output_param = ros::this_node::getName();
  output_param.append("/output_topic");

  std::string image_topic;
  std::string output_topic;

  node->param<std::string>(image_param, image_topic, "/camera_1/image_raw");
  node->param<std::string>(output_param, output_topic, "/ball_coords");

  // TODO add ability to change subscribed topic
  image_sub = node->subscribe(image_topic, 1, imageCallback);
  coords_pub = node->advertise<spear_msgs::BallCoords>(output_topic, 10);

  ros::spin();

  return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Find x,y coords and radius of all circles in image
  std::vector<Vec3f> circles;
  circles = getCircles(cv_bridge::toCvShare(msg, "bgr8")->image);

  if (circles.size() > 0) {
    // Generate message
    spear_msgs::BallCoords outMsg;
    for (unsigned int i = 0; i < circles.size(); i++) {
      spear_msgs::BallCoord coord;
      coord.number = i;
      coord.x_pos = circles[i][0];
      coord.y_pos = circles[i][1];
      coord.radius = circles[i][2];

      outMsg.balls.push_back(coord);

      ROS_INFO("Ball found! (X, Y, Radius) -> (%.2f, %.2f, %.2f)",
               circles[i][0], circles[i][1], circles[i][2]);
    }

    coords_pub.publish(outMsg);
  }
}

// TODO Fix issue with detection when there is a saturation/value gradient
std::vector<Vec3f> getCircles(Mat image) {
  // Convert to HSV format
  Mat fullImageHSV;
  cvtColor(image, fullImageHSV, CV_BGR2HSV);

  // Image processing to improve image quality
  Mat imgThresholded;
  inRange(fullImageHSV, Scalar(29, 86, 100), Scalar(64, 255, 255),
          imgThresholded);
  erode(imgThresholded, imgThresholded,
        getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  dilate(imgThresholded, imgThresholded,
         getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  dilate(imgThresholded, imgThresholded,
         getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  erode(imgThresholded, imgThresholded,
        getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  GaussianBlur(imgThresholded, imgThresholded, Size(9, 9), 2, 2);

  // Circles are held as [ X, Y, radius ]
  std::vector<Vec3f> circles;
  HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1,
               imgThresholded.rows / 8, 30, 30, 0, 0);

  return circles;
}
