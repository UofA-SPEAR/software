#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rover2/BallCoords.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

ros::NodeHandle* node;
ros::Subscriber image_sub;
ros::Publisher coords_pub;


/** @brief Callback function for recieving raw frames from camera
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

/** @brief Function to find the yellow circles within an image.
 * Assumes image is in BGR format.
 */
std::vector<Vec3f> getCircles(Mat image);


int main(int argc, char** argv) {
    ros::init(argc, argv, "ball_detector");
    node = new ros::NodeHandle;

    // TODO add ability to change subscribed topic
    image_sub = node->subscribe("/camera_1/raw", 1, imageCallback);
    coords_pub = node->advertise<rover2::BallCoords>("/ball_coords", 10);

    ros::spin();

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    getCircles(cv_bridge::toCvShare(msg, "bgr8")->image);

}


std::vector<Vec3f> getCircles(Mat image) {

    // Convert to HSV format
    Mat fullImageHSV;
    cvtColor(image, fullImageHSV, CV_BGR2HSV);

    // Image processing to improve image quality
    Mat imgThresholded;
    inRange(fullImageHSV, Scalar(29, 86, 6), Scalar(64, 255, 255), imgThresholded);
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    GaussianBlur(imgThresholded, imgThresholded, Size(9, 9), 2, 2);

    // Circles are held as [ X, Y, radius ]
    std::vector<Vec3f> circles;
    HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.rows/8, 30, 30, 0, 0);
    for (size_t i = 0; i < circles.size(); i++){
        std::cout << "X: " << circles[i][0] << " Y: " << circles[i][1] << " Radius: " << circles[i][2] << "\n";
    }

    return circles;

}
