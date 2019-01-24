#include <ros/ros.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image;
    image = imread( argv[1], 1);

    if (!image.data){
        printf("No image data \n");
        return -1;
    }
    Mat fullImageHSV;
    cvtColor(image, fullImageHSV, CV_BGR2HSV);

    Mat imgThresholded;
    inRange(fullImageHSV, Scalar(29, 86, 6), Scalar(64, 255, 255), imgThresholded);
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    GaussianBlur(imgThresholded, imgThresholded, Size(9, 9), 2, 2);

    std::vector<Vec3f> circles;
    HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.rows/8, 30, 30, 0, 0);
    for (size_t i = 0; i < circles.size(); i++){
        rectangle(image, Point(circles[i][0]-circles[i][2], circles[i][1]-circles[i][2]), Point(circles[i][0]+circles[i][2], circles[i][1]+circles[i][2]), Scalar(255, 0, 0));
        std::cout << "X: " << circles[i][0] << " Y: " << circles[i][1] << " Radius: " << circles[i][2] << "\n";
    }
    waitKey(0);
    return 0;
}
