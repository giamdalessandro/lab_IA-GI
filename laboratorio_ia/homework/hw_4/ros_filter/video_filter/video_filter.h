#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>


cv::Mat gaussianSmoothing(cv::Mat frame, ros::Publisher& filter_pub);
cv::Mat cannyEdges(cv::Mat frame, ros::Publisher& filter_pub);
