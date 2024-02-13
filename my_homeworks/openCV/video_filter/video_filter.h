#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <image_transport/image_transport.h>


cv::Mat gaussianSmoothing(cv::Mat frame, image_transport::Publisher& pub);
cv::Mat sobelFilter(cv::Mat frame, image_transport::Publisher& pub);
cv::Mat cannyEdges(cv::Mat frame, image_transport::Publisher& pub);
