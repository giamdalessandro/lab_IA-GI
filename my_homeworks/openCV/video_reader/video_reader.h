#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <image_transport/image_transport.h>

class VideoReader{
public:
  VideoReader(ros::NodeHandle& nh, image_transport::ImageTransport& it);
  void gaussianCallback(const sensor_msgs::ImageConstPtr& msg);
  void cannyCallback(const sensor_msgs::ImageConstPtr& msg);
  void sobelCallback(const sensor_msgs::ImageConstPtr& msg);
  void subscribeFiltered();
  cv::VideoWriter gauss_video;
  cv::VideoWriter canny_video;
  cv::VideoWriter sobel_video;
  
protected:
  ros::NodeHandle& _nh;
  image_transport::ImageTransport& _it;
  std::string _gauss_topic;
  std::string _canny_topic;
  std::string _video_topic;
  std::string _sobel_topic;
  image_transport::Subscriber _gauss_sub;
  image_transport::Subscriber _canny_sub;
  image_transport::Subscriber _sobel_sub;
};
