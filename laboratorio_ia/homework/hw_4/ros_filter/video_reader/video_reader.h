#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

// #define VIDEO_TOPIC  "video_frame"
// #define GAUSS_TOPIC  "gauss_smooth"
// #define CANNY_TOPIC  "canny_edges"

class VideoReader{
public:
  VideoReader(ros::NodeHandle& nh);
  ~VideoReader();
  void gaussianCallback(const sensor_msgs::ImagePtr& msg);
  void cannyCallback(const sensor_msgs::ImagePtr& msg);
  void subscribeFiltered();
protected:
  std::string _gauss_topic;
  std::string _canny_topic;
  std::string _video_topic;
  ros::NodeHandle& _nh;
};
