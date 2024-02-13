#include "video_reader.h"

VideoReader::VideoReader(ros::NodeHandle& nh) : _nh(nh) {
  _gauss_topic = "/gauss_smooth";    
  _canny_topic = "/canny_edges";
  _video_topic = "/video_frame";
}

VideoReader::~VideoReader(){
  cv::destroyAllWindows();
  //delete;
}

void VideoReader::gaussianCallback(const sensor_msgs::ImagePtr& msg) {
  //sensor_msgs::ImageConstPtr& msg;
  
  cv_bridge::CvImagePtr cv_ptr= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::imshow("Gauss Window", cv_ptr->image);
  cv::waitKey(10);

  return;
}

void VideoReader::cannyCallback(const sensor_msgs::ImagePtr& msg) {
  //sensor_msgs::ImageConstPtr& msg;
  
  cv_bridge::CvImagePtr cv_ptr= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::imshow("Canny Window", cv_ptr->image);
  cv::waitKey(10);
  
  return;
}

void VideoReader::subscribeFiltered() {
  _nh.subscribe(_gauss_topic, 100, &VideoReader::gaussianCallback, this);
  _nh.subscribe(_canny_topic, 100, &VideoReader::cannyCallback, this);
}
