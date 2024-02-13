#include "video_reader.h"

VideoReader::VideoReader(ros::NodeHandle& nh, image_transport::ImageTransport& it):
  _nh(nh),
  _it(it) {
  _gauss_topic = "/gauss_smooth";    
  _canny_topic = "/canny_edges";
  _video_topic = "/video_frame";
  _sobel_topic = "/sobel_filter";
}

void VideoReader::gaussianCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::cerr << "[READER]: gauss image read" << std::endl;

  try {
    cv_bridge::CvImageConstPtr cv_ptr= cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("Gauss Window", cv_ptr->image);
    //gauss_video.write(cv_ptr->image);
    cv::waitKey(20);
  
  } catch (cv_bridge::Exception& e){
    ROS_ERROR("[READER]: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  return;
}


void VideoReader::sobelCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::cerr << "[READER]: sobel image read" << std::endl;

  try {
    cv_bridge::CvImageConstPtr cv_ptr= cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("Sobel Window", cv_ptr->image);
    //sobel_video.write(cv_ptr->image);
    cv::waitKey(15);
  
  } catch (cv_bridge::Exception& e){
    ROS_ERROR("[READER]: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  return;
}


void VideoReader::cannyCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::cerr << "[READER]: canny image read" << std::endl;

  try {
    cv_bridge::CvImageConstPtr cv_ptr= cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("Canny Window", cv_ptr->image);
    //canny_video.write(cv_ptr->image);
    cv::waitKey(10);
    
  } catch (cv_bridge::Exception& e){
    ROS_ERROR("[READER]: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  return;
}

void VideoReader::subscribeFiltered() {
  std::cerr << "[READER]: Subsrcibing to topics" << std::endl;
  _gauss_sub= _it.subscribe(_gauss_topic, 100, &VideoReader::gaussianCallback, this);
  _canny_sub= _it.subscribe(_canny_topic, 100, &VideoReader::cannyCallback, this);
  _sobel_sub= _it.subscribe(_sobel_topic, 100, &VideoReader::sobelCallback, this);
}
