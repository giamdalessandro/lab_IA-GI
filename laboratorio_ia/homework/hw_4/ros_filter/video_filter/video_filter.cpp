#include "video_filter.h"


cv::Mat gaussianSmoothing(cv::Mat frame, ros::Publisher& filter_pub) {
  if(frame.empty()) {
    std::cerr << "[GSMOOTH]: Error opening video frame" << std::endl;
  }
  frame.convertTo(frame, cv::DataType<float>::type);
  cv::normalize(frame, frame, 0, 1.0, cv::NORM_MINMAX, cv::DataType<float>::type);

  // Compute the image derivatives along x and y using Sobel
  cv::Mat dx_img, dy_img;
  cv::Sobel(frame, dx_img, cv::DataType<float>::type, 1, 0, 3);
  cv::Sobel(frame, dy_img, cv::DataType<float>::type, 0, 1, 3);
  
  // Compute the gradient magnitude image
  cv::Mat gradient_mag_img, abs_dx_img, abs_dy_img, binary_img;
  abs_dx_img= cv::abs(dx_img);
  abs_dy_img= cv::abs(dy_img);
  gradient_mag_img= 0.5*(abs_dx_img + abs_dy_img);
  
  // Binarize the image
  cv::threshold(gradient_mag_img, binary_img, 0.4, 1.0, cv::THRESH_BINARY);

  // pubblicare messagio immagine nel topic
  cv_bridge::CvImage ros_img;
  ros_img.image = gradient_mag_img;
  ros_img.encoding = sensor_msgs::image_encodings::BGR8; //"bgr8";

  sensor_msgs::ImagePtr msg = ros_img.toImageMsg();
  filter_pub.publish(msg);
  
  return gradient_mag_img;
}


cv::Mat cannyEdges(cv::Mat frame, ros::Publisher& filter_pub) {
  if(frame.empty()) {
    std::cerr << "[CEDGES]: Error opening video frame" << std::endl;
  }
  //TODO
  //modifica del frame
  cv::Mat gaussian_blurred_img, box_blurred_img;
  float gaussian_stddev = 1.0;
  
  frame.convertTo(frame, cv::DataType<float>::type);
  cv::normalize(frame, frame, 0, 1.0, cv::NORM_MINMAX, cv::DataType<float>::type);
  
  // Filter the image with a 3x3 box filter
  cv::boxFilter(frame, box_blurred_img, -1, cv::Size(3,3));
  // Filter the image with a gaussian filter with std dev = 1
  cv::GaussianBlur(frame, gaussian_blurred_img, cv::Size(0,0), gaussian_stddev);

  // pubblicare messagio immagine nel topic
  cv_bridge::CvImage ros_img;
  ros_img.image = gaussian_blurred_img;
  ros_img.encoding = sensor_msgs::image_encodings::BGR8; //"bgr8";

  sensor_msgs::ImagePtr msg = ros_img.toImageMsg();
  filter_pub.publish(msg);
  
  return gaussian_blurred_img;
}

