#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Dense>

namespace utils{

  class Canvas{
  public:
    Canvas(const size_t& rows_, const size_t& cols_, const float& resolution_);
    ~Canvas();
    void resize(const size_t& rows_, const size_t& cols_);
    void colorPoint(const Eigen::Vector2f& point_, const cv::Vec3b& color=cv::Vec3b(0,0,0));
    inline void show(){
      cv::imshow("map",*_img);
      cv::waitKey(0.1);
    }
  protected:
    cv::Mat* _img;
    float _resolution;
  };

}
