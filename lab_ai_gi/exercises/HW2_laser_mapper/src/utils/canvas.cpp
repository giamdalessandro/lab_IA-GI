#include "canvas.h"

namespace utils{

  Canvas::Canvas(const size_t& rows_, const size_t& cols_, const float& resolution_){
    _img = new cv::Mat(rows_,cols_,CV_8UC3,cv::Scalar(255,255,255));
    _resolution = resolution_;
  }

  Canvas::~Canvas(){
    delete _img;
  }

  void Canvas::resize(const size_t& rows_, const size_t& cols_){
    _img->create(rows_,cols_,CV_8UC3);
  }

  void Canvas::colorPoint(const Eigen::Vector2f& point, const cv::Vec3b& color){

    Eigen::Vector2i indicies(point.x()/_resolution,point.y()/_resolution);

    if(indicies.x() < 0 || indicies.x() > _img->rows)
      return;
    if(indicies.y() < 0 || indicies.y() > _img->cols)
      return;

    _img->at<cv::Vec3b>(indicies.x(),indicies.y()) = color;

  }

}
