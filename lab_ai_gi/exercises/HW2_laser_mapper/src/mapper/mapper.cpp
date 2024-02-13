#include "mapper.h"

LaserMapper::LaserMapper(ros::NodeHandle& nh_) : _nh(nh_){
  _listener = new tf::TransformListener;
  _odom_frame_id = "";
  _laser_topic = "";
}

LaserMapper::~LaserMapper(){
  delete _listener;
}

void LaserMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_){
  tf::StampedTransform transform;
  // Get laser transform in odom frame using the tf listener
  
  
  
  //

  Eigen::Isometry2f T = convertPose2D(transform);
  // Extract points from raw laser scan and paint them on canvas

  // for (something)
  //
  //  Eigen::Vector2f p

  //
  //  Eigen::Vector2f transformed_point = T * p
  //
  //
  _canvas->show();
}

void LaserMapper::subscribe(){

  _laser_sub = _nh.subscribe(_laser_topic,10,&LaserMapper::laserCallback,this);

}
