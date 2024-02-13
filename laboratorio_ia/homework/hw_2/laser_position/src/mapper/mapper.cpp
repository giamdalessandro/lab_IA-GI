#include "mapper.h"
#include<math.h>
LaserMapperPosition::LaserMapperPosition(ros::NodeHandle& nh_) : _nh(nh_){
  _listener = new tf::TransformListener;
  _odom_frame_id = "";
  _laser_topic = "/scan";
}

LaserMapperPosition::~LaserMapperPosition(){
  delete _listener;
}

void LaserMapperPosition::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_){
  std::cerr << "Let there be callback..." << std::endl;
  tf::StampedTransform transform;
  // Get laser transform in odom frame using the tf listener
  //Time time = msg_->header.stamp 
  ros::Time time = ros::Time::now();

  // i frame_id vanno controllati
  if(_listener->canTransform(_odom_frame_id, msg_->header.frame_id, msg_->header.stamp)) {
    _listener->lookupTransform(_odom_frame_id, msg_->header.frame_id, msg_->header.stamp, transform);
    
  } else {
    std::cerr << "Ooops, I died" << std::endl;
  }
  
  Eigen::Isometry2f T = convertPose2D(transform);
  // Extract points from raw laser scan and paint them on canvas
  int n = msg_->ranges.size();


  Eigen::Vector2f p;
  for(int i= 0; i<msg_->ranges.size(); i++) {
    Eigen::Vector2f p;
    
    float angle= msg_->angle_min + i*msg_->angle_increment;
    
    float x= msg_->ranges[i]*cos(angle);
    float y= msg_->ranges[i]*sin(angle);
    
    p= Eigen::Vector2f(x,y);

    //std::cerr << "Printing on canvas " << i << "th point" << std::endl;
    Eigen::Vector2f transformed_point= T * p;
    _canvas->colorPoint(transformed_point, cv::Vec3b(60,20,220));  //crimson red, (b,g,r) non (r,g,b)
  }

  std::cerr << "...and there was callback" << std::endl;
  _canvas->show();
}

void LaserMapperPosition::subscribe(){

  _laser_sub = _nh.subscribe(_laser_topic,10,&LaserMapperPosition::laserCallback,this);

}
