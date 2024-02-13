#include "mapOdom.h"
#include <math.h>

LaserMapper::LaserMapper(ros::NodeHandle& nh_) : _nh(nh_) {
  _listener = new tf::TransformListener;
  _broadcaster = new tf::TransformBroadcaster;
  _odom_frame_id = "odom";    // da verficare
  _map_topic = "/base_scan";     // da verificare
}

LaserMapper::~LaserMapper(){
  delete _listener;
}

void LaserMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_) {
  std::cerr << "Laser callback: " << std::endl;
  tf::StampedTransform transform;
}


void LaserMapper::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_) {
  std::cerr <<"Odom callback" << std::endl;
  std::cerr<< "\tPos =  (" << msg_->pose.pose.position.x << ", "<< msg_->pose.pose.position.y <<") " << std::endl;
  
  float x = msg_->pose.pose.position.x;
  float y = msg_->pose.pose.position.y;
}

void LaserMapper::publish() {
  while(nh.ok()){
    _broadcaster.sendTransform(
      tf::StampedTransform(
	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
	              ros::Time::now(), "base_link", "base_laser"));
    r.sleep();
  }
}

void LaserMapper::subscribe() {
  _map_sub = _nh.subscribe(_map_topic, 10, &LaserMapper::laserCallback, this);
  _odom_sub = _nh.subscribe(_odom_frame_id, 100, &LaserMapper::odomCallback, this);
}
