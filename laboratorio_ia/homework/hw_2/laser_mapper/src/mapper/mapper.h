#pragma once
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "geometry_utils_fd.h"
#include "canvas.h"

class LaserMapper{
public:
  LaserMapper(ros::NodeHandle& nh_);
  ~LaserMapper();
  inline void setOdomFrameId(const std::string& odom_frame_id_){_odom_frame_id=odom_frame_id_;}
  inline void setLaserTopic(const std::string& laser_topic_){_laser_topic=laser_topic_;}
  inline void setCanvas(utils::Canvas& canvas_){_canvas=&canvas_;}
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_);
  void subscribe();
protected:
  std::string _odom_frame_id;
  std::string _laser_topic;
  utils::Canvas* _canvas;
  tf::TransformListener* _listener;
  ros::NodeHandle& _nh;
  ros::Subscriber _laser_sub;
  ros::Subscriber _odom_sub;
};
