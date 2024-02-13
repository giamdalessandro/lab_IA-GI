#pragma once
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>


class LaserMapper{
public:
  LaserMapper(ros::NodeHandle& nh_);
  ~LaserMapper();
  inline void setOdomFrameId(const std::string& odom_frame_id_){_odom_frame_id=odom_frame_id_;}
  inline void setMapTopic(const std::string& map_topic_){_map_topic=map_topic_;}
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_);
  void publish();
  void subscribe();
protected:
  std::string _odom_frame_id;
  std::string _map_topic;
  tf::TransformListener* _listener;
  tf::TransformBroadcaster* _broadcaster;
  ros::NodeHandle& _nh;
  ros::Subscriber _map_sub;
  ros::Subscriber _odom_sub;
};
