#include "ros/ros.h"
#include "mapOdom.h"

using namespace std;

int main(int argc, char** argv){

  ros::init(argc, argv, "mo_tf_publisher");
  ros::NodeHandle nh("~");

  ros::Rate r(100);
 
  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("/map"));
  cerr << "[string] _map_topic: " << map_topic << endl;

  std::string odom_frame_id;
  nh.param("odom_frame_id", odom_frame_id, std::string("/odom"));
  cerr << "[string] _odom_frame_id: " << odom_frame_id << endl;
  
  LaserMapper mapper(nh);
  mapper.setLaserTopic(laser_topic);
  mapper.setOdomFrameId(odom_frame_id);
  mapper.subscribe();

  ros::spin();

}
