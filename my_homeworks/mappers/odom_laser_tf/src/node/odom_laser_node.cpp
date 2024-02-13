#include "ros/ros.h"
#include "mapper.h"

using namespace std;
using namespace utils;

int main(int argc, char** argv){

  ros::init(argc, argv, "laser_mapper_node");

  ros::NodeHandle nh("~");

  std::string laser_topic;
  nh.param("laser_topic", laser_topic, std::string("/scan"));
  cerr << "[string] _laser_topic: " << laser_topic << endl;

  std::string odom_frame_id;
  nh.param("odom_frame_id", odom_frame_id, std::string("/odom"));
  cerr << "[string] _odom_frame_id: " << odom_frame_id << endl;

  int rows;
  nh.param("rows", rows, 500);
  cerr << "[int] _rows: " << rows << endl;

  int cols;
  nh.param("cols", cols, 500);
  cerr << "[int] _cols: " << cols << endl;

  double resolution;
  nh.param("resolution", resolution, 0.1);
  cerr << "[double] _resolution: " << resolution << endl;

  Canvas canvas(rows,cols,resolution);

  // for(int i= 0; i< 1000; i++) {
  //   Eigen::Vector2f p = Eigen::Vector2f(i,i);
  //   canvas.colorPoint(p);
  // }
  LaserMapper mapper(nh);

  mapper.setCanvas(canvas);
  mapper.setLaserTopic(laser_topic);
  mapper.setOdomFrameId(odom_frame_id);

  mapper.subscribe();

  ros::spin();

}
