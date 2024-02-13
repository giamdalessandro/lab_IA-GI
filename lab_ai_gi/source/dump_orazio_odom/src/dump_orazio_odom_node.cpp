#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>

// Sugar
const char* banner[] = {
  "Write the trajectory followed by the robot on a text file.",
  "usage :",
  " rosrun dump_orazio_odom dump_orazio_odom_node [options]",
  " options:",
  " -o <string>         :   name of the output file ( default trajectory.txt )",
  " -odom_topic <string> :   the topic where the odometry is published ( default /odom )",
  " -h :   this help",
  0
};

void printBanner(const char** banner_) {
  const char** b = banner_;
  while(*b) {
    std::cerr << *b << std::endl;
    b++;
  }
}

// Output file stream
std::ofstream* output_file;

// Callback
void odomCallback(const nav_msgs::OdometryPtr& msg){
  geometry_msgs::Point& robot_position = msg->pose.pose.position;
  ROS_INFO("Odom msg received !");
  *output_file << robot_position.x << " " << robot_position.y << std::endl;
}

int main(int argc, char** argv){

  // Parse command line
  std::string odom_topic = "/odom";
  std::string filename = "trajectory.txt";
  int c=1;
  while(c<argc){
    if (! strcmp(argv[c], "-h")){
      printBanner(banner);
      return 0;
    }
    if (! strcmp(argv[c], "-odom_topic")){
      ++c;
      odom_topic = argv[c];
    } else if (! strcmp(argv[c],"-o")){
      ++c;
      filename = argv[c];
    } else {
      std::cerr << "[Error] : Option not recognized" << std::endl;
      return 0;
    }
    ++c;
  }

  // Create the subscriber
  output_file = new std::ofstream(filename);
  ros::init(argc, argv, "dump_orazio_odom_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe(odom_topic,1000,odomCallback);
  ros::spin();
  output_file->close();
  return 0;
}
