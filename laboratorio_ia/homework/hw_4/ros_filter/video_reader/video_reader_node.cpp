#include "ros/ros.h"
#include "video_reader.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "video_reader");

  ros::NodeHandle nh("~");

  VideoReader reader(nh);
  
  reader.subscribeFiltered();
  
  //cv::destroyAllWindows();
  ros::spin();
}
