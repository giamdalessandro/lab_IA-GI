#include "ros/ros.h"
#include "video_filter.cpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "video_filter");

  ros::NodeHandle nh;

  // inizializza publisher
  ros::Publisher filter_pub = nh.advertise<sensor_msgs::Image>("video_frame", 100);
  //ros::Rate loop_rate(10);
  ros::Publisher gauss_pub = nh.advertise<sensor_msgs::Image>("gauss_smooth", 100);
  ros::Publisher canny_pub = nh.advertise<sensor_msgs::Image>("canny_edges", 100);

  cv::VideoCapture cap("/media/zascerta/DATA/CINEbriamo/Bojack Horseman/BoJack Horseman S01e01-12[Webrip 720p-H264-Ita Aac](TNT Village)/BoJack Horseman S01e01 - La storia di BoJack Horseman cap. 1 - [AiR.GrouP].mkv");
  
  if(!cap.isOpened()){
    std::cerr << "Error opening video stream or file" << std::endl;
    return -1;
  }

  while(ros::ok()) { 
    cv::Mat frame;
    cap >> frame;

    if (frame.empty()) break;
    // pubblicare nel topic 
    cv_bridge::CvImage ros_img;
    ros_img.image = frame;
    ros_img.encoding = sensor_msgs::image_encodings::BGR8; //"bgr8";

    sensor_msgs::ImagePtr msg = ros_img.toImageMsg();
    filter_pub.publish(msg);

    cv::Mat gmat= gaussianSmoothing(frame, gauss_pub);
    cv::Mat cedge= cannyEdges(frame, canny_pub);
    
    //cv::imshow("frame video", frame);
    //cv::imshow("gauss video", gmat);
    //cv::imshow("cedge video", cedge);

    //loop_rate.sleep(); 
    cv::waitKey(1);
  }

  cap.release();

  // Closes all the frames
  cv::destroyAllWindows();
  
  ros::spin();
}
