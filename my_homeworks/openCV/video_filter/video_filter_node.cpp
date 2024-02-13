#include "ros/ros.h"
#include "video_filter.cpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "video_filter");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher filter_pub = it.advertise("video_frame", 1);
  image_transport::Publisher gauss_pub = it.advertise("gauss_smooth", 1);
  image_transport::Publisher sobel_pub = it.advertise("sobel_filter", 1);
  image_transport::Publisher canny_pub = it.advertise("canny_edges", 1);

  cv::VideoCapture cap(0);
  //cv::VideoCapture cap("/media/zascerta/DATA/CINEbriamo/Bojack Horseman/BoJack Horseman S01e01-12[Webrip 720p-H264-Ita Aac](TNT Village)/BoJack Horseman S01e01 - La storia di BoJack Horseman cap. 1 - [AiR.GrouP].mkv");
 
  if(!cap.isOpened()){
    std::cerr << "Error opening video stream or file" << std::endl;
    return -1;
  }

  ros::Rate loop_rate(10);
  while(ros::ok()) { 
    cv::Mat frame;
    cap >> frame;

    if (frame.empty()) break;   
    cv::Mat gmat= gaussianSmoothing(frame, gauss_pub);
    cv::Mat sobel= sobelFilter(frame, sobel_pub);
    cv::Mat cedge= cannyEdges(frame, canny_pub);
    //cv::imshow("frame video", frame);
    //cv::imshow("gauss video", gmat);
    //cv::imshow("cedge video", cedge);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    filter_pub.publish(msg);
    
    //cv::waitKey(10);
    loop_rate.sleep(); 
  }

  cap.release();

  // Closes all the frames
  cv::destroyAllWindows();
  
  ros::spin();
}
