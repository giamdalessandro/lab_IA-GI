#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;


class Listener{
public:
    cv::VideoWriter video;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        try{
            cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
            video.write(cv_bridge::toCvShare(msg, "bgr8")->image);
            cv::waitKey(2);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
   // Default resolution of the frame is obtained.The default resolution is system dependent. 
  int frame_width =1280;// cap.get(CV_CAP_PROP_FRAME_WIDTH); 
  int frame_height = 720; //cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
  
  cv::VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height));
    Listener listener;
    listener.video = video;
  image_transport::Subscriber sub = it.subscribe("video_publisher/camera/image", 1, &Listener::imageCallback, &listener);
  ros::spin();

  video.release();
  cv::destroyWindow("view");
}
