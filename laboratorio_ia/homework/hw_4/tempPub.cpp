#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char**argv){
    ros::init(argc, argv, "video_reader_node");
    ros::NodeHandle nh("~");

    string video_topic;
    ros::Publisher video_pub = n.advertise<std_msgs::String>("chatter",1000);

    VideoCapture cap("chaplin.mp4");

    if(!cap.isOpened()){
        cerr <<"Error opening video stream or file" <<endl;
        return -1;
    }

    cv::Mat frame;

    while(ros::ok()){
        cap >> frame;
        
        if(frame.empty()){
            break;
        }

        cvImage ros_img;
        ros_img->image = frame;
        ros_img->encodings = bgr8;

        sensorMsg::ImagePtr msg = ros_img->toImageMsg();
    }

    cap.release();
    
    destroyAllWindows();
    return 0;
}
