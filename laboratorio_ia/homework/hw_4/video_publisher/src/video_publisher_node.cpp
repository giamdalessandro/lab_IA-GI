#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sstream>
#include<string.h>
using namespace std;
using namespace cv;
Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;


int main(int argc, char**argv){

    if(argc < 3){
        cerr<<"Usage .video_publisher_node <video_url> <type_filter>"<<endl;
        cerr<<"<type_filter> = gaussian, edge_detection"<<endl;
        return 1;
    }
    if(argv[1] == NULL) {
        cerr <<"Inserisci il percorso dell'imagine come parametro" <<endl;
        return 1;
    }
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    //cv::VideoCapture cap("chaplin.mp4");
    // Convert the passed as command line parameter index for the video device to an integer
/*     std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    // Check if it is indeed a number
    if(!(video_sourceCmd >> video_source)) return 1; */

    cv::VideoCapture cap(argv[1]);
    char* filter = argv[2];

    if(!cap.isOpened()){
        cerr <<"Error opening video stream or file" <<endl;
        return -1;
    }
    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
    cerr<<"width "<<frame_width<<endl;
    cerr<<"height "<<frame_height<<endl;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    cv::Mat gaussian_blurred_img;
    cv::Mat box_blurred_img;

    float gaussian_stddev = 1.0;
    ros::Rate loop_rate(5);
    while(ros::ok()){
        cap >> frame;

        if(strcmp(filter,"gaussian") == 0){
            //frame.convertTo(frame,cv::DataType<float>::type);
            //cv::normalize(frame, frame,0,1.0,cv::NORM_MINMAX, cv::DataType<float>::type);
            cv::blur(frame, frame, Size(5,5));
            //cv::boxFilter(frame,box_blurred_img, -1, cv::Size(3,3));
            cv::GaussianBlur( frame, gaussian_blurred_img, cv::Size(3,3), gaussian_stddev );

            // Compute the image derivatives along x and y using Sobel
            cv::Mat dx_img, dy_img;
            cv::Sobel(frame, dx_img, cv::DataType<float>::type, 1, 0, 3);
            cv::Sobel(frame, dy_img, cv::DataType<float>::type, 0, 1, 3);
            cv::Mat gradient_mag_img, abs_dx_img, abs_dy_img, binary_img;
            // Compute the gradient magnitude image
            abs_dx_img = cv::abs(dx_img);
            abs_dy_img = cv::abs(dy_img);
            gradient_mag_img = 0.5*(abs_dx_img + abs_dy_img);
            // Binarize the image
            cv::threshold ( gradient_mag_img, dst, 0.4,1.0,cv::THRESH_BINARY );    
        }else if(strcmp(filter,"edge_detection") == 0) {
            /*edge detection */
            src = frame;
            cvtColor( src, src_gray, CV_BGR2GRAY );
            blur( src_gray, detected_edges, Size(3,3) );
            /// Canny detector
            Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
            /// Using Canny's output as a mask, we display our result
            dst = Scalar::all(0);
            //applico alla matrice src la mask detected_edges e la copio su dst
            src.copyTo( dst, detected_edges);
        }else{
            cerr<<"<type_filter> not correct. Bad."<<endl;
            return 1;
        }
        if(!frame.empty()){
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
            //cerr<<"Published frame" <<endl;
            pub.publish(msg);
            cv::waitKey(1);    
        }

        ros::spinOnce();
        //loop_rate.sleep();
    }

    cap.release();
    
    return 0;
}
