#include "video_reader.cpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "video_reader");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  //cv::startWindowThread();

  int frame_width= 1280;   //cap.get(CV_CAP_PROP_FRAME_WIDTH); 
  int frame_height= 720;   //cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
  
  cv::VideoWriter gau_video("outgau.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(frame_width,frame_height));
  cv::VideoWriter can_video("outcan.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(frame_width,frame_height));
  cv::VideoWriter sob_video("outsob.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(frame_width,frame_height));
  
  VideoReader reader(nh, it);
  reader.gauss_video= gau_video;
  reader.canny_video= can_video;
  reader.sobel_video= sob_video;
  reader.subscribeFiltered();
  
  ros::spin();
}

