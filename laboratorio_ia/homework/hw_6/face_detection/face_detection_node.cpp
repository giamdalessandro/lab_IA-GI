#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <iostream>
#include <stdio.h>

//using namespace cv;


std::string frontal_face_cascade_name = "/media/zascerta/DATA/Universalis/LabAIGI/homework/detectionNN/face_detection/haarcascade_frontalface_alt.xml";
std::string profile_face_cascade_name = "/media/zascerta/DATA/Universalis/LabAIGI/homework/detectionNN/face_detection/haarcascade_profileface.xml";
std::string eye_cascade_name = "/media/zascerta/DATA/Universalis/LabAIGI/homework/detectionNN/face_detection/haarcascade_eye.xml";
cv::CascadeClassifier frontal_face_cascade;
cv::CascadeClassifier profile_face_cascade;
cv::CascadeClassifier eye_cascade;


std::string window_name = "Capture - Face detection";
cv::RNG rng(12345);

/** @function detectAndDisplay */
void detectAndDisplay(cv::Mat frame) {
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;

  cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);

  //-- Detect faces
  frontal_face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));

  for(size_t i = 0; i < faces.size(); i++) {
    cv::Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
    cv::ellipse(frame, center, cv::Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar(255, 0, 255), 2, 8, 0);

    cv::Mat faceROI = frame_gray(faces[i]);
    std::vector<cv::Rect> eyes;

    eye_cascade.detectMultiScale(faceROI, eyes, 1.4, 4, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50)); 

    for(size_t j = 0; j < eyes.size(); j++) {
      cv::Point center(faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5);
      int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
      cv::circle(frame, center, radius, cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    
    // //-- In each face, detect profile
    // std::vector<cv::Rect> profile;
    // profile_face_cascade.detectMultiScale(frame_gray, profile, 1.4, 4, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));    //faceROI

    // for(size_t j = 0; j < profile.size(); j++) {
    //   cv::Point center(profile[i].x + profile[j].x + profile[j].width*0.5, profile[i].y + profile[j].y + profile[j].height*0.5);
    //   int radius = cvRound((profile[j].width + profile[j].height)*0.25);
    //   cv::circle(frame, center, radius, cv::Scalar(255, 0, 0), 4, 8, 0);
    // }
  }
  //-- Show what you got
  cv::imshow(window_name, frame);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "face_detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  
  //-- 1. Load the cascades
  if(!frontal_face_cascade.load(frontal_face_cascade_name)) {
    std::cerr << "[frontalface]: Error while loading HAAR cascades." << std::endl;
    return -1;
  }
  // if(!profile_face_cascade.load(profile_face_cascade_name)) {
  //   std::cerr << "[profile]: Error while loading HAAR cascades." << std::endl;
  //   return -1;
  // }
  if(!eye_cascade.load(eye_cascade_name)) {
    std::cerr << "[profile]: Error while loading HAAR cascades." << std::endl;
    return -1;
  }
    
  cv::VideoCapture capture(0);
  cv::Mat frame;

  //-- 2. Read the video stream
  if(!capture.isOpened()){
    std::cerr << "Error opening video stream or file" << std::endl;
    return -1;
  }

  while(ros::ok()) {
    //frame = cvQueryFrame(capture);
    capture >> frame;
    //-- 3. Apply the classifier to the frame
    if(!frame.empty()) {
      detectAndDisplay(frame);
    } else {
      printf(" --(!) No captured frame -- Break!");
      break;
    }
	  
    int c = cv::waitKey(10);
    if((char)c == 'c') { break; }
  }

  capture.release();
  cv::destroyAllWindows();
  
  //ros::spin();
  return 0;
}
