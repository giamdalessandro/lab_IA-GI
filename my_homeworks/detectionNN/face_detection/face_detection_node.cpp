#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>


std::string frontal_face_cascade_name = "/media/zascerta/DATA/Universalis/LabAIGI/homework/detectionNN/face_detection/haarcascade_frontalface_alt.xml";
std::string profile_face_cascade_name = "/media/zascerta/DATA/Universalis/LabAIGI/homework/detectionNN/face_detection/haarcascade_profileface.xml";
std::string eye_cascade_name = "/media/zascerta/DATA/Universalis/LabAIGI/homework/detectionNN/face_detection/haarcascade_eye.xml";
cv::CascadeClassifier frontal_face_cascade;
cv::CascadeClassifier profile_face_cascade;
cv::CascadeClassifier eye_cascade;


cv::RNG rng(12345);
cv::Mat best_face;

void detectAndDisplay(cv::Mat frame) {
  int found_best= 0;
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray; 
  cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);

  // Detect faces
  frontal_face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));
  //best_face= frame(faces[0]);
  
  for(size_t i = 0; i < faces.size(); i++) {
    cv::Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
    cv::ellipse(frame, center, cv::Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar(255, 0, 0), 1, 8, 0);

    if(frame(faces[i]).rows > best_face.rows) {
      best_face= frame(faces[i]);
      found_best= 1;
    }
    
    cv::Mat faceROI = frame_gray(faces[i]);
    std::vector<cv::Rect> eyes;

    // detect eyes, in each face
    eye_cascade.detectMultiScale(faceROI, eyes, 1.4, 4, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50)); 

    for(size_t j = 0; j < eyes.size(); j++) {
      cv::Point center(faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5);
      int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
      cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 1, 8, 0);
    }
  }

  // detect profile
  std::vector<cv::Rect> profile;
  profile_face_cascade.detectMultiScale(frame_gray, profile, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));    //faceROI

  for(size_t j = 0; j < profile.size(); j++) {
    std::cout << "[detection] detected profile of "<< j << std::endl;
    cv::Point center(profile[j].x + profile[j].width*0.5, profile[j].y + profile[j].height*0.5);
    int radius = cvRound((profile[j].width + profile[j].height)*0.25);
    cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 1, 6, 0);
    //cv::rectangle(frame, cv::Point(profile[j].x,profile[j].y), cv::Point(profile[j].x + profile[j].width, profile[j].y + profile[j].height), cv::Scalar(0, 255, 0), 1, 8, 0);
  }

  cv::imshow("Face detection -- Webcam", frame);
  if(found_best) cv::imshow("Best Face", best_face);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "face_detection");
  ros::NodeHandle nh;
  
  // Load the cascades
  if(!frontal_face_cascade.load(frontal_face_cascade_name) ||
     !profile_face_cascade.load(profile_face_cascade_name) ||
     !eye_cascade.load(eye_cascade_name)) {
    
    std::cerr << "[frontalface]: Error while loading HAAR cascades." << std::endl;
    return -1;
  }

  // Read the video stream from webcam
  cv::VideoCapture capture(0);
  cv::Mat frame;
  
  if(!capture.isOpened()){std::cerr << "Error opening video stream or file" << std::endl; return -1;}

  while(ros::ok()) {
    capture >> frame;
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
