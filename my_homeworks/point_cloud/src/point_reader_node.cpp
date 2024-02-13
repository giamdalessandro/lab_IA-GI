#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#define DEBUG 1


pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildPointCloud(const cv::Mat& depth_img_rect_reg, const cv::Mat& rgb_img_rect) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  //cloudPtr->width= rgb_img_rect.rows;
  //cloudPtr->height= rgb_img_rect.cols;
  cloudPtr->is_dense= false;
  if(DEBUG) std::cout <<"[DEBUG]: reading depth and rgb images"<< std::endl;

  // visualize settings
  float fx = 512, fy = 512, cx = 320, cy = 240;
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix<<fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f;
  Eigen::Matrix4f t_mat;
  t_mat.setIdentity();
  t_mat.block<3,3>(0,0) = cameraMatrix.inverse();        

  cv::Mat scaled_depth_img;
  depth_img_rect_reg.convertTo(scaled_depth_img, CV_32F, 0.001);

  for(int r= 0; r< rgb_img_rect.rows; r++) {
    for(int c= 0; c< rgb_img_rect.cols; c++) {
      float d= scaled_depth_img.at<float>(r,c);
      Eigen::Vector4f v= t_mat* Eigen::Vector4f(c*d,r*d,d,1.0);
      const cv::Vec3b& clr= rgb_img_rect.at<cv::Vec3b>(r,c);

      pcl::PointXYZRGB pt;
      if(d == 0) {
	pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
      } else {
	pt.z = v[2];  
	pt.y = -v[1];  
	pt.x = -v[0]; 
	// colors
	uchar blue= clr[0];
	uchar green= clr[1];
	uchar red= clr[2];
	uint32_t rgb = (static_cast<uint32_t>(red) << 16|static_cast<uint32_t>(green) << 8|static_cast<uint32_t>(blue));
	pt.rgb = *reinterpret_cast<float*>(&rgb);
      }
      cloudPtr->points.push_back(pt);
    }
  }

  return cloudPtr;
}
    
void callback(const sensor_msgs::ImageConstPtr& depth_img_rect_reg, const sensor_msgs::ImageConstPtr& rgb_img_rect) {
  cv::Mat depth_img, rgb_img;
  try {
    cv_bridge::CvImageConstPtr dp_ptr= cv_bridge::toCvShare(depth_img_rect_reg, "16UC1");
    cv_bridge::CvImageConstPtr im_ptr= cv_bridge::toCvShare(rgb_img_rect, sensor_msgs::image_encodings::BGR8);
    depth_img= dp_ptr->image;
    rgb_img= im_ptr->image;
  
  } catch (cv_bridge::Exception& e){
    ROS_ERROR("[ERROR]: Could not convert from '%s' to '16UC1'.", depth_img_rect_reg->encoding.c_str());
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr readCloud= buildPointCloud(depth_img, rgb_img);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Bag Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(readCloud);

  if(DEBUG) std::cout <<  "[DEBUG]: visualizing point cloud" << std::endl;
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(readCloud, rgb, "bag cloud");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "bag cloud");
  viewer->initCameraParameters();
  viewer->addCoordinateSystem(0.5);
    
  while(!viewer->wasStopped()) 
    viewer->spinOnce(10);
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_reader");
  ros::NodeHandle nh;
  
  std::string depth_topic= "/camera/depth_registered/hw_registered/image_rect_raw";
  std::string rgb_topic= "/camera/rgb/image_rect_color";

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), depth_sub, rgb_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  cv::destroyAllWindows();

  return 0;
}
