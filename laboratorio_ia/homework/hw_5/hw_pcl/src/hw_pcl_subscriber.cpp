#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;


pcl::PointCloud< pcl::PointXYZRGB >::Ptr depth2PointCloud ( const cv::Mat &depth_img,
                                                            const cv::Mat &rgb_img,
                                                            const Eigen::Matrix3f &camera_matrix,
                                                            float min_depth = 0.01f, float max_depth = 10.0f )
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
  pc_ptr->points.resize ( depth_img.total() );
  Eigen::Matrix4f t_mat;
  t_mat.setIdentity();
  t_mat.block<3, 3> ( 0, 0 ) = camera_matrix.inverse();
  cv::Mat scaledDepthImg;
  depth_img.convertTo(scaledDepthImg,CV_32F,0.001);
  ROS_INFO("Creating point cloud from <%d, %d>",depth_img.rows, depth_img.cols);
  int i_pt = 0;
  for ( int y = 0; y < depth_img.rows; y++ )
  {
    const float *depth = scaledDepthImg.ptr<float> ( y );
    const uchar *bgr = rgb_img.ptr<uchar> ( y );
    uchar r, g, b;
    for ( int x = 0; x < depth_img.cols; x++, depth++, bgr+=3 )
    {
      float d = *depth;
      if (true)
      {
        Eigen::Vector4f point = t_mat*Eigen::Vector4f ( x*d,y*d,d,1.0 );

        b = bgr[0];
        g = bgr[1];
        r = bgr[2];

        uint32_t rgb = ( static_cast<uint32_t> ( r ) << 16 |
                         static_cast<uint32_t> ( g ) << 8 |
                         static_cast<uint32_t> ( b ) );

        pc_ptr->points[i_pt].x = point[0];
        pc_ptr->points[i_pt].y = point[1];
        pc_ptr->points[i_pt].z = point[2];

        pc_ptr->points[i_pt++].rgb = *reinterpret_cast<float*> ( &rgb );
      }
    }
  }

  pc_ptr->points.resize ( i_pt );
  pc_ptr->width = ( int ) pc_ptr->points.size();
  pc_ptr->height = 1;

  return pc_ptr;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr depth2PC ( const cv::Mat &depth_img,
                                                    const cv::Mat &image,
                                                    const Eigen::Matrix3f &camera_matrix,
                                                    float min_depth = 0.01f, float max_depth = 10.0f )
{
  cv::Mat scaledDepthImg;
  depth_img.convertTo(scaledDepthImg,CV_32F,0.001);
  Eigen::Matrix4f t_mat;
  t_mat.setIdentity();
  t_mat.block<3, 3> ( 0, 0 ) = camera_matrix.inverse();

  //per i punti errati
  const float bad_point = numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  cloud->is_dense = false;
  cloud->points.resize(image.rows*image.cols);
  for(size_t r = 0; r < image.rows; r++){
      for(size_t c =0; c < image.cols;c++){
          float d = scaledDepthImg.at<float>(r,c);
          float x = c, y = r;
          Eigen::Vector4f v = t_mat* Eigen::Vector4f(x*d,y*d,d,1.0);
          //vrappa v su point
          pcl::PointXYZRGBNormal point;
          point.x = v[0];
          point.y = v[1];
          point.z = v[2];
          // pack r/g/b into rgb
          uchar blue = image.at<cv::Vec3b>(r,c)[0];
          uchar green = image.at<cv::Vec3b>(r,c)[1];
          uchar red = image.at<cv::Vec3b>(r,c)[2];
          uint32_t rgb = (static_cast<uint32_t>(red) << 16 |static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
          point.rgb = *reinterpret_cast<float*>(&rgb);
          //set bad point 
          if(d == 0){
              point.x = point.y = point.z = bad_point;
          }else{
            cloud->points.push_back(point);
          }
      }
  }
  cloud->width = (uint32_t) image.rows;
  cloud->height = image.cols;

}


void callback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr&  depth_image){

    float fx = 512, fy = 512, cx = 320, cy = 240;
        
  Eigen::Matrix3f camera_matrix;
  camera_matrix<< fx,   0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f;
  
  cv::Mat rgb_img, depth_img;
  cout<<"Msg arrived: rgb "<<rgb_image->width<<"x"<<rgb_image->height
      <<" Depth: "<<depth_image->width<<"x"<<depth_image->height<<endl;
  try{  
    rgb_img = cv_bridge::toCvShare(rgb_image, "bgr8")->image;
    depth_img = cv_bridge::toCvShare(depth_image, "16UC1")->image;
    ROS_INFO("RGB_IMG <%d,%d>", rgb_img.rows, rgb_img.cols);
        // Wait for ESCAPE key
    //while( cv::waitKey() != 27 );
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr = depth2PointCloud( depth_img, rgb_img, camera_matrix );
    ROS_INFO("Converted PointCLoud");
    ROS_INFO("Cloud_ptr <%d,%d>", cloud_ptr->width, cloud_ptr->height);
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                                              3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce ( 1 );
    }
    
    return;

  }catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", depth_image->encoding.c_str());
  }

}


int main(int argc, char **argv){
    ros::init(argc, argv, "PCL_listener");
    ros::NodeHandle nh;
    //cv::namedWindow("view"); ///camera/depth_registered/hw_registered/image_rect_raw
    //cv::startWindowThread(); ///camera/depth_registered/image_raw
    ///camera/rgb/image_rect_color"
    if(argc < 3){
      cerr<<"Usage subscriber <depth_topic> <rgb_topic>";
      return -1;
    }
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[1],1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[2],1);   
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    cv::destroyWindow("view");
}
