#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//sensor_msgs
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//pcl libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
//std
#include <iostream>
#include <string>

#define DEBUG 1

#define NUM_IMAGES  180
#define IMG_WIDTH   640   //dim image -> 640x480
#define IMG_HEIGHT  480

void passThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cut_cloud){
    pcl::PassThrough<pcl::PointXYZRGB> pass_through;
    pass_through.setInputCloud(in_cloud);
    pass_through.setFilterLimits(0.0, 0.85);
    pass_through.setFilterFieldName("z");
    pass_through.filter(*cut_cloud);
}

void voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr subsamp_cloud){
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(in_cloud);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01);
    voxel_grid.filter(*subsamp_cloud);
}

pcl::PointCloud<pcl::Normal>::Ptr addNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud) {
  if(DEBUG) std::cout << "[DEBUG]: adding points normals..." << std::endl;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  
  ne.setInputCloud(in_cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(5);  //ne.setRadiusSearch(0.03);
  ne.compute(*normals);
  
  return normals;
}

// legge da file .pcd e appilica ICP
pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignAll() {
  std::string abs_path = "/media/zascerta/DATA/Universalis/LabAIGI/homework/point_cloud/dataset/output_cloud/";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::string path = abs_path + "cloud_001.pcd";
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud_in) == -1){
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    exit(1);
  }
  if(!cloud_in->is_dense){
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in, indices);
  }
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
    
  Eigen::Matrix4f globalTransform, previousGlobalTransform, alignmentTransform;
  globalTransform.setIdentity();

  for(int i= 2; i< 4; i++){
    std::string path = abs_path + "cloud_00" + std::to_string(i) + ".pcd";
    cloud_target = cloud_in;
    previousGlobalTransform = globalTransform;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud_in) == -1){
      PCL_ERROR("Couldn't read file test_pcd.pcd \n");
      exit;
    }
    if(!cloud_in->is_dense){
      cloud_in->is_dense = false;
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in, indices);
    }
    std::cout <<"Converging "<<i<<" cloud"<< std::endl;
    gicp.setInputSource(cloud_in);
    gicp.setInputTarget(cloud_target);
    gicp.align(*final);
    std::cout <<"Cloud "<<i<<" has converged:"<< gicp.hasConverged() << " score:" << gicp.getFitnessScore() <<std::endl;
    std::cout << gicp.getFinalTransformation() << std::endl;
    alignmentTransform = gicp.getFinalTransformation();
    globalTransform = previousGlobalTransform * alignmentTransform;
  }

  std::string save_path= abs_path + "myFinalCloud.pcd";
  pcl::io::savePCDFileASCII(save_path,*final);
  cout<<"Saved GICP cloud"<<endl;
  return final;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "convert_pointcloud_to_image");

  std::string data_path= "/media/zascerta/DATA/Universalis/LabAIGI/homework/point_cloud/dataset/rgbd-scenes/kitchen_small/kitchen_small_1/kitchen_small_1_";
  std::string image_path, depth_path;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  
  for(int i=1; i<= NUM_IMAGES; i++) {
    cv::Mat image, input_depth, scaled_depth_img;
    image_path= data_path + std::to_string(i) + ".png";    
    image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);          // read the image file
    if(!image.data) {std::cout <<  "[ERROR]: Could not open or find the image" << std::endl; return -1;}

    depth_path= data_path + std::to_string(i) + "_depth.png";
    input_depth = cv::imread(depth_path, cv::IMREAD_ANYDEPTH);    // read the depth file
    if(!input_depth.data) {std::cout <<  "[ERROR]: Could not open or find the image" << std::endl; return -1;}
    
    input_depth.convertTo(scaled_depth_img, CV_32F, 0.001);

    // associate point cloud with colors and surface normals
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudPtr->width= IMG_WIDTH;
    cloudPtr->height= IMG_HEIGHT;
    cloudPtr->is_dense= false;
    //cloudPtr->points.resize(cloudPtr->width*cloudPtr->height);
    if(DEBUG) std::cout <<  "[DEBUG]: accamacca" << std::endl;

    float fx = 512, fy = 512, cx = 320, cy = 240;
    Eigen::Matrix3f cameraMatrix;
    cameraMatrix<<fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f;
    //as re-projection matrix
    Eigen::Matrix4f t_mat;
    t_mat.setIdentity();
    t_mat.block<3,3>(0,0) = cameraMatrix.inverse();        

    for(int r= 0; r< image.rows; r++) {
      for(int c= 0; c< image.cols; c++) {
	pcl::PointXYZRGB pt;
        float d = scaled_depth_img.at<float>(r,c);
	const cv::Vec3b& clr= image.at<cv::Vec3b>(r,c);
	Eigen::Vector4f v = t_mat* Eigen::Vector4f(c*d,r*d,d,1.0);
	//if(DEBUG) std::cout <<  "[DEBUG]: uelalla row: " << r << " col: " << c << " depth: " << d << std::endl;
	
    	if(d == 0) {
	  pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
	} else {
	  pt.z = v[2];  //d;
	  pt.y = -v[1];  //pt.z*(float(r)-cy)/fy;
	  pt.x = -v[0];  //pt.z*(float(c)-cx)/fx;
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
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudPtr);

    pcl::PointCloud<pcl::Normal>::Ptr norm_cloud= addNormal(cloudPtr); 
    if(DEBUG) std::cout <<  "[DEBUG]: added normals ..." << std::endl;

    // visualizzazione nuvola
    if(DEBUG) std::cout <<  "[DEBUG]: visualizzazione" << std::endl;
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudPtr, rgb, "Input cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloudPtr, norm_cloud, 50, 0.05, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Input cloud");
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.5);
    
    viewer->spin();
    viewer->resetCamera();
    viewer->removePointCloud("normals");
    viewer->removePointCloud("Input cloud");
    
    // save the point cluod on .pcd files 
    std::string save_path= "/media/zascerta/DATA/Universalis/LabAIGI/homework/point_cloud/dataset/output_cloud/cloud_00" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFileASCII(save_path, *cloudPtr);
    if(DEBUG) std::cerr << "Saved " << cloudPtr->points.size() << " data points to cloud_00" << i << ".pcd." << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // alignedCloud = alignAll();

    std::string abs_path = "/media/zascerta/DATA/Universalis/LabAIGI/homework/point_cloud/dataset/output_cloud/";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (abs_path + "myFinalCloud.pcd", *alignedCloud) == -1){
      PCL_ERROR("Couldn't read file myFinalCloud.pcd \n");
      exit;
    }
    
    voxelize(alignedCloud,alignedCloud);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    passThrough(alignedCloud, cutCloud);
    
    //visualize alignedCloud
    // if(DEBUG) std::cout <<  "[DEBUG]: visualizzazione alignedCloud" << std::endl;
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZRGB>(cutCloud, rgb, "Cut cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cut cloud");
    // viewer->initCameraParameters();
    // viewer->addCoordinateSystem(0.5);
    
    // viewer->spin();
    // viewer->resetCamera();
    // viewer->removePointCloud("Cut cloud");
  }
  
  ros::spin();      //where she stops nobody knows
  return 0;
}

