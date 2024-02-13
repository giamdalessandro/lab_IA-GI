#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace std;
using namespace cv;

#define NUM_IMAGES 3
#define RANDMAX 100
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr alignAll();
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr);
void voxelize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr );
void visualize(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud);

int main(int argc, char **argv){

    if(argc < 1){
        cerr<<"Usage .pcl_node <img> <img_depth>"<<endl;
        return 1;
    }
    ros::init(argc, argv, "pcl_generator");
    ros::NodeHandle nh;
    Mat image, depth, scaledDepthImg;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    for(int i = 1; i < NUM_IMAGES; i++){
        const char *path = "/home/frank/university/laboratorio_ia/homework/hw_5/rgbd-scenes/table_small/table_small_1/table_small_1_%d.png";
        cout<<"Opening "<<format(path, i)<<endl;
        image = imread(format(path,i),CV_LOAD_IMAGE_COLOR);
        if(!image.data || image.empty()){
            cout<<"could not open "<<format(path, i)<<endl;
            return -1;
        }
        //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
        //imshow( "Display window", image );
        cout<<"Opened "<<format(path, i)<<endl;
        const char* img_depth = "/home/frank/university/laboratorio_ia/homework/hw_5/rgbd-scenes/table_small/table_small_1/table_small_1_%d_depth.png";
        depth = imread(format(img_depth,i), IMREAD_ANYDEPTH);
        if(! depth.data || depth.empty()){
            cout <<  "Could not open or find the image depth" << std::endl ;
            return -1;
        }
        // Show our image inside it.
        //imshow( "Display window", depth );                   
        depth.convertTo(scaledDepthImg,CV_32F,0.001);
        // as camera matrix, use following default matrix
        float fx = 512, fy = 512, cx = 320, cy = 240;
        Eigen::Matrix3f cameraMatrix;
        cameraMatrix<<fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f;
        
        //as re-projection matrix
        Eigen::Matrix4f t_mat;
        t_mat.setIdentity();
        t_mat.block<3,3>(0,0) = cameraMatrix.inverse();
        
        //per i punti errati
        const float bad_point = numeric_limits<float>::quiet_NaN();

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        cloud->is_dense = false;

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
                }
                cloud->points.push_back(point);
            }
        }
        cloud->width = (uint32_t) image.rows;
        cloud->height = image.cols;

        pcl::io::savePCDFileASCII(format("output/myCloud_%d.pcd",i),*cloud);
        cout<<"saved "<<format("output/myCloud_%d.pcd",i)<<endl<<endl;
        //visualize(cloud);
    }    
    //COMMENT 2 SECOND LINE AND DECOMMENT 3 LINE to avoid Gicp Algorithm
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr alignedCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    alignedCloud = alignAll();
    //TESTING 
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr alignedCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("output/myFinalCloud.pcd", *alignedCloud) == -1){
        PCL_ERROR("Couldn't read file myFinalCloud.pcd \n");
        exit;
    }
    voxelize(alignedCloud,alignedCloud);
    visualize(alignedCloud);
}


pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr alignAll(){
    const char * path = "output/myCloud_%d.pcd";
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (format(path, 1), *cloud_in) == -1){
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        exit;
    }
    if (!cloud_in->is_dense){
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in, indices);
    }
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> gicp;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    Eigen::Matrix4f globalTransform, previousGlobalTransform, alignmentTransform;
    globalTransform.setIdentity();

    for(int i = 2; i < NUM_IMAGES; i++){
        cloud_target = cloud_in;
        previousGlobalTransform = globalTransform;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (format(path, i), *cloud_in) == -1){
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            exit;
        }
        if (!cloud_in->is_dense){
            cloud_in->is_dense = false;
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in, indices);
        }
        cout<<"Converging "<<i<<" cloud"<<endl;
        gicp.setInputSource(cloud_in);
        gicp.setInputTarget(cloud_target);
        gicp.align(*final);
        cout<<"Cloud "<<i<<" has converged:"<< gicp.hasConverged() << " score:" << gicp.getFitnessScore() <<endl;
        cout<< gicp.getFinalTransformation() << endl;
        alignmentTransform = gicp.getFinalTransformation();
        globalTransform = previousGlobalTransform * alignmentTransform;
    }

    pcl::io::savePCDFileASCII("output/myFinalCloud.pcd",*final);
    cout<<"Saved GICP cloud"<<endl;
    return final;
}
        

Vec3f normalize(Mat depth, float x, float y){
    //fatta per un solo punto x,y

    Mat normals(depth.size(), CV_32FC3);

    float dzdx = (depth.at<float>(x+1, y) - depth.at<float>(x-1, y)) / 2.0;
    float dzdy = (depth.at<float>(x, y+1) - depth.at<float>(x, y-1)) / 2.0;

    Vec3f d(-dzdx, -dzdy, 1.0f);
    Vec3f n = normalize(d);
    //normals.at<Vec3f>(x, y) = n; Matrice normale, usata se fatta nel for
    return n;
}

void passThrough(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cut_cloud){
    pcl::PassThrough<pcl::PointXYZRGBNormal> pass_through;
    pass_through.setInputCloud(in_cloud);
    pass_through.setFilterLimits(0.0, 0.5);
    pass_through.setFilterFieldName("z");
    pass_through.filter(*cut_cloud);
}
/*
    Downsampling
*/
void voxelize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subsamp_cloud){
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_grid;
    voxel_grid.setInputCloud(in_cloud);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01);
    voxel_grid.filter(*subsamp_cloud);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void visualize(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(cloud);

    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}