/*
 *          png2pcd_batch   -   simple command line utility to convert depth and rgb frames
 *          from png format to PCL pointcloud.
 * */

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> // imread

#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace pcl;
using namespace std;
#define NUM 3
/*
 * represent camera intrinsics params
 * scale factor used for convert depth values from unsigned short to float
 * for kinect-based dataset - usually 0.001
 *
 * */ 

typedef struct Intr{
    int width;
    int height;
    float fx;
    float fy;
    float cx;
    float cy;
    float scale_factor;
} Intr;

/*  Predefined data for freiburg dataset 3 
 *  */
const Intr DEFAULT_CAM_PARAMS   =   {

                                    640,
                                    480,
                                    535.4f,
                                    539.2f,
                                    320.1f,
                                    247.6f,
                                    ( 1.f / 5000.f )
                                };

const string DEFAULT_CFG_FILE   =   "cam_params.cfg";

void usage(){
    cout    <<  "Usage: "   << endl
            <<  "1 case: png2pcd_batch association_file" << endl
            <<  "\t\t   where association_file is a text file which should be parsed by USER implemented rule" << endl
            <<  "\t\t   for details and example of implementation check \"parse_freiburg\" function in source" << endl
            <<  "2 case: png2pcd_batch folder_with_rgb folder_with_depth" << endl
            <<  "\t\t   1st folder should contain png files with RGB data () " << endl
            <<  "\t\t   2nd folder should contain png files with DEPTH data () " << endl;
    exit(0);
}

bool load_camera_intrinsics ( const string & cam_param_file_name, Intr & param ){
    fstream cam_param; 
    cam_param.open ( cam_param_file_name.c_str(), std::ios::in );
    if ( !cam_param.is_open() ) {
        cerr << "NOTE:  If you want to use custom camera params - create file named "<< endl
             << "\t\t"<< DEFAULT_CFG_FILE << "and add there values of width height fx fy cx cy scale_factor" << endl
             << "\t\t"<< " for your dataset. Whitespace as separator." << endl
             << "\t\t"<< " Every depth measurment is multiplied by scale_factor." << endl
             << "NOTE: Load default values - instrinsics for freiburg dataset!" << endl;
        param = DEFAULT_CAM_PARAMS;
        return false;
    }
    else
        cam_param >> param.width >> param.height >> param.fx >> param.fy >> param.cx >> param.cy >> param.scale_factor;

    return true;
}


template<class T>void set_pixel ( T & pcl_pixel, cv::Mat & src, int x, int y, Intr& cam_params ){
    cerr << "set_pixel: Error - do not have proper specification for type: " << typeid(T).name() << endl;
    throw;
}

template <>void set_pixel ( RGB & pcl_color_pixel, cv::Mat & src, int x, int y, Intr& cam_params ){
    uint32_t rgb;
    cv::Vec3b cur_rgb = src.at<cv::Vec3b>(y,x);// b,g,r
    rgb =   ( static_cast<int> ( cur_rgb [ 2 ] ) ) << 16 |
            ( static_cast<int> ( cur_rgb [ 1 ] ) ) << 8 |
            ( static_cast<int> ( cur_rgb [ 0 ] ) );
    
    pcl_color_pixel.rgba = static_cast < uint32_t > ( rgb );
}

template <>void set_pixel ( pcl::PointXYZ & xyz_pcl_pixel, cv::Mat & src, int x, int y, Intr& cam_params ){
    xyz_pcl_pixel.z = src.at<unsigned short>( y * cam_params.width + x ) * cam_params.scale_factor;
    xyz_pcl_pixel.x = xyz_pcl_pixel.z * ( x - cam_params.cx ) / cam_params.fx;
    xyz_pcl_pixel.y = xyz_pcl_pixel.z * ( y - cam_params.cy ) / cam_params.fy;
}

template < class T> bool load_cloud ( const string & file_name, PointCloud<T> & pcl_cloud , Intr& cam_params ){

     cv::Mat cur_mat =   cv::imread ( file_name.c_str(), -1 );
	 cv::Size s      =   cur_mat.size();
	 int width       =   s.width;
	 int height      =   s.height;
     int nchannels   =   cur_mat.channels();
     int step        =   cur_mat.step;

     pcl_cloud.width   = width;
     pcl_cloud.height  = height;
     pcl_cloud.is_dense = true;
     pcl_cloud.points.resize ( width * height );

     for ( int y = 0; y < height; y++ )
        for ( int x = 0; x < width; x++ )
        {
            T   current_pixel;
            set_pixel <T> ( current_pixel, cur_mat, x, y, cam_params );
            pcl_cloud(x,y) = current_pixel;
        }

}
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(cloud);

    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main ( int argc, char* argv[] ){
  
    vector<string> depth_names;
    vector<string> rgb_names;
    vector<string> pcd_file_names;
    cerr<<"putting names"<<endl;
    cerr<<"putting names"<<endl;


    const char *imgFilename = "/home/frank/university/laboratorio_ia/homework/hw_5/rgbd-scenes/table_small/table_small_1/table_small_1_%d.png";
    const char* depthFilename = "/home/frank/university/laboratorio_ia/homework/hw_5/rgbd-scenes/table_small/table_small_1/table_small_1_%d_depth.png";
    const char* pcdFilename = "output/theirCloud_%d.pcd";
    if(argc == 1){
        for(int i = 0 ; i < NUM; i++){
            cerr<<"putting names"<<endl;
            rgb_names.push_back(cv::format(imgFilename, i+1));
            depth_names.push_back(cv::format(depthFilename, i+1));
            pcd_file_names.push_back(cv::format(pcdFilename, i+1));
        }
    }
    // do we have data for continue ?
    if ( depth_names.empty() || depth_names.size() != rgb_names.size() )
        usage();

    Intr cam_params; 
    load_camera_intrinsics ( DEFAULT_CFG_FILE, cam_params );

    for ( int i = 0; i < pcd_file_names.size(); i++){
        PointCloud < RGB >          current_color_cloud;
        PointCloud < PointXYZ >     current_xyz_cloud;
        PointCloud < PointXYZRGBA > current_xyzrgb_cloud;
        cout<<i<<" ciao bro"<<endl;
        load_cloud<RGB> (rgb_names[i], current_color_cloud, cam_params);
        
        load_cloud<PointXYZ> (depth_names[i], current_xyz_cloud, cam_params );

        copyPointCloud ( current_xyz_cloud, current_xyzrgb_cloud );
        for ( size_t ii = 0; ii < current_color_cloud.size (); ++ii )
            current_xyzrgb_cloud.points[ii].rgba = current_color_cloud.points[ii].rgba;
        /*
        pcl::io::savePCDFileASCII<pcl::PointXYZRGBA> ( pcd_file_names[i], current_xyzrgb_cloud );
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (current_xyzrgb_cloud);
        visualize(cloud);
        */
    }
    return 0;
}




