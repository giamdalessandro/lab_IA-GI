# Point Cloud Library


## Tutorial
- http://wiki.ros.org/pcl/Tutorials


# What I Did
```
catkin_create_pkg hw_pcl  pcl_conversions pcl_ros roscpp sensor_msgs
```

### Tip

- Get point to `cvMat`: https://answers.opencv.org/question/12963/how-to-get-pixels-value-from-a-picture/
- format https://answers.opencv.org/question/95323/possible-to-convert-value-of-a-point-to-string-opencv-c/
- Superficie normale: https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
- Calcolo superficie normale: https://answers.opencv.org/question/82453/calculate-surface-normals-from-depth-image-using-neighboring-pixels-cross-product/

- PassThrough http://pointclouds.org/documentation/tutorials/passthrough.php
- slides originali: http://profs.scienze.univr.it/~bloisi/corsi/lezioni/pcl.pdf


## Filter NaN points
https://github.com/daviddoria/Examples/blob/master/c%2B%2B/PCL/Filters/RemoveNaNFromPointCloud/RemoveNaNFromPointCloud.cpp
