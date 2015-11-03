#include "reg_visualizer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

void RegVisualizer::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    viewer.setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}


void RegVisualizer::viewPCD(pcl::PointCloud<PointT>::Ptr cloud, std::string name, int r, int g, int b){
    //*viewer= pcl::visualization::PCLVisualizer("Matrix transformation example");

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(cloud, r, g, b);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, cloud_color_handler, name);

    //viewer.addCoordinateSystem (1.0, name, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void RegVisualizer::viewICPResult(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_targ, pcl::PointCloud<PointT>::Ptr cloud_aligned){
    //*viewer= pcl::visualization::PCLVisualizer("Matrix transformation example");

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_handler(cloud_in, 255, 255, 255);
    viewer.addPointCloud (cloud_in, cloud_in_color_handler, "cloud_in");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_targ_color_handler(cloud_targ, 0, 0, 255);
    viewer.addPointCloud (cloud_targ, cloud_targ_color_handler, "cloud_targ");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_aligned_color_handler(cloud_aligned, 255, 0, 0);
    viewer.addPointCloud (cloud_aligned, cloud_aligned_color_handler, "cloud_aligned");

    //viewer.addCoordinateSystem (1.0, name, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_in");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_targ");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_aligned");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_transformed");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
}


