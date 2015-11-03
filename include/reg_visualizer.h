#ifndef REG_VISUALIZER
#define REG_VISUALIZER

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;



class RegVisualizer {
    public:

    void rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void viewPCD(pcl::PointCloud<PointT>::Ptr cloud, std::string name="point_cloud", int r=255, int g=255, int b=255);
    void viewICPResult(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_targ, pcl::PointCloud<PointT>::Ptr cloud_aligned);

};


#endif
