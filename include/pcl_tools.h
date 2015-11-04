#ifndef PCL_TOOLS_H
#define PCL_TOOLS_H


#include <ios>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Geometry>

#include "allparams.h"

typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


class pcl_tools
{
public:
    pcl_tools(params *config);
    int MAX_NUM_SCANS;
    struct transformation_relation{
        //int id;
        bool is_parent;
        int parent_id;
        Eigen::Matrix4f init_guess;
        Eigen::Matrix4f T;
        Eigen::Matrix4f T_ndt;
        bool completed;
        bool ok;
        //struct transformation_relation* parent;
    };
    Eigen::Matrix4f apply_icp(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out, bool viewResult=true);
    Eigen::Matrix4f do_icp(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
    Eigen::Matrix4f do_gicp(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
    Eigen::Matrix4f do_ndt(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f init_guess);
    int apply_icp(std::string path_in, std::string path_out);
    pcl::PointCloud<PointT>::Ptr loadPCD(std::string path);
    pcl::PointCloud<PointT>::Ptr transform_pcd(pcl::PointCloud<PointT>::Ptr source_cloud, Eigen::Matrix4f transform_matrix);
    pcl::PointCloud<PointT>::Ptr getSlice(pcl::PointCloud<PointT>::Ptr cloud, float z1, float z2, std::string field_name = "z");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSliceRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float z1, float z2, std::string field_name = "z");
    void savePCD(pcl::PointCloud<PointT>::Ptr cloud, std::string path);
    Eigen::Matrix3f quaternion_to_rotation(float x, float y, float z, float w);
    Eigen::Matrix4f createTransformationMatrix(Eigen::Matrix3f rotation, Eigen::Vector3f translation);
    void getPCDStatistics(std::string dir, int low_ind, int up_ind);
    pcl::PointCloud<PointT>::Ptr projectPCD(pcl::PointCloud<PointT>::Ptr cloud, float a, float b, float c, float d);
    Eigen::Matrix4f getInitialGuess(int input, int target);
    Eigen::Matrix4f getTransformation(int input, int target);
    int getInitialGuesses(std::string filename);
    int getTransformations(std::string filename, Eigen::Matrix4f transformation_matrices[]);
    int computeGlobalTransformations();
    int topMostParent(int id);


    struct transformation_relation transformations[83];
    Eigen::Matrix4f transformations_icp[83], transformations_ndt[83];

private:
    params *config_;

};

#endif // PCL_TOOLS_H
