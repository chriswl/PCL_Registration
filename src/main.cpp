#include <ios>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <sstream>
#include <vector>

//Transform
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

#include "pcl_tools.h"

// params and cl interface
#include <boost/program_options.hpp>
#include "allparams.h"
#include "cl_options.h"

using namespace std;
struct timespec t1, t2;
double elapsed_time;
volatile long long i;


int file_exists (const char *filename)
{
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
}

void start_timer_(){
    clock_gettime(CLOCK_MONOTONIC,  &t1);
}

void end_timer_(string message = "Elapsed time:"){
    clock_gettime(CLOCK_MONOTONIC,  &t2);
    elapsed_time = (t2.tv_sec - t1.tv_sec) + (double) (t2.tv_nsec - t1.tv_nsec) * 1e-9;
    std::cout << message << " " << elapsed_time << " seconds" << std::endl;
}


void print_pcd_xyz(pcl::PointCloud<PointT>::Ptr cloud){
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;
}


int main(int argc, char **argv) {
    po::variables_map vm;
    read_config(argc, argv, vm);
    params *config = new params(vm);

    pcl_tools pcl_tool(config);

    pcl_tool.getInitialGuesses(config->get_tf_directory() + "initial_guesses.txt");

    fstream icp_result, ndt_result, gicp_result_fh;
    fstream overall_icp, overall_ndt;
    if (config->do_icp()) {
        icp_result.open((config->get_tf_directory() + "icp_results.txt").c_str(), ios::out);
        overall_icp.open((config->get_tf_directory() + "overall_icp.txt").c_str(), ios::out);
    }
    if (config->do_ndt()) {
        ndt_result.open((config->get_tf_directory() + "ndt_results.txt").c_str(), ios::out);
        overall_ndt.open((config->get_tf_directory() + "overall_ndt.txt").c_str(), ios::out);
    }
    if (config->do_gicp())
        gicp_result_fh.open((config->get_tf_directory() + "gicp_results.txt").c_str(), ios::out);


    for(int i=1; i<=pcl_tool.MAX_NUM_SCANS; i++){
        if(pcl_tool.transformations[i-1].is_root){
            std::cout << "scan_" << i <<  " is a parent transformation" << std::endl;
            if (config->do_icp()) {
                overall_icp << "scan_" << i <<  " to scan_" << i << " \n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n\n";
                icp_result << "scan_" << i << " to scan_" << i << std::endl;
                icp_result << pcl_tool.transformations[i-1].init_guess << std::endl << std::endl;
            }
            if (config->do_ndt()) {
                ndt_result << "scan_" << i << " to scan_" << i << std::endl;
                ndt_result << pcl_tool.transformations[i-1].init_guess << std::endl << std::endl;
                overall_ndt << "scan_" << i <<  " to scan_" << i << " \n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n\n";
            }
            if (config->do_gicp()) {
                gicp_result_fh << "scan_" << i << " to scan_" << i << std::endl;
                gicp_result_fh << pcl_tool.transformations[i-1].init_guess << std::endl << std::endl;
            }
            continue;
        }

        if (!pcl_tool.transformations[i-1].ok) {
            std::cout << "Transformation problematic, scan_" << i << " is being skipped" << std::endl;
            continue;
        }

        if(pcl_tool.transformations[i-1].parent_id > 83 || pcl_tool.transformations[i-1].parent_id < 1) {
            std::cout << "Parent id problematic, scan_" << i << " is being skipped" << std::endl;
            continue;
        }

        //Read input cloud
        std::string input_filename = config->get_scan_filename(i);
        if (!file_exists(input_filename.c_str())) {
            std::cerr << input_filename << " does not exist" << std::endl;
            continue;
        }

        std::cout << "Reading input cloud " << input_filename << std::endl;
        start_timer_();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename, *cloud_in_rgb) == -1) {
            string temp(input_filename);
            PCL_ERROR ("Couldn't read %s\n", temp.c_str());
            continue;
        }

        pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_in_rgb, *cloud_in);
        end_timer_("Input cloud loaded in:");
        if(cloud_in->points.size() == 0){
            std::cout << input_filename << " empty" << std::endl;
            continue;
        }

        //Read target cloud
        std::string target_filename = config->get_scan_filename(pcl_tool.transformations[i-1].parent_id);

        if (!file_exists(target_filename.c_str())) {
            std::cerr << target_filename << " does not exist" << std::endl;
            continue;
        }

        std::cout << "Reading target cloud " << target_filename << std::endl;
        start_timer_();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targ_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (target_filename, *cloud_targ_rgb) == -1) {
            string temp(target_filename);
            PCL_ERROR ("Couldn't read %s\n", temp.c_str());
            continue;
        }

        pcl::PointCloud<PointT>::Ptr cloud_targ(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_targ_rgb, *cloud_targ);
        end_timer_("Target cloud loaded in:");

        if(cloud_targ->points.size() == 0){
            std::cout << target_filename << " empty" << std::endl;
            cloud_in.reset();
            cloud_targ.reset();
            continue;
        }

        if(config->get_filter_xy_range()){
            cloud_in = pcl_tool.getSlice(cloud_in, -50, 50, "x");
            cloud_in = pcl_tool.getSlice(cloud_in, -50, 50, "y");
            cloud_targ = pcl_tool.getSlice(cloud_targ, -50, 50, "x");
            cloud_targ = pcl_tool.getSlice(cloud_targ, -50, 50, "y");
        }

        // transform by initial guess
        pcl::PointCloud<PointT>::Ptr cloud_in_transformed = pcl_tool.transform_pcd(cloud_in, pcl_tool.transformations[i-1].init_guess);

        Eigen::Matrix4f ndt_init_guess;
        ndt_init_guess.setIdentity();

        //Whole data
        //Eigen::Matrix4f transform_matrix_ndt = pcl_tool.apply_ndt(cloud_in_transformed, cloud_targ, ndt_init_guess);
        //Eigen::Matrix4f transform_matrix_icp = pcl_tool.apply_icp(cloud_in_transformed, cloud_targ, true);

        // Only specific z range
        pcl::PointCloud<PointT>::Ptr cloud_in_transformed_filtered = pcl_tool.getSlice(cloud_in_transformed, 2.5, 20);
        pcl::PointCloud<PointT>::Ptr cloud_targ_filtered = pcl_tool.getSlice(cloud_targ, 2.5, 20);

        Eigen::Matrix4f transform_matrix_ndt;
        if (config->do_ndt()) {
            std::cout << "Applying NDT" << std::endl;
            start_timer_();
            transform_matrix_ndt = pcl_tool.do_ndt(cloud_in_transformed_filtered, cloud_targ_filtered, ndt_init_guess);
            end_timer_("NDT completed in:");

            ndt_result << "scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
            ndt_result << transform_matrix_ndt << std::endl << std::endl;
        }

        Eigen::Matrix4f transform_matrix_gicp;
        if (config->do_gicp()) {
            std::cout << "Applying GICP" << std::endl;
            start_timer_();
            transform_matrix_gicp = pcl_tool.do_gicp(cloud_in_transformed_filtered, cloud_targ_filtered);
            end_timer_(" completed in:");

            gicp_result_fh << "scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
            gicp_result_fh << transform_matrix_gicp << std::endl << std::endl;
        }

        Eigen::Matrix4f transform_matrix_icp;
        if (config->do_icp()) {
            std::cout << "ICP-Aligning scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
            start_timer_();
            transform_matrix_icp = pcl_tool.do_icp(cloud_in_transformed_filtered, cloud_targ_filtered);
            end_timer_("Pair aligned in:");

            // write pairwise transforms to file
            icp_result << "scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
            icp_result << transform_matrix_icp << std::endl << std::endl;
        }

        // break if parent transforms are not complete
        if (!pcl_tool.transformations[pcl_tool.transformations[i-1].parent_id-1].completed){
            std::cerr << "Parent transformation is not completed for scan_" << i << std::endl;
            cloud_in.reset();
            cloud_targ.reset();
            cloud_in_transformed.reset();
            continue;
        }

        // apply transformation
        if (config->do_ndt()) {
            pcl_tool.transformations[i-1].T_ndt = pcl_tool.transformations[pcl_tool.transformations[i-1].parent_id-1].T_ndt * transform_matrix_ndt * pcl_tool.transformations[i-1].init_guess;
        }
        if (config->do_icp()) {
            pcl_tool.transformations[i-1].T = pcl_tool.transformations[pcl_tool.transformations[i-1].parent_id-1].T * transform_matrix_icp * pcl_tool.transformations[i-1].init_guess;
        }
        pcl_tool.transformations[i-1].completed = true;


        // save transformed point cloud
        stringstream output_filename;
        output_filename << config->get_output_scan_directory() << "scan_" << i << "_" << config->get_resolution() << ".pcd";

        //RGB Version
        //pcl_tool.savePCD((pcl_tool.transform_pcd(cloud_in, pcl_tool.transformations[i-1].T)), output_filename.str());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_rgb_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_in_rgb, *cloud_in_rgb_transformed, pcl_tool.transformations[i-1].T);
        pcl::io::savePCDFile(output_filename.str(), *cloud_in_rgb_transformed, true);

        // write global frame transforms to file
        // seems like this relies on the numbers in the transform tree being ascending
        int top_parent = pcl_tool.topMostParent(i);
        if(top_parent >= 1 && top_parent <= pcl_tool.MAX_NUM_SCANS){

            if (config->do_icp()) {
                overall_icp << "scan_" << i << " to scan_" << top_parent << std::endl;
                overall_icp << pcl_tool.transformations[i-1].T << std::endl << std::endl;
            }

            if (config->do_ndt()) {
                overall_ndt << "scan_" << i << " to scan_" << top_parent << std::endl;
                overall_ndt << pcl_tool.transformations[i-1].T_ndt << std::endl << std::endl;
            }
        }

        cloud_in.reset();
        cloud_targ.reset();
        cloud_in_transformed.reset();
        cloud_in_rgb.reset();
        cloud_targ_rgb.reset();
    }

    if (config->do_icp()) {
        icp_result.close();
        overall_icp.close();
    }
    if (config->do_gicp()) {
        gicp_result_fh.close(); }
    if (config->do_ndt()) {
        ndt_result.close();
        overall_ndt.close();
    }

    return 0;
}

