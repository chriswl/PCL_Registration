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

#include "pcl_tools.h"

#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

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


// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

void create_launch_file(int min_index, int max_index){
    //Create launch file for icp_result
    for(int i=min_index; i<=max_index; i++){
        std::cout << "<node pkg=\"pcl_ros\" type=\"pcd_to_pointcloud\" name=\"cloud_" << i << "\" args=\"/home/mustafasezer/dataset/Downsampled_Projected_3.5_4.5/scan_" << i << ".pcd 0.1\" output=\"screen\">\n\t<param name=\"frame_id\" value=\"iser_transform_1\" />\n\t<remap from=\"cloud_pcd\" to=\"cloud_" << i << "\" />\n</node>\n\n";
    }
}


int main(int argc, char** argv) {
    /* pcl_tools pcl_tool; */
    if (argc != 3) {
        std::cout << "3 args needed" << std::endl;
        return(-1);
    }

    string filename(argv[1]);
    string out_filename(argv[2]);

    downsample_pcd(filename, out_filename, 0.2f);
};

