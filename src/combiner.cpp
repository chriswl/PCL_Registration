#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>
#include <sstream>
#include <iostream> //std::cout


std::vector<int> parse_number(std::string name) {
    std::vector<int> values;
    for(int i=0; i < name.size(); ++i) {
        int value;
        std::stringstream ss;
        ss << name.at(i);
        if (ss >> value) {//return false if conversion does not succeed
            values.push_back(value);
            // std::cout <<" v " << value << " " << ss.str() <<  std::endl;
        } else if (!values.empty()) {
            break;
        }
    }
    return values;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "combiner");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Duration(0.5).sleep(); // to update transforms

    if (argc == 1) {
        std::cout << "Usage: combiner scaling_factor(as float) pcd_filenames " << std::endl;
        return -1;
    }

    float scaling_factor;
    scaling_factor = std::strtof(argv[1], NULL);
    if (scaling_factor == 0.0) {
        std::cout << "No scaling factor given!! Aborting." << std::endl;
        return -1;
    }

    sensor_msgs::PointCloud2 total_cloud;
    for (int i=2; i < argc; i++) {
        std::string filename(argv[i]);
        std::cout << "Working on " << filename << ", scaling!! " << scaling_factor << std::endl;

        sensor_msgs::PointCloud2 cloud;
        pcl::io::loadPCDFile(filename, cloud);

        std::cout << "loaded " << filename << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> scale_cloud;
        pcl::fromROSMsg(cloud, scale_cloud);

        for (size_t j = 0; j < scale_cloud.points.size (); ++j){
            scale_cloud.points[j].x = scaling_factor*scale_cloud.points[j].x;
            scale_cloud.points[j].y = scaling_factor*scale_cloud.points[j].y;
            scale_cloud.points[j].z = scaling_factor*scale_cloud.points[j].z;
        }

        pcl::toROSMsg(scale_cloud, cloud);
        std::stringstream frame;
        frame << "scene_";

        std::vector<int> v;
        v = parse_number(filename);
        for(std::vector<int>::iterator it = v.begin(); it != v.end(); ++it) {
            frame << *it;
            // std::cout << *it;
        }
        cloud.header.frame_id = frame.str();

        std::cout << "Looking up transform " << frame.str() << " to map" << std::endl;
        pcl_ros::transformPointCloud("/map", cloud, cloud, listener);
        // std::cout << "res" << cloud[0] << "to map" << std::endl;

        // total_cloud += cloud;
        sensor_msgs::PointCloud2 temp_cloud;
        temp_cloud = total_cloud;

        pcl::concatenatePointCloud(cloud, temp_cloud, total_cloud);
        // std::cout << "Filtered size: " << total_cloud.points.size() << std::endl;
    }
    /* pcl::PCLPointCloud2 out_cloud; */
    /* pcl::fromROSMsg(total_cloud, out_cloud); */

    pcl::io::destructiveSavePCDFile("out.pcd", total_cloud);
}


