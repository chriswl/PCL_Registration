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

    ros::Duration(0.5).sleep();
    sensor_msgs::PointCloud2 total_cloud;
    for (int i=1; i < argc; i++) {
        std::string filename(argv[i]);
        std::cout << "Working on " << filename << std::endl;

        sensor_msgs::PointCloud2 cloud;
        pcl::io::loadPCDFile(filename, cloud);

        std::cout << "loaded " << filename << std::endl;
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


