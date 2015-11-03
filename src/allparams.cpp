#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/program_options.hpp>

#include "allparams.h"

namespace po = boost::program_options;
using namespace std;

params::params(po::variables_map vm) {
    tf_directory_ = vm["tf_directory"].as<string>();
    original_pointcloud_directory_ = vm["original_pointcloud_directory"].as<string>();
    quaternion_directory_ = vm["quaternion_directory"].as<string>();
    output_scan_directory_ = vm["output_scan_directory"].as<string>();
    resolution_ = vm["resolution"].as<float>();
    filter_xy_range_ = vm["filter_xy_range"].as<bool>();
    do_ndt_ = vm["do_ndt"].as<bool>();
}

std::string params::get_tf_directory() { return tf_directory_;};
std::string params::get_original_pointcloud_directory() { return original_pointcloud_directory_;};
std::string params::get_quaternion_directory() { return quaternion_directory_;};
std::string params::get_output_scan_directory() { return output_scan_directory_;};
float params::get_resolution() { return resolution_; };
bool params::get_filter_xy_range() { return filter_xy_range_; };
bool params::do_ndt() { return do_ndt_; }; 
std::string params::get_scan_filename(int scan_ix) {
    std::stringstream ss;
    ss << original_pointcloud_directory_ << "cuboids_irgblabel_" << std::setw(2) << std::setfill('0') << scan_ix << "." << resolution_ << ".pcd";
    return ss.str();
}
