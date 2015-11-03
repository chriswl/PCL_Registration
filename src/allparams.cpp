#include <iostream>
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
}

std::string params::get_tf_directory() { return tf_directory_;};
std::string params::get_original_pointcloud_directory() { return original_pointcloud_directory_;};
std::string params::get_quaternion_directory() { return quaternion_directory_;};
std::string params::get_output_scan_directory() { return output_scan_directory_;};

