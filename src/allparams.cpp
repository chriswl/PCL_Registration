#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

#include "allparams.h"

namespace po = boost::program_options;
using namespace std;

typedef std::pair<int, std::vector<int> > concentration_params;
typedef std::vector<concentration_params> dirichlet_params;

params::params(po::variables_map vm) {
    tf_directory_ = vm["tf_directory"].as<string>();
    original_pointcloud_directory_ = vm["original_pointcloud_directory"].as<string>();
    quaterion_directory_ = vm["quaterion_directory"].as<string>();
    output_scan_directory_ = vm["output_scan_directory"].as<string>();
}

