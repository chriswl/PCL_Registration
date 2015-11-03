#ifndef RTREE_PARAMS
#define RTREE_PARAMS

#include <boost/program_options.hpp>
namespace po = boost::program_options;

class params {
    public:
        params(po::variables_map);

        std::string get_tf_directory_();
        std::string get_original_pointcloud_directory();
        std::string get_quaterion_directory();
        std::string get_output_scan_directory();

    private:
        std::string tf_directory_;
        std::string original_pointcloud_directory_;
        std::string quaterion_directory_;
        std::string output_scan_directory_;
};

std::string params::get_tf_directory_() {return tf_directory_;};
std::string params::get_original_pointcloud_directory() { return original_pointcloud_directory_;};
std::string params::get_quaterion_directory() { return quaterion_directory_;};
std::string params::get_output_scan_directory() { return output_scan_directory_;};

#endif
