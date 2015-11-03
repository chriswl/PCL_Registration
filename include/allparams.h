#ifndef PCLREG_PARAMS
#define PCLREG_PARAMS

#include <boost/program_options.hpp>
namespace po = boost::program_options;

class params {
    public:
        params(po::variables_map);

        std::string get_tf_directory();
        std::string get_original_pointcloud_directory();
        std::string get_quaternion_directory();
        std::string get_output_scan_directory();

    private:
        std::string tf_directory_;
        std::string original_pointcloud_directory_;
        std::string quaternion_directory_;
        std::string output_scan_directory_;
};

#endif
