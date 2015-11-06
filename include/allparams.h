#ifndef PCLREG_PARAMS
#define PCLREG_PARAMS

#include <boost/program_options.hpp>
namespace po = boost::program_options;

class params {
    public:
        params(po::variables_map);

        std::string get_tf_directory();
        std::string get_initial_estimate_path();
        std::string get_original_pointcloud_directory();
        std::string get_quaternion_directory();
        std::string get_output_scan_directory();
        float get_resolution();
        float get_init_max_correspondence_distance();
        float get_max_correspondence_distance_step();
        bool get_filter_xy_range();
        bool do_icp();
        bool do_ndt();
        bool do_gicp();
        int get_n_iterations();

        std::string get_scan_filename(int scan_ix);
        std::string get_result_path(std::string method, int scan_ix);

    private:
        std::string tf_directory_;
        std::string initial_guesses_filename_;
        std::string original_pointcloud_directory_;
        std::string quaternion_directory_;
        std::string output_scan_directory_;
        float resolution_;
        float init_max_correspondence_distance_;
        float max_correspondence_distance_step_;
        bool filter_xy_range_;
        bool do_ndt_;
        bool do_icp_;
        bool do_gicp_;
        int n_iterations_;
};

#endif
