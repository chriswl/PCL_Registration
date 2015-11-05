#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <iterator>

namespace po = boost::program_options;
using namespace std;

// A helper function to simplify the main part.
template <class T>
ostream& operator<<(ostream& os, const vector<T>& v) {
    copy(v.begin(), v.end(), ostream_iterator<T>(os, " "));
    return os;
}

int read_config(int ac, char* av[], po::variables_map &vm) {
    try {
        string config_file;

        // Declare a group of options that will be
        // allowed only on command line
        po::options_description generic("Generic options");
        generic.add_options()
            ("scan_ix", po::value<int>()->default_value(-1), "scan index")
            ("config,c", po::value<string>(&config_file)->default_value("multiple_sources.cfg"),
                "name of a file of a configuration.");

        // Hidden options, will be allowed both on command line and
        // in config file, but will not be shown to the user.
        po::options_description hidden("Hidden options");
        hidden.add_options()
            ("tf_directory", po::value<string>()->default_value("/home/landsiedel/data/labels/transforms/"))
            ("initial_guesses_filename", po::value<string>()->default_value("overall_initial_estimates.txt"))
            ("original_pointcloud_directory", po::value<string>()->default_value("/home/landsiedel/data/scans/original/"))
            ("quaternion_directory", po::value<string>()->default_value("/home/landsiedel/data/transforms/quaternions/"))
            ("output_scan_directory", po::value<string>()->default_value("/home/landsiedel/data/scans/transformed_scans/"))
            ("resolution", po::value<float>()->default_value(0.1))
            ("init_max_correspondence_distance", po::value<float>()->default_value(1.0))
            ("max_correspondence_distance_step", po::value<float>()->default_value(0.1))
            ("do_icp", po::value<bool>()->default_value(true))
            ("do_ndt", po::value<bool>()->default_value(true))
            ("do_gicp", po::value<bool>()->default_value(true))
            ("n_iterations", po::value<int>()->default_value(10))
            ("filter_xy_range", po::value<bool>()->default_value(false));

        po::options_description cmdline_options;
        cmdline_options.add(generic);

        po::positional_options_description p;
        p.add("scan_ix", 1);

        store(po::command_line_parser(ac, av).options(cmdline_options).positional(p).run(), vm);
        notify(vm);

        po::options_description config_file_options;
        config_file_options.add(hidden);

        ifstream ifs(config_file.c_str());
        if (!ifs) {
            cout << "can not open config file: " << config_file << "\n";
            return 0;
        } else {
            store(parse_config_file(ifs, config_file_options), vm);
            notify(vm);
        }

    } catch (exception& e) {
        cerr << "CL parsing Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
