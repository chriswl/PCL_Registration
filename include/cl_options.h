#ifndef PCLREG_CL_OPTIONS
#define PCLREG_CL_OPTIONS

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int read_config(int ac, char* av[], po::variables_map &vm);

#endif
