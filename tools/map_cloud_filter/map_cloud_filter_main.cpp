#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <pcl/io/ply_io.h>

#include "map_cloud_filter.hpp"

namespace
{

void usage()
{
  std::cerr <<
    "Usage: map_cloud_filter INPUT.ply OUTPUT.ply [options]\n"
    "  --config PATH             YAML configuration file\n"
    "  --mean-k N                 statistical neighbors (default 20, 0 disables)\n"
    "  --stddev VALUE             statistical threshold (default 2.0)\n"
    "  --radius METERS            neighbor radius (default 0.12, 0 disables)\n"
    "  --min-neighbors N          neighbors required in radius (default 4)\n"
    "  --cluster-tolerance METERS cluster connection radius (default 0.12)\n"
    "  --min-cluster-size N       discard smaller clusters (default 20, 0 disables)\n"
    "  --min-z METERS             optional map-frame lower height\n"
    "  --max-z METERS             optional map-frame upper height\n";
}

const char * valueAfter(int & index, int argc, char ** argv)
{
  if (++index >= argc) {
    throw std::runtime_error(std::string("missing value after ") + argv[index - 1]);
  }
  return argv[index];
}

int integerAfter(int & index, int argc, char ** argv)
{
  return std::stoi(valueAfter(index, argc, argv));
}

double doubleAfter(int & index, int argc, char ** argv)
{
  return std::stod(valueAfter(index, argc, argv));
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    usage();
    return 2;
  }
  try {
    const std::filesystem::path input_path(argv[1]);
    const std::filesystem::path output_path(argv[2]);
    if (input_path == output_path) {
      throw std::runtime_error("input and output paths must be different");
    }

    luxi_map_tools::FilterParameters parameters;
    for (int index = 3; index < argc; ++index) {
      const std::string option(argv[index]);
      if (option == "--config") {
        parameters = luxi_map_tools::loadFilterParameters(
          valueAfter(index, argc, argv), parameters);
      }
    }
    for (int index = 3; index < argc; ++index) {
      const std::string option(argv[index]);
      if (option == "--config") {
        valueAfter(index, argc, argv);
      } else if (option == "--mean-k") {
        parameters.mean_k = integerAfter(index, argc, argv);
      } else if (option == "--stddev") {
        parameters.standard_deviation = doubleAfter(index, argc, argv);
      } else if (option == "--radius") {
        parameters.radius = doubleAfter(index, argc, argv);
      } else if (option == "--min-neighbors") {
        parameters.minimum_neighbors = integerAfter(index, argc, argv);
      } else if (option == "--cluster-tolerance") {
        parameters.cluster_tolerance = doubleAfter(index, argc, argv);
      } else if (option == "--min-cluster-size") {
        parameters.minimum_cluster_size = integerAfter(index, argc, argv);
      } else if (option == "--min-z") {
        parameters.minimum_z = doubleAfter(index, argc, argv);
      } else if (option == "--max-z") {
        parameters.maximum_z = doubleAfter(index, argc, argv);
      } else {
        throw std::runtime_error("unknown option: " + option);
      }
    }
    luxi_map_tools::validateFilterParameters(parameters);

    auto input = std::make_shared<luxi_map_tools::MapCloud>();
    if (pcl::io::loadPLYFile(input_path.string(), *input) < 0) {
      throw std::runtime_error("cannot read input PLY: " + input_path.string());
    }
    luxi_map_tools::FilterReport report;
    const auto output = luxi_map_tools::filterMapCloud(input, parameters, report);
    if (output->empty()) {
      throw std::runtime_error("all points were rejected; output was not written");
    }
    if (!output_path.parent_path().empty()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    if (pcl::io::savePLYFileBinary(output_path.string(), *output) < 0) {
      throw std::runtime_error("cannot write output PLY: " + output_path.string());
    }

    const double removed_percent = 100.0 *
      static_cast<double>(report.input_points - report.output_points) /
      static_cast<double>(report.input_points);
    std::cout << "input=" << report.input_points << " finite=" << report.finite_points
              << " height=" << report.after_height_filter
              << " statistical=" << report.after_statistical_filter
              << " radius=" << report.after_radius_filter
              << " output=" << report.output_points << " removed=" << std::fixed
              << std::setprecision(2) << removed_percent << "%\n"
              << "Filtered PLY: " << output_path << '\n';
  } catch (const std::exception & error) {
    std::cerr << "map_cloud_filter: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
