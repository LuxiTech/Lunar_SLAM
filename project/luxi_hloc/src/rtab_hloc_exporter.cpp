#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>

#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct Arguments
{
  std::filesystem::path database;
  std::filesystem::path output;
  int jpeg_quality{95};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage: " << executable
    << " --database MAP.db --output HLOC_MAP_DIRECTORY [--jpeg-quality 95]\n";
}

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help" || option == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + option);
    }
    const std::string value = argv[++index];
    if (option == "--database") {
      arguments.database = value;
    } else if (option == "--output") {
      arguments.output = value;
    } else if (option == "--jpeg-quality") {
      arguments.jpeg_quality = std::stoi(value);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (arguments.database.empty() || arguments.output.empty()) {
    throw std::invalid_argument("--database and --output are required");
  }
  if (!std::filesystem::is_regular_file(arguments.database)) {
    throw std::invalid_argument("database does not exist: " + arguments.database.string());
  }
  if (arguments.jpeg_quality < 1 || arguments.jpeg_quality > 100) {
    throw std::invalid_argument("--jpeg-quality must be in [1, 100]");
  }
  return arguments;
}

std::string matrix_header()
{
  std::ostringstream output;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      output << ",t" << row << column;
    }
  }
  return output.str();
}

void append_matrix(std::ostream & output, const rtabmap::Transform & transform)
{
  output << std::setprecision(12);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      output << ',' << transform(row, column);
    }
  }
  output << ",0,0,0,1";
}

cv::Mat depth_as_millimeters(const cv::Mat & depth, double & depth_scale)
{
  depth_scale = 0.001;
  if (depth.type() == CV_16UC1) {
    return depth;
  }
  if (depth.type() == CV_32FC1) {
    cv::Mat converted;
    depth.convertTo(converted, CV_16UC1, 1000.0);
    return converted;
  }
  throw std::runtime_error("depth image is neither CV_16UC1 nor CV_32FC1");
}

int export_database(const Arguments & arguments)
{
  std::unique_ptr<rtabmap::DBDriver> driver(rtabmap::DBDriver::create());
  if (!driver || !driver->openConnection(arguments.database.string(), false, true)) {
    throw std::runtime_error("unable to open RTAB database read-only");
  }

  std::set<int> node_ids;
  driver->getAllNodeIds(node_ids, false, true, true);
  if (node_ids.empty()) {
    throw std::runtime_error("RTAB database contains no valid keyframes");
  }
  const std::map<int, rtabmap::Transform> optimized_poses = driver->loadOptimizedPoses();

  const auto image_directory = arguments.output / "reference_images";
  const auto depth_directory = arguments.output / "reference_depth";
  std::filesystem::create_directories(image_directory);
  std::filesystem::create_directories(depth_directory);

  std::ofstream frames(arguments.output / "frames.csv", std::ios::trunc);
  if (!frames) {
    throw std::runtime_error("unable to create frames.csv");
  }
  frames
    << "node_id,image_name,depth_name,stamp,map_id,fx,fy,cx,cy,width,height,depth_scale"
    << matrix_header() << '\n';

  std::size_t exported = 0;
  std::size_t skipped = 0;
  for (const int node_id : node_ids) {
    rtabmap::SensorData data;
    driver->getNodeData(node_id, data, true, false, false, false);
    cv::Mat image;
    cv::Mat depth;
    data.uncompressData(&image, &depth);
    if (image.empty() || depth.empty() || data.cameraModels().size() != 1) {
      ++skipped;
      continue;
    }
    const rtabmap::CameraModel & model = data.cameraModels().front();
    if (!model.isValidForProjection()) {
      ++skipped;
      continue;
    }

    rtabmap::Transform node_pose;
    int map_id = -1;
    int weight = 0;
    std::string label;
    double stamp = data.stamp();
    rtabmap::Transform ground_truth;
    std::vector<float> velocity;
    rtabmap::GPS gps;
    rtabmap::EnvSensors sensors;
    if (!driver->getNodeInfo(
        node_id, node_pose, map_id, weight, label, stamp, ground_truth, velocity, gps, sensors))
    {
      ++skipped;
      continue;
    }
    const auto optimized = optimized_poses.find(node_id);
    const rtabmap::Transform map_from_base =
      optimized != optimized_poses.end() ? optimized->second : node_pose;
    if (map_from_base.isNull() || model.localTransform().isNull()) {
      ++skipped;
      continue;
    }
    const rtabmap::Transform map_from_camera = map_from_base * model.localTransform();

    std::ostringstream stem;
    stem << "node_" << std::setw(6) << std::setfill('0') << node_id;
    const std::string image_name = "reference_images/" + stem.str() + ".jpg";
    const std::string depth_name = "reference_depth/" + stem.str() + ".png";
    const std::vector<int> jpeg_parameters{cv::IMWRITE_JPEG_QUALITY, arguments.jpeg_quality};
    if (!cv::imwrite((arguments.output / image_name).string(), image, jpeg_parameters)) {
      throw std::runtime_error("failed to write " + image_name);
    }
    double depth_scale = 0.0;
    const cv::Mat depth_millimeters = depth_as_millimeters(depth, depth_scale);
    if (!cv::imwrite((arguments.output / depth_name).string(), depth_millimeters)) {
      throw std::runtime_error("failed to write " + depth_name);
    }

    frames
      << node_id << ',' << image_name << ',' << depth_name << ','
      << std::setprecision(17) << stamp << ',' << map_id << ','
      << model.fx() << ',' << model.fy() << ',' << model.cx() << ',' << model.cy() << ','
      << image.cols << ',' << image.rows << ',' << depth_scale;
    append_matrix(frames, map_from_camera);
    frames << '\n';
    ++exported;
  }
  driver->closeConnection(false);

  if (exported == 0) {
    throw std::runtime_error("no keyframe had RGB, depth, calibration and a valid pose");
  }
  std::cout
    << "Exported " << exported << " reference frames to " << arguments.output
    << " (skipped " << skipped << ")\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return export_database(parse_arguments(argc, argv));
  } catch (const std::exception & exception) {
    std::cerr << "rtab_hloc_exporter: " << exception.what() << '\n';
    print_usage(argv[0]);
    return 1;
  }
}
