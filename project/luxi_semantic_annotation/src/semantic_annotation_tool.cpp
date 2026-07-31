#include "luxi_semantic_annotation/semantic_annotation.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

nlohmann::json read_json(const std::string & path)
{
  if (path == "-") {
    nlohmann::json input;
    std::cin >> input;
    return input;
  }
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open annotation input: " + path);
  }
  nlohmann::json input;
  stream >> input;
  return input;
}

void usage()
{
  std::cerr
    << "Usage:\n"
    << "  semantic_annotation_tool inspect OCTOMAP\n"
    << "  semantic_annotation_tool validate OCTOMAP MAP_ID INPUT_JSON\n"
    << "  semantic_annotation_tool save OCTOMAP MAP_ID INPUT_JSON OUTPUT_JSON\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 3) {
      usage();
      return 2;
    }
    const std::string command = argv[1];
    auto tree = luxi_semantic_annotation::load_octomap(argv[2]);
    if (command == "inspect" && argc == 3) {
      std::cout << luxi_semantic_annotation::summary_json(
        luxi_semantic_annotation::inspect_octomap(*tree)).dump() << '\n';
      return 0;
    }
    if (command == "validate" && argc == 5) {
      std::cout << luxi_semantic_annotation::validate_annotation(
        read_json(argv[4]), *tree, argv[3]).dump() << '\n';
      return 0;
    }
    if (command == "save" && argc == 6) {
      const auto annotation = luxi_semantic_annotation::validate_annotation(
        read_json(argv[4]), *tree, argv[3]);
      luxi_semantic_annotation::save_annotation_atomic(argv[5], annotation);
      std::cout << annotation.dump() << '\n';
      return 0;
    }
    usage();
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "semantic_annotation_tool: " << error.what() << '\n';
    return 1;
  }
}
