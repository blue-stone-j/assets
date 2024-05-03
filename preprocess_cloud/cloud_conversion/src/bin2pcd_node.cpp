#include "glog/logging.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <string>
#include <iostream>

#include <yaml-cpp/yaml.h>

std::string pcd_path, bin_path;

int main(int argc, char **argv)
{
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;

  std::cout << std::endl;
  std::string config_file = "../config/params.yaml";
  // std::cout << config_file_path << std::endl;

  YAML::Node config_node = YAML::LoadFile(config_file);

  std::string field;

  field = "pcd_path";
  pcd_path = config_node[field].as<std::string>();

  field = "bin_path";
  bin_path = config_node[field].as<std::string>();
}