#include "glog/logging.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <string>
#include <iostream>

#include <yaml-cpp/yaml.h>

int pcd2bin(pcl::PointCloud<pcl::PointXYZI> cloud, std::string bin_name)
{
  std::ofstream bin_file(bin_name.c_str(), std::ios::out | std::ios::binary);
  if (!bin_file.good())
  {
    std::cout << "Couldn't open " << bin_name << std::endl;
  }

  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    //! Should adjust according to point type
    bin_file.write((char *)&cloud.points[i].x, 3 * sizeof(float));
    bin_file.write((char *)&cloud.points[i].intensity, sizeof(float));
  }
  bin_file.close();

  return 0;
}

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

  if (!pcd_path.compare(pcd_path.length() - 4, 4, ".pcd"))
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::io::loadPCDFile(pcd_path, cloud);
    std::string bin_name = pcd_path.substr(0, pcd_path.size() - 4) + ".bin";
    int res = pcd2bin(cloud, bin_name);
    return res;
  }

  if (!pcd_path.compare(pcd_path.length() - 1, 1, "/"))
  {
    if (bin_path.compare(pcd_path.length() - 1, 1, "/"))
    {
      return -2;
    }
    // char *cmd;
    // cmd = std::string("mkdir -p " + bin_path).c_str();
    system(std::string("mkdir -p " + bin_path).c_str());
    std::cout << pcd_path << std::endl;
    for (const auto &entry : std::filesystem::directory_iterator(pcd_path))
    {
      auto filename = entry.path().filename().string();

      // filename = bin_path + filename;

      // If you want to check if it's a file
      if (!std::filesystem::is_regular_file(entry.status()))
      {
        continue;
      }
      if (filename.find(".pcd") != std::string::npos)
      {
        std::cout << entry.path().string() << std::endl;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::io::loadPCDFile(entry.path().string(), cloud);
        std::string bin_name = bin_path + filename.substr(0, pcd_path.size() - 5) + ".bin";
        std::cout << bin_name << std::endl;
        int res = pcd2bin(cloud, bin_name);
        if (res != 0)
        {
          LOG(FATAL) << "Failed to generate " << bin_name;
        }
      }
    }
    return 0;
  }
  return -1;
}