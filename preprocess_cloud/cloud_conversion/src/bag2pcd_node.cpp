#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "glog/logging.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <string>
#include <iostream>

#include <yaml-cpp/yaml.h>

std::string topic_cloud = "/lidar_points";
std::string bag_name, pcd_dest;

int main(int argc, char **argv)
{
  //***** 设置日志 *****//
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::cout << std::endl;
  std::string config_file = "../config/params.yaml";
  // std::cout << config_file_path << std::endl;

  YAML::Node config_node = YAML::LoadFile(config_file);

  std::string field;

  field = "bag_name";
  bag_name = config_node[field].as<std::string>();

  field = "topic_cloud";
  topic_cloud = config_node[field].as<std::string>();

  field = "pcd_dest";
  pcd_dest = config_node[field].as<std::string>();

  rosbag::Bag i_bag;
  i_bag.open(bag_name, rosbag::bagmode::Read);

  rosbag::View view(i_bag); // read all topics

  for (auto m : view)
  {
    // 得到该帧数据的topic
    std::string topic = m.getTopic();

    if (topic == topic_cloud)
    {
      sensor_msgs::PointCloud2 cloudMsg = *(m.instantiate<sensor_msgs::PointCloud2>());

      static pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::moveFromROSMsg(cloudMsg, cloud);
      long long time = m.getTime().toSec() * 1000;

      std::string name = pcd_dest + std::to_string(time) + ".pcd";
      pcl::io::savePCDFileBinary(name, cloud);
      std::cout << name << std::endl;
    }
  }

  i_bag.close();
  return 0;
}
