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
std::string bag_name, bin_dest;

int main(int argc, char **argv)
{
  //***** 设置日志 *****//
  // default FLAGS_log_dir is '/tmp';
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

  field = "bin_dest";
  bin_dest = config_node[field].as<std::string>();

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

      std::string name = bin_dest + std::to_string(time) + ".bin";
      std::ofstream bin_file(name.c_str(), std::ios::out | std::ios::binary);
      if (!bin_file.good())
      {
        std::cout << "Couldn't open " << name << std::endl;
      }

      for (size_t i = 0; i < cloud.points.size(); ++i)
      {
        //! Should adjust according to point type
        bin_file.write((char *)&cloud.points[i].x, 3 * sizeof(float));
        bin_file.write((char *)&cloud.points[i].intensity, sizeof(float));
      }
      bin_file.close();
      std::cout << name << std::endl;
    }
  }

  i_bag.close();
  return 0;
}
