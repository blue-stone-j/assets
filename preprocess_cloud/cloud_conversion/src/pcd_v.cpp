#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
  std::string config_file = "../config/params.yaml";

  YAML::Node config_node = YAML::LoadFile(config_file);

  std::string field;

  field = "pcd";
  std::string pcd = config_node[field].as<std::string>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *cloud);

  pcl::visualization::PCLVisualizer viewer("点云查看器");

  // 添加坐标轴
  viewer.addCoordinateSystem(5.0);

  // 添加网格
  // 设置网格范围和间隔
  int grid_range = 30;       // 网格范围为30米
  float grid_interval = 1.0; // 网格间隔为1米

  // 在X-Y平面绘制网格
  for (int i = -grid_range; i <= grid_range; i++)
  {
    if (i % 5 == 0)
    { // 每5米绘制一条主线
      // 线条在Y轴方向
      viewer.addLine(pcl::PointXYZ(i, -grid_range, 0), pcl::PointXYZ(i, grid_range, 0), 1, 0, 0, "line_y_" + std::to_string(i));
      // 线条在X轴方向
      viewer.addLine(pcl::PointXYZ(-grid_range, i, 0), pcl::PointXYZ(grid_range, i, 0), 0, 1, 0, "line_x_" + std::to_string(i));
    }
    else
    { // 次要线条
      // 次要线条在Y轴方向
      viewer.addLine(pcl::PointXYZ(i, -grid_range, 0), pcl::PointXYZ(i, grid_range, 0), 0.5, 0.5, 0.5, "line_y_minor_" + std::to_string(i));
      // 次要线条在X轴方向
      viewer.addLine(pcl::PointXYZ(-grid_range, i, 0), pcl::PointXYZ(grid_range, i, 0), 0.5, 0.5, 0.5, "line_x_minor_" + std::to_string(i));
    }
  }

  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景颜色为深灰

  // pcl::PointXYZ minPt, maxPt;
  // pcl::getMinMax3D(*cloud, minPt, maxPt);
  // double z_min = minPt.z;
  // double z_max = maxPt.z;
  // // 对每个点应用颜色映射
  // for (size_t i = 0; i < cloud->points.size(); ++i)
  // {
  //   double z = cloud->points[i].z;
  //   double normalized = (z - z_min) / (z_max - z_min); // 将z值归一化到[0, 1]
  //   // 创建渐变色
  //   double r = normalized;       // 红色分量：z值越高，红色分量越强
  //   double g = 1.0 - normalized; // 绿色分量：z值越低，绿色分量越强
  //   double b = 0.5;              // 蓝色分量：常量
  //   viewer.addSphere(cloud->points[i], 0.001, r, g, b, "sphere" + std::to_string(i));
  // }
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }

  return 0;
}
