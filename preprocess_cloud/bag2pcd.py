import rosbag
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pcl

# 替换为您的bag文件路径和感兴趣的话题名称
bag_path = '2023-04-13-10-39-35.orig.bag'
topic_name = '/lidar1/points'

with rosbag.Bag(bag_path, 'r') as bag:
    counter = 0
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        #print("Message type:", type(msg))
        print("Message fields:", msg.fields)
        if isinstance(msg, PointCloud2):
            # 使用read_points来解析PointCloud2消息
            cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
            
            # 创建PCL点云对象
            pcl_cloud = pcl.PointCloud()
            pcl_cloud.from_list(cloud_points)
            
            # 构造文件名并保存点云到PCD文件
            filename = f'output_{counter}.pcd'
            pcl.save(pcl_cloud, filename)
            print(f"Saved {filename}")
            counter += 1

