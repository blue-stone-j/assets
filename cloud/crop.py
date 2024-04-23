import open3d as o3d
import numpy as np

def filter_points(pcd_file, output_file):
    # 加载PCD文件
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 将点云转换为NumPy数组
    points = np.asarray(pcd.points)

    # 计算点云中每个点的距离原点的水平距离
    horizontal_distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)

    # 设置过滤条件：x > 0 且水平距离小于20米
    mask = (points[:, 0] > 0) & (horizontal_distances < 20)

    # 应用过滤条件
    filtered_points = points[mask]

    # 创建新的点云对象
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    # 保存过滤后的点云到新的PCD文件
    o3d.io.write_point_cloud(output_file, filtered_pcd)
    print(f"Filtered point cloud saved to {output_file}")

# 示例用法
filter_points("slope.pcd", "slope1.pcd")
