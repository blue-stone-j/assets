import open3d as o3d
import numpy as np

def adjust_and_save_pcd(pcd_file, output_file):
    # 加载PCD文件
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 将点云转换为NumPy数组
    points = np.asarray(pcd.points)

    # 将每个点的z值减小300
    points[:, 0] += 0
    points[:, 1] += 0
    points[:, 2] += 5
    print(points[:, 0])
    print(points[:, 1])
    print(points[:, 2])

    # 创建新的点云对象
    adjusted_pcd = o3d.geometry.PointCloud()
    adjusted_pcd.points = o3d.utility.Vector3dVector(points)

    # 保存调整后的点云到新的PCD文件
    o3d.io.write_point_cloud(output_file, adjusted_pcd)
    print(f"Adjusted point cloud saved to {output_file}")

# 示例用法
adjust_and_save_pcd("slope.pcd", "slope.pcd")

