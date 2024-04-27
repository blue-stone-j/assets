import open3d as o3d
import numpy as np

def colorize_point_cloud_by_z(file_path):
    # 加载点云
    point_cloud = o3d.io.read_point_cloud(file_path)

    # 获取点云中所有点的 Z 坐标
    points = np.asarray(point_cloud.points)
    z_coords = points[:, 2]

    # 计算 Z 的最小值和最大值
    z_min, z_max = np.min(z_coords), np.max(z_coords)

    # 计算每个点的颜色
    colors = np.zeros((len(z_coords), 3))  # 初始化颜色数组
    for i, z in enumerate(z_coords):
        # 归一化 Z 值
        normalized_z = (z - z_min) / (z_max - z_min)
        # 创建渐变颜色，从蓝色到红色
        colors[i] = [normalized_z, 1 - normalized_z, 0]  # 红色和蓝色渐变

    # 设置点云的颜色
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0, 0, 0])

    # 创建并添加XOY平面网格
    grid = o3d.geometry.TriangleMesh.create_box(width=2, height=2, depth=0.01)  # 创建一个薄盒子作为网格
    grid.translate([-1, -1, -0.002])  # 将网格中心移动到原点附近的XOY平面
    grid.paint_uniform_color([0.5, 0.5, 0.5])  # 设置网格的颜色

    # Here are two methods to visualize point cloud below 

    # method1
    # 可视化点云
    # o3d.visualization.draw_geometries([point_cloud, axis, grid])

    # method2
    # 初始化 GUI 应用
    app = o3d.visualization.gui.Application.instance
    app.initialize()

    # 创建可视化窗口
    vis = o3d.visualization.O3DVisualizer("3D Visualizer", 1024, 768)
    vis.add_geometry("Point Cloud", point_cloud)
    vis.add_geometry("Coordinate Frame", axis)
    # fov(angle), eye(camera position), front(face to this position), up(camera up direction)
    vis.setup_camera(60, [20, 0, -20], [0, 0, 0], [0, 0, 1])
    vis.add_geometry("XOY Grid", grid)

    # 显示窗口
    app.add_window(vis)
    app.run()

# 指定 PCD 文件的路径
file_path = 'flat.pcd'
colorize_point_cloud_by_z(file_path)
