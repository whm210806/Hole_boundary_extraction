import open3d as o3d

# 读取 PCD 文件
pcd = o3d.io.read_point_cloud("/home/mike/sick_Ws/cluster_0.pcd")

# 创建一个坐标轴框架，设置尺寸和位移
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

# 可视化点云和坐标轴
o3d.visualization.draw_geometries([pcd])
