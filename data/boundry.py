import open3d as o3d
point_cloud = o3d.io.read_point_cloud('point_cloud_full.pcd')
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add original point cloud and boundary points to the visualizer
vis.add_geometry(point_cloud)

# Render the scene
vis.run()

# Close the visualizer window
vis.destroy_window()

ply_point_cloud = o3d.data.DemoCropPointCloud()
pcd = o3d.t.io.read_point_cloud(ply_point_cloud.point_cloud_path)

boundarys, mask = pcd.compute_boundary_points(0.02, 30)
# TODO: not good to get size of points.
print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd.point.positions.shape[0]} points.")

boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
pcd = pcd.paint_uniform_color([0.6, 0.6, 0.6])
o3d.visualization.draw_geometries([pcd.to_legacy(), boundarys.to_legacy()],
                                      zoom=0.3412,
                                      front=[0.3257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

pcd_path = 'point_cloud_full.pcd'

# Read point cloud from your PCD file
pcd = o3d.io.read_point_cloud(pcd_path)
#pcd.estimate_normals(pcd, knn=30)
# Compute boundary points
boundarys, mask = pcd.compute_boundary_points(0.02, 30)

print(f"Detected {boundarys.point.positions.shape[0]} boundary points from {pcd.point.positions.shape[0]} points.")

# Color boundary points in red and original point cloud in gray
boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
pcd = pcd.paint_uniform_color([0.6, 0.6, 0.6])

vis = o3d.visualization.Visualizer()
vis.create_window()

# Add original point cloud and boundary points to the visualizer
vis.add_geometry(boundarys)
vis.add_geometry(pcd)

# Render the scene
vis.run()

# Close the visualizer window
vis.destroy_window()