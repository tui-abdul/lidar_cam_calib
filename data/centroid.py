import open3d as o3d
import numpy as np
# Load the 3D points from the PCD file
pcd = o3d.io.read_point_cloud("point_cloud_full.pcd")

# Compute the centroid of the loaded points
points = np.asarray(pcd.points)
print(points)
# Calculate centroid
centroid = np.mean(points, axis=0)
print(centroid)
# Create a point cloud for the centroid
centroid_pcd = o3d.geometry.PointCloud()
centroid_pcd.points = o3d.utility.Vector3dVector([centroid])

# Enlarge the centroid point for better visibility
centroid_pcd.paint_uniform_color([1, 0, 0])  # Color the centroid point red
centroid_pcd = centroid_pcd.voxel_down_sample(voxel_size=0.1)  # Enlarge the centroid point
axis_length = 1.0
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length)
# Visualize the centroid point
o3d.visualization.draw_geometries([pcd, centroid_pcd,axes], window_name="Centroid Visualization", width=800, height=600)
