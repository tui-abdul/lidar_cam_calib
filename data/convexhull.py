import open3d as o3d
import numpy as np

# Read the PCD file
pcd = o3d.io.read_point_cloud("point_cloud_full.pcd")

# Compute the convex hull
hull, _ = pcd.compute_convex_hull()

# Extract the boundary points from the convex hull
boundary_points = []

for i, face in enumerate(hull.triangles):
    for j in range(3):
        vertex_index = face[j]
        vertex_point = hull.vertices[vertex_index]
        
        # Check if the vertex is on the boundary
        is_boundary_vertex = True
        for other_face in hull.triangles:
            if i != np.where(hull.triangles == vertex_index)[0][0]:
                if vertex_index in other_face:
                    is_boundary_vertex = False
                    break
        
        if is_boundary_vertex:
            boundary_points.append(vertex_point)
np.save('boundary_points.npy',boundary_points)

# Convert the list of boundary points to an Open3D point cloud
boundary_pcd = o3d.geometry.PointCloud()
boundary_pcd.points = o3d.utility.Vector3dVector(boundary_points)

# Color the boundary points red
boundary_colors = np.tile([1, 0, 0], (len(boundary_points), 1))
boundary_pcd.colors = o3d.utility.Vector3dVector(boundary_colors)

# Visualize the original point cloud and boundary points
o3d.visualization.draw_geometries([ boundary_pcd])
