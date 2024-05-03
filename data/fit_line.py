import numpy as np
import pyransac3d as pyrsc
import open3d as o3d

# Generate some sample points
np.random.seed(0)
points = np.random.rand(100, 3)  # 100 random points in 3D space


# Create a Line object
line = pyrsc.Line()

# Fit line to the data
A, B, inliers = line.fit(points, thresh=0.2, maxIteration=1000)

print(f"A (3D slope of the line): {A}")
print(f"B (Axis interception): {B}")

# Convert inliers to Open3D point cloud
inlier_points = points[inliers]
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(inlier_points)

# Create line segment for visualization
line_segment = o3d.geometry.LineSet()
line_points = np.vstack([B, B + A])
line_segment.points = o3d.utility.Vector3dVector(line_points)
line_segment.lines = o3d.utility.Vector2iVector([[0, 1]])

# Create visualizer
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()

# Add point cloud and line segment to visualizer
visualizer.add_geometry(point_cloud)
visualizer.add_geometry(line_segment)

# Run visualizer
visualizer.run()

# Close visualizer
visualizer.destroy_window()
