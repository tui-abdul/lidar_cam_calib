import numpy as np
import open3d as o3d

def fit_rectangle_to_points_on_plane(points, width, height):
    # Compute the centroid of the 3D points
    centroid = np.mean(points, axis=0)

    # Project the 3D points onto the plane defined by the normal
    # We assume that the plane equation is already defined

    # Define the plane equation (normal vector and a point on the plane)
    normal = np.array([0, 0, 1])  # Corrected to a NumPy array
    point_on_plane = centroid  # Use the centroid as a point on the plane

    # Project the 3D points onto the plane
    projected_points = []
    for point in points:
        d = np.dot(normal, point - point_on_plane)
        projected_point = point - d * normal
        projected_points.append(projected_point)
    projected_points = np.array(projected_points)

    # Fit a rectangle to the projected points
    # Here, you can use any method to fit a rectangle to the 2D points
    # For simplicity, we'll just use the bounding box of the points
    min_x = np.min(projected_points[:, 0])
    max_x = np.max(projected_points[:, 0])
    min_y = np.min(projected_points[:, 1])
    max_y = np.max(projected_points[:, 1])

    # Define rectangle corners
    rectangle_corners = np.array([
        [min_x, min_y, 0],
        [max_x, min_y, 0],
        [max_x, max_y, 0],
        [min_x, max_y, 0]
    ])

    return rectangle_corners

# Example noisy boundary 3D points
np.random.seed(0)
n_points = 100
boundary_points = np.random.rand(n_points, 3) * 10  # Random points in [0, 10) range
boundary_points = np.load('boundary_points.npy')
# Define rectangle dimensions
width = 0.607
height = 0.85

# Fit rectangle to points on the plane
rectangle_corners = fit_rectangle_to_points_on_plane(boundary_points, width, height)

# Visualize the result using Open3D
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(boundary_points)

# Create a LineSet to represent the rectangle
lines = [[0, 1], [1, 2], [2, 3], [3, 0]]
colors = [[1, 0, 0] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(rectangle_corners),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)

# Visualize
o3d.visualization.draw_geometries([pcd, line_set], window_name="Rectangle Fitting", width=800, height=600)


# Visualize
o3d.visualization.draw_geometries([pcd, line_set], window_name="Rectangle Fitting", width=800, height=600)




#boundary_points = np.load('boundary_points.npy')




