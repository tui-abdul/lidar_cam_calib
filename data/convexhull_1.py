import numpy as np
from scipy.spatial import ConvexHull
import open3d as o3d

# Function to read PCD file and return 3D points
def read_pcd(filename):
    pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(pcd.points)
    return points

# Read PCD file
filename = "point_cloud_full.pcd"
points = read_pcd(filename)

'''
# Compute convex hull
hull = ConvexHull(points)

# Extract boundary points
boundary_points = points[hull.vertices]

# Visualize the convex hull
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(boundary_points)
o3d.visualization.draw_geometries([pcd])
'''
import numpy as np
from scipy.spatial import ConvexHull
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def find_rectangle_corners(points):
    # Compute centroid (mean) of the 3D points
    centroid = np.mean(points, axis=0)
    
    # Center the points around the centroid
    centered_points = points - centroid
    
    # Perform PCA to find the principal directions (vectors) of the point cloud
    pca = PCA(n_components=2)
    pca.fit(centered_points)
    principal_components = pca.components_
    
    # Project the centered points onto the plane defined by the two principal directions
    projected_points = np.dot(centered_points, principal_components.T)
    
    # Find convex hull of the projected points
    hull = ConvexHull(projected_points)
    
    # Extract convex hull vertices as rectangle corners
    corners = projected_points[hull.vertices]
    print("corners",corners.shape)
    # Transform back to the original coordinate system
    rotated_corners = np.dot(corners, principal_components) + centroid
    
    return rotated_corners



# Find rectangle corners
corners = find_rectangle_corners(points)
#   print("corners",corners.shape)
# Plot 3D points and rectangle corners
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', label='3D Points')
ax.scatter(corners[:, 0], corners[:, 1], corners[:, 2], c='r', s=100, label='Rectangle Corners')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Find Rectangle Corners in 3D Points')
ax.legend()
plt.show()
