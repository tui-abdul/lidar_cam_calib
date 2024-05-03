import numpy as np
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
# Generate some sample 3D points (replace this with your actual data)
np.random.seed(0)
num_points = 100
points_3d = np.random.rand(num_points, 3) * 10
pcd = o3d.io.read_point_cloud("point_cloud_full.pcd")
points_3d = np.asarray(pcd.points)
# Add some outliers
outlier_indices = np.random.choice(num_points, size=num_points // 10, replace=False)
points_3d[outlier_indices] = np.random.uniform(-10, 10, size=(len(outlier_indices), 3))

# Fit RANSAC to find the best-fitting rectangle
ransac = RANSACRegressor()
ransac.fit(points_3d[:, :2], points_3d[:, 2])

# Extract the parameters of the plane
rectangle_params = ransac.estimator_.coef_
rectangle_intercept = ransac.estimator_.intercept_

# Plot the original 3D points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], c='b', marker='o', label='3D Points')

# Plot the best-fitting rectangle
x = np.linspace(points_3d[:, 0].min(), points_3d[:, 0].max(), 10)
y = np.linspace(points_3d[:, 1].min(), points_3d[:, 1].max(), 10)
X, Y = np.meshgrid(x, y)
Z = rectangle_params[0] * X + rectangle_params[1] * Y + rectangle_intercept
ax.plot_surface(X, Y, Z, alpha=0.5, color='r')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.legend()
plt.show()
