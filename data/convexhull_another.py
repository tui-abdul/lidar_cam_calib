import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import open3d as o3d
import pyransac3d as pyrsc
from scipy.spatial import HalfspaceIntersection
# Generate some random 3D points
#np.random.seed(0)
#points = np.random.rand(20, 3)

def line_detect(points, all_points):
        line = pyrsc.Line()
        print("points",points.shape)
        # Fit line to the data
        A, B, inliers = line.fit(points, thresh=1, maxIteration=100)

        print(f"A (3D slope of the line): {A}")
        print(f"B (Axis interception): {B}")

        # Convert inliers to Open3D point cloud
        inlier_points = points[inliers]
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(all_points)

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
        return A, B

def select_line():
    pcd = o3d.io.read_point_cloud("point_cloud_full.pcd")
    points = np.asarray(pcd.points)
    # Create convex hull
    print(points.shape)
    hull = ConvexHull(points)
    print(points[ hull.vertices].shape )
    # Plotting
    boundary_points = points[ hull.vertices]

    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    boundary_pcd = o3d.geometry.PointCloud()
    boundary_pcd.points = o3d.utility.Vector3dVector(points[ hull.vertices])
    # Add original point cloud and boundary points to the visualizer
    vis.add_geometry( boundary_pcd)
    #vis.add_geometry(pcd)
    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    # Render the scene
    vis.run()
    vis.destroy_window()
    # Close the visualizer window

    pick_points_index = vis.get_picked_points()
    obj_points = np.zeros((len(pick_points_index),3))

    for i,point_obj in enumerate(pick_points_index):
        obj_points[i]  = boundary_points[point_obj.index,:] 
    return obj_points, points



def find_intersection_point(slope1, intercept1, slope2, intercept2):
    # Define the halfspaces corresponding to the lines
    halfspace1 = [[0, -1, -slope1, intercept1], [0, 1, slope1, -intercept1]]
    halfspace2 = [[0, -1, -slope2, intercept2], [0, 1, slope2, -intercept2]]

    # Compute the intersection of the halfspaces
    hs = HalfspaceIntersection(np.vstack((halfspace1, halfspace2)), np.zeros(4))
    intersection_point = hs.intersections[0]

    return intersection_point











def euclidean_distance(point1, point2):
    """
    Calculate the Euclidean distance between two 3D points.
    
    Parameters:
        point1 (tuple or list): Coordinates of the first point (x, y, z).
        point2 (tuple or list): Coordinates of the second point (x, y, z).
    
    Returns:
        float: Euclidean distance between the two points.
    """
    point1 = np.array(point1)
    point2 = np.array(point2)
    distance = np.sqrt(np.sum((point2 - point1)**2))
    return distance




# Find intersection point


obj_points, points = select_line()

A1,B1 = line_detect(obj_points,points)
obj_points, points = select_line()
A2,B2 = line_detect(obj_points,points)

obj_points, points = select_line()

A3,B3 = line_detect(obj_points,points)
obj_points, points = select_line()
A4,B4 = line_detect(obj_points,points)

x1,y1,z1 = find_intersection_point(A1, B1, A2, B2)
x2,y2,z2 = find_intersection_point(A2, B2, A3, B3)
x3,y3,z3 = find_intersection_point(A3, B3, A4, B4)
x4,y4,z4 = find_intersection_point(A4, B4, A1, B1)

corners =[[x4,y4,z4],[ x1,y1,z1],[x2,y2,z2],[x3,y3,z3]] 

#corners, points = select_line()
print(corners)
print('W',euclidean_distance(corners[0], corners[1] ))
print('right H',euclidean_distance(corners[1], corners[2] ))
print('left H',euclidean_distance(corners[0], corners[3] ))
print('W',euclidean_distance(corners[3], corners[2] ))
print(corners)

rectangle = o3d.geometry.PointCloud()
rectangle.points = o3d.utility.Vector3dVector(corners)

# Visualize the rectangle
o3d.visualization.draw_geometries([rectangle])