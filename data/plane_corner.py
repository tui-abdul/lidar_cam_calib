import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import pcl
#from pclpy import pcl

def plane_segmentation():
   
    pcd = o3d.io.read_point_cloud("point_cloud.pcd")
    print(np.array(pcd.points))
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print('pcd',pcd)
    print('inliers',inliers,len(inliers))
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([inlier_cloud])# , outlier_cloud])#,
                                    #zoom=0.8,
                                    #front=[-0.4999, -0.1659, -0.8499],
                                    #lookat=[2.1813, 2.0619, 2.0999],
                                    #up=[0.1204, -0.9852, 0.1215])
    return plane_model
def planar_patch_detection(planer):
    
    #pcd = o3d.io.read_point_cloud("point_cloud.pcd")
    pcd = o3d.utility.Vector3dVector(planer)
    #print(type(pcd))
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = pcd
    print(point_cloud.points)
    assert (point_cloud.has_normals())

    # using all defaults
    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=60,
        coplanarity_deg=75,
        outlier_ratio=0.75,
        min_plane_edge_length=0,
        min_num_points=0,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    print(oboxes[0])
    print("Detected {} patches".format(len(oboxes)))

    geometries = []
    for obox in oboxes:
        mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
        mesh.paint_uniform_color(obox.color)
        geometries.append(mesh)
        geometries.append(obox)
    geometries.append(pcd)

    o3d.visualization.draw_geometries(geometries,
                                    zoom=0.62,
                                    front=[0.4361, -0.2632, -0.8605],
                                    lookat=[2.4947, 1.7728, 1.5541],
                                    up=[-0.1726, -0.9630, 0.2071])
def cluster_points():
    
    pcd = o3d.io.read_point_cloud("point_cloud.pcd")

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])#,
                                    #zoom=0.455,
                                    #front=[-0.4999, -0.1659, -0.8499],
                                    #lookat=[2.1813, 2.0619, 2.0999],
                                    #up=[0.1204, -0.9852, 0.1215])
def pick_points(file):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    pcd = o3d.io.read_point_cloud("point_cloud.pcd")
    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def plane_fitting():
    

    # Load the board point cloud (replace "board_point_cloud.pcd" with your file name)
    board_cloud = o3d.io.read_point_cloud("point_cloud.pcd")
    board_points = np.asarray(board_cloud.points)
    print('board',board_points.shape)
    # Define the plane equation
    plane_equation = np.array([0.85, -0.45, -0.29, -1.82])

    # Normalize the plane equation
    plane_normal = plane_equation[:3] / np.linalg.norm(plane_equation[:3])
    plane_d = plane_equation[3] / np.linalg.norm(plane_equation[:3])

    # Project each point onto the plane
    projected_points = []
    for point in board_points:
        distance = np.dot(plane_normal, point) - plane_d
        projected_point = point - distance * plane_normal
        projected_points.append(projected_point)
    #print("11",projected_points)
    # Convert the projected points back to Open3D format

    projected_cloud = o3d.geometry.PointCloud()
    projected_cloud.points = o3d.utility.Vector3dVector(np.asarray(projected_points))

    # Plot the projected points
    #o3d.visualization.draw_geometries([projected_cloud])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(projected_cloud)
    
    vis.run()
    vis.destroy_window()
    pick_points_index = vis.get_picked_points()
    print('picked_points',pick_points_index)
    points = np.asarray(projected_cloud.points)[pick_points_index] 
    print(points)
    #return np.asarray(projected_cloud.points)
    return points

def line(cloud_np):

    # Load point cloud data
    cloud = pcl.PointCloud(cloud_np.astype('float32'))
    # Create the segmentation object
    seg = cloud.make_segmenter()
    #seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_LINE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_MaxIterations(10)
    seg.set_distance_threshold(0.01)
    indices, model = seg.segment()
    # Extract line inliers
    # Extract the line points
    cloud_line = cloud.extract(indices, negative=True)
    print(model)
    print(indices)
    print(np.asarray(cloud_line).shape)
    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(np.asarray(cloud_line))

    # Visualize the line points
    o3d.visualization.draw_geometries([cloud_o3d])





def organise_pc(pf):

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pf)

    # Create a KDTree from the point cloud
    kdtree = o3d.geometry.KDTreeFlann(point_cloud)

    # Define the number of scan beams
    num_scan_beams = 5

    # Divide the points into scan beams
    # You can adjust the division method according to your specific requirements
    scan_beam_indices = np.linspace(0, len(pf), num_scan_beams + 1, dtype=int)

    # Organize points into scan beams
    scan_beams = []
    for i in range(num_scan_beams):
        # Extract points within each scan beam
        points_in_beam = []
        for j in range(scan_beam_indices[i], scan_beam_indices[i + 1]):
            [_, idx, _] = kdtree.search_knn_vector_3d(pf[j], 2)
            points_in_beam.append(pf[idx[0]])
        scan_beams.append(np.array(points_in_beam))
    projected_cloud = o3d.geometry.PointCloud()
    projected_cloud.points = o3d.utility.Vector3dVector(np.asarray(scan_beams[1] ))
    o3d.visualization.draw_geometries([ projected_cloud] )
    # Convert the list of scan beams to a numpy array
    #scan_beams = np.array(scan_beams)

def line_ransac(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Fit lines using RANSAC
    lines = []

    while len(points) > 50:  # Loop until fewer than 50 points are left
        # Estimate normals
        pcd.estimate_normals()

        # Compute distances from points to estimated normals
        distances = pcd.compute_nearest_neighbor_distance()

        # Find the point with the largest median distance as the seed
        #seed = np.argmax(distances)

        # Use RANSAC to fit a line
        _, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

        if len(inliers) < 50:
            break

        # Extract inliers
        inlier_cloud = pcd.select_by_index(inliers)
        line_model, _ = inlier_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

        # Extract line parameters
        line_start = line_model[0]
        line_end = line_model[1]
        lines.append([line_start, line_end])

        # Remove inliers from point cloud
        pcd = pcd.select_by_index(inliers, invert=True)

    # Visualize
    o3d.visualization.draw_geometries([pcd, o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(lines))])


def find_edge_points(projected_points):
    edge_points = []
    for scan_beam in projected_points:
        min_y_point = scan_beam[np.argmin(scan_beam[1]), :]  # Finding point with minimum y-coordinate
        max_y_point = scan_beam[np.argmax(scan_beam[1]), :]  # Finding point with maximum y-coordinate
        edge_points.extend([min_y_point, max_y_point])
    return edge_points



# Step 6: Finding Corner Points
def fit_lines(edge_points):
    corner_points = []
    for i in range(0, len(edge_points), 2):  # Processing pairs of edge points
        x_coords = [edge_points[i][0], edge_points[i+1][0]]
        y_coords = [edge_points[i][1], edge_points[i+1][1]]
        line_coeffs = np.polyfit(x_coords, y_coords, 1)  # Fitting a line through the edge points
        corner_points.append((x_coords[0], np.polyval(line_coeffs, x_coords[0])))  # Adding the corner point
    return corner_points



# Step 7: Calculating Diagonal Midpoint
def calculate_diagonal_midpoint(corner_points):
    diagonal_midpoint = ((corner_points[0][0] + corner_points[2][0]) / 2, 
                         (corner_points[0][1] + corner_points[2][1]) / 2)
    return diagonal_midpoint






if __name__ == '__main__':
    #planar_patch_detection()
    plane_model = plane_segmentation() # plane equation
    #cluster_points()
    # Start subscriber
    #pick_points("point_cloud.pcd")
    #projected_points = plane_fitting()
    #line(projected_points)
    #organise_pc(projected_points)
    #line_ransac(projected_points)
    #projected_points = np.asarray(projected_points)
    #planar_patch_detection(projected_points)
    #print("projected points",projected_points)
    #edge_points = find_edge_points(projected_points)
    #corner_points = fit_lines(edge_points)
    #diagonal_midpoint = calculate_diagonal_midpoint(corner_points)
    '''
    # Creating Open3D point cloud
    point_cloud = o3d.geometry.PointCloud()
    points = np.array(projected_points).reshape(-1, 3)
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Creating Open3D lines for fitted lines
    lines = []
    for edge_point_pair in zip(edge_points[::2], edge_points[1::2]):
        lines.append([edge_point_pair[0], edge_point_pair[1]])
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.array(lines).reshape(-1, 3)),
        lines=o3d.utility.Vector2iVector(np.array([[i, i+1] for i in range(0, len(lines)*2, 2)]))
    )

    # Creating Open3D arrow for normal vector
    normal_vector = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.2, cone_height=0.1
    )
    normal_vector.transform(np.array([[1, 0, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]]))  # Adjusting direction to match coordinate system
    normal_vector.translate(np.mean(points, axis=0))  # Translating to the centroid of the point cloud

    # Creating Open3D sphere for diagonal midpoint
    diagonal_midpoint_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
    diagonal_midpoint_sphere.translate(diagonal_midpoint)

    # Visualization
    o3d.visualization.draw_geometries([point_cloud, line_set, normal_vector, diagonal_midpoint_sphere]) 
    '''