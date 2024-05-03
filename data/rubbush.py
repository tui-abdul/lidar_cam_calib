import open3d as o3d
import numpy as np

a = np.random.randn(100,3) 

PC = o3d.geometry.PointCloud()
PC.points = o3d.utility.Vector3dVector(a)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(PC)
ctr = vis.get_view_control()  # Everything good
vis.run()
vis.close()

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(PC)
ctr = vis.get_view_control()  # ERROR: [Open3D ERROR] GLFW Error: The GLFW library is not initialized
vis.run() 
