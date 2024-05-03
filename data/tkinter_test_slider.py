import open3d as o3d
import tkinter as tk
from tkinter import ttk
import numpy as np

# Initialize tkinter root window
root = tk.Tk()
root.title("Open3D with Tkinter")

# Initialize Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()

# Read point cloud from a PCD file
point_cloud = o3d.io.read_point_cloud('point_cloud.pcd')
vis.add_geometry(point_cloud)

# Create a tkinter slider
slider_label = ttk.Label(root, text="Point Cloud Density")
slider_label.pack(pady=20)

slider = ttk.Scale(root, from_=10, to=100, orient=tk.HORIZONTAL)
slider.pack(pady=20)

# Function to update the point cloud density
def update_point_cloud_density(value):
    density = int(slider.get())
    
    # Generate random point cloud data based on the density
    points = np.random.rand(density, 3)
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    # Update the point cloud geometry and render the scene
    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()

# Bind the slider's value change event to the update_point_cloud_density function
slider.bind("<ButtonRelease-1>", lambda event: update_point_cloud_density(slider.get()))

# Initial point cloud rendering
vis.poll_events()
vis.update_renderer()

# Start the tkinter main loop
root.mainloop()

# Close Open3D visualizer window when the tkinter window is closed
vis.destroy_window()
