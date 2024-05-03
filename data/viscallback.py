

import open3d as o3d
from pynput import keyboard



   
point_cloud = o3d.io.read_point_cloud('point_cloud.pcd')

def refresh_visualization(vis):
    # This function will be called when the key is pressed.
    # You can add your code here to update the visualization.
    print("The visualization has been refreshed!")
    #vis = o3d.visualization.VisualizerWithVertexSelection()
    print(type(vis))
    vis.update_geometry(point_cloud)
    vis.update_renderer()
    vis.poll_events()
    vis.run()
    return False  # Return False to keep the window open
#vis = o3d.visualization.VisualizerWithVertexSelection()
# Create a VisualizerWithKeyCallback instance
vis = o3d.visualization.VisualizerWithKeyCallback()

# Create a window
vis.create_window()

# Load your point cloud or geometry
# Replace 'your_point_cloud.ply' with the path to your point cloud file
#point_cloud = o3d.io.read_point_cloud('point_cloud.pcd')
vis.add_geometry(point_cloud)

# Register the key callback for the 'R' key to refresh the visualization
vis.register_key_callback(ord('R'), refresh_visualization)

# Run the visualization
vis.run()



