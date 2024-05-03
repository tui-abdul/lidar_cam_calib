import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np

def on_value_changed(slider, value):
    #print(f"Value of {slider.set_text()} changed to {value}")
    print(value)

def main():
    app = gui.Application.instance
    app.initialize()

    window = app.create_window("Open3D Sliders", width=640, height=480)
    w = window
    em = w.theme.font_size
    margin = 0.5 * em

    layout = gui.Vert(0, gui.Margins(margin))
    window.add_child(layout)

    for i in range(1, 7):
        slider = gui.Slider(gui.Slider.DOUBLE)
        slider.set_limits(0.0, 100.0)
        #slider.double_value(50.0)
        slider.set_on_value_changed(lambda value: on_value_changed(slider, value))
        #slider.text = f"Slider {i}"
        layout.add_child(slider)

    # Load point cloud
    point_cloud = o3d.io.read_point_cloud('point_cloud.pcd')

    # Create a SceneWidget to display the point cloud
    scene = gui.SceneWidget()
    scene.scene = rendering.Open3DScene(window.renderer)
    material = rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    scene.scene.add_geometry("point_cloud", point_cloud,material)
    center = point_cloud.get_center()
    bbox = point_cloud.get_axis_aligned_bounding_box()
    fov = 60.0
    camera_position = center + np.array([0, 0, bbox.get_max_extent() * 1.5])
    scene.setup_camera(fov, bbox, camera_position)

    # Add the SceneWidget to the window
    layout.add_child(scene)

    app.run()

if __name__ == "__main__":
    main()
