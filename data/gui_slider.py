import glob
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import os
import platform
import sys

def scale_value(value, min_in, max_in, min_out, max_out):
    scaled_value = ((value - min_in) / (max_in - min_in)) * (max_out - min_out) + min_out
    return scaled_value

def call_back(number):
    print('hello', scale_value(number,0,1000,-3,3))

def call_back1(number):
    print('hello', scale_value(number,0,1000,-3,3))

#pcd = o3d.io.read_point_cloud("point_cloud.pcd")
#vis =o3d.visualization.Visualizer()

#vis.create_window()
#vis.add_geometry(pcd)
#opt = vis.get_render_option()
#opt.show_coordinate_frame = True
gui.Application.instance.initialize()

window = gui.Application.instance.create_window("Test", 200, 400)


w = window  # for more concise code


em = w.theme.font_size
vgrid = gui.VGrid(2, 0)  # 1 column, 0 spacing between items

# Add the slider to the layout

slider = gui.Slider(gui.Slider.INT) 
slider.set_limits(0, 1000)
slider.double_value = 50
slider.set_on_value_changed(call_back)

slider1 = gui.Slider(gui.Slider.INT)
slider1.set_limits(0, 1000)
slider1.set_on_value_changed(call_back1)
window.add_child(vgrid)
window.add_child(slider)
#window.add_child(slider)
gui.Application.instance.run()
exit()
#print(pick_points_index[0].coord  )
#gui.Application.instance.destroy_window()