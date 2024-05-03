import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np


def on_value_changed(slider, value):
    #print(f"Value of {slider.set_text()} changed to {value}")

    print(slider.double_value)

def main_slider():
    app = gui.Application.instance
    app.initialize()

    window = app.create_window("Open3D Sliders", width=640, height=400)
    w = window
    em = w.theme.font_size
    margin = 0.5 * em

    layout = gui.Vert(0, gui.Margins(margin))
    window.add_child(layout)

    #for i in range(1, 7):
    slider = gui.Slider(gui.Slider.DOUBLE)
    slider.set_limits(-3.0, +3.0)
    value = slider.double_value
    slider.set_on_value_changed(lambda value: on_value_changed(slider, value))
    #slider.text = f"Slider {i}"
    
    layout.add_child(slider)
    print('value',value)


    app.run()
    return value

if __name__ == "__main__":
    main_slider()
