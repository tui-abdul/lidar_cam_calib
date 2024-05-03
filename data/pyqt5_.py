import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QSlider, QLabel, QPushButton

class SliderGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.update_slider = {} 

        self.setWindowTitle("Slider GUI")
        self.setGeometry(100, 100, 600, 300)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Slider names
        self.slider_names = [
            "X Upper Bound",
            "X Lower Bound",
            "Y Upper Bound",
            "Y Lower Bound",
            "Z Upper Bound",
            "Z Lower Bound"
        ]

        # Create six sliders
        self.sliders = []
        for name in self.slider_names:
            slider = QSlider()
            slider.setOrientation(0x1)  # Qt.Horizontal
            slider.setMinimum(0)
            slider.setMaximum(600)
            slider.setValue(300)
            slider.sliderMoved.connect(self.update_label)
            layout.addWidget(slider)
            self.sliders.append(slider)

        # Create labels to display the slider values
        self.label_values = {}
        for name in self.slider_names:
            label = QLabel(f"{name}: 0.00")
            layout.addWidget(label)
            self.label_values[name] = label

        # Create an Exit button
        exit_button = QPushButton("Exit", self)
        exit_button.clicked.connect(self.on_exit)
        layout.addWidget(exit_button)

    def update_label(self):
        slider = self.sender()
        index = self.sliders.index(slider)
        name = self.slider_names[index]
        value = (slider.value() - 300) / 100
        self.label_values[name].setText(f"{name}: {value:.2f}")
        self.update_slider[name] = value 
        return self.update_slider
    def value(self):
        return self.update_slider
    def on_exit(self):
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SliderGUI()
    window.show()
    var = window.value()
    app.exec()
    print('hello',var)