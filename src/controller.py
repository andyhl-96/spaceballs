from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
import sys

class AppWindow(QWidget):
    def __init__(self):
        super().__init__()

        # Set up the basic window
        self.setWindowTitle("CONTROL PANEL")
        self.setGeometry(100, 100, 640, 960)

        # Layout setup
        self.layout = QVBoxLayout()

        # Set layout for the window
        self.setLayout(self.layout)

        self.button = QPushButton("Test", self)
        self.button.clicked.connect(self.test_button)
        self.layout.addWidget(self.button)

    def test_button(self):
        file = open("temp", "w")
        file.write("hello")
        file.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Set up the Open3D window with PyQt5
    window = AppWindow()
    window.show()

    # Start the Qt event loop
    sys.exit(app.exec_())