
import time
import math
import subprocess
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QSlider
from PyQt5.QtGui import QPixmap, QPainter, QPen
from PyQt5.QtCore import Qt, QPoint


class DrillingConfig(QWidget):
    def __init__(self, parent=None):
        super(DrillingConfig, self).__init__(parent)
        self.layout = QVBoxLayout(self)

        self.length_label = QLabel('Workpiece Length:')
        self.length_input = QLineEdit()
        self.layout.addWidget(self.length_label)
        self.layout.addWidget(self.length_input)

        self.width_label = QLabel('Workpiece Width:')
        self.width_input = QLineEdit()
        self.layout.addWidget(self.width_label)
        self.layout.addWidget(self.width_input)

        
        self.configure_button = QPushButton('Configure Workpiece')
        self.configure_button.clicked.connect(self.configure_workpiece)
        self.layout.addWidget(self.configure_button)

        self.randomize_button = QPushButton('Randomize Geometry and Position')
        self.randomize_button.clicked.connect(self.randomize_geometry)
        self.layout.addWidget(self.randomize_button)
        
        self.set_holes_button = QPushButton('Set Holes')
        self.set_holes_button.clicked.connect(self.set_holes)
        self.layout.addWidget(self.set_holes_button)
        
        self.put_robot_button = QPushButton('Put_Drilling_Robot')
        self.put_robot_button.clicked.connect(self.put_robot)
        self.layout.addWidget(self.put_robot_button)
        
        self.do_drilling = QPushButton('Execute Drilling')
        self.do_drilling.clicked.connect(self.execute_drilling)
        self.layout.addWidget(self.do_drilling)
        # Store selected position
        self.selected_position = None


    
    def configure_workpiece(self):
        length_text = self.length_input.text()
        width_text = self.width_input.text()
        # yaw_degrees = self.yaw_slider.value() * 20
        # yaw_radians = math.radians(yaw_degrees)
        # position_index = self.selected_position[0] * 3 + self.selected_position[1] + 1

        if not length_text or not width_text:
            QMessageBox.warning(self, "Input Error", "Please enter both length and width.")
            return
        try:
            length = float(length_text)
            width = float(width_text)
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Length and width must be numeric values.")
            return
        command1 = ['rosrun', 'panda_gazebo', 'configure_geometry.py', str(length), str(width)]
        # command2 = ['rosrun', 'panda_gazebo', 'configure_workpiece_position.py', str(position_index), str(yaw_radians)]
        try:
            subprocess.run(command1, check=True)
            # subprocess.run(command2, check=True)
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {e}")
    
    
    def randomize_geometry(self):
        try:
            self.process1=subprocess.run(['rosrun', 'panda_gazebo', 'modify_geometry.py'])
            time.sleep(.01)
            self.process2=subprocess.run(['rosrun', 'panda_gazebo', 'randomize_workpiece_position.py'], check=True)
        except   subprocess.CalledProcessError as e:
            print(f"Error launching Gazebo: {e}")

    def set_holes(self):
        try:
            self.process1=subprocess.run(['rosrun', 'panda_gazebo', 'randomize_hole_position.py'])
        except   subprocess.CalledProcessError as e:
            print(f"Error launching Gazebo: {e}")
    
    def put_robot(self):
        try:
            subprocess.Popen(['rqt_plot'])
            self.put_robot_in_gazebo = subprocess.Popen(['roslaunch', 'panda_gazebo', 'put_robot_in_world.launch','load_gripper:=false','gripper:=drill'])
        except   subprocess.CalledProcessError as e:
            print(f"Error launching Gazebo: {e}")

    def execute_drilling(self):
        try:
            self.process1=subprocess.run(['rosrun', 'pick_and_place', 'drilling.py'])
        except   subprocess.CalledProcessError as e:
            print(f"Error launching Gazebo: {e}")
