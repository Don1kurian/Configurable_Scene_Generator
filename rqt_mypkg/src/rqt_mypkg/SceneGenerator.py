import subprocess

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from drilling_config import DrillingConfig

class SceneGenerationHMIPlugin(Plugin):

    def __init__(self, context):
        super(SceneGenerationHMIPlugin, self).__init__(context)
        self.setObjectName('SceneGenerationHMIPlugin')

        # Define page identifiers
        self.PAGE_SCENE_SELECTION = 0
        self.PAGE_DRILLING_CONFIG = 1
        # self.PAGE_WELDING_CONFIG = 2

        # Initialize current page
        self.current_page = self.PAGE_SCENE_SELECTION

        # Initialize Gazebo process variable
        self.gazebo_process = None

        # Create QWidget
        self._widget = QWidget()
        self.layout = QVBoxLayout()
        self._widget.setLayout(self.layout)

        # Add label for scene selection
        self.scene_select_label = QLabel('Select Scene Configuration:')
        self.layout.addWidget(self.scene_select_label)

        # Add buttons for scene selection
        self.welding_button = QPushButton('Welding Scene')
        self.welding_button.clicked.connect(self.show_drilling_config)
        self.layout.addWidget(self.welding_button)

        self.drilling_button = QPushButton('Drilling Scene')
        self.drilling_button.clicked.connect(self.show_drilling_config)
        self.layout.addWidget(self.drilling_button)

        # Add button to kill Gazebo
        self.kill_gazebo_button = QPushButton('Kill Gazebo')
        self.kill_gazebo_button.clicked.connect(self.kill_gazebo)
        self.kill_gazebo_button.hide()  # Initially hidden
        self.layout.addWidget(self.kill_gazebo_button)

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Show initial page
        self.show_scene_selection()

    def show_scene_selection(self):
        # Show scene selection page
        self.scene_select_label.setText('Select Scene Configuration:')
        # self.welding_button.show()
        self.drilling_button.show()
        # Hide kill Gazebo button
        self.kill_gazebo_button.hide()
        # Terminate Gazebo if it's running
        if self.gazebo_process:
            self.gazebo_process.terminate()

        # Update current page
        self.current_page = self.PAGE_SCENE_SELECTION

    def show_drilling_config(self):
        # Show drilling configuration page
        self.scene_select_label.setText('Drilling Configuration:')
        # self.welding_button.hide()
        self.drilling_button.hide()
        self.welding_button.hide()
        # Show kill Gazebo button
        self.kill_gazebo_button.show()
        # Launch Gazebo
        try:
            self.gazebo_process = subprocess.Popen(['roslaunch', 'panda_gazebo', 'start_workscene.launch'])
        except subprocess.CalledProcessError as e:
            print(f"Error launching Gazebo: {e}")
        drilling_config = DrillingConfig()
        self.layout.addWidget(drilling_config)
        # Update current page
        self.current_page = self.PAGE_DRILLING_CONFIG


    def kill_gazebo(self):
        # Kill Gazebo process
        if self.gazebo_process:
            self.gazebo_process.terminate()
            print("Gazebo process terminated.")
        else:
            print("No Gazebo process running.")

    def shutdown_plugin(self):
        # Terminate Gazebo if it's running
        if self.gazebo_process:
            self.gazebo_process.terminate()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
