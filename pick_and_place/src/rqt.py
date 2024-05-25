from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton
from python_qt_binding.QtCore import Qt
import subprocess

class RosrunControlPlugin(Plugin):
    def __init__(self, context):
        super(RosrunControlPlugin, self).__init__(context)
        self.setObjectName('RosrunControlPlugin')

        self._widget = QWidget()
        self._widget.setObjectName('RosrunControlWidget')

        self._layout = QVBoxLayout(self._widget)

        # Create buttons for different rosrun commands
        self._button1 = QPushButton('Run Node 1', self._widget)
        self._button2 = QPushButton('Run Node 2', self._widget)
        # Add more buttons as needed

        # Connect buttons to corresponding functions
        self._button1.clicked.connect(self.run_node_1)
        self._button2.clicked.connect(self.run_node_2)
        # Connect more buttons to corresponding functions

        # Add buttons to layout
        self._layout.addWidget(self._button1)
        self._layout.addWidget(self._button2)
        # Add more buttons to layout as needed

        context.add_widget(self._widget)

    def run_node_1(self):
        subprocess.Popen(['rosrun', 'your_package_name', 'node_1'])

    def run_node_2(self):
        subprocess.Popen(['rosrun', 'your_package_name', 'node_2'])
    # Add more functions for other buttons as needed
