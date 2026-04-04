from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGroupBox, QLabel, QSlider,
    QPushButton, QLineEdit, QGridLayout, QTextEdit, QTabWidget
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont

import sys
from PyQt6.QtWidgets import QApplication
import qdarktheme
import pyqtgraph as pg  # Import pyqtgraph for real-time plotting

# Add ROS2 imports
import rclpy
from rclpy.node import Node
from indy_iface.msg import JointInfo
from indy_iface.srv import SetPos, SetAllPos, GetGains, SetGains
import threading

from commons import load_config, ensure_config


class ROS2Worker(QThread):
    """Background thread for ROS2 communication"""
    joint_state_updated = pyqtSignal(list)  # Signal for joint state updates
    gains_updated = pyqtSignal(list, list)  # Signal for gains updates (kp, kd)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.node = None
        
    def run(self):
        """Main thread function"""
        rclpy.init()
        self.node = Node('indy_ui_controller')
        
        # Create service clients
        self.set_pos_client = self.node.create_client(SetPos, 'set_pos')
        self.set_all_pos_client = self.node.create_client(SetAllPos, 'set_all_pos')
        self.get_gains_client = self.node.create_client(GetGains, 'get_gains')
        self.set_gains_client = self.node.create_client(SetGains, 'set_gains')
        
        # Create subscriber
        self.joint_sub = self.node.create_subscription(
            JointInfo,
            'joint_info',
            self.joint_callback,
            10
        )
        
        # Spin ROS2
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
    
    def joint_callback(self, msg):
        """Handle joint state updates"""
        positions = list(msg.actual_pos)
        self.joint_state_updated.emit(positions)
    
    def stop(self):
        """Stop the worker thread"""
        self.running = False


class JointControl(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Collaborative Robot Control")
        self.setMinimumSize(1400, 800)

        ensure_config()
        config = load_config()
        joint_config = config["UI"]["JointSpace"]

        # Create main layout with tabs
        main_layout = QVBoxLayout()
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        
        # Create tabs
        self.position_tab = self.create_position_tab(joint_config)
        self.gains_tab = self.create_gains_tab(joint_config)
        self.graphs_tab = self.create_graphs_tab()  # Add graphs tab
        
        self.tab_widget.addTab(self.position_tab, "Position Control")
        self.tab_widget.addTab(self.gains_tab, "Gains Control")
        self.tab_widget.addTab(self.graphs_tab, "Real-Time Graphs")  # Add graphs tab to UI
        
        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # Start ROS2 worker thread
        self.ros2_worker = ROS2Worker()
        self.ros2_worker.joint_state_updated.connect(self.update_joint_states)
        self.ros2_worker.start()
        
        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # Update every second
        
        self.last_update_time = None

        # Initialize data for graphs
        self.graph_data = {
            "position": [[] for _ in range(6)],
            "velocity": [[] for _ in range(6)],
            "torque": [[] for _ in range(6)],
        }

    def create_position_tab(self, joint_config):
        """Create position control tab"""
        tab_widget = QWidget()
        main_layout = QHBoxLayout()

        # Left: Joint controls
        joint_group = QGroupBox("Joint Space Control")
        joint_layout = QGridLayout()
        self.joint_sliders = []
        self.target_edits = []
        self.actual_edits = []

        # Add headers
        headers = ["Joint", "-", "Slider", "+", "Target", "Value", "Actual", "Value"]
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            joint_layout.addWidget(label, 0, col)

        for i in range(6):
            row = i + 1
            joint = joint_config.get(f"Joint{i+1}", {})
            name = joint.get("name", f"Joint {i+1}")
            min_val = joint.get("min", -180)
            max_val = joint.get("max", 180)

            label = QLabel(name)
            label.setFixedWidth(80)
            btn_minus = QPushButton("<")
            btn_plus = QPushButton(">")
            btn_minus.setFixedWidth(30)
            btn_plus.setFixedWidth(30)
            
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setSingleStep(1)
            slider.setFixedWidth(200)
            
            target_edit = QLineEdit("0.0")
            target_edit.setFixedWidth(80)
            actual_edit = QLineEdit("0.0")
            actual_edit.setFixedWidth(80)
            actual_edit.setReadOnly(True)
            actual_edit.setStyleSheet("background-color: #f0f0f0;")

            # Connect signals
            slider.valueChanged.connect(lambda val, idx=i: self.update_target(val, idx))
            btn_minus.clicked.connect(lambda _, idx=i: self.adjust_slider(idx, -1))
            btn_plus.clicked.connect(lambda _, idx=i: self.adjust_slider(idx, +1))
            target_edit.returnPressed.connect(lambda idx=i: self.update_from_edit(idx))

            self.joint_sliders.append(slider)
            self.target_edits.append(target_edit)
            self.actual_edits.append(actual_edit)

            joint_layout.addWidget(label, row, 0)
            joint_layout.addWidget(btn_minus, row, 1)
            joint_layout.addWidget(slider, row, 2)
            joint_layout.addWidget(btn_plus, row, 3)
            joint_layout.addWidget(QLabel("Target:"), row, 4)
            joint_layout.addWidget(target_edit, row, 5)
            joint_layout.addWidget(QLabel("Actual:"), row, 6)
            joint_layout.addWidget(actual_edit, row, 7)

        joint_group.setLayout(joint_layout)
        
        # Layout
        main_layout.addWidget(joint_group)
        tab_widget.setLayout(main_layout)
        return tab_widget

    def create_gains_tab(self, joint_config):
        """Create gains control tab"""
        tab_widget = QWidget()
        main_layout = QVBoxLayout()

        # Gains control group
        gains_group = QGroupBox("PID Gains Control")
        gains_layout = QGridLayout()
        
        # Initialize gains storage
        self.kp_edits = []
        self.kd_edits = []
        
        # Add headers
        headers = ["Joint", "Kp Value", "Kd Value"]
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
            gains_layout.addWidget(label, 0, col)

        # Create gain controls for each joint
        for i in range(6):
            row = i + 1
            joint = joint_config.get(f"Joint{i+1}", {})
            name = joint.get("name", f"Joint {i+1}")
            
            # Joint name
            joint_label = QLabel(name)
            joint_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            joint_label.setFixedWidth(100)
            
            # Kp control
            kp_edit = QLineEdit("1.0")
            kp_edit.setFixedWidth(120)
            kp_edit.setPlaceholderText("Kp value")
            
            # Kd control
            kd_edit = QLineEdit("0.1")
            kd_edit.setFixedWidth(120)
            kd_edit.setPlaceholderText("Kd value")
            
            self.kp_edits.append(kp_edit)
            self.kd_edits.append(kd_edit)
            
            gains_layout.addWidget(joint_label, row, 0)
            gains_layout.addWidget(kp_edit, row, 1)
            gains_layout.addWidget(kd_edit, row, 2)

        gains_group.setLayout(gains_layout)
        
        # Layout
        main_layout.addWidget(gains_group)
        tab_widget.setLayout(main_layout)
        return tab_widget

    def create_graphs_tab(self):
        """Create real-time graphs tab"""
        tab_widget = QWidget()
        main_layout = QVBoxLayout()

        # Create position graph
        self.position_plot = pg.PlotWidget(title="Position")
        self.position_plot.setLabel('left', 'Position (°)')
        self.position_plot.setLabel('bottom', 'Time (s)')
        self.position_curves = [
            self.position_plot.plot(pen=pg.mkPen(color=color, width=2))
            for color in ['b', 'g', 'r', 'c', 'm', 'y']
        ]
        main_layout.addWidget(self.position_plot)

        # Create velocity graph
        self.velocity_plot = pg.PlotWidget(title="Velocity")
        self.velocity_plot.setLabel('left', 'Velocity (°/s)')
        self.velocity_plot.setLabel('bottom', 'Time (s)')
        self.velocity_curves = [
            self.velocity_plot.plot(pen=pg.mkPen(color=color, width=2))
            for color in ['b', 'g', 'r', 'c', 'm', 'y']
        ]
        main_layout.addWidget(self.velocity_plot)

        # Create torque graph
        self.torque_plot = pg.PlotWidget(title="Torque")
        self.torque_plot.setLabel('left', 'Torque (Nm)')
        self.torque_plot.setLabel('bottom', 'Time (s)')
        self.torque_curves = [
            self.torque_plot.plot(pen=pg.mkPen(color=color, width=2))
            for color in ['b', 'g', 'r', 'c', 'm', 'y']
        ]
        main_layout.addWidget(self.torque_plot)

        tab_widget.setLayout(main_layout)
        return tab_widget

    def update_joint_states(self, positions):
        """Update joint states from ROS2 (called from worker thread)"""
        import time
        self.last_update_time = time.time()

        # Update actual positions
        for i, pos in enumerate(positions[:6]):  # Only update first 6 joints
            if i < len(self.actual_edits):
                self.actual_edits[i].setText(f"{pos:.2f}")

        # Update graphs
        current_time = time.time()
        for i in range(6):
            # Example velocity and torque calculations
            velocity = positions[i] * 0.1  # Replace with actual velocity data
            torque = positions[i] * 0.05  # Replace with actual torque data

            # Append data to graph storage
            self.graph_data["position"][i].append((current_time, positions[i]))
            self.graph_data["velocity"][i].append((current_time, velocity))
            self.graph_data["torque"][i].append((current_time, torque))

            # Update position graph
            self.position_curves[i].setData(
                [point[0] for point in self.graph_data["position"][i]],
                [point[1] for point in self.graph_data["position"][i]]
            )

            # Update velocity graph
            self.velocity_curves[i].setData(
                [point[0] for point in self.graph_data["velocity"][i]],
                [point[1] for point in self.graph_data["velocity"][i]]
            )

            # Update torque graph
            self.torque_curves[i].setData(
                [point[0] for point in self.graph_data["torque"][i]],
                [point[1] for point in self.graph_data["torque"][i]]
            )

    def update_status(self):
        """Update connection status"""
        import time
        current_time = time.time()
        
        if self.last_update_time and (current_time - self.last_update_time) < 2.0:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.status_label.setText("Status: No data")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    qdarktheme.setup_theme("light")

    window = JointControl()
    window.show()

    sys.exit(app.exec())