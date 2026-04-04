from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGroupBox, QLabel, QSlider,
    QPushButton, QLineEdit, QGridLayout, QTextEdit, QTabWidget, QComboBox
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont

import sys
from PyQt6.QtWidgets import QApplication
import qdarktheme

# Add ROS2 imports
import rclpy
from rclpy.node import Node
from indy_iface.msg import JointInfo
from indy_iface.srv import SetPos, SetAllPos, GetGains, SetGains, GetControlMode, SetControlMode
import threading

from commons import load_config, ensure_config


class ROS2Worker(QThread):
    """Background thread for ROS2 communication"""
    joint_state_updated = pyqtSignal(list)  # Signal for joint state updates
    gains_updated = pyqtSignal(list, list)  # Signal for gains updates (kp, kd)
    control_mode_updated = pyqtSignal(int)  # Signal for control mode updates
    
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
        self.get_control_mode_client = self.node.create_client(GetControlMode, 'get_control_mode')
        self.set_control_mode_client = self.node.create_client(SetControlMode, 'set_control_mode')
        
        # Create subscriber
        self.joint_sub = self.node.create_subscription(
            JointInfo,
            'indy_state',
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
    
    def set_joint_position(self, joint_index, position):
        """Set single joint position (thread-safe)"""
        if not self.set_pos_client.wait_for_service(timeout_sec=1.0):
            return False
            
        req = SetPos.Request()
        req.joint_index = joint_index
        req.position = float(position)
        
        future = self.set_pos_client.call_async(req)
        return True
    
    def set_all_positions(self, positions):
        """Set all joint positions (thread-safe)"""
        if not self.set_all_pos_client.wait_for_service(timeout_sec=1.0):
            return False
            
        req = SetAllPos.Request()
        req.positions = [float(p) for p in positions]
        
        future = self.set_all_pos_client.call_async(req)
        return True
    
    def get_gains(self):
        """Get current gains from robot"""
        if not self.get_gains_client.wait_for_service(timeout_sec=1.0):
            return False

        req = GetGains.Request()
        future = self.get_gains_client.call_async(req)

        def handle_response():
            try:
                # Wait for the response with timeout
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                if future.done():
                    response = future.result()
                    if response.success:
                        kp = list(response.kp)
                        kd = list(response.kd)
                        self.gains_updated.emit(kp, kd)
                        return True
                    else:
                        print("Received response, but success=False.")
                else:
                    print("Timeout waiting for gains response.")
            except Exception as e:
                print(f"Error getting gains: {e}")
            return False

        threading.Thread(target=handle_response, daemon=True).start()
        return True
    
    def set_gains(self, kp_values, kd_values):
        """Set gains to robot"""
        if not self.set_gains_client.wait_for_service(timeout_sec=1.0):
            return False
            
        req = SetGains.Request()
        req.kp = [float(k) for k in kp_values]
        req.kd = [float(k) for k in kd_values]
        
        future = self.set_gains_client.call_async(req)
        return True

    def get_control_mode(self):
        """Get current control mode from robot"""
        if not self.get_control_mode_client.wait_for_service(timeout_sec=1.0):
            return False

        req = GetControlMode.Request()
        future = self.get_control_mode_client.call_async(req)

        def handle_response():
            try:
                # Wait for result with timeout
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                if future.done():
                    response = future.result()
                    if hasattr(response, "control_mode"):
                        self.control_mode_updated.emit(response.control_mode)
                else:
                    print("Timeout waiting for control mode response.")
            except Exception as e:
                print(f"Error getting control mode: {e}")

        threading.Thread(target=handle_response, daemon=True).start()
        return True

    def set_control_mode(self, mode):
        """Set control mode on robot"""
        if not self.set_control_mode_client.wait_for_service(timeout_sec=1.0):
            return False
        req = SetControlMode.Request()
        req.control_mode = int(mode)
        future = self.set_control_mode_client.call_async(req)
        return True
    
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
        
        self.tab_widget.addTab(self.position_tab, "Position Control")
        self.tab_widget.addTab(self.gains_tab, "Gains Control")
        
        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # Start ROS2 worker thread
        self.ros2_worker = ROS2Worker()
        self.ros2_worker.joint_state_updated.connect(self.update_joint_states)
        self.ros2_worker.gains_updated.connect(self.update_gains_display)
        self.ros2_worker.control_mode_updated.connect(self.update_control_mode_display)
        self.ros2_worker.start()
        
        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.timeout.connect(self.ros2_worker.get_control_mode)
        self.status_timer.start(1000)  # Update every second
        
        self.last_update_time = None

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

        # --- Add control mode selector and monitor at the bottom ---
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.addItems([
            "Gravity Compensation", "Full Dynamics", "Computed Torque", "Adaptive Control"
        ])
        self.set_mode_btn = QPushButton("Set Mode")
        self.set_mode_btn.clicked.connect(self.set_control_mode)
        self.control_mode_monitor = QLineEdit("Unknown")
        self.control_mode_monitor.setReadOnly(True)
        self.control_mode_monitor.setFixedWidth(180)
        self.control_mode_monitor.setStyleSheet("background-color: #f0f0f0;")

        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Control Mode:"))
        mode_layout.addWidget(self.control_mode_combo)
        mode_layout.addWidget(self.set_mode_btn)
        mode_layout.addWidget(QLabel("Current:"))
        mode_layout.addWidget(self.control_mode_monitor)

        joint_layout.addLayout(mode_layout, len(joint_config) + 2, 0, 1, 8)

        joint_group.setLayout(joint_layout)
        
        # Right: Control panel
        control_panel = self.create_control_panel()
        
        # Layout
        main_layout.addWidget(joint_group)
        main_layout.addWidget(control_panel)
        
        tab_widget.setLayout(main_layout)
        return tab_widget

    def set_control_mode(self):
        mode = self.control_mode_combo.currentIndex()
        self.ros2_worker.set_control_mode(mode)

    def update_control_mode_display(self, mode):
        modes = [
            "Gravity Compensation", "Full Dynamics", "Computed Torque", "Adaptive Control"
        ]
        if 0 <= mode < len(modes):
            self.control_mode_monitor.setText(modes[mode])
        else:
            self.control_mode_monitor.setText("Unknown")

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
        
        # Buttons layout
        buttons_layout = QHBoxLayout()
        
        # Get gains button
        self.get_gains_btn = QPushButton("Get Current Gains")
        self.get_gains_btn.clicked.connect(self.get_gains)
        self.get_gains_btn.setFixedHeight(40)
        self.get_gains_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        
        # Set gains button
        self.set_gains_btn = QPushButton("Set Gains to Robot")
        self.set_gains_btn.clicked.connect(self.set_gains)
        self.set_gains_btn.setFixedHeight(40)
        self.set_gains_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        
        # Set default gains button
        self.set_default_gains_btn = QPushButton("Set Default Gains")
        self.set_default_gains_btn.clicked.connect(self.set_default_gains)
        self.set_default_gains_btn.setFixedHeight(40)
        self.set_default_gains_btn.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold;")
        
        buttons_layout.addWidget(self.get_gains_btn)
        buttons_layout.addWidget(self.set_gains_btn)
        buttons_layout.addWidget(self.set_default_gains_btn)
        
        # Status and log
        self.gains_status_label = QLabel("Gains Status: Ready")
        self.gains_status_label.setStyleSheet("color: blue; font-weight: bold;")
        
        self.gains_log_display = QTextEdit()
        self.gains_log_display.setMaximumHeight(150)
        self.gains_log_display.setReadOnly(True)
        
        # Layout
        main_layout.addWidget(gains_group)
        main_layout.addLayout(buttons_layout)
        main_layout.addWidget(self.gains_status_label)
        main_layout.addWidget(QLabel("Gains Log:"))
        main_layout.addWidget(self.gains_log_display)
        
        tab_widget.setLayout(main_layout)
        return tab_widget

    def create_control_panel(self):
        """Create the right control panel"""
        control_group = QGroupBox("Robot Control")
        control_layout = QVBoxLayout()
        
        # Connection status
        self.status_label = QLabel("Status: Connecting...")
        self.status_label.setStyleSheet("color: orange; font-weight: bold;")
        control_layout.addWidget(self.status_label)
        
        # Control buttons
        self.send_all_btn = QPushButton("Send All Positions")
        self.send_all_btn.clicked.connect(self.send_all_positions)
        self.send_all_btn.setFixedHeight(40)
        control_layout.addWidget(self.send_all_btn)
        
        self.home_btn = QPushButton("Go Home")
        self.home_btn.clicked.connect(self.go_home)
        self.home_btn.setFixedHeight(40)
        control_layout.addWidget(self.home_btn)
        
        # Log display
        self.log_display = QTextEdit()
        self.log_display.setMaximumHeight(200)
        self.log_display.setReadOnly(True)
        control_layout.addWidget(QLabel("Position Log:"))
        control_layout.addWidget(self.log_display)
        
        control_group.setLayout(control_layout)
        return control_group

    def update_joint_states(self, positions):
        """Update joint states from ROS2 (called from worker thread)"""
        import time
        self.last_update_time = time.time()
        
        for i, pos in enumerate(positions[:6]):  # Only update first 6 joints
            if i < len(self.actual_edits):
                self.actual_edits[i].setText(f"{pos:.2f}")

    def update_gains_display(self, kp_values, kd_values):
        """Update gains display from ROS2 (called from worker thread)"""
        for i, (kp, kd) in enumerate(zip(kp_values[:6], kd_values[:6])):
            if i < len(self.kp_edits):
                self.kp_edits[i].setText(f"{kp:.3f}")
                self.kd_edits[i].setText(f"{kd:.3f}")
        
        self.gains_status_label.setText("Gains Status: Updated from robot")
        self.gains_status_label.setStyleSheet("color: green; font-weight: bold;")
        self.gains_log_message("Successfully retrieved gains from robot")

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

    def update_target(self, value, index):
        """Update target value when slider changes"""
        self.target_edits[index].setText(str(value))
        
        # Send to robot (non-blocking)
        if self.ros2_worker.set_joint_position(index, value):
            self.log_message(f"Sent Joint {index+1} to {value}°")
        else:
            self.log_message(f"Failed to send Joint {index+1}")

    def update_from_edit(self, index):
        """Update slider when edit field changes"""
        try:
            value = float(self.target_edits[index].text())
            self.joint_sliders[index].setValue(int(value))
        except ValueError:
            self.log_message(f"Invalid value for Joint {index+1}")

    def adjust_slider(self, index, delta):
        """Adjust slider by delta"""
        slider = self.joint_sliders[index]
        slider.setValue(slider.value() + delta)

    def send_all_positions(self):
        """Send all joint positions"""
        positions = []
        for edit in self.target_edits:
            try:
                positions.append(float(edit.text()))
            except ValueError:
                positions.append(0.0)
        
        if self.ros2_worker.set_all_positions(positions):
            self.log_message(f"Sent all positions: {positions}")
        else:
            self.log_message("Failed to send all positions")

    def go_home(self):
        """Set all joints to home position (0)"""
        for slider in self.joint_sliders:
            slider.setValue(0)
        self.log_message("Moving to home position")

    def get_gains(self):
        """Get current gains from robot"""
        if self.ros2_worker.get_gains():
            self.gains_log_message("Requesting gains from robot...")
            self.gains_status_label.setText("Gains Status: Getting gains...")
            self.gains_status_label.setStyleSheet("color: orange; font-weight: bold;")
        else:
            self.gains_log_message("Failed to request gains - service not available")
            self.gains_status_label.setText("Gains Status: Service unavailable")
            self.gains_status_label.setStyleSheet("color: red; font-weight: bold;")

    def set_gains(self):
        """Set gains to robot"""
        try:
            kp_values = []
            kd_values = []
            
            for i in range(6):
                kp_values.append(float(self.kp_edits[i].text()))
                kd_values.append(float(self.kd_edits[i].text()))
            
            if self.ros2_worker.set_gains(kp_values, kd_values):
                self.gains_log_message(f"Setting gains - Kp: {kp_values}, Kd: {kd_values}")
                self.gains_status_label.setText("Gains Status: Setting gains...")
                self.gains_status_label.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.gains_log_message("Failed to set gains - service not available")
                self.gains_status_label.setText("Gains Status: Service unavailable")
                self.gains_status_label.setStyleSheet("color: red; font-weight: bold;")
                
        except ValueError:
            self.gains_log_message("Error: Invalid gain values")
            self.gains_status_label.setText("Gains Status: Invalid values")
            self.gains_status_label.setStyleSheet("color: red; font-weight: bold;")

    def set_default_gains(self):
        """Set default gains values"""
        default_kp = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        default_kd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        
        for i in range(6):
            self.kp_edits[i].setText(str(default_kp[i]))
            self.kd_edits[i].setText(str(default_kd[i]))
        
        self.gains_log_message("Set default gains values")
        self.gains_status_label.setText("Gains Status: Default values set")
        self.gains_status_label.setStyleSheet("color: blue; font-weight: bold;")

    def log_message(self, message):
        """Add message to position log display"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_display.append(f"[{timestamp}] {message}")
        # Auto-scroll to bottom
        self.log_display.verticalScrollBar().setValue(
            self.log_display.verticalScrollBar().maximum()
        )

    def gains_log_message(self, message):
        """Add message to gains log display"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.gains_log_display.append(f"[{timestamp}] {message}")
        # Auto-scroll to bottom
        self.gains_log_display.verticalScrollBar().setValue(
            self.gains_log_display.verticalScrollBar().maximum()
        )

    def closeEvent(self, event):
        """Clean up when closing"""
        self.ros2_worker.stop()
        self.ros2_worker.wait(3000)  # Wait up to 3 seconds for clean shutdown
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    qdarktheme.setup_theme("light")

    window = JointControl()
    window.show()

    sys.exit(app.exec())