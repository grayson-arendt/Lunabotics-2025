#!/usr/bin/env python3
import signal
import sys
import os
import math
import re
import threading
import time
from queue import Queue

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import ui_widgets
from gui_styles import (Colors, MAIN_STYLESHEET, Styles,
                        ACTION_BTN_CSS, ACTION_BTN_ACTIVE_CSS, ACTION_BTN_DISABLED_CSS,
                        ESTOP_BTN_NORMAL_CSS, ESTOP_BTN_ACTIVE_CSS,
                        CAN_RESTART_SUCCESS_CSS)
from ros_interface import RobotInterface

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QIcon, QImage, QPalette, QPixmap, QFont
from PyQt5.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                              QMainWindow, QTextEdit, QPushButton, QSizePolicy, QVBoxLayout,
                              QWidget)

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# --- Layout ---
WINDOW_DEFAULT_WIDTH    = 1600
WINDOW_DEFAULT_HEIGHT   = 900
WINDOW_MIN_WIDTH        = 1000
WINDOW_MIN_HEIGHT       = 650
SIDEBAR_MIN_WIDTH       = 300
SIDEBAR_COLLAPSED_WIDTH = 30
CAMERA_MIN_WIDTH        = 320
CAMERA_MIN_HEIGHT       = 240

# --- Teleop ---
TELEOP_LINEAR_SPEED_DEFAULT  = 0.35
TELEOP_ANGULAR_SPEED_DEFAULT = 0.25
TELEOP_MAX_LINEAR_SPEED      = 0.75
TELEOP_MAX_ANGULAR_SPEED     = 0.75
TELEOP_SPEED_INCREMENT       = 1.1
TELEOP_SPEED_DECREMENT       = 0.9
TELEOP_BUCKET_SPEED          = 0.05
TELEOP_BUCKET_LIMIT_MIN      = -1.57
TELEOP_BUCKET_LIMIT_MAX      = 0.1

# --- Timers ---
UI_UPDATE_INTERVAL_MS   = 100
CAMERA_UPDATE_INTERVAL_MS = 33

# --- Bandwidth ---
BANDWIDTH_MAX_MBPS           = 4.0
BANDWIDTH_WARN_THRESHOLD     = 0.75
BANDWIDTH_CRITICAL_THRESHOLD = 0.90

# Key to teleop action mappings
_TELEOP_MOVE_KEYS = {
    Qt.Key_W: 'w', Qt.Key_A: 'a', Qt.Key_S: 's', Qt.Key_D: 'd',
    Qt.Key_Up: 'up', Qt.Key_Down: 'down',
}
_TELEOP_SPEED_KEYS = {
    Qt.Key_Q: ('linear', TELEOP_SPEED_INCREMENT),
    Qt.Key_Z: ('linear', TELEOP_SPEED_DECREMENT),
    Qt.Key_E: ('angular', TELEOP_SPEED_INCREMENT),
    Qt.Key_C: ('angular', TELEOP_SPEED_DECREMENT),
}


class LunabotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init()
        temp_node = Node('temp_param_reader')
        temp_node.declare_parameter('mode', True)
        mode_val = temp_node.get_parameter('mode').value
        temp_node.destroy_node()

        self._log_queue = Queue()
        
        self.robot = RobotInterface(mode_param=mode_val)
        self.robot.on_robot_state_update = self._queue_robot_state_update
        self.robot.on_control_state_update = self._queue_control_state_update
        self.robot.on_log = self._queue_log

        self.sidebar_collapsed = False
        self.fisheye_camera_position = 0.0
        self.fisheye_showing_deposit = True
        self.full_auto_active = False
        self.emergency_stopped = False
        self.excavation_angle = 1.59

        self.teleop_keys = {
            'w': False, 'a': False, 's': False, 'd': False,
            'up': False, 'down': False,
        }
        self.linear_speed = TELEOP_LINEAR_SPEED_DEFAULT
        self.angular_speed = TELEOP_ANGULAR_SPEED_DEFAULT
        self.max_linear_speed = TELEOP_MAX_LINEAR_SPEED
        self.max_angular_speed = TELEOP_MAX_ANGULAR_SPEED

        self.last_camera_frame_ids = {'front': None, 'rear': None, 'fisheye': None}
        self._fisheye_frame_count = 0
        self._fisheye_fps_time = time.monotonic()

        self.init_ui()

        self._ros_executor = SingleThreadedExecutor()
        self._ros_executor.add_node(self.robot.node)
        self._ros_thread = threading.Thread(target=self._ros_executor.spin, daemon=True)
        self._ros_thread.start()

        self.robot.publish_excavation_angle(self.excavation_angle)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(UI_UPDATE_INTERVAL_MS)

        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_cameras)
        self.camera_timer.start(CAMERA_UPDATE_INTERVAL_MS)

    def _queue_robot_state_update(self):
        self._log_queue.put(('robot_state', None))
    
    def _queue_control_state_update(self, msg):
        self._log_queue.put(('control_state', msg))
    
    def _queue_log(self, text):
        self._log_queue.put(('log', text))

    def init_ui(self):
        self.setWindowTitle('Lunabot Control Panel')

        try:
            pkg_dir = get_package_share_directory('lunabot_gui')
            icon_path = os.path.join(pkg_dir, 'resource', 'lunabot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
        except Exception:
            pass

        screen = QApplication.primaryScreen().geometry()
        w = min(WINDOW_DEFAULT_WIDTH, int(screen.width() * 0.9))
        h = min(WINDOW_DEFAULT_HEIGHT, int(screen.height() * 0.9))
        self.setGeometry((screen.width() - w) // 2, (screen.height() - h) // 2, w, h)
        self.setMinimumSize(WINDOW_MIN_WIDTH, WINDOW_MIN_HEIGHT)
        self.setStyleSheet(MAIN_STYLESHEET)

        main_widget = QWidget()
        main_widget.setObjectName("centralWidget")
        main_widget.setStyleSheet("background-color: #1a1a1a;")
        self.setCentralWidget(main_widget)

        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)
        main_widget.setLayout(main_layout)

        main_layout.addWidget(self.create_content_area(), 10)

        sidebar = self.create_sidebar()
        sidebar.setMinimumWidth(SIDEBAR_MIN_WIDTH)
        main_layout.addWidget(sidebar, 0)

    # -------------------------------------------------------------------------
    # Layout builders
    # -------------------------------------------------------------------------

    def create_content_area(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(4, 0, 0, 4)
        layout.setSpacing(5)
        widget.setLayout(layout)
        layout.addWidget(self.create_camera_displays(), 10)
        layout.addWidget(self.create_bottom_row(), 0)
        return widget

    def create_camera_displays(self):
        row = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(10)
        row.setLayout(row_layout)

        fisheye_container = QWidget()
        fisheye_layout = QVBoxLayout()
        fisheye_layout.setContentsMargins(0, 0, 0, 0)
        fisheye_layout.setSpacing(4)
        fisheye_container.setLayout(fisheye_layout)

        fisheye_group = QGroupBox("Fisheye Camera")
        fisheye_group.setAutoFillBackground(True)
        fisheye_group.setStyleSheet(
            "QGroupBox { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2a2a2a, stop:1 #252525); padding-top: 20px; }")
        eye_layout = QVBoxLayout()
        eye_layout.setContentsMargins(4, 4, 4, 4)
        eye_layout.setSpacing(0)

        cam_overlay = QWidget()
        cam_grid = QGridLayout(cam_overlay)
        cam_grid.setContentsMargins(0, 0, 0, 0)
        cam_grid.setSpacing(0)

        self.fisheye_camera_label = QLabel("No camera feed")
        self.fisheye_camera_label.setAlignment(Qt.AlignCenter)
        self.fisheye_camera_label.setMinimumSize(CAMERA_MIN_WIDTH, CAMERA_MIN_HEIGHT)
        self.fisheye_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fisheye_camera_label.setStyleSheet(Styles.camera_label())
        cam_grid.addWidget(self.fisheye_camera_label, 0, 0)

        self.fisheye_fps_label = QLabel("FPS: --")
        self.fisheye_fps_label.setAlignment(Qt.AlignBottom | Qt.AlignRight)
        self.fisheye_fps_label.setFont(QFont("Monospace", 9))
        self.fisheye_fps_label.setStyleSheet("color: white; background-color: transparent; padding: 0px 6px 6px 0px;")
        self.fisheye_fps_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.fisheye_fps_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        cam_grid.addWidget(self.fisheye_fps_label, 0, 0)

        eye_layout.addWidget(cam_overlay)
        fisheye_group.setLayout(eye_layout)
        fisheye_layout.addWidget(fisheye_group)

        self.swap_fisheye_btn = QPushButton("Switch to Bucket View")
        self.swap_fisheye_btn.setStyleSheet(ACTION_BTN_CSS)
        self.swap_fisheye_btn.setMaximumHeight(32)
        self.swap_fisheye_btn.clicked.connect(self.swap_fisheye_view)
        fisheye_layout.addWidget(self.swap_fisheye_btn)

        row_layout.addWidget(fisheye_container)
        return row

    def create_bottom_row(self):
        row = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(3)
        row.setLayout(row_layout)

        left_col = QWidget()
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(3)
        left_col.setLayout(left_layout)

        mode_bw = QWidget()
        mode_bw_layout = QHBoxLayout()
        mode_bw_layout.setContentsMargins(0, 0, 0, 0)
        mode_bw_layout.setSpacing(3)
        mode_bw.setLayout(mode_bw_layout)

        mode_group = ui_widgets.create_mode_group(self)
        if not self.robot.is_real_mode:
            mode_group.setEnabled(False)
        mode_bw_layout.addWidget(mode_group, 2)
        mode_bw_layout.addWidget(ui_widgets.create_bandwidth_group(self), 3)

        left_layout.addWidget(mode_bw)
        left_layout.addWidget(ui_widgets.create_condensed_telemetry_group(self))
        left_layout.addWidget(ui_widgets.create_bucket_state_widget(self))
        left_layout.addStretch()

        row_layout.addWidget(left_col, 1)
        row_layout.addWidget(self.create_terminal_output(), 1)
        return row

    def create_terminal_output(self):
        group = QGroupBox("Terminal Output")
        group.setAutoFillBackground(True)
        group.setStyleSheet(
            "QGroupBox { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); padding-top: 16px; }")
        layout = QVBoxLayout()
        layout.setContentsMargins(3, 6, 3, 2)
        layout.setSpacing(2)
        group.setLayout(layout)

        self.terminal_text = QTextEdit()
        self.terminal_text.setReadOnly(True)
        self.terminal_text.document().setMaximumBlockCount(500)
        self.terminal_text.setFont(QFont("Monospace", 9))
        self.terminal_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: #0d0d0d;
                color: {Colors.TEXT_MAIN};
                border: 1px solid #2a2a2a;
                border-radius: 2px;
            }}
            QScrollBar:vertical {{
                background: #0d0d0d;
                width: 10px;
                border: none;
            }}
            QScrollBar::handle:vertical {{
                background: #2a2a2a;
                min-height: 20px;
                border-radius: 4px;
                margin: 2px;
            }}
            QScrollBar::handle:vertical:hover {{ background: #3a3a3a; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: none; }}
        """)
        layout.addWidget(self.terminal_text)
        return group

    def create_sidebar(self):
        self.sidebar_container = QWidget()
        container_layout = QHBoxLayout()
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)
        self.sidebar_container.setLayout(container_layout)

        self.sidebar_widget = QWidget()
        self.sidebar_widget.setStyleSheet("background-color: #1a1a1a;")
        self.sidebar_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.sidebar_widget.setMinimumWidth(300)
        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(4, 4, 4, 4)
        sidebar_layout.setSpacing(6)
        self.sidebar_widget.setLayout(sidebar_layout)

        sidebar_layout.addWidget(ui_widgets.create_controls_reference_group(self))
        sidebar_layout.addWidget(ui_widgets.create_excavation_angle_group(self))
        sidebar_layout.addWidget(ui_widgets.create_hardware_group(self))
        sidebar_layout.addWidget(ui_widgets.create_launch_group(self))
        sidebar_layout.addWidget(ui_widgets.create_action_control_group(self))

        if not self.robot.is_real_mode:
            teleop_group = ui_widgets.create_teleop_control_group(self)
            teleop_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding)
            sidebar_layout.addWidget(teleop_group, 0)

        sidebar_layout.addStretch(1)
        container_layout.addWidget(self.sidebar_widget)

        edge_container = QWidget()
        edge_container.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        edge_container.setFixedWidth(32)
        edge_layout = QHBoxLayout()
        edge_layout.setContentsMargins(0, 0, 0, 0)
        edge_layout.setSpacing(0)
        edge_container.setLayout(edge_layout)

        divider = QWidget()
        divider.setFixedWidth(2)
        divider.setStyleSheet(f"background-color: {Colors.BORDER};")
        edge_layout.addWidget(divider)

        self.edge_tab = QPushButton("◀\n\nC\nO\nN\nT\nR\nO\nL\nS")
        self.edge_tab.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.BG_MAIN};
                color: {Colors.TEXT_MAIN};
                border: none;
                font-size: 12px;
                font-weight: bold;
                padding: 10px 5px;
                letter-spacing: 2px;
            }}
        """)
        self.edge_tab.setFixedWidth(30)
        self.edge_tab.clicked.connect(self.toggle_sidebar)
        edge_layout.addWidget(self.edge_tab)

        container_layout.addWidget(edge_container)
        return self.sidebar_container

    # -------------------------------------------------------------------------
    # Camera / view actions
    # -------------------------------------------------------------------------

    def swap_fisheye_view(self):
        self.fisheye_showing_deposit = not self.fisheye_showing_deposit
        degrees = 0 if self.fisheye_showing_deposit else 180
        self.set_fisheye_camera_position(math.radians(degrees))
        self.swap_fisheye_btn.setText(
            "Switch to Bucket View" if self.fisheye_showing_deposit else "Switch to Deposit View")

    def set_fisheye_camera_position(self, position_radians):
        self.fisheye_camera_position = position_radians
        self.robot.publish_camera_position(position_radians)
        degrees = math.degrees(position_radians)
        if hasattr(self, 'camera_pos_label'):
            self.camera_pos_label.setText(f"Position: {degrees:.0f} deg")
        self.robot.node.get_logger().info(f'Fisheye camera position: {degrees:.1f} deg')

    def rotate_fisheye_camera(self, delta_radians):
        new_pos = self.fisheye_camera_position + delta_radians
        while new_pos > math.pi:
            new_pos -= 2 * math.pi
        while new_pos < -math.pi:
            new_pos += 2 * math.pi
        self.set_fisheye_camera_position(new_pos)

    # -------------------------------------------------------------------------
    # Sidebar actions
    # -------------------------------------------------------------------------

    def toggle_sidebar(self):
        self.sidebar_collapsed = not self.sidebar_collapsed
        if self.sidebar_collapsed:
            self.sidebar_widget.hide()
            self.sidebar_container.setMinimumWidth(SIDEBAR_COLLAPSED_WIDTH)
            self.sidebar_container.setMaximumWidth(SIDEBAR_COLLAPSED_WIDTH)
            self.edge_tab.setText("▶\n\nC\nO\nN\nT\nR\nO\nL\nS")
        else:
            self.sidebar_widget.show()
            self.sidebar_container.setMinimumWidth(SIDEBAR_MIN_WIDTH)
            self.sidebar_container.setMaximumWidth(16777215)
            self.edge_tab.setText("◀\n\nC\nO\nN\nT\nR\nO\nL\nS")

    def toggle_mode(self):
        self.robot.publish_mode_switch()

    def start_can_interface(self):
        self.robot.start_can_interface()

    def launch_hardware(self):
        if self.robot.launch_processes.get('hardware') is not None:
            self.robot.stop_system('hardware')
        else:
            self.robot.launch_hardware()

    def launch_system(self, system_name):
        if self.robot.launch_processes.get(system_name) is not None:
            self.robot.stop_system(system_name)
        else:
            self.robot.launch_system(system_name)

    def launch_rviz(self):
        self.robot.launch_rviz()

    def restart_can(self):
        if self.robot.restart_can_interface() and hasattr(self, 'can_restart_btn'):
            self.can_restart_btn.setStyleSheet(CAN_RESTART_SUCCESS_CSS)
            QTimer.singleShot(2000, lambda: self.can_restart_btn.setStyleSheet(ACTION_BTN_CSS))

    # -------------------------------------------------------------------------
    # Operation callbacks (shared helper)
    # -------------------------------------------------------------------------

    def _set_operation_status(self, text, style_type):
        color_map = {
            'idle': Colors.TEXT_DIM,
            'active': Colors.STATUS_SUCCESS,
            'warning': Colors.STATUS_WARNING,
            'error': Colors.STATUS_ERROR,
            'info': Colors.STATUS_INFO,
        }
        color = color_map.get(style_type, Colors.TEXT_DIM)
        
        if text.startswith('Status:'):
            message = text[7:].strip()
            formatted_text = f'<span style="color: {Colors.TEXT_MAIN};">Status:</span> <span style="color: {color};">{message}</span>'
            self.operation_status_label.setText(formatted_text)
        else:
            self.operation_status_label.setText(text)
        
        self.operation_status_label.setStyleSheet(f"background-color: {Colors.BG_TRANSPARENT}; font-weight: bold;")

    # -------------------------------------------------------------------------
    # Excavation
    # -------------------------------------------------------------------------

    def send_excavate_goal(self):
        if self.robot.is_excavating:
            self.robot.node.get_logger().info('Canceling excavation')
            self.robot.cancel_excavate_goal(self._queue_excavate_cancel_callback)
        else:
            self.robot.is_excavating = True
            self.excavate_btn.setText("Cancel Excavation")
            self.excavate_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            self._set_operation_status("Status: Excavating...", 'warning')
            self.robot.send_excavate_goal(
                self._queue_excavate_goal_response_callback,
                self._queue_excavate_result_callback,
                self._queue_excavate_cancel_callback,
            )

    def _queue_excavate_goal_response_callback(self, goal_handle):
        self._log_queue.put(('excavate_response', goal_handle))

    def _queue_excavate_result_callback(self, future):
        self._log_queue.put(('excavate_result', future))

    def _queue_excavate_cancel_callback(self, future):
        self._log_queue.put(('excavate_cancel', future))

    def excavate_goal_response_callback(self, goal_handle):
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Excavate goal rejected')
            self.robot.is_excavating = False
            self.excavate_btn.setText("Excavate")
            self._set_operation_status("Status: Goal rejected", 'error')
        else:
            self.robot.node.get_logger().info('Excavate goal accepted')

    def excavate_result_callback(self, future):
        result = future.result().result
        self.robot.is_excavating = False
        self.excavate_btn.setText("Excavate")
        self.excavate_btn.setStyleSheet(ACTION_BTN_CSS)
        if result.success:
            self.robot.node.get_logger().info('Excavation completed')
            self._set_operation_status("Status: Excavation completed", 'active')
        else:
            self.robot.node.get_logger().error('Excavation failed')
            self._set_operation_status("Status: Excavation failed", 'error')

    def excavate_cancel_callback(self, future):
        self.robot.node.get_logger().info('Excavation canceled')
        self.robot.is_excavating = False
        self.excavate_btn.setText("Excavate")
        self.excavate_btn.setStyleSheet(ACTION_BTN_CSS)
        self._set_operation_status("Status: Excavation canceled", 'warning')

    # -------------------------------------------------------------------------
    # Deposit
    # -------------------------------------------------------------------------

    def send_deposit_goal(self):
        if self.robot.is_depositing:
            self.robot.node.get_logger().info('Canceling deposit')
            self.robot.cancel_deposit_goal(self._queue_deposit_cancel_callback)
        else:
            self.robot.is_depositing = True
            self.deposit_btn.setText("Cancel Deposit")
            self.deposit_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            self._set_operation_status("Status: Depositing...", 'warning')
            self.robot.send_deposit_goal(
                self._queue_deposit_goal_response_callback,
                self._queue_deposit_result_callback,
                self._queue_deposit_cancel_callback,
            )

    def _queue_deposit_goal_response_callback(self, goal_handle):
        self._log_queue.put(('deposit_response', goal_handle))

    def _queue_deposit_result_callback(self, future):
        self._log_queue.put(('deposit_result', future))

    def _queue_deposit_cancel_callback(self, future):
        self._log_queue.put(('deposit_cancel', future))

    def deposit_goal_response_callback(self, goal_handle):
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Deposit goal rejected')
            self.robot.is_depositing = False
            self.deposit_btn.setText("Deposit")
            self._set_operation_status("Status: Goal rejected", 'error')
        else:
            self.robot.node.get_logger().info('Deposit goal accepted')

    def deposit_result_callback(self, future):
        result = future.result().result
        self.robot.is_depositing = False
        self.deposit_btn.setText("Deposit")
        self.deposit_btn.setStyleSheet(ACTION_BTN_CSS)
        if result.success:
            self.robot.node.get_logger().info(f'Deposit completed: {result.message}')
            self._set_operation_status(f"Status: {result.message}", 'active')
        else:
            self.robot.node.get_logger().error(f'Deposit failed: {result.message}')
            self._set_operation_status(f"Status: Failed - {result.message}", 'error')

    def deposit_cancel_callback(self, future):
        self.robot.node.get_logger().info('Deposit canceled')
        self.robot.is_depositing = False
        self.deposit_btn.setText("Deposit")
        self.deposit_btn.setStyleSheet(ACTION_BTN_CSS)
        self._set_operation_status("Status: Deposit canceled", 'warning')

    # -------------------------------------------------------------------------
    # Emergency stop / full auto
    # -------------------------------------------------------------------------

    def emergency_stop(self):
        if not self.emergency_stopped:
            self.robot.node.get_logger().error('Emergency stop!')
            self.robot.publish_emergency_stop()
            if self.robot.is_excavating:
                self.robot.cancel_excavate_goal(lambda f: None)
                self.robot.is_excavating = False
                self.excavate_btn.setText("Excavate")
                self.excavate_btn.setStyleSheet(ACTION_BTN_CSS)
            if self.robot.is_depositing:
                self.robot.cancel_deposit_goal(lambda f: None)
                self.robot.is_depositing = False
                self.deposit_btn.setText("Deposit")
                self.deposit_btn.setStyleSheet(ACTION_BTN_CSS)
            if self.full_auto_active:
                self.robot.stop_navigation_client()
                self.full_auto_active = False
                self.auto_btn.setText("One Cycle Auto")
                self.auto_btn.setStyleSheet(ACTION_BTN_CSS)
            self.emergency_stopped = True
            self.emergency_stop_btn.setText("Re-Enable Robot")
            self.emergency_stop_btn.setStyleSheet(ESTOP_BTN_ACTIVE_CSS)
            self._set_operation_status("Status: Disabled", 'error')
        else:
            self.robot.node.get_logger().info('Re-enabling robot')
            self.robot.publish_re_enable()
            self.emergency_stopped = False
            self.emergency_stop_btn.setText("Emergency Stop")
            self.emergency_stop_btn.setStyleSheet(ESTOP_BTN_NORMAL_CSS)

    def increase_excavation_angle(self):
        self.excavation_angle = min(1.70, self.excavation_angle + 0.01)
        self.excavation_angle_value_label.setText(f"{self.excavation_angle:.2f}")
        self.robot.publish_excavation_angle(self.excavation_angle)

    def decrease_excavation_angle(self):
        self.excavation_angle = max(1.50, self.excavation_angle - 0.01)
        self.excavation_angle_value_label.setText(f"{self.excavation_angle:.2f}")
        self.robot.publish_excavation_angle(self.excavation_angle)

    def send_full_auto_goal(self):
        if self.full_auto_active:
            self.robot.node.get_logger().info('Stopping one cycle auto')
            self.robot.stop_navigation_client()
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.auto_btn.setStyleSheet(ACTION_BTN_CSS)
            self._set_operation_status("Status: One Cycle Auto Stopped", 'warning')
            return

        if self.robot.is_excavating or self.robot.is_depositing:
            self.robot.node.get_logger().warning('Operation already in progress')
            return

        self.robot.node.get_logger().info('Starting one cycle auto')
        self.full_auto_active = True
        self.auto_btn.setText("Stop One Cycle Auto")
        self.auto_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
        self._set_operation_status("Status: One Cycle Auto Active...", 'warning')

        if not self.robot.launch_navigation_client():
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.auto_btn.setStyleSheet(ACTION_BTN_CSS)
            self._set_operation_status("Status: Failed to start", 'error')

    # -------------------------------------------------------------------------
    # Teleop
    # -------------------------------------------------------------------------

    def adjust_speed(self, speed_type, multiplier):
        if speed_type == 'linear':
            self.linear_speed = min(self.max_linear_speed, max(0.0, self.linear_speed * multiplier))
        elif speed_type == 'angular':
            self.angular_speed = min(self.max_angular_speed, max(0.0, self.angular_speed * multiplier))
        speed_html = (
            f'<div style="text-align: right; line-height: 1.2;">'
            f'<span style="color: #aaa; font-size: 13px;">Lin: {self.linear_speed:.2f} m/s | Ang: {self.angular_speed:.2f} rad/s</span>'
            f'<br><span style="color: #888; font-size: 11px; font-style: italic;">Q/Z: Linear Speed | E/C: Angular Speed</span>'
            f'</div>'
        )
        self.speed_label.setText(speed_html)
        self.robot.node.get_logger().info(
            f"Speed: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")

    def teleop_key_press(self, key):
        if self.robot.is_real_mode:
            return
        self.teleop_keys[key] = True
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_active_style)
        self.publish_teleop_velocity()

    def teleop_key_release(self, key):
        if self.robot.is_real_mode:
            return
        self.teleop_keys[key] = False
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_button_style)
        self.publish_teleop_velocity()

    def publish_teleop_velocity(self):
        linear_x = 0.0
        angular_z = 0.0
        if self.teleop_keys['w']:
            linear_x = self.linear_speed
        elif self.teleop_keys['s']:
            linear_x = -self.linear_speed
        if self.teleop_keys['a']:
            angular_z = self.angular_speed
        elif self.teleop_keys['d']:
            angular_z = -self.angular_speed
        self.robot.publish_velocity(linear_x, angular_z)

        if self.teleop_keys['up']:
            new_pos = max(self.robot.bucket_position - TELEOP_BUCKET_SPEED, TELEOP_BUCKET_LIMIT_MIN)
            self.robot.publish_bucket_position(new_pos)
            self.robot.bucket_position = new_pos
        elif self.teleop_keys['down']:
            new_pos = min(self.robot.bucket_position + TELEOP_BUCKET_SPEED, TELEOP_BUCKET_LIMIT_MAX)
            self.robot.publish_bucket_position(new_pos)
            self.robot.bucket_position = new_pos

    # -------------------------------------------------------------------------
    # ROS / UI update callbacks
    # -------------------------------------------------------------------------

    def handle_robot_state_update(self):
        if hasattr(self, 'mode_label') and self.robot.is_real_mode:
            is_manual = (self.robot.robot_mode == "MANUAL")
            self.mode_label.setText("Manual" if is_manual else "Auto")
            if is_manual:
                self.mode_label.setStyleSheet(
                    f"color: {Colors.STATUS_SUCCESS}; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Auto")
            else:
                self.mode_label.setStyleSheet(
                    f"color: {Colors.STATUS_ERROR}; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Manual")

        _system_btns = [
            ('pointlio_btn',  'pointlio',  'Point-LIO',      'Stop Point-LIO'),
            ('mapping_btn',   'mapping',   'RTAB-Map',       'Stop RTAB-Map'),
            ('nav2_btn',      'nav2',      'Navigation2',    'Stop Navigation2'),
            ('hardware_btn',  'hardware',  'Launch Hardware', 'Stop Hardware'),
        ]
        for attr, key, off_text, on_text in _system_btns:
            if hasattr(self, attr):
                btn = getattr(self, attr)
                running = self.robot.launch_processes.get(key)
                btn.setText(on_text if running else off_text)
                btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS if running else ACTION_BTN_CSS)

        current_status = self.operation_status_label.text()
        preserve = any(keyword in current_status for keyword in [
            "Excavating", "Depositing", "One Cycle Auto Active", "Navigating"
        ])

        if self.robot.robot_disabled:
            if not preserve:
                self._set_operation_status("Status: Disabled", 'error')
            self.excavate_btn.setEnabled(False)
            self.excavate_btn.setStyleSheet(ACTION_BTN_DISABLED_CSS)
            self.deposit_btn.setEnabled(False)
            self.deposit_btn.setStyleSheet(ACTION_BTN_DISABLED_CSS)
            self.auto_btn.setEnabled(False)
            self.auto_btn.setStyleSheet(ACTION_BTN_DISABLED_CSS)
            if not self.emergency_stopped:
                self.emergency_stopped = True
                self.emergency_stop_btn.setText("Re-Enable Robot")
                self.emergency_stop_btn.setStyleSheet(ESTOP_BTN_ACTIVE_CSS)
        else:
            if not preserve:
                self._set_operation_status("Status: Active", 'active')
            
            self.excavate_btn.setEnabled(True)
            self.deposit_btn.setEnabled(True)
            self.auto_btn.setEnabled(True)
            
            if self.robot.is_excavating:
                self.excavate_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            else:
                self.excavate_btn.setStyleSheet(ACTION_BTN_CSS)
            
            if self.robot.is_depositing:
                self.deposit_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            else:
                self.deposit_btn.setStyleSheet(ACTION_BTN_CSS)
            
            if self.full_auto_active:
                self.auto_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            else:
                self.auto_btn.setStyleSheet(ACTION_BTN_CSS)
            
            if self.emergency_stopped:
                self.emergency_stopped = False
                self.emergency_stop_btn.setText("Emergency Stop")
                self.emergency_stop_btn.setStyleSheet(ESTOP_BTN_NORMAL_CSS)

    def handle_control_state_update(self, msg):
        if msg.is_excavating:
            self.excavate_btn.setText("Cancel Excavation")
            self.excavate_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            if not self.robot.robot_disabled:
                self.excavate_btn.setEnabled(True)
            self._set_operation_status("Status: Excavating", 'warning')
        else:
            self.excavate_btn.setText("Excavate")
            self.excavate_btn.setStyleSheet(ACTION_BTN_CSS)
            if not self.robot.robot_disabled:
                self.excavate_btn.setEnabled(True)

        if msg.is_depositing:
            self.deposit_btn.setText("Cancel Deposit")
            self.deposit_btn.setStyleSheet(ACTION_BTN_ACTIVE_CSS)
            if not self.robot.robot_disabled:
                self.deposit_btn.setEnabled(True)
            self._set_operation_status("Status: Depositing", 'warning')
        else:
            self.deposit_btn.setText("Deposit")
            self.deposit_btn.setStyleSheet(ACTION_BTN_CSS)
            if not self.robot.robot_disabled:
                self.deposit_btn.setEnabled(True)

        if msg.is_navigating:
            self._set_operation_status("Status: Navigating", 'info')
        elif msg.status_message and not (msg.is_excavating or msg.is_depositing):
            self._set_operation_status(f"Status: {msg.status_message}", 'idle')
        elif not (msg.is_excavating or msg.is_depositing or msg.is_navigating):
            self._set_operation_status("Status: Idle", 'idle')

    def update_ui(self):
        # Process queued callbacks from ROS thread
        while not self._log_queue.empty():
            try:
                event_type, data = self._log_queue.get_nowait()
                if event_type == 'robot_state':
                    self.handle_robot_state_update()
                elif event_type == 'control_state':
                    self.handle_control_state_update(data)
                elif event_type == 'log':
                    self.append_log(data)
                elif event_type == 'excavate_response':
                    self.excavate_goal_response_callback(data)
                elif event_type == 'excavate_result':
                    self.excavate_result_callback(data)
                elif event_type == 'excavate_cancel':
                    self.excavate_cancel_callback(data)
                elif event_type == 'deposit_response':
                    self.deposit_goal_response_callback(data)
                elif event_type == 'deposit_result':
                    self.deposit_result_callback(data)
                elif event_type == 'deposit_cancel':
                    self.deposit_cancel_callback(data)
            except:
                break
        
        self.bandwidth_total_label.setText(f"{self.robot.bandwidth_avg_total:.2f} Mbps")
        self.bandwidth_total_current_label.setText(f"{self.robot.bandwidth_total:.2f} Mbps")
        self.bandwidth_rx_current_label.setText(f"{self.robot.bandwidth_rx:.2f} Mbps")
        self.bandwidth_tx_current_label.setText(f"{self.robot.bandwidth_tx:.2f} Mbps")

        pct = min(100, int((self.robot.bandwidth_avg_total / BANDWIDTH_MAX_MBPS) * 100))
        self.bandwidth_progress.setValue(pct)
        if pct >= int(BANDWIDTH_CRITICAL_THRESHOLD * 100):
            bw_color = Colors.STATUS_ERROR
        elif pct >= int(BANDWIDTH_WARN_THRESHOLD * 100):
            bw_color = Colors.STATUS_WARNING
        else:
            bw_color = Colors.STATUS_SUCCESS
        self.bandwidth_progress.setStyleSheet(f"QProgressBar::chunk {{ background-color: {bw_color}; }}")

        self.linear_vel_label.setText(f"{self.robot.linear_velocity:.2f}")
        self.angular_vel_label.setText(f"{self.robot.angular_velocity:.2f}")

        self.position_x_label.setText(f"{self.robot.position_x:.2f}")
        self.position_y_label.setText(f"{self.robot.position_y:.2f}")
        self.position_z_label.setText(f"{self.robot.position_z:.2f}")
        self.orientation_roll_label.setText(f"{self.robot.orientation_roll:.2f}")
        self.orientation_pitch_label.setText(f"{self.robot.orientation_pitch:.2f}")
        self.orientation_yaw_label.setText(f"{self.robot.orientation_yaw:.2f}")

        self.bucket_angle_label.setText(f"{self.robot.bucket_position:.2f} rad")

        if self.robot.vibration_duty_cycle > 0.01:
            self.vibration_state_label.setText(f"ON ({self.robot.vibration_duty_cycle:.2f})")
            self.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_SUCCESS};")
        else:
            self.vibration_state_label.setText("OFF")
            self.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_ERROR};")

        self.power_voltage_label.setText(f"{self.robot.power_voltage:.2f}")
        self.power_current_label.setText(f"{self.robot.power_current:.2f}")
        self.power_watts_label.setText(f"{self.robot.power_watts:.2f}")
        self.power_energy_label.setText(f"{self.robot.power_energy_wh:.2f}")
        self.power_temp_label.setText(f"{self.robot.power_temp:.1f}")

        if not self.robot.is_real_mode and any(self.teleop_keys.values()):
            self.publish_teleop_velocity()

        self.bucket_slider.set_position(self.robot.bucket_position)

    def update_cameras(self):
        if not CV_AVAILABLE:
            return

        prev_id = self.last_camera_frame_ids.get('fisheye')
        self.update_camera_display(self.fisheye_camera_label, self.robot.fisheye_camera_image, 'fisheye')
        if self.last_camera_frame_ids.get('fisheye') != prev_id:
            self._fisheye_frame_count += 1

        now = time.monotonic()
        elapsed = now - self._fisheye_fps_time
        if elapsed >= 1.0:
            self.fisheye_fps_label.setText(f"FPS: {self._fisheye_frame_count / elapsed:.1f}")
            self._fisheye_frame_count = 0
            self._fisheye_fps_time = now

    def update_camera_display(self, label, cv_image, camera_name):
        if cv_image is None:
            if label.pixmap() is not None:
                label.clear()
                label.setText("No camera feed")
                self.last_camera_frame_ids[camera_name] = None
            return

        frame_id = id(cv_image)
        if self.last_camera_frame_ids.get(camera_name) == frame_id:
            return
        self.last_camera_frame_ids[camera_name] = frame_id

        height, width = cv_image.shape[:2]
        scale = min(label.width() / width, label.height() / height)
        new_w, new_h = int(width * scale), int(height * scale)

        resized = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(qt_image))

    # -------------------------------------------------------------------------
    # Key events
    # -------------------------------------------------------------------------

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            event.accept()
            return
        if event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showNormal()
            event.accept()
            return
        if not self.robot.is_real_mode and not event.isAutoRepeat():
            if event.key() in _TELEOP_MOVE_KEYS:
                self.teleop_key_press(_TELEOP_MOVE_KEYS[event.key()])
                event.accept()
                return
            if event.key() in _TELEOP_SPEED_KEYS:
                self.adjust_speed(*_TELEOP_SPEED_KEYS[event.key()])
                event.accept()
                return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not self.robot.is_real_mode and not event.isAutoRepeat():
            if event.key() in _TELEOP_MOVE_KEYS:
                self.teleop_key_release(_TELEOP_MOVE_KEYS[event.key()])
                event.accept()
                return
        super().keyReleaseEvent(event)

    # -------------------------------------------------------------------------
    # Logging
    # -------------------------------------------------------------------------

    def append_log(self, message):
        if not hasattr(self, 'terminal_text'):
            return
        level_colors = {
            'DEBUG': '#757575',
            'INFO':  '#e0e0e0',
            'WARN':  '#ffca28',
            'ERROR': '#ef5350',
            'FATAL': '#ff5252',
        }
        try:
            match = re.match(r'\[(\w+)\]', message)
            if match:
                color = level_colors.get(match.group(1), '#e0e0e0')
                html = f'<span style="color:{color}">{self.ansi_to_html(message)}</span>'
            else:
                html = self.ansi_to_html(message)
            self.terminal_text.append(html)
            self.terminal_text.verticalScrollBar().setValue(
                self.terminal_text.verticalScrollBar().maximum())
        except Exception as e:
            self.terminal_text.append(
                message.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))

    def ansi_to_html(self, text):
        ansi_colors = {
            '30': '#000000', '31': '#ef5350', '32': '#66bb6a', '33': '#ffca28',
            '34': '#42a5f5', '35': '#ab47bc', '36': '#26c6da', '37': '#e0e0e0',
            '90': '#757575', '91': '#ff5252', '92': '#69f0ae', '93': '#ffd54f',
            '94': '#448aff', '95': '#e040fb', '96': '#18ffff', '97': '#ffffff',
        }
        result = text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
        open_spans = 0

        def replace_color(match):
            nonlocal open_spans
            codes = match.group(1).split(';')
            if '0' in codes or codes == ['']:
                if open_spans > 0:
                    open_spans -= 1
                    return '</span>'
                return ''
            styles = []
            for code in codes:
                if code == '1':
                    styles.append('font-weight:bold')
                elif code in ansi_colors:
                    styles.append(f'color:{ansi_colors[code]}')
            if styles:
                open_spans += 1
                return f'<span style="{";".join(styles)}">'
            return ''

        result = re.sub(r'\x1b\[([0-9;]*)m', replace_color, result)
        result += '</span>' * open_spans
        return result

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def _do_shutdown(self):
        self._ros_executor.shutdown(timeout_sec=1.0)
        self._ros_thread.join(timeout=2.0)
        self.ui_timer.stop()
        self.camera_timer.stop()
        QApplication.processEvents()
        self.robot.shutdown()

    def closeEvent(self, event):
        self._do_shutdown()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    app.setApplicationName("Lunabot")
    app.setApplicationDisplayName("Lunabot Control Panel")
    app.setDesktopFileName("lunabot_gui")

    try:
        pkg_dir = get_package_share_directory('lunabot_gui')
        icon_path = os.path.join(pkg_dir, 'resource', 'lunabot_icon.png')
        if os.path.exists(icon_path):
            app.setWindowIcon(QIcon(icon_path))
    except Exception:
        pass

    app.setStyle('Fusion')

    palette = QPalette()
    palette.setColor(QPalette.Window,          QColor(*Colors.PALETTE_WINDOW))
    palette.setColor(QPalette.WindowText,      QColor(255, 255, 255))
    palette.setColor(QPalette.Base,            QColor(*Colors.PALETTE_BASE))
    palette.setColor(QPalette.AlternateBase,   QColor(*Colors.PALETTE_WINDOW))
    palette.setColor(QPalette.ToolTipBase,     QColor(*Colors.PALETTE_TOOLTIP_BASE))
    palette.setColor(QPalette.ToolTipText,     QColor(255, 255, 255))
    palette.setColor(QPalette.Text,            QColor(255, 255, 255))
    palette.setColor(QPalette.Button,          QColor(*Colors.PALETTE_WINDOW))
    palette.setColor(QPalette.ButtonText,      QColor(255, 255, 255))
    palette.setColor(QPalette.BrightText,      QColor(255, 0, 0))
    palette.setColor(QPalette.Link,            QColor(*Colors.PALETTE_LINK))
    palette.setColor(QPalette.Highlight,       QColor(*Colors.PALETTE_HIGHLIGHT))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    signal.signal(signal.SIGINT,  lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    window = LunabotGUI()
    app.aboutToQuit.connect(window._do_shutdown)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
