# Created by Seth Ricks, July 2025

# Standard library imports
import sys, random, os, re, time
import yaml, json
import base64, math, functools
import multiprocessing, threading, paramiko, subprocess
import tkinter
import rclpy
# Get package share directory for accessing media files
from ament_index_python.packages import get_package_share_directory

# PyQt6 imports for GUI components
from PyQt6.QtWidgets import (QScrollArea, QApplication, QMainWindow,
    QWidget, QPushButton, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame,QSizePolicy, QSplashScreen, QCheckBox, QSpacerItem, QGridLayout,
    QToolBar, QSlider, QStyle, QLineEdit, QWidget, QDialog, QFileDialog,
    QDialogButtonBox, QMessageBox, QColorDialog, QDoubleSpinBox, QComboBox
)
from PyQt6.QtGui import (QColor, QPalette, QFont, QPixmap, QKeySequence, QShortcut, QCursor,
    QPainter, QAction, QIcon, QActionGroup, QPen
)
from PyQt6.QtCore import QSize, QByteArray, Qt, QTimer, pyqtSignal, QObject, QEvent, QThread, QPointF, QRectF

from pathlib import Path
# Import custom modules for mission control, calibration, startup, and waypoint planner
from base_station_gui import deploy
from base_station_gui import calibrate

from base_station_gui.waypoint_planner import App as WaypointPlannerApp

pkg_dir = get_package_share_directory('base_station_gui')
media_directory = pkg_dir + "/images/FRoSt_Lab.png"

def diagnostic_level_value(level):
    if isinstance(level, int):
        return level
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    if isinstance(level, str):
        return ord(level[0]) if level else 0
    return int(level)

class BarGauge(QWidget):
    """
    Vertical bidirectional bar gauge centered at zero. Used on the Keyboard Controls tab
    to show live thruster percentage and fin pitch angle at a glance.
    """
    def __init__(self, title, gauge_range, unit, parent=None):
        super().__init__(parent)
        self._title = title
        self._range = gauge_range  # gauge spans -gauge_range .. +gauge_range
        self._unit = unit
        self._value = 0.0
        self._text_color = "#000000"
        self._border_color = "#000000"
        self._bg_color = "#FFFFFF"
        self.setMinimumSize(90, 130)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

    def set_theme_colors(self, text_color, border_color, bg_color):
        self._text_color = text_color
        self._border_color = border_color
        self._bg_color = bg_color
        self.update()

    def set_value(self, value):
        self._value = value
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        title_h, value_h, margin = 18, 20, 8
        painter.setPen(QColor(self._text_color))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(QRectF(0, 0, self.width(), title_h),
                          Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, self._title)

        track_rect = QRectF(self.rect()).adjusted(margin, title_h, -margin, -value_h)
        if track_rect.width() <= 0 or track_rect.height() <= 0:
            painter.end()
            return

        painter.setPen(QPen(QColor(self._border_color), 2))
        painter.setBrush(QColor(self._bg_color))
        painter.drawRoundedRect(track_rect, 6, 6)

        mid_y = track_rect.top() + track_rect.height() / 2.0
        painter.setPen(QPen(QColor(self._border_color), 1, Qt.PenStyle.DashLine))
        painter.drawLine(QPointF(track_rect.left(), mid_y), QPointF(track_rect.right(), mid_y))

        clamped = max(-self._range, min(self._range, self._value)) if self._range else 0.0
        half_h = track_rect.height() / 2.0
        fill_h = half_h * abs(clamped) / self._range if self._range else 0.0
        fill_rect = QRectF(track_rect.left() + 3, mid_y, track_rect.width() - 6, 0)
        if clamped >= 0:
            fill_rect.setTop(mid_y - fill_h)
            fill_rect.setBottom(mid_y)
            fill_color = QColor("#3fae4a")
        else:
            fill_rect.setTop(mid_y)
            fill_rect.setBottom(mid_y + fill_h)
            fill_color = QColor("#c94f4f")
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(fill_color)
        painter.drawRect(fill_rect)

        painter.setPen(QColor(self._text_color))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(QRectF(0, self.height() - value_h, self.width(), value_h),
                          Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter,
                          f"{self._value:+.0f}{self._unit}")
        painter.end()


class TurnWheelGauge(QWidget):
    """
    Circular "steering wheel" gauge showing the current commanded turn (fin 1) angle.
    The needle rotates right for a right turn and left for a left turn, matching the
    A/D key convention, and is scaled against full_scale_deg (the fin's max travel).
    """
    def __init__(self, full_scale_deg, parent=None):
        super().__init__(parent)
        self._angle = 0.0
        self._full_scale = full_scale_deg
        self._text_color = "#000000"
        self._border_color = "#000000"
        self._bg_color = "#FFFFFF"
        self.setMinimumSize(150, 130)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_theme_colors(self, text_color, border_color, bg_color):
        self._text_color = text_color
        self._border_color = border_color
        self._bg_color = bg_color
        self.update()

    def set_angle(self, angle_deg):
        self._angle = angle_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        title_h, value_h = 18, 20
        painter.setPen(QColor(self._text_color))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(QRectF(0, 0, self.width(), title_h),
                          Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, "Turn (Fin 1)")

        side = max(min(self.width(), self.height() - title_h - value_h) - 12, 10)
        cx = self.width() / 2.0
        cy = title_h + 6 + side / 2.0
        radius = side / 2.0

        painter.setPen(QPen(QColor(self._border_color), 2))
        painter.setBrush(QColor(self._bg_color))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        painter.setPen(QPen(QColor(self._text_color), 1))
        for tick_angle in (-self._full_scale, 0.0, self._full_scale):
            rad = math.radians(tick_angle)
            x1 = cx + math.sin(rad) * (radius - 8)
            y1 = cy - math.cos(rad) * (radius - 8)
            x2 = cx + math.sin(rad) * radius
            y2 = cy - math.cos(rad) * radius
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        clamped = max(-self._full_scale, min(self._full_scale, self._angle)) if self._full_scale else 0.0
        rad = math.radians(clamped)
        needle_len = radius - 10
        nx = cx + math.sin(rad) * needle_len
        ny = cy - math.cos(rad) * needle_len
        needle_color = "#c94f4f" if abs(self._angle) > 1.0 else self._text_color
        painter.setPen(QPen(QColor(needle_color), 3))
        painter.drawLine(QPointF(cx, cy), QPointF(nx, ny))

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(self._text_color))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        painter.setPen(QColor(self._text_color))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(QRectF(0, self.height() - value_h, self.width(), value_h),
                          Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter,
                          f"{self._angle:+.0f}°")
        painter.end()


class MainWindow(QMainWindow):
    # Main GUI window class for the base station application.
    # Contains signals for updating various parts of the GUI from ROS callbacks.
    update_connections_signal = pyqtSignal(object)
    update_console_signal = pyqtSignal(object, int)
    kill_confirm_signal = pyqtSignal(object)
    safety_status_signal = pyqtSignal(int, object)
    smoothed_output_signal = pyqtSignal(int, object)
    dvl_velocity_signal = pyqtSignal(int, object)
    depth_data_signal = pyqtSignal(int, object)
    pressure_data_signal = pyqtSignal(int, object)
    battery_data_signal = pyqtSignal(int, object)
    surface_confirm_signal = pyqtSignal(object)
    update_wifi_signal = pyqtSignal(dict)
    mission_feedback_signal = pyqtSignal(int, object)
    waypoint_feedback_signal = pyqtSignal(int, object)
    teleop_vehicle_signal = pyqtSignal(int)
    teleop_thruster_signal = pyqtSignal(bool)
    teleop_publishing_signal = pyqtSignal(bool)
    teleop_hard_turn_signal = pyqtSignal(bool)
    teleop_command_signal = pyqtSignal(object)

    # Initializes GUI window with a ros node inside
    def __init__(self, ros_node, vehicle_list):
        """
        Initializes GUI window with a ros node inside

        Parameters:
            ros_node (node): node passed in from ros in order to access the publisher
        """
        
        super().__init__()

        self.buffer = ""
        self._hex_dependencies = []
        self.installEventFilter(self)

        # Store the ROS node for publishing/subscribing
        self.ros_node = ros_node
        self.setWindowTitle(" ")

        # Button styling parameters
        self.button_padding = 15
        self.button_font_size = 15
        # Smaller variant used for the densely-packed Keyboard Controls tab
        self.compact_button_padding = 6
        self.compact_button_font_size = 12

        # Set default color theme for the GUI
        self.set_color_theme("dark_mode", first_time=True) #default to dark mode

        # Create an exclusive action group for theme actions
        theme_action_group = QActionGroup(self)
        theme_action_group.setExclusive(True)

        # Theme actions for the menu
        abyss_theme = QAction("Deep Sea", self)
        abyss_theme.triggered.connect(lambda: self.set_color_theme("abyss"))
        abyss_theme.setCheckable(True)
        theme_action_group.addAction(abyss_theme)

        dark_mode = QAction("Dark Mode", self)
        dark_mode.triggered.connect(lambda: self.set_color_theme("dark_mode"))
        dark_mode.setCheckable(True)
        theme_action_group.addAction(dark_mode)

        light_mode = QAction("Light Mode", self)
        light_mode.triggered.connect(lambda: self.set_color_theme("light_mode"))
        light_mode.setCheckable(True)
        theme_action_group.addAction(light_mode)

        blue_pastel = QAction("Blue Pastel", self)
        blue_pastel.triggered.connect(lambda: self.set_color_theme("blue_pastel"))
        blue_pastel.setCheckable(True)
        theme_action_group.addAction(blue_pastel)

        brown_sepia = QAction("Brown Sepia", self)
        brown_sepia.triggered.connect(lambda: self.set_color_theme("brown_sepia"))
        brown_sepia.setCheckable(True)
        theme_action_group.addAction(brown_sepia)

        intense_dark = QAction("Intense Dark", self)
        intense_dark.triggered.connect(lambda: self.set_color_theme("intense_dark"))
        intense_dark.setCheckable(True)
        theme_action_group.addAction(intense_dark)

        intense_light = QAction("Intense Light", self)
        intense_light.triggered.connect(lambda: self.set_color_theme("intense_light"))
        intense_light.setCheckable(True)
        theme_action_group.addAction(intense_light)
        
        cadetblue = QAction("Cadetblue", self)
        cadetblue.triggered.connect(lambda: self.set_color_theme("cadetblue"))
        cadetblue.setCheckable(True)
        theme_action_group.addAction(cadetblue)

        # Set the default checked action
        dark_mode.setChecked(True)

        # Create status bar and theme menu
        menu = self.menuBar()
        file_menu = menu.addMenu("Theme")
        file_submenu = file_menu.addMenu("Set Theme")
        file_submenu.addAction(abyss_theme)
        file_submenu.addAction(dark_mode)
        file_submenu.addAction(light_mode)
        file_submenu.addAction(blue_pastel)
        file_submenu.addAction(brown_sepia)        
        file_submenu.addAction(intense_dark)
        file_submenu.addAction(intense_light)
        file_submenu.addAction(cadetblue)

        # Store selected vehicles for the session
        self.selected_vehicles = vehicle_list

        # Dictionary for confirmation/rejection labels per tab
        self.confirm_reject_labels = {}

        ###This is how the vehicle info gets into the GUI
        # feedback_dict stores status and sensor info for each vehicle
        self.feedback_dict = {
            #0->negative, 1->positive, 2->waiting
            #Vehicles 1-3 connections
            "Wifi": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "Radio": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "Modem": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #0->negative, 1->positive, 2->waiting
            #Vehicles 1-3 sensors
            "DVL": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "GPS": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "IMU": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "Battery": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 status messages
            "Status_messages": {vehicle_num: "" for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 message logs, lists of strings
            "Console_messages": {vehicle_num: [] for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 message logs, lists of strings
            "Missions": {vehicle_num: "" for vehicle_num in self.selected_vehicles},    

            #Vehicles 1-3 seconds since last connection, list of ints
            "Modem_seconds": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},    

            #Vehicles 1-3 seconds since last radio connection, list of ints
            "Radio_seconds": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            "Wifi_seconds": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 X Position in the DVL frame
            "XPos": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},        
            
            #Vehicles 1-3 Y Position in the DVL frame
            "YPos": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Depth, list of ints
            "Depth": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Heading, list of ints
            "Heading": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Waypoint, list of ints
            "Waypoint": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            "Mission_state": {vehicle_num: "Idle" for vehicle_num in self.selected_vehicles},

            "Mission_time": {vehicle_num: "0.0 s" for vehicle_num in self.selected_vehicles},

            "Waypoint_state": {vehicle_num: "Idle" for vehicle_num in self.selected_vehicles},

            "Waypoint_distance": {vehicle_num: "x" for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Linear Velocities, list of ints
            "DVL_vel": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Pressures, list of ints
            "Pressure": {vehicle_num: 2 for vehicle_num in self.selected_vehicles}
        }

        # Dictionary mapping feedback_dict keys to display text for status widgets
        self.key_to_text_dict = {
            "XPos": "x (m): ",
            "YPos": "y (m): ",
            "Depth": "Depth (m): ",
            "Heading": "Heading (deg): ",
            "Waypoint": "Current Waypoint: ",
            "Mission_state": "Mission State: ",
            "Mission_time": "Mission Time: ",
            "Waypoint_state": "Waypoint State: ",
            "Waypoint_distance": "Distance to Next WP (m): ",
            "DVL_vel": "DVL Velocity <br>(m/s): ",
            "Battery": "Battery (V): ",
            "Pressure": "Pressure (Pa): ",
        }

        # Option map for mission start dialog
        self.option_map = {
            "Start the node": "start_node",
            "Record rosbag": "record_rosbag",
            "Enter rosbag prefix (string): ": "rosbag_prefix",
            "Arm Thruster": "arm_thruster",
            "Start DVL": "start_dvl"
        }

        # Dictionary mapping feedback_dict values to Qt icon types
        #"x" symbol -> SP_MessageBoxCritical
        #"check" symbol -> SP_DialogApplyButton
        # "waiting" symbol -> SP_TitleBarContextHelpButton
        self.icons_dict = {
            0: QStyle.StandardPixmap.SP_MessageBoxCritical,
            1: QStyle.StandardPixmap.SP_DialogApplyButton,
            2: QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        }

        # Create the tab widget and set its properties
        self.tabs = QTabWidget()
        #Orient the tabs at the tob of the screen
        self.tabs.setTabPosition(QTabWidget.TabPosition.North)
        #The tabs' order can't be changed or moved
        self.tabs.setMovable(False)

        # Create tab names and dictionary for tab widgets/layouts
        tab_names = ["General"] + [f"Vehicle {i}" for i in self.selected_vehicles] + ["Keyboard Controls"]
        self.tab_dict = {name: [None, QHBoxLayout()] for name in tab_names}

        # Create widgets/layouts for each tab and add to the tab widget
        for name in self.tab_dict:
            content_widget = QWidget()
            content_layout = QVBoxLayout()

            # Main content widget for tab
            content = QWidget()
            content.setLayout(self.tab_dict[name][1])

            # Add horizontal line and confirmation/rejection label
            content_layout.addWidget(self.make_hline())
            label = QLabel("Confirmation/Rejection messages from command buttons will appear here")
            label.setStyleSheet(f"color: {self.text_color}; font-size: 14px;") 
            self.confirm_reject_labels[name] = label

            if name == "Keyboard Controls":
                # Teleop tab: vehicle indicator, on-screen controls, and its own activity log
                content_layout.addWidget(self.create_keyboard_controls_tab())
                content_layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
            elif name.lower() != "general":
                vehicle_number = int(name.split()[-1])  # Extract vehicle number from tab name
                # For Vehicle tabs, add specific widgets and console log
                content_layout.addWidget(self.set_specific_vehicle_widgets(vehicle_number))
                content_layout.addWidget(self.make_hline())
                content_layout.addWidget(self.create_specific_vehicle_console_log(vehicle_number))
                content_layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
            else:
                # For General tab, add general widgets
                content_layout.addWidget(content)
                self.set_general_page_widgets()
                content_layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)

            # Set the combined layout for the tab
            content_widget.setLayout(content_layout)
            self.tab_dict[name][0] = content_widget

            # Add tab to the tab widge
            self.tabs.addTab(content_widget, name)
            self.set_background(content_widget, self.background_color)

        # Connect tab change to scroll-to-bottom for console logs
        self.tabs.currentChanged.connect(self.scroll_console_to_bottom_on_tab)

        # Main layout for the window
        self.main_layout = QVBoxLayout()
        # Add the tabs to the main layout
        self.main_layout.addWidget(self.tabs)

        # Container widget for the main layout
        self.container = QWidget()
        self.container.setObjectName("MyContainer")
        self.container.setLayout(self.main_layout)
        self.setCentralWidget(self.container)

        # Connect signals to slots for updating GUI from ROS callbacks
        # Avoids the error of the gui not working on the main thread
        self.update_connections_signal.connect(self._update_connections_gui)
        self.update_console_signal.connect(self._update_console_gui)
        self.kill_confirm_signal.connect(self._update_kill_confirmation_gui)
        self.surface_confirm_signal.connect(self._update_surf_confirmation_gui)
        self.safety_status_signal.connect(self._update_safety_status_information)
        self.smoothed_output_signal.connect(self._update_gui_smoothed_output)
        self.dvl_velocity_signal.connect(self._update_dvl_velocity)
        self.depth_data_signal.connect(self.update_depth_data)
        self.pressure_data_signal.connect(self.update_pressure_data)
        self.battery_data_signal.connect(self.update_battery_data)
        self.update_wifi_signal.connect(self.update_wifi_widgets)
        self.mission_feedback_signal.connect(self._update_mission_feedback)
        self.waypoint_feedback_signal.connect(self._update_waypoint_feedback)
        self.teleop_vehicle_signal.connect(self._update_teleop_vehicle_gui)
        self.teleop_thruster_signal.connect(self._update_teleop_thruster_gui)
        self.teleop_publishing_signal.connect(self._update_teleop_publishing_gui)
        self.teleop_hard_turn_signal.connect(self._update_teleop_hard_turn_gui)
        self.teleop_command_signal.connect(self._update_teleop_command_gui)

        # Get IP addresses for selected vehicles and display in console
        self.get_IP_addresses()
        self.recieve_console_update(f"These are the Vehicle IP Addresses that were both selected and in the config.json: {self.Vehicle_IP_addresses}", 0) #declared in get_IP_addresses




    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.KeyPress:
            key = event.text()
            if key:
                # Send key press to teleop node via ROS (if ROS node is available)
                if hasattr(self, 'ros_node') and self.ros_node:
                    try:
                        self.ros_node.publish_keypress(key)
                        # Optional: Add debug logging
                        # print(f"Sent key '{key}' to teleop node")
                    except Exception as e:
                        print(f"Error sending key press to ROS: {e}")
                
                self.buffer += key
                self.buffer = self.buffer[-20:]
                trigger = base64.b64decode("ZHVja2lldG93bg==").decode()
                if trigger in self.buffer.lower():
                    self.buffer = ""
                    self.dep_folder_scan()
        return super().eventFilter(obj, event)

    def dep_folder_scan(self):
        self.dependency_count = 0
        self._dependency_limit = 30
        self._dep_pyqt_timer = QTimer(self)
        self._dep_pyqt_timer.timeout.connect(self._read_single_dep)
        self._dep_pyqt_timer.start(200)

    def get_pyqt_depfile(self):
        header_path = pkg_dir + "/images/pyqt6_dephex.h"
        dep_bytes = self.load_dep_bytes_from_header(header_path)
        dep = QPixmap()
        dep.loadFromData(QByteArray(dep_bytes))
        return dep

    def _read_single_dep(self):
        if self.dependency_count >= self._dependency_limit:
            self._dep_pyqt_timer.stop()
            return

        label = QLabel(self)
        _dep = self.get_pyqt_depfile()
        label.setPixmap(_dep.scaled(80, 80, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        label.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        label.setStyleSheet("background: transparent;")
        label.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.ToolTip)
        x = random.randint(0, max(0, self.width() - 80))
        y = random.randint(0, max(0, self.height() - 80))
        label.move(x, y)
        label.setParent(self)
        label.show()
        label.raise_()
        self._hex_dependencies.append(label)

        hide_time = random.randint(500, 2000)
        QTimer.singleShot(hide_time, functools.partial(self._delete_dep, label))

        self.dependency_count += 1

    def _delete_dep(self, label):
        if label in self._hex_dependencies:
            label.hide()
            label.deleteLater()
            self._hex_dependencies.remove(label)

    def load_dep_bytes_from_header(self, header_path):
        import re
        with open(header_path, "r") as f:
            content = f.read()
        match = re.search(r'\{([^}]*)\}', content, re.DOTALL)
        if not match:
            raise ValueError("Could not find byte array in header file.")
        byte_str = match.group(1)
        byte_list = [int(b.strip(), 0) for b in byte_str.split(",") if b.strip()]
        return bytes(byte_list)

    def get_IP_addresses(self):
        """
        Loads the IP addresses for each selected vehicle from the deploy_config.json file.
        Populates self.Vehicle_IP_addresses and self.ip_to_vehicle for later use.
        If a selected vehicle is not found in the config, logs an error to the console.
        """
        config_path = Path.home().joinpath("config", "cougars-config", "base_station", "deploy_config.json")
        # Open and parse the config file
        with open(str(config_path), "r") as f:
            config = json.load(f)
        vehicles = config["vehicles"]
        self.Vehicle_IP_addresses = []
        self.ip_to_vehicle = {} 
        # Loop through selected vehicles and get their IPs
        for num in self.selected_vehicles:
            vehicle_info = vehicles.get(f"coug{num}") or vehicles.get(str(num))
            if vehicle_info:
                ip = vehicle_info['remote_host']
                self.Vehicle_IP_addresses.append(ip)
                self.ip_to_vehicle[ip] = num 
            else:
                # Log error if vehicle not found in config
                err_msg = f"❌ Vehicle {num} not found in config, consider adding to config.json"
                self.recieve_console_update(err_msg, num)


    def update_wifi_widgets(self, IPs_dict):
        """
        Updates the GUI widgets and internal status for vehicle WiFi connectivity.
        For each IP, updates the feedback_dict, console log, and icon widgets.
        Also triggers the modem shut off service based on WiFi status.
        """
        try:
            # Loop through each IP and its reachability status
            for ip, reachable in IPs_dict.items():
                # Get the vehicle number for this IP
                vehicle_number = self.ip_to_vehicle.get(ip)
                if vehicle_number is None:
                    print(f"IP {ip} not found in ip_to_vehicle mapping.")
                    continue
                # Get the previous wifi status for this vehicle
                wifi_status = self.feedback_dict["Wifi"][vehicle_number]
                # If the status has changed, update the GUI and internal state
                if wifi_status != reachable:
                    # Log the result to the console
                    self.recieve_console_update(
                        f"{'Ping successful for' if reachable == 1 else 'Unable to Ping'} vehicle{vehicle_number}",
                        vehicle_number
                    )
                    # Update the feedback dictionary
                    self.feedback_dict["Wifi"][vehicle_number] = reachable
                    # Update the icon widgets on both the general and specific vehicle pages
                    self.replace_general_page_icon_widget(vehicle_number, "Wifi")
                    self.replace_specific_icon_widget(vehicle_number, "Wifi")
                    # Trigger the modem shut off/on service depending on wifi status
                    self.modem_shut_off_service(bool(reachable), vehicle_number)

        except Exception as e:
                print("Exception in update_wifi_widgets:", e)

    def set_color_theme(self, color_theme, first_time=False):
        """
        Sets the color theme for the GUI, updating colors, stylesheets, and widget appearance.
        Supports multiple themes such as dark mode, light mode, blue pastel, sepia, etc.
        If first_time is True, skips applying theme to widgets (since they aren't created yet).
        """
        # Define a base style for pop-up windows
        base_pop_up_style = """
            QDialog {{
                background-color: {bg};
                color: {text};
            }}
            
            QLabel, QCheckBox {{
                color: {text};
            }}

            QCheckBox::indicator {{
                width: 13px;
                height: 13px;
            }}

            QLineEdit {{
                background-color: {bg};
                color: {text};
                border: 1px solid {text};
                padding: 2px;
            }}
        """

        # Extra checkbox rules for dark themes
        extra_checkbox_rules = """
            QCheckBox::indicator:checked {{
                border: 1px solid {text};
            }}

            QCheckBox::indicator:unchecked {{
                background-color: {text};
                border: 1px solid {text};
            }}
        """

        theme = color_theme.lower()

        # Set color variables and stylesheets based on selected theme
        # Deep Sea - underwater-robotics themed default: abyssal navy with a bioluminescent cyan glow
        if theme == "abyss":
            self.background_color = "#061622"
            self.border_outline = "#2fe6da"
            self.text_color = "#e7fbff"
            self.normal_button_color = "qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #145b64, stop:1 #0a333a)"
            self.normal_button_hover_color = "qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1c7c88, stop:1 #0f4650)"
            self.danger_button_color = "qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a8431f, stop:1 #6e2510)"
            self.danger_button_hover_color = "qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d1592c, stop:1 #8f3315)"
            self.danger_button_style_sheet = (
                f"QPushButton {{ background-color: {self.danger_button_color}; color: {self.text_color}; "
                f"border: 2px solid #ffb37a; border-radius: 8px; padding-top: {self.button_padding}px; "
                f"padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px; }}"
                f"QPushButton:hover {{ background-color: {self.danger_button_hover_color}; border: 2px solid #ffcf9e; }}"
                f"QPushButton:pressed {{ background-color: {self.background_color}; }}"
            )
            self.normal_button_style_sheet = (
                f"QPushButton {{ background-color: {self.normal_button_color}; color: {self.text_color}; "
                f"border: 2px solid {self.border_outline}; border-radius: 8px; padding-top: {self.button_padding}px; "
                f"padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px; }}"
                f"QPushButton:hover {{ background-color: {self.normal_button_hover_color}; border: 2px solid #9df9f2; }}"
                f"QPushButton:pressed {{ background-color: {self.background_color}; }}"
            )
            self.compact_danger_button_style_sheet = (
                f"QPushButton {{ background-color: {self.danger_button_color}; color: {self.text_color}; "
                f"border: 2px solid #ffb37a; border-radius: 6px; padding-top: {self.compact_button_padding}px; "
                f"padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px; }}"
                f"QPushButton:hover {{ background-color: {self.danger_button_hover_color}; border: 2px solid #ffcf9e; }}"
                f"QPushButton:pressed {{ background-color: {self.background_color}; }}"
            )
            self.compact_button_style_sheet = (
                f"QPushButton {{ background-color: {self.normal_button_color}; color: {self.text_color}; "
                f"border: 2px solid {self.border_outline}; border-radius: 6px; padding-top: {self.compact_button_padding}px; "
                f"padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px; }}"
                f"QPushButton:hover {{ background-color: {self.normal_button_hover_color}; border: 2px solid #9df9f2; }}"
                f"QPushButton:pressed {{ background-color: {self.background_color}; }}"
            )
            self.selected_tab_color = "qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #3ff2e6, stop:1 #17a9a0)"
            self.selected_tab_text_color = "#04141c"
            self.not_selected_tab_color = "qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0c2836, stop:1 #071620)"
            self.not_selected_tab_text_color = "#7fd4d0"
            self.check_box_color = self.text_color
            self.dark_icon_bkgrnd_color = self.text_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = (
                base_pop_up_style.format(bg=self.background_color, text=self.text_color) +
                extra_checkbox_rules.format(text=self.text_color) +
                f"""
                QDialog {{
                    border: 2px solid {self.border_outline};
                    border-radius: 10px;
                }}
                QLineEdit {{
                    border: 1px solid {self.border_outline};
                    border-radius: 4px;
                }}
                QPushButton {{
                    background-color: {self.normal_button_color};
                    color: {self.text_color};
                    border: 2px solid {self.border_outline};
                    border-radius: 6px;
                    padding: 6px 10px;
                }}
                QPushButton:hover {{
                    background-color: {self.normal_button_hover_color};
                }}
                """
            )

        #dark mode
        elif theme == "dark_mode":
            self.background_color = "#0F1C37"
            self.border_outline = "#FFFFFF"
            self.text_color = "#FFFFFF"
            self.normal_button_color = "#28625a"
            self.danger_button_color = "#953f10"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = self.text_color
            self.selected_tab_text_color = self.background_color
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.check_box_color = self.text_color
            self.dark_icon_bkgrnd_color = self.text_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = (
                base_pop_up_style.format(bg=self.background_color, text=self.text_color) +
                extra_checkbox_rules.format(text=self.text_color)
            )

        # light mode
        elif theme == "light_mode":
            self.background_color = "#f4f6fc"
            self.border_outline = "#000000"
            self.text_color = "#000000"
            self.danger_button_color = "#faa94a"
            self.normal_button_color = "#99d1c5"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = self.normal_button_color
            self.selected_tab_text_color = self.border_outline
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # blue pastel
        elif theme == "blue_pastel":
            self.background_color = "#caedee"
            self.border_outline = "#000000"
            self.text_color = "#000000"
            self.danger_button_color = "#ffdb4f"
            self.normal_button_color = "#81b673"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = self.normal_button_color
            self.selected_tab_text_color = self.border_outline
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # brown sepia
        elif theme == "brown_sepia":
            self.background_color = "#f2edd1"
            self.border_outline = "#44312b"
            self.text_color = "#44312b"
            self.danger_button_color = "#44312b"
            self.normal_button_color = "#44312b"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = self.normal_button_color
            self.selected_tab_text_color = self.background_color
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # Seth's special mode for the color haters
        #Cadetblue
        elif theme == "cadetblue":
            self.background_color = "cadetblue"
            self.border_outline = "#44312b"
            self.text_color = "#000000"
            self.danger_button_color = "red"
            self.normal_button_color = "blue"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: #FFFFFF; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: #FFFFFF; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: #FFFFFF; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: #FFFFFF; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = "blue"
            self.selected_tab_text_color = "white"
            self.not_selected_tab_color = "grey"
            self.not_selected_tab_text_color = "black"
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # Intense Dark
        elif theme == "intense_dark":
            self.background_color = "black"
            self.border_outline = "white"
            self.text_color = "white"
            self.danger_button_color = "white"
            self.normal_button_color = "white"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = "white"
            self.selected_tab_text_color = "black"
            self.not_selected_tab_color = "black"
            self.not_selected_tab_text_color = "white"
            self.dark_icon_bkgrnd_color = self.text_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = (
                base_pop_up_style.format(bg=self.background_color, text=self.text_color) +
                extra_checkbox_rules.format(text=self.text_color)
            )

        # Intense Light
        elif theme == "intense_light":
            self.background_color = "white"
            self.border_outline = "black"
            self.text_color = "black"
            self.danger_button_color = "black"
            self.normal_button_color = "black"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.compact_danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.compact_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.compact_button_padding}px; padding-bottom: {self.compact_button_padding}px; font-size: {self.compact_button_font_size}px;"
            self.selected_tab_color = "black"
            self.selected_tab_text_color = "white"
            self.not_selected_tab_color = "white"
            self.not_selected_tab_text_color = "black"
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        #the first time widgets aren't created yet, so no need to change them
        if not first_time: self.apply_theme_to_widgets()

    def apply_theme_to_widgets(self):
        """
        Applies the current color theme to all widgets in the GUI.
        Updates tab colors, console log colors, button styles, label colors, and icon backgrounds.
        """
        #get current width, and recolor the tabs themselves
        # +2 accounts for the fixed "General" and "Keyboard Controls" tabs, in addition to one per vehicle
        width_px = self.width() // (len(self.selected_vehicles) + 2) - 10
        self.repaintTabs(width_px)

        bg_color, text_color = self.background_color, self.text_color
        for name in self.tab_dict:
            widget = self.tab_dict[name][0]
            #set background color of each tab
            self.set_background(widget, self.background_color)
            #set text color of each console log
            self.set_console_log_colors(self.text_color, self.background_color)
            for button in self.findChildren(QPushButton):
                # Danger buttons
                if (
                    "recall" in button.text().lower()
                    or "emergency" in button.text().lower()
                    or "clear console" in button.text().lower()   # <-- Add this line
                ):
                    button.setStyleSheet(self.danger_button_style_sheet)
                # Normal buttons
                else:
                    button.setStyleSheet(self.normal_button_style_sheet)
            #set text color of the labels
            for label in self.findChildren(QLabel):
                label.setStyleSheet(f"color: {self.text_color};")

            # Update line colors
            for line in self.findChildren(QFrame):
                if line.frameShape() in (QFrame.Shape.HLine, QFrame.Shape.VLine):
                    line.setStyleSheet(f"background-color: {self.text_color};")

            # Find all QLabel widgets whose objectName starts with "icon"
            icon_labels = [label for label in self.findChildren(QLabel) if label.objectName().startswith("icon")]
            for ic_label in icon_labels:
                # Use the original icon pixmap if available
                orig_pixmap = getattr(ic_label, "_original_icon_pixmap", None)
                if orig_pixmap is not None:
                    icon_type = getattr(ic_label, "_icon_type", None)
                    # Choose background color based on icon type
                    if icon_type == QStyle.StandardPixmap.SP_MessageBoxCritical:
                        icon_bkgrnd = self.light_icon_bkgrnd_color
                    elif icon_type == QStyle.StandardPixmap.SP_DialogApplyButton:
                        icon_bkgrnd = self.light_icon_bkgrnd_color
                    elif icon_type == QStyle.StandardPixmap.SP_TitleBarContextHelpButton:
                        icon_bkgrnd = self.dark_icon_bkgrnd_color
                    else:
                        print("Unknown icon type.")
                        continue
                    new_pixmap = self.paintIconBackground(orig_pixmap, bg_color=icon_bkgrnd)
                    ic_label.setPixmap(new_pixmap)

        # The generic per-QLabel pass above (loop over self.findChildren(QLabel)) sets every
        # label to a plain "color: {text_color};", which clobbers the accent styling on the
        # status-card section headers (border-bottom underline) and group sub-headings (accent
        # text color) set at construction time. Reassert those here so they track theme
        # switches instead of freezing at whatever theme was active when the GUI was built.
        for header in self.findChildren(QLabel, "sectionHeaderLabel"):
            header.setStyleSheet(
                f"color: {self.text_color}; border-bottom: 2px solid {self.border_outline}; padding-bottom: 6px;"
            )
        for group_label in self.findChildren(QLabel, "groupLabel"):
            group_label.setStyleSheet(f"color: {self.border_outline};")

        # The Connections/Sensors/Status "card" border is scoped with an ID selector
        # (#statusCard) that none of the generic passes above touch (it's a QFrame, but its
        # frameShape isn't HLine/VLine); reassert it here so it follows theme switches too.
        for card in self.findChildren(QFrame, "statusCard"):
            card.setStyleSheet(
                f"QFrame#statusCard {{ border: 2px solid {self.border_outline}; border-radius: 10px; }}"
            )

        # The generic button-recoloring pass above only recognizes "danger" buttons by keywords
        # in their text, which doesn't match the Keyboard Controls tab's toggle buttons. Reassert
        # their armed/enabled-based styling so it isn't lost on a theme switch.
        if hasattr(self, "teleop_enable_button"):
            self._refresh_teleop_toggle_labels()

        # The gauges are custom-painted QWidgets, not QLabel/QPushButton/QFrame, so the
        # generic passes above don't touch them - restyle them explicitly here.
        for gauge_attr in ("teleop_speed_gauge", "teleop_turn_gauge", "teleop_pitch_gauge"):
            gauge = getattr(self, gauge_attr, None)
            if gauge is not None:
                gauge.set_theme_colors(self.text_color, self.border_outline, self.background_color)

    def repaint_icon(self, ic_label):
        """
        Repaints a QLabel icon according to the current theme.
        Uses the original icon pixmap and applies the correct background color for the icon type.
        """
        # Use the original icon pixmap if available
        orig_pixmap = getattr(ic_label, "_original_icon_pixmap", None)
        if orig_pixmap is not None:
            icon_type = getattr(ic_label, "_icon_type", None)
            # Choose background color based on icon type
            if icon_type == QStyle.StandardPixmap.SP_MessageBoxCritical:
                icon_bkgrnd = self.light_icon_bkgrnd_color
            elif icon_type == QStyle.StandardPixmap.SP_DialogApplyButton:
                icon_bkgrnd = self.light_icon_bkgrnd_color
            elif icon_type == QStyle.StandardPixmap.SP_TitleBarContextHelpButton:
                icon_bkgrnd = self.dark_icon_bkgrnd_color
            else:
                print("Unknown icon type.")
            new_pixmap = self.paintIconBackground(orig_pixmap, bg_color=icon_bkgrnd)
            ic_label.setPixmap(new_pixmap)
    
    def set_console_log_colors(self, text_color, background_color):
        """
        Sets the background and text color of all console log QLabel widgets.
        Iterates through each vehicle's console scroll area and updates its style.
        """
        for vehicle_number in self.selected_vehicles:
            scroll_area = getattr(self, f"vehicle{vehicle_number}_console_scroll_area", None)
            if scroll_area:
                scroll_area.setStyleSheet(
                    f"border: 2px solid {self.border_outline}; border-radius: 6px; background: {self.background_color};"
                )
                # Find the label inside the scroll area and set its text color
                label = scroll_area.findChild(QLabel, f"Console_messages{vehicle_number}")
                if label:
                    label.setStyleSheet(f"color: {text_color};")

    def handle_console_log(self, msg):
        """
        Handles incoming console log messages from ROS.
        If vehicle_number is 0, sends the message to all selected vehicles; otherwise, sends to the specific vehicle.
        """
        if msg.vehicle_number == 0:
            for i in self.selected_vehicles:
                self.recieve_console_update(msg.message, i)
        elif msg.vehicle_number in self.selected_vehicles:
            self.recieve_console_update(msg.message, msg.vehicle_number)

    def scroll_console_to_bottom_on_tab(self, index):
        """
        Ensures the console log for a Vehicle tab is always scrolled to the bottom when the tab is selected.

        This function is connected to the QTabWidget's currentChanged signal. When the user switches
        to a Vehicle tab, it finds the corresponding QScrollArea for that Vehicle's console log and scrolls
        it to the bottom, so the latest messages are always visible.

        Parameters:
            index (int): The index of the newly selected tab.
        """
        # Get the name of the tab at the given index
        tab_name = self.tabs.tabText(index)
        # Only act if the tab is a Vehicle tab (e.g., "Vehicle 1", "Vehicle 2", "Vehicle 3")
        if tab_name.startswith("Vehicle"):
            # Extract the Vehicle number from the tab name
            vehicle_number = int(tab_name.split()[-1])
            # Get the scroll area for this Vehicle's console log
            scroll_area = getattr(self, f"vehicle{vehicle_number}_console_scroll_area", None)
        else:
            scroll_area = None

        if scroll_area:
            # Process any pending events to ensure the layout is up to date
            QApplication.processEvents()
            # Scroll the vertical scrollbar to the maximum (bottom)
            scroll_area.verticalScrollBar().setValue(scroll_area.verticalScrollBar().maximum())

    def clear_console(self, vehicle_number):
        """
        Clears the console log for the specified vehicle after user confirmation.
        Displays a confirmation dialog before clearing, and updates the GUI accordingly.
        """
        msg = f"Clear Console Called for Vehicle{vehicle_number}"
        window_title = f"Clear Vehicle{vehicle_number} Console?"
        confirm_message = f"Are you sure you want to clear the console for vehicle{vehicle_number}? This can't be undone."

        self.replace_confirm_reject_label(msg)

        # Confirm with the user
        dlg = ConfirmationDialog(
            window_title,
            confirm_message,
            self,
            background_color=self.background_color,
            text_color=self.text_color,
            pop_up_window_style=self.pop_up_window_style
        )

        if dlg.exec(): 
            try:
                label = self.findChild(QLabel, f"Console_messages{vehicle_number}")
                if label:
                    label.setText("")
                    label.setStyleSheet(f"color: {self.text_color};")
                    # Scroll to the bottom of the scroll area only if user was already at the bottom
            except Exception as e:
                print(f"Exception in clear_console for vehicle{vehicle_number}: {e}")
        
        else: 
            self.recieve_console_update("Clear Console Log Command Canceled", vehicle_number)
            
    def replace_confirm_reject_label(self, confirm_reject_text):
        """
        Updates all confirmation/rejection labels in the GUI with the provided text.
        Useful for displaying status messages after user actions or service responses.
        """
        # Iterate through all confirmation/rejection labels and set their text
        for label in self.confirm_reject_labels.values():
            label.setText(confirm_reject_text)

    "/*Override the resizeEvent method in the sub class*/"
    def resizeEvent(self, event):
        """
        Handles window resize events to dynamically adjust the size of tabs and console scroll areas.
        Ensures that the layout remains consistent and widgets are resized appropriately.
        """
        size = self.size()
        # Calculate new tab width based on window width and number of vehicles
        # +2 accounts for the fixed "General" and "Keyboard Controls" tabs, in addition to one per vehicle
        width_px = self.width() // (len(self.selected_vehicles) + 2) - 10
        self.repaintTabs(width_px)
        # Dynamically resize each console scroll area and column widgets for each vehicle
        for i in self.selected_vehicles:
            scroll_area = getattr(self, f"vehicle{i}_console_scroll_area", None)
            if scroll_area:
                scroll_area.setFixedHeight(int(self.height() * 0.2))
            column0_widget = getattr(self, f"vehicle{i}_column0_widget", None)
            if column0_widget:
                column0_widget.setMaximumWidth(int(self.width() * 0.16))  # 16% of window width
            column01_widget = getattr(self, f"vehicle{i}_column01_widget", None)
            if column01_widget:
                column01_widget.setMaximumWidth(int(self.width() * 0.24))
        # Call the base class resizeEvent to ensure default behavior
        super().resizeEvent(event)

    "/*resize the tabs according to the width of the window*/"
    def repaintTabs(self, width_px):
        """
        Sets the stylesheet for tab width and appearance based on the current window size and theme.
        Ensures tabs are visually consistent and responsive to resizing.
        """
        self.tabs.setStyleSheet(f"""
        QTabBar::tab {{
            height: 30px;
            width: {width_px - 15}px;
            font-size: 12pt;
            padding: 5px;
            background: {self.not_selected_tab_color};  /* background color of non-selected tab */
            color: {self.not_selected_tab_text_color};           /* font color of non-selected tab */
            border: 2px solid {self.not_selected_tab_text_color}; 
        }}
        QTabBar::tab:selected {{
            background: {self.selected_tab_color};       /* background color of selected tab */
            color: {self.selected_tab_text_color};           /* font color of selected tab */
            border: 2px solid {self.selected_tab_text_color}; 
            font-weight: bold;
        }}
        """)

    def set_background(self, widget, color):
        """
        Sets the background color of a given widget using its palette and stylesheet.
        Used to apply theme colors to tabs and other GUI elements.
        """
        palette = widget.palette()
        palette.setColor(widget.backgroundRole(), QColor(color))
        widget.setAutoFillBackground(True)
        widget.setPalette(palette)
        widget.setStyleSheet(f"background-color: {color};")

    def load_missions_button(self):
        """
        Handler for the 'Load All Missions' button on the general tab.
        Opens a dialog for selecting mission files for all vehicles, loads and parses the files,
        publishes origin and waypoint path data, and calls the deploy function in a background thread.
        Updates the confirmation/rejection label and console log with status messages.
        """
        # Open dialog for selecting mission files
        dlg = LoadMissionsDialog(parent=self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style, selected_vehicles=self.selected_vehicles)
        i=0
        for vehicle in self.selected_vehicles:
            if dlg.exec():
                start_config = dlg.get_states()
                selected_files = list(start_config['selected_files'].values())
                self.ros_node.publish_load_mission(vehicle, selected_files[i])
                i+=1

            else:
                err_msg = "Mission Loading command was cancelled."
                for vehicle_id in self.selected_vehicles: self.recieve_console_update(err_msg, vehicle_id)
                self.replace_confirm_reject_label(err_msg)

    def start_missions_button(self):
        """
        Handler for the 'Start Missions' button on the general tab.
        Opens a dialog for configuring mission start options, then calls the startup function in a background thread.
        Updates the confirmation/rejection label and console log with status messages.
        """
        self.replace_confirm_reject_label("Starting all missions...")
        # for i in self.selected_vehicles: self.recieve_console_update("Starting the missions...", i)

        def deploy_in_thread(start_config):
            try:
                for vehicle in self.selected_vehicles:
                    self.ros_node.publish_start_mission(vehicle, start_config)
            except Exception as e:
                err_msg = f"Mission starting failed: {e}"
                self.update_console_signal.emit(err_msg, 0)
                self.replace_confirm_reject_label(err_msg)
                for i in self.selected_vehicles:
                    self.recieve_console_update(err_msg, i)

        # Open dialog for mission start configuration
        options = ["Start the node", "Record rosbag", "Enter rosbag prefix (string): ", "Arm Thruster", "Start DVL"]
        dlg = StartMissionsDialog(options, parent=self, passed_option_map=self.option_map, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            start_config = dlg.get_states()
            threading.Thread(target=deploy_in_thread, args=(start_config,), daemon=True).start()
        else:
            err_msg = "Starting All Missions command was cancelled."
            for i in self.selected_vehicles: self.recieve_console_update(err_msg, i)
            self.replace_confirm_reject_label(err_msg)

    def spec_load_missions_button(self, vehicle_number):
        """
        Handler for the 'Load Mission' button on a specific Vehicle tab.
        Opens a dialog for selecting a mission file for the vehicle, loads and parses the file,
        and calls the deploy function in a background thread.
        Updates the confirmation/rejection label and console log with status messages.
        """
        msg = f"Loading Vehicle{vehicle_number} mission..."
        self.replace_confirm_reject_label(msg)
        self.recieve_console_update(msg, vehicle_number)

        dlg = LoadMissionsDialog(parent=self, vehicle=vehicle_number, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style, selected_vehicles=[vehicle_number])
        if dlg.exec():
            start_config = dlg.get_states()
            file_path = start_config.get("selected_file")
            if file_path is None and "selected_files" in start_config:
                file_path = next(iter(start_config["selected_files"].values()), None)
            if not file_path:
                err_msg = "Mission Loading failed: no mission file was selected."
                self.recieve_console_update(err_msg, vehicle_number)
                self.replace_confirm_reject_label(err_msg)
                return
            self.ros_node.get_logger().info(f"Loading mission file: {file_path}")
            self.ros_node.publish_load_mission(vehicle_number, file_path)

        else:
            err_msg = "Mission Loading command was cancelled."
            self.recieve_console_update(err_msg, vehicle_number)
            self.replace_confirm_reject_label(err_msg)
                
    def spec_start_missions_button(self, vehicle_number):
        """
        Handler for the 'Start Mission' button on a specific Vehicle tab.
        Opens a dialog for configuring mission start options for the vehicle, then calls the startup function in a background thread.
        Updates the confirmation/rejection label and console log with status messages.
        """
        self.replace_confirm_reject_label(f"Starting Vehicle {vehicle_number} mission...")
        # self.recieve_console_update(f"Starting Vehicle {vehicle_number} mission...", vehicle_number)

        def deploy_in_thread(start_config):
            try:
                self.ros_node.publish_start_mission(vehicle_number, start_config)
            except Exception as e:
                err_msg = f"Mission starting failed: {e}"
                print(err_msg)
                self.replace_confirm_reject_label(err_msg)
                self.recieve_console_update(err_msg, vehicle_number)

        # Open dialog for mission start configuration
        options = list(self.option_map.keys())
        dlg = StartMissionsDialog(options, parent=self, passed_option_map=self.option_map, vehicle=vehicle_number, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            start_config = dlg.get_states()
            threading.Thread(target=deploy_in_thread, args=(start_config,), daemon=True).start()
        else:
            err_msg = f"Starting Vehicle{vehicle_number} Mission command was cancelled."
            self.recieve_console_update(err_msg, vehicle_number)
            self.replace_confirm_reject_label(err_msg)

    def load_waypoint_button(self): 
        """
        Handler for the 'Plot Waypoints' button on the general tab.
        Launches the waypoint planner application in a separate process to avoid GUI conflicts.
        Updates the confirmation/rejection label and console log when the planner is closed.
        """
        msg = f"Loading waypoint planner on general page..."
        self.replace_confirm_reject_label(msg)
        for i in self.selected_vehicles: self.recieve_console_update(msg, i)

        def run_waypoint_planner():
            root = tkinter.Tk()
            app = WaypointPlannerApp(root)
            root.mainloop()

        # Use multiprocessing to avoid GUI conflicts
        p = multiprocessing.Process(target=run_waypoint_planner)
        p.start()

        # Poll for process completion and update label
        def check_planner_closed():
            if not p.is_alive():
                msg = "Waypoint planner closed successfully"
                self.replace_confirm_reject_label(msg)
                for i in self.selected_vehicles: self.recieve_console_update(msg, i)
            else:
                QTimer.singleShot(500, check_planner_closed)  # check again in 0.5s

        QTimer.singleShot(500, check_planner_closed)

    def copy_bags(self):
        """
        Handler for the 'Copy Bags to Base Station' button on the general tab.
        Starts the bag synchronization process for all selected vehicles.
        Updates the confirmation/rejection label and console log with status messages.
        """
        msg = "Starting bag sync for all vehicles..."
        self.replace_confirm_reject_label(msg)
        for i in self.selected_vehicles: 
            # self.recieve_console_update(msg, i)
            threading.Thread(target=self.run_sync_bags, args=(i,), daemon=True).start()  

    def spec_copy_bags(self, vehicle_number):
        """
        Handler for the 'Copy Bag to Base Station' button on a specific Vehicle tab.
        Starts the bag synchronization process for the selected vehicle.
        Updates the confirmation/rejection label and console log with status messages.
        """
        msg = f"Starting bag sync for Vehicle {vehicle_number}..."
        self.replace_confirm_reject_label(msg)
        # self.recieve_console_update(msg, vehicle_number)
        # Run the sync operation in a separate thread to avoid blocking the GUI
        threading.Thread(target=self.run_sync_bags, args=(vehicle_number,), daemon=True).start()  

    def run_calibrate_script(self, vehicle_number):
        """
        Runs the vehicle calibration script in a separate thread.
        Used for calibrating all vehicles or a specific vehicle.
        """
        threading.Thread(target=self.run_calibrate_script_threaded, args=(vehicle_number,), daemon=True).start()

    def run_calibrate_script_threaded(self, vehicle_number):
        """
        Threaded function to run the vehicle calibration script.
        Calls the calibrate.main function with the appropriate vehicle list.
        """
        if not vehicle_number: vehicles = self.selected_vehicles
        else: vehicles = [vehicle_number]
        calibrate.main(self.ros_node, vehicles)

    #used by copy bags
    def run_sync_bags(self, vehicle_number):
        """
        Bootstraps passwordless SSH key auth (via paramiko, generating/copying a key if needed),
        then uses rsync to sync mission log (rosbag) folders from the specified vehicle's
        ~/cougars-frost/mission_logs into this base station's local mission_logs folder.
        rsync only transfers files that are missing or different, and --partial keeps a
        partially-transferred file instead of discarding it, so an interrupted sync resumes
        cleanly and re-running the sync never duplicates or re-downloads already-synced data.
        Reports success or failure through the confirmation/rejection label and console log.
        """

        try:
            vehicle_suffix = str(vehicle_number)
            ip_address = f"192.168.0.10{vehicle_suffix}"
            remote_user = "frostlab"
            local_folder = os.path.expanduser("~/mission_logs")

            # Ensure local folder exists
            os.makedirs(local_folder, exist_ok=True)

            # Bootstrap passwordless SSH key auth so the rsync subprocess below can run
            # non-interactively (it will fail fast via BatchMode instead of hanging on a
            # password prompt if this step didn't already establish key-based trust).
            ssh = self.get_ssh_connection(ip_address, remote_user)
            if ssh is None:
                raise RuntimeError(f"Could not establish SSH access to {ip_address}")
            ssh.close()
            self.recieve_console_update(f"Verified SSH access to Vehicle {vehicle_number}", vehicle_number)

            remote_path = f"{remote_user}@{ip_address}:~/cougars-frost/mission_logs/"
            rsync_cmd = [
                "rsync",
                "-avz",            # archive mode (recursive, preserves perms/times) + verbose + compression
                "--partial",       # keep partially-transferred files so an interrupted sync can resume
                "--timeout=30",    # abort if the connection stalls instead of hanging forever
                "-e", "ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10",
                remote_path,
                local_folder + "/",
            ]

            self.recieve_console_update(
                f"Starting rsync of mission logs from Vehicle {vehicle_number}...", vehicle_number)

            process = subprocess.Popen(
                rsync_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            for line in process.stdout:
                line = line.strip()
                if line:
                    self.recieve_console_update(f"[rsync] {line}", vehicle_number)
            return_code = process.wait(timeout=600)

            if return_code == 0:
                success_msg = f"Mission log sync completed successfully for Vehicle {vehicle_number}"
                self.replace_confirm_reject_label(success_msg)
                self.recieve_console_update(success_msg, vehicle_number)
            else:
                error_msg = f"rsync exited with code {return_code} while syncing Vehicle {vehicle_number}"
                self.replace_confirm_reject_label(error_msg)
                self.recieve_console_update(error_msg, vehicle_number)

        except Exception as e:
            error_msg = f"Failed to sync mission logs: {str(e)}"
            self.replace_confirm_reject_label(error_msg)
            self.recieve_console_update(error_msg, vehicle_number)

    def get_ssh_connection(self, ip_address, remote_user):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip_address, username=remote_user)
            return ssh
        except (paramiko.AuthenticationException, paramiko.SSHException):
            # Run ssh-copy-id to add the key
            self.recieve_console_update(f"SSH Authentication failed for {ip_address}. Attempting to copy SSH key. Enter password in the terminal", 0)
            try:
                if self.ensure_ssh_key():
                    subprocess.run(["ssh-copy-id", f"{remote_user}@{ip_address}"], check=True)
                    self.recieve_console_update(f"SSH key copied successfully to {ip_address}.", 0)
                    return self.get_ssh_connection(ip_address, remote_user)
            except subprocess.CalledProcessError as e:
                self.recieve_console_update(f"Failed to copy SSH key to {ip_address}: {e}", 0)
        except Exception as e:
            self.recieve_console_update(f"Failed to connect to {ip_address}: {e}", 0)
            return None

    def ensure_ssh_key(self, key_path="~/.ssh/id_rsa"):
        key_path = os.path.expanduser(key_path)
        pub_key_path = key_path + ".pub"
        # Check if both private and public key exist
        if os.path.exists(key_path) and os.path.exists(pub_key_path):
            return True  # SSH key exists
        else:
            # Generate a new SSH key with ssh-keygen
            subprocess.run(["ssh-keygen", "-t", "rsa", "-b", "4096", "-f", key_path, "-N", ""], check=True)
            return os.path.exists(key_path) and os.path.exists(pub_key_path)

    def local_vehicle_params_path(self, vehicle_num):
        """
        Path to this vehicle's vehicle_params.yaml in the base station's local
        cougars-config checkout (config/cougars-config/coug{N}/vehicle_params.yaml).
        """
        return os.path.expanduser(f"~/config/cougars-config/coug{vehicle_num}/vehicle_params.yaml")

    def load_vehicle_kinematics_params(self, vehicle_num):
        """
        Loads the vehicle kinematics parameters from the vehicle using paramiko or falls back to local params file.
        Returns the vehicle and base kinematics parameters.
        """

        vehicle_kinematics = None
        base_kinematics = None

        # Try to get the params path from the vehicle
        config_path = Path.home().joinpath("config", "cougars-config", "base_station", "deploy_config.json")
        with open(config_path, "r") as f:
            config = json.load(f)
        vehicles = config["vehicles"]
        vehicle_info = vehicles.get(f"coug{vehicle_num}") or vehicles.get(str(vehicle_num))
        if vehicle_info:
            remote_user = vehicle_info["remote_user"]
            remote_host = vehicle_info["remote_host"]
            remote_param_path = f"{config['config_path']}/coug{vehicle_num}/vehicle_params.yaml"
            try:
                # Connect via SSH and SFTP
                ssh = self.get_ssh_connection(remote_host, remote_user)
                sftp = ssh.open_sftp()
                with sftp.open(remote_param_path, "r") as remote_file:
                    file_content = remote_file.read().decode()
                    data = yaml.safe_load(file_content)
                    vehicle_key = f"coug{vehicle_num}"
                    try:
                        vehicle_kinematics = data[vehicle_key]['coug_kinematics']['ros__parameters']
                    except KeyError:
                        self.replace_confirm_reject_label(f"Could not find kinematics in remote file for coug{vehicle_num}")
                sftp.close()
                ssh.close()
            except Exception as e:
                self.replace_confirm_reject_label(f"SSH error: {e}")

        # If can't get params from vehicle, fallback to local cougars-config copy
        params_path = self.local_vehicle_params_path(vehicle_num)
        if os.path.exists(params_path):
            with open(params_path, 'r') as f:
                data = yaml.safe_load(f)
            vehicle_key = f"coug{vehicle_num}"
            try:
                base_kinematics = data[vehicle_key]['coug_kinematics']['ros__parameters']
            except KeyError:
                base_kinematics = None

        return vehicle_kinematics, base_kinematics

    def create_new_param_file(self, vehicle_num):
        """
        Creates a new vehicle_params.yaml for the given vehicle number by copying the
        example template (config/config_example/coug1/vehicle_params.yaml) and replacing
        'coug1' with 'coug{vehicle_num}'. The new file is saved into the local
        cougars-config checkout at config/cougars-config/coug{vehicle_num}/vehicle_params.yaml.

        Parameters:
            vehicle_num (int): Vehicle number to create the param file for.
        """

        template_path = os.path.expanduser("~/cougars-frost/config/config_example/coug1/vehicle_params.yaml")
        new_param_path = self.local_vehicle_params_path(vehicle_num)

        os.makedirs(os.path.dirname(new_param_path), exist_ok=True)

        # Read template and replace 'coug1' with 'coug{vehicle_num}'
        with open(template_path, "r") as f:
            content = f.read()
        content = content.replace("coug1:", f"coug{vehicle_num}:")
        content = content.replace("vehicle_ID: 1", f"vehicle_ID: {vehicle_num}")

        with open(new_param_path, "w") as f:
            f.write(content)

        msg = f"Created new param file for Vehicle {vehicle_num} at {new_param_path}"
        self.recieve_console_update(msg, vehicle_num)

    def save_param_file(self, vehicle_num, fin_list):
        """
        Saves updated fin calibration parameters to the local cougars-config YAML file
        for the given vehicle. Replaces the top, right, and left fin offsets in the
        file using regex.
        Parameters:
            vehicle_num (int): Vehicle number.
            fin_list (list): List of fin offsets [top, right, left].
        """
        # Build the path to the params file for this vehicle
        params_path = self.local_vehicle_params_path(vehicle_num)

        # Read the current file contents
        with open(params_path, "r") as f:
            content = f.read()

        # Replace the offsets using regex to match any value
        content = re.sub(r'(top_fin_offset:\s*)(-?\d+\.?\d*)', r'\g<1>{}'.format(float(fin_list[0])), content)
        content = re.sub(r'(right_fin_offset:\s*)(-?\d+\.?\d*)', r'\g<1>{}'.format(float(fin_list[1])), content)
        content = re.sub(r'(left_fin_offset:\s*)(-?\d+\.?\d*)', r'\g<1>{}'.format(float(fin_list[2])), content)

        # Write the updated content back to the file
        with open(params_path, "w") as f:
            f.write(content)

    def calibrate_fins(self):
        """
        Opens the fin calibration dialog for all selected vehicles.
        Loads current parameters, shows a loading dialog, and allows the user to adjust fin offsets.
        Saves changes to params and updates the ROS node parameters.
        """
        # Show loading dialog
        loading_dialog = LoadingDialog(
            message="Loading fin \ncalibration data...",
            parent=self,
            background_color=self.background_color,
            text_color=self.text_color
        )
        loading_dialog.show()
        QApplication.processEvents()  # Ensure it appears immediately

        def after_worker(vehicle_params_dict, params_found_dict, base_params_problems, vehicle_params_problems):
            loading_dialog.close()
            # Show warnings if any params files are missing or have errors
            if base_params_problems or vehicle_params_problems:
                warning_lines = []
                if vehicle_params_problems:
                    warning_lines.append(
                        f"The following vehicle params files weren't found or had errors: {vehicle_params_problems}"
                    )
                if base_params_problems:
                    warning_lines.append(
                        f"The following base station params files weren't found or had errors (if not found they were created): {base_params_problems}"
                    )
                warning_msg = "\n".join(warning_lines)
                QTimer.singleShot(0, lambda: QMessageBox.warning(
                    self, "Params File Warning", warning_msg
                ))

            def publish_fins(vehicle_num, fins, pub_type):
                fins_out = [float(f) for f in fins]
                self.ros_node.publish_fins(fins_out, vehicle_num, pub_type)

            for i in self.selected_vehicles:
                self.recieve_console_update("Loading Fin Calibration Window...", i)

            # Open the calibration dialog
            dlg = CalibrateFinsDialog(
                parent=self,
                background_color=self.background_color,
                text_color=self.text_color,
                pop_up_window_style=self.pop_up_window_style,
                selected_vehicles=self.selected_vehicles,
                passed_ros_node=self.ros_node,
                on_slider_change=publish_fins,
                vehicle_init_params=vehicle_params_dict
            )

            if dlg.exec():
                fin_states = dlg.get_states()
                for key, states in fin_states.items():
                    self.save_param_file(key, states)
                    self.ros_node.set_single_parameter("top_fin_offset", float(states[0]), key)
                    self.ros_node.set_single_parameter("right_fin_offset", float(states[1]), key)
                    self.ros_node.set_single_parameter("left_fin_offset", float(states[2]), key)

                    self.recieve_console_update("Vehicle Kinematics param set", int(key))
                    self.recieve_console_update("Fin Calibration Saved to Params", int(key))
            else:
                for i in self.selected_vehicles: self.recieve_console_update("Canceling Fin Calibration", i)

        # Start worker thread to load parameters
        self.cal_fins_worker = CalibrateFinsWorker(
            self.selected_vehicles,
            self.load_vehicle_kinematics_params,
            self.create_new_param_file
        )
        self.cal_fins_worker.finished.connect(after_worker)
        self.cal_fins_worker.start()

    #Connected to the "kill" signal
    def emergency_shutdown_button(self, vehicle_number):
        """
        Handler for 'Emergency Shutdown' button, with confirmation dialog.
        Sends a shutdown request to the ROS service for the specified vehicle.
        Updates the GUI and console log with status messages.
        """
        dlg = ConfirmationDialog("Emergency Shutdown?", "Are you sure you want to initiate emergency shutdown?", self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            self.replace_confirm_reject_label("Starting Emergency Shutdown...")
            self.ros_node.publish_emergency_kill(vehicle_number)
        else:
            self.replace_confirm_reject_label("Canceling Emergency Shutdown command...")
            self.recieve_console_update(f"Canceling Emergency Shutdown for Vehicle {vehicle_number}", vehicle_number)

    #Connected to the "surface" signal
    def emergency_surface_button(self, vehicle_number):
        """
        Handler for 'Emergency Surface' button, with confirmation dialog.
        Sends a surface request to the ROS service for the specified vehicle.
        Updates the GUI and console log with status messages.
        """
        dlg = ConfirmationDialog("Emergency Surface?", "Are you sure you want to initiate emergency surface?", self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            self.replace_confirm_reject_label("Starting Emergency Surface...")
            self.ros_node.publish_emergency_surface(vehicle_number)
        else:
            self.replace_confirm_reject_label("Canceling Emergency Surface command...")
            self.recieve_console_update(f"Canceling Emergency Surface for Vehicle {vehicle_number}", vehicle_number)

    #Opens the relay/strobe control dialog for the given vehicle
    def hardware_control_button(self, vehicle_number):
        """
        Handler for 'Relay / Strobe Control' button.
        Opens a dialog that sends relay/strobe auto/on/off commands to the vehicle over radio.
        """
        dlg = HardwareControlDialog(
            vehicle_number,
            self.ros_node,
            self,
            background_color=self.background_color,
            text_color=self.text_color,
            pop_up_window_style=self.pop_up_window_style,
        )
        dlg.exec()

    #Connected to the "ModemControl" service in base_station_interfaces
    # def modem_shut_off_service(self, shutoff:bool, vehicle_id:int):
    #     """
    #     Handler for modem shut off/on service.
    #     Sends a request to the ROS service to shut off or turn on the modem for the specified vehicle.
    #     Updates the GUI and console log with status messages.
    #     Parameters:
    #         shutoff (bool): True to shut off modem, False to turn on.
    #         vehicle_id (int): Vehicle number.
    #     """
    #     message = ModemControl.Request()
    #     message.modem_shut_off = shutoff
    #     message.vehicle_id = vehicle_id
    #     if shutoff: self.replace_confirm_reject_label("Wifi Connected, Shutting Off Modem")
    #     else: self.replace_confirm_reject_label("Wifi Disconnected, Turning On Modem")
        
    #     if shutoff: self.recieve_console_update(f"Wifi Connected, Shutting Off Modem", vehicle_id)
    #     else: self.recieve_console_update(f"Wifi Disconnected, Turning On Modem", vehicle_id)

    #     future = self.ros_node.cli3.call_async(message)
    #     # Add callback to handle response
    #     future.add_done_callback(partial(self.handle_service_response, action="Modem Shut off Service", vehicle_number=vehicle_id)) #0->for all vehicles
    #     return future

    #used by various buttons to handle services dynamically
    def handle_service_response(self, future, action, vehicle_number):
        """
        Handles the result of an asynchronous ROS service call.
        Updates the confirmation/rejection label and console log based on the service response.
        Parameters:
            future: The future object from the async service call.
            action (str): Description of the action/service.
            vehicle_number (int): Vehicle number (0 for all vehicles).
        """
        try:
            response = future.result()
            if response.success:
                message = f"{action} Service Initiated Successfully"
                self.replace_confirm_reject_label(message)
                # if not vehicle_number:
                #     for i in self.selected_vehicles:
                #         self.recieve_console_update(message, i)
                # elif vehicle_number in self.selected_vehicles:
                #     self.recieve_console_update(message, vehicle_number)
            else:
                message = f"{action} Service Initialization Failed"
                self.replace_confirm_reject_label(message)
                # if not vehicle_number:
                #     for i in self.selected_vehicles:
                #         self.recieve_console_update(message, i)
                # elif vehicle_number in self.selected_vehicles:
                #     self.recieve_console_update(message, vehicle_number)
       
        except Exception as e:
            self.replace_confirm_reject_label(f"{action} service call failed: {e}")
            if vehicle_number in self.selected_vehicles: self.recieve_console_update(f"{action} service call failed: {e}", vehicle_number)
            else: print(f"{action} service call failed: {e}")

    def publish_origin_command(self):
        """
        Handler for the general-tab origin button.
        Prompts for an origin and publishes it on the global /origin topic.
        """
        origin_values = getattr(self, "last_origin_values", (40.247125, -111.647000, 1420.00))
        dlg = OriginDialog(
            origin_values,
            self,
            background_color=self.background_color,
            text_color=self.text_color,
            pop_up_window_style=self.pop_up_window_style,
        )
        if not dlg.exec():
            self.replace_confirm_reject_label("Publish Origin command was cancelled.")
            for i in self.selected_vehicles:
                self.recieve_console_update("Publish Origin command was cancelled.", i)
            return

        latitude, longitude, altitude = dlg.get_origin()
        self.last_origin_values = (latitude, longitude, altitude)
        self.replace_confirm_reject_label("Publishing origin over WiFi...")
        for i in self.selected_vehicles:
            self.recieve_console_update(
                f"Publishing origin over WiFi: lat={latitude}, lon={longitude}, alt={altitude}",
                i,
            )

        self.ros_node.publish_origin((latitude, longitude, altitude))

    def make_vline(self):
        """
        Creates and returns a vertical line QFrame for use in layouts.
        Used to visually separate columns in the GUI.
        """
        Vline = QFrame()
        Vline.setFrameShape(QFrame.Shape.VLine)
        Vline.setFrameShadow(QFrame.Shadow.Sunken)
        Vline.setStyleSheet(f"background-color: {self.text_color};")
        return Vline

    def make_hline(self):
        """
        Creates and returns a horizontal line QFrame for use in layouts.
        Used to visually separate sections in the GUI.
        """
        Hline = QFrame()
        Hline.setFrameShape(QFrame.Shape.HLine)
        Hline.setFrameShadow(QFrame.Shadow.Sunken)
        Hline.setStyleSheet(f"background-color: {self.text_color};")
        return Hline

    def set_general_page_widgets(self):
        """
        Sets up the widgets and layouts for the General tab.
        Creates the first column for general options and additional columns for each selected vehicle.
        Adds vertical lines between columns and initializes layouts for each vehicle.
        """
        self.general_page_layout = self.tab_dict["General"][1]

        # Create the first column (General Options)
        self.general_page_C0_widget = QWidget()
        self.general_page_C0_layout = QVBoxLayout()
        self.general_page_C0_widget.setLayout(self.general_page_C0_layout)

        # Store widgets and layouts for each Vehicle column
        self.general_page_vehicle_widgets = {}
        self.general_page_vehicle_layouts = {}

        # Add the first column to the layout
        self.general_page_layout.addWidget(self.general_page_C0_widget)

        # For each selected Vehicle, create a column and add to the layout, separated by vertical lines
        for idx, vehicle_number in enumerate(self.selected_vehicles):
            self.general_page_layout.addWidget(self.make_vline())
            widget = QWidget()
            layout = QVBoxLayout()
            widget.setLayout(layout)
            self.general_page_layout.addWidget(widget)
            self.general_page_vehicle_widgets[vehicle_number] = widget
            self.general_page_vehicle_layouts[vehicle_number] = layout

        # Add the buttons to the first column
        self.set_general_page_C0_widgets()

        # Set the widgets for each Vehicle column
        for vehicle_number in self.selected_vehicles:
            self.set_general_page_C1_widgets(self.general_page_vehicle_layouts[vehicle_number], vehicle_number)

    def set_general_page_C0_widgets(self):
        """
        Sets up the widgets for the first column on the General tab.
        Adds general option buttons such as Load Missions, Start Missions, Plot Waypoints, Copy Bags, Calibrate, and Publish Origin.
        Styles and arranges the buttons and labels vertically.
        """
        general_label = QLabel("General Options:")
        general_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        general_label.setStyleSheet(f"color: {self.text_color};")
        general_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        #Load All Missions button
        self.Load_missions_button = QPushButton("Load All Missions")
        self.Load_missions_button.clicked.connect(self.load_missions_button)
        self.Load_missions_button.setStyleSheet(self.normal_button_style_sheet)

        #Start All Missions button
        self.Start_missions_button = QPushButton("Start All Missions")
        self.Start_missions_button.clicked.connect(self.start_missions_button)
        self.Start_missions_button.setStyleSheet(self.normal_button_style_sheet)

        #Plot Waypoints button
        self.plot_waypoints_button = QPushButton("Plot Waypoints")
        self.plot_waypoints_button.clicked.connect(self.load_waypoint_button)
        self.plot_waypoints_button.setStyleSheet(self.normal_button_style_sheet)        
        
        #Copy Bags to Base Station
        self.copy_bags_button = QPushButton("Copy Bags to Base Station")
        self.copy_bags_button.clicked.connect(self.copy_bags)
        self.copy_bags_button.setStyleSheet(self.normal_button_style_sheet)

        #Calibrate All Vehicles 
        self.sync_all_vehicles_button = QPushButton("Calibrate All Vehicles")
        self.sync_all_vehicles_button.clicked.connect(lambda: self.run_calibrate_script(0))
        self.sync_all_vehicles_button.setStyleSheet(self.normal_button_style_sheet)

        # Calibrate fins button
        self.calibrate_fins_button = QPushButton("Calibrate Fins (In Progress)")
        self.calibrate_fins_button.clicked.connect(self.calibrate_fins)
        self.calibrate_fins_button.setStyleSheet(self.normal_button_style_sheet)

        # Publish the global origin over WiFi
        self.publish_origin_button = QPushButton("Publish Origin")
        self.publish_origin_button.clicked.connect(self.publish_origin_command)
        self.publish_origin_button.setStyleSheet(self.normal_button_style_sheet)

        # Add widgets to the layout
        self.general_page_C0_layout.addWidget(general_label, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.Load_missions_button, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.Start_missions_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.plot_waypoints_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.copy_bags_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.sync_all_vehicles_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.calibrate_fins_button)
        self.general_page_C0_layout.addSpacing(20)

        self.general_page_C0_layout.addWidget(self.publish_origin_button)
        self.general_page_C0_layout.addSpacing(20)

        # Add spacer to push the rest of the buttons down
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        self.general_page_C0_layout.addItem(spacer)
            
    #template to set the rest of widgets on the rest of the columns on the general page
    def set_general_page_C1_widgets(self, layout, vehicle_number):
        """
        Sets up the widgets for each Vehicle column on the General tab.
        Adds section labels, connection and sensor icons, and emergency status for each vehicle.
        """
        title_label = QLabel(f"Vehicle {vehicle_number}:")
        title_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {self.text_color};")
        title_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout.addWidget(title_label, alignment=Qt.AlignmentFlag.AlignTop)
        layout.addSpacing(20)

        #section labels for each column
        section_titles = ["Connections", "Sensors", "Emergency Status"]
        for title in section_titles:
            label = QLabel(title)
            label.setFont(QFont("Arial", 15))
            label.setStyleSheet(f"color: {self.text_color};")
            layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
            layout.addSpacing(20)
            #repeated tab_spacing variable used throughout the file, to keep tabs consistent
            self.tab_spacing = 60

            #The connections section contains the wifi, radio, and modem connections for each Vehicle respectively
            if title == "Connections": 
                wifi_widget = self.create_icon_and_text("Wifi", self.icons_dict[self.feedback_dict["Wifi"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(wifi_widget)
                layout.addSpacing(20)

                radio_widget = self.create_icon_and_text("Radio", self.icons_dict[self.feedback_dict["Radio"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(radio_widget)
                layout.addSpacing(20)

                modem_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(modem_widget)
                layout.addSpacing(40)

            #The connections section contains the DVL, GPS, and IMU sensors connections for each Vehicle respectively
            elif title == "Sensors":
                DVL_sensor_widget = self.create_icon_and_text("DVL", self.icons_dict[self.feedback_dict["DVL"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(DVL_sensor_widget)
                layout.addSpacing(20)

                GPS_sensor_widget = self.create_icon_and_text("GPS", self.icons_dict[self.feedback_dict["GPS"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(GPS_sensor_widget)
                layout.addSpacing(20)
                
                IMU_sensor_widget = self.create_icon_and_text("IMU", self.icons_dict[self.feedback_dict["IMU"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(IMU_sensor_widget)
                layout.addSpacing(40)

            #The status section contains the status message for each Vehicle respectively
            elif title == "Emergency Status":
                status = "No Data Recieved"
                label = QLabel(f"{status}", font=QFont("Arial", 13))
                label.setObjectName(f"Status_messages{vehicle_number}")
                label.setStyleSheet(f"color: {self.text_color};")
                layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
                layout.addSpacing(40)

        # Add spacer to push content up
        spacer = QSpacerItem(0, 0, QSizePolicy .Policy.Minimum, QSizePolicy.Policy.Expanding)
        layout.addItem(spacer)

    #This is used to set the widgets on the other tabs, namely Vehicle1, Vehicle2, Vehicle3, etc
    def set_specific_vehicle_widgets(self, vehicle_number):
        """
        Sets up the widgets for a specific Vehicle tab.
        Arranges columns for connections/sensors, status, and buttons, separated by vertical lines.
        Returns the container widget for the tab.
        """
        temp_container = QWidget()
        temp_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        temp_layout = QHBoxLayout(temp_container)
        temp_layout.setSpacing(0)
        temp_layout.setContentsMargins(0, 0, 0, 0)

        #col0 contains connections and sensor icons
        temp_layout.addWidget(self.create_specific_vehicle_column0(vehicle_number), alignment=Qt.AlignmentFlag.AlignTop)
        #col1 contains mission and vehicle1 status
        temp_layout.addWidget(self.create_specific_vehicle_column01(vehicle_number), alignment=Qt.AlignmentFlag.AlignTop)

        #vline
        temp_layout.addWidget(self.make_vline())
        #col2 - seconds since last connected and buttons
        temp_layout.addWidget(self.create_vehicle_buttons_column(vehicle_number))
        return temp_container

    #The scrolling log at the bottom of the specific vehicle tabs. 
    def create_specific_vehicle_console_log(self, vehicle_number): 
        """
        Creates the scrollable console log area for a specific Vehicle tab.
        Adds a title label and a message label inside a QScrollArea for displaying log messages.
        Returns the container widget holding the scrollable log.
        """
        temp_container = QWidget()
        temp_layout = QVBoxLayout(temp_container)
        setattr(self, f"vehicle{vehicle_number}_console_layout", temp_layout)
        setattr(self, f"vehicle{vehicle_number}_console_widget", temp_container)

        # First text label (bold title)
        title_text = "Console Log"
        title_label = QLabel(title_text)
        title_label.setWordWrap(True)
        title_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {self.text_color};")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        temp_layout.addWidget(title_label)

        # Second text label (wrapped long message)
        message_text = ""

        # Create a QLabel from the message_text for displaying the log
        message_label = QLabel(message_text)
        message_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse | Qt.TextInteractionFlag.TextSelectableByKeyboard)
        message_label.setWordWrap(True) # Enable word wrapping for readability
        font = QFont()
        #second font is for emojis, that aren't available in arial
        font.setFamily("Arial, Noto Color Emoji")
        font.setPointSize(13)
        message_label.setFont(font)
        message_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop) 
        message_label.setContentsMargins(0, 0, 0, 0)
        message_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        message_label.setObjectName(f"Console_messages{vehicle_number}")

        # Create a QWidget to hold the message label, and a layout for it
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.addWidget(message_label)

        # Create QScrollArea and set scroll content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(scroll_content)
        scroll_area.setStyleSheet(
            f"border: 2px solid {self.border_outline}; border-radius: 6px; background: {self.background_color};"
        )
        # Store the scroll area as an attribute for dynamic resizing
        setattr(self, f"vehicle{vehicle_number}_console_scroll_area", scroll_area)

        # Add scroll_area to the layout
        temp_layout.addWidget(scroll_area)

        # Add a spacer to push content up and allow for vertical expansion
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        temp_layout.addItem(spacer)

        # Return the container widget holding the scrollable log
        return temp_container

    # ---- Keyboard Controls tab ----

    KEYBOARD_CONTROLS_INSTRUCTIONS = (
        "W / Up Arrow     -  Fins up\n"
        "S / Down Arrow   -  Fins down\n"
        "A / Left Arrow   -  Turn left\n"
        "D / Right Arrow  -  Turn right\n"
        "Space            -  Thruster +5 (up to 100)\n"
        "R  (or , . -)    -  Thruster -5 (down to -100, i.e. reverse)\n"
        "Q                -  Arm / disarm thruster\n"
        "E                -  Switch to next vehicle\n"
        "Z                -  Enable / disable keyboard controls\n"
        "T                -  Toggle hard turn mode (see below)\n\n"
        "Hard turn mode: while enabled, holding A/D snaps the turn fin straight to its\n"
        "full angle instead of stepping toward it, and releasing springs it back to 0.\n\n"
        "You can also just type these keys anywhere in this window."
    )

    # Mirrors teleop_couguv_key.cpp's default max_fin_value param (degrees of fin travel).
    # If that param is ever changed on launch, this display scale won't automatically follow it.
    TELEOP_MAX_FIN_ANGLE_DEG = 70.0

    def create_keyboard_controls_tab(self):
        """
        Builds the Keyboard Controls tab: a currently-controlled-vehicle indicator, instructions,
        live speed/turn/pitch gauges, and on-screen controls mirroring every key binding.
        Both the on-screen buttons and physical typing go through the same
        ros_node.publish_keypress() call, so they're equivalent to the teleop node.

        Kept compact (small button padding, tight spacing, a 2x2 grid for the toggle
        buttons instead of a stack) so the whole tab fits on screen without scrolling on
        typical laptop displays; still wrapped in a QScrollArea as a safety net for smaller ones.
        """
        # Buttons that use the compact style sheet instead of the app-wide normal one; tracked
        # so _refresh_teleop_toggle_labels can re-assert that sizing after a theme switch (the
        # generic per-theme recoloring pass in apply_theme_to_widgets would otherwise restyle
        # every QPushButton, including these, back to full size).
        self.teleop_compact_buttons = []

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(6)
        layout.setContentsMargins(10, 8, 10, 8)

        self.teleop_vehicle_label = QLabel("Currently controlling: waiting for status...")
        self.teleop_vehicle_label.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        self.teleop_vehicle_label.setStyleSheet(f"color: {self.text_color};")
        self.teleop_vehicle_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(self.teleop_vehicle_label)

        instructions_label = QLabel(self.KEYBOARD_CONTROLS_INSTRUCTIONS)
        instructions_label.setFont(QFont("Courier New", 9))
        instructions_label.setStyleSheet(f"color: {self.text_color};")
        layout.addWidget(instructions_label)

        layout.addWidget(self._build_teleop_live_gauges())

        self.teleop_enable_button = QPushButton()
        self.teleop_enable_button.clicked.connect(lambda: self._send_teleop_key('z'))

        self.teleop_arm_button = QPushButton()
        self.teleop_arm_button.clicked.connect(lambda: self._send_teleop_key('q'))

        self.teleop_hard_turn_button = QPushButton()
        self.teleop_hard_turn_button.clicked.connect(lambda: self._send_teleop_key('t'))

        switch_vehicle_button = QPushButton("Switch Vehicle (E)")
        switch_vehicle_button.setStyleSheet(self.compact_button_style_sheet)
        switch_vehicle_button.clicked.connect(lambda: self._send_teleop_key('e'))
        self.teleop_compact_buttons.append(switch_vehicle_button)

        # 2x2 grid instead of 4 stacked full-width buttons - halves the vertical space they use
        toggle_grid = QGridLayout()
        toggle_grid.setSpacing(6)
        toggle_grid.addWidget(self.teleop_enable_button, 0, 0)
        toggle_grid.addWidget(self.teleop_arm_button, 0, 1)
        toggle_grid.addWidget(self.teleop_hard_turn_button, 1, 0)
        toggle_grid.addWidget(switch_vehicle_button, 1, 1)
        layout.addLayout(toggle_grid)

        layout.addWidget(self._build_teleop_fin_pad())
        layout.addWidget(self._build_teleop_thruster_row())

        self._refresh_teleop_toggle_labels()

        self.teleop_scroll_area = QScrollArea()
        self.teleop_scroll_area.setWidget(container)
        self.teleop_scroll_area.setWidgetResizable(True)
        self.teleop_scroll_area.setStyleSheet(f"border: none; background: {self.background_color};")
        return self.teleop_scroll_area

    def _build_teleop_live_gauges(self):
        """
        Builds the live thruster/turn/pitch gauge row: reads the same UCommandBase message
        that's actually being sent to the vehicle (see recieve_teleop_command), so it shows
        what's really commanded rather than just echoing button presses.
        """
        container = QWidget()
        row = QHBoxLayout(container)

        self.teleop_speed_gauge = BarGauge("Thruster", 100.0, "%")
        self.teleop_turn_gauge = TurnWheelGauge(self.TELEOP_MAX_FIN_ANGLE_DEG)
        self.teleop_pitch_gauge = BarGauge("Fins Up/Down", self.TELEOP_MAX_FIN_ANGLE_DEG, "°")

        for gauge in (self.teleop_speed_gauge, self.teleop_turn_gauge, self.teleop_pitch_gauge):
            gauge.set_theme_colors(self.text_color, self.border_outline, self.background_color)
            row.addWidget(gauge)

        return container

    def _build_teleop_fin_pad(self):
        pad_widget = QWidget()
        grid = QGridLayout(pad_widget)
        grid.setSpacing(4)
        grid.addWidget(self._make_teleop_key_button("Fins Up (W)", 'w'), 0, 1)
        grid.addWidget(self._make_teleop_key_button("Turn Left (A)", 'a'), 1, 0)
        grid.addWidget(self._make_teleop_key_button("Turn Right (D)", 'd'), 1, 2)
        grid.addWidget(self._make_teleop_key_button("Fins Down (S)", 's'), 2, 1)
        return pad_widget

    def _build_teleop_thruster_row(self):
        row_widget = QWidget()
        row = QHBoxLayout(row_widget)
        row.setSpacing(4)
        row.addWidget(self._make_teleop_key_button("Thruster - (R)", 'r'))
        row.addWidget(self._make_teleop_key_button("Thruster + (Space)", ' '))
        return row_widget

    def _make_teleop_key_button(self, text, key):
        button = QPushButton(text)
        button.setStyleSheet(self.compact_button_style_sheet)
        button.clicked.connect(lambda: self._send_teleop_key(key))
        self.teleop_compact_buttons.append(button)
        return button

    def _send_teleop_key(self, key):
        self.ros_node.publish_keypress(key)

    def _refresh_teleop_toggle_labels(self):
        enabled = getattr(self, "teleop_publishing_enabled", False)
        armed = getattr(self, "teleop_thruster_enabled", False)
        hard_turn = getattr(self, "teleop_hard_turn_mode", False)

        self.teleop_enable_button.setText("Disable Keyboard Controls (Z)" if enabled else "Enable Keyboard Controls (Z)")
        self.teleop_enable_button.setStyleSheet(self.compact_danger_button_style_sheet if enabled else self.compact_button_style_sheet)

        self.teleop_arm_button.setText("Disarm Thruster (Q)" if armed else "Arm Thruster (Q)")
        self.teleop_arm_button.setStyleSheet(self.compact_danger_button_style_sheet if armed else self.compact_button_style_sheet)

        self.teleop_hard_turn_button.setText("Disable Hard Turn Mode (T)" if hard_turn else "Enable Hard Turn Mode (T)")
        self.teleop_hard_turn_button.setStyleSheet(self.compact_danger_button_style_sheet if hard_turn else self.compact_button_style_sheet)

        # The generic per-theme recoloring pass (apply_theme_to_widgets) restyles every
        # QPushButton at full size; re-assert compact sizing on the rest of this tab's buttons.
        for button in getattr(self, "teleop_compact_buttons", []):
            button.setStyleSheet(self.compact_button_style_sheet)

        # The scroll area's own background is set once at construction and isn't touched by
        # the generic per-tab recoloring pass, so it goes stale on a theme switch; refresh it.
        if hasattr(self, "teleop_scroll_area"):
            self.teleop_scroll_area.setStyleSheet(f"border: none; background: {self.background_color};")

    def recieve_teleop_vehicle(self, vehicle_id):
        self.teleop_vehicle_signal.emit(vehicle_id)

    def _update_teleop_vehicle_gui(self, vehicle_id):
        self.teleop_vehicle_label.setText(f"Currently controlling: Vehicle {vehicle_id}")
        # teleop_couguv_key.cpp resets fins/thruster to 0 for the newly-controlled vehicle on
        # switch; reflect that immediately instead of showing the previous vehicle's last values.
        self._reset_teleop_gauges()

    def recieve_teleop_thruster(self, armed):
        self.teleop_thruster_signal.emit(armed)

    def _update_teleop_thruster_gui(self, armed):
        self.teleop_thruster_enabled = armed
        self._refresh_teleop_toggle_labels()

    def recieve_teleop_publishing(self, enabled):
        self.teleop_publishing_signal.emit(enabled)

    def _update_teleop_publishing_gui(self, enabled):
        self.teleop_publishing_enabled = enabled
        self._refresh_teleop_toggle_labels()
        if not enabled:
            # No new commands are published while disabled, so the gauges would otherwise
            # keep showing stale last-commanded values; zero them to signal "not commanding".
            self._reset_teleop_gauges()

    def recieve_teleop_hard_turn(self, hard_turn_mode):
        self.teleop_hard_turn_signal.emit(hard_turn_mode)

    def _update_teleop_hard_turn_gui(self, hard_turn_mode):
        self.teleop_hard_turn_mode = hard_turn_mode
        self._refresh_teleop_toggle_labels()

    def recieve_teleop_command(self, msg):
        self.teleop_command_signal.emit(msg)

    def _update_teleop_command_gui(self, msg):
        """
        Drives the live gauges from the UCommandBase message teleop_couguv_key.cpp actually
        publishes to /keyboard_controls. fin[0] (turn) and fin[2] are sign-flipped by the
        teleop node relative to the operator's A/D-positive convention, so undo that here.
        """
        fins = list(msg.ucommand.fin)
        turn_angle = -fins[0] if len(fins) > 0 else 0.0
        pitch_angle = fins[1] if len(fins) > 1 else 0.0

        if hasattr(self, "teleop_speed_gauge"):
            self.teleop_speed_gauge.set_value(msg.ucommand.thruster)
        if hasattr(self, "teleop_turn_gauge"):
            self.teleop_turn_gauge.set_angle(turn_angle)
        if hasattr(self, "teleop_pitch_gauge"):
            self.teleop_pitch_gauge.set_value(pitch_angle)

    def _reset_teleop_gauges(self):
        if hasattr(self, "teleop_speed_gauge"):
            self.teleop_speed_gauge.set_value(0.0)
        if hasattr(self, "teleop_turn_gauge"):
            self.teleop_turn_gauge.set_angle(0.0)
        if hasattr(self, "teleop_pitch_gauge"):
            self.teleop_pitch_gauge.set_value(0.0)

    def paintIconBackground(self, icon_pixmap, bg_color="#28625a", diameter=24):
        """
        Draws a colored circle behind the given icon pixmap.
        Used for visually highlighting status icons in the GUI.

        Parameters:
            icon_pixmap (QPixmap): The icon to draw.
            bg_color (str): The background color (hex or color name).
            diameter (int): The diameter of the background circle.

        Returns:
            QPixmap: The new pixmap with the background.
        """
        # Create a transparent pixmap
        result = QPixmap(diameter, diameter)
        result.fill(Qt.GlobalColor.transparent)
        painter = QPainter(result)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QColor(bg_color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(0, 0, diameter, diameter)
        # Center the icon in the circle
        icon_size = icon_pixmap.size()
        x = (diameter - icon_size.width()) // 2
        y = (diameter - icon_size.height()) // 2
        painter.drawPixmap(x, y, icon_pixmap)
        painter.end()
        return result

    def make_icon_label(self, icon, text, vehicle_number, icon_pg_type): 
        """
        Creates a QLabel for a status icon, sets its pixmap and stores original icon and type.
        Used for displaying connection/sensor status in the GUI.

        Parameters:
            icon (QStyle.StandardPixmap): The Qt standard icon type.
            text (str): The label text (used for naming).
            vehicle_number (int): Vehicle number for naming.
            icon_pg_type (int): 0 for general, 1 for specific tab.

        Returns:
            QLabel: The icon label widget.
        """
        icon_label = QLabel()
        icon_pixmap = self.style().standardIcon(icon).pixmap(16, 16)
        icon_label._original_icon_pixmap = icon_pixmap  # Store original
        icon_label._icon_type = icon # Store the icon type (e.g., QStyle.StandardPixmap.SP_MessageBoxCritical)
        
        if icon == QStyle.StandardPixmap.SP_MessageBoxCritical:
            icon_bkgrnd = self.light_icon_bkgrnd_color
        elif icon == QStyle.StandardPixmap.SP_DialogApplyButton:
            icon_bkgrnd = self.light_icon_bkgrnd_color
        elif icon == QStyle.StandardPixmap.SP_TitleBarContextHelpButton:
            icon_bkgrnd = self.dark_icon_bkgrnd_color
        else:
            print("Unknown icon type.")
            return
        bg_pixmap = self.paintIconBackground(icon_pixmap, bg_color=icon_bkgrnd)
        icon_label.setPixmap(bg_pixmap)
        icon_label.setObjectName(f"icon_{text}{vehicle_number}{icon_pg_type}")
        icon_label.setContentsMargins(0, 0, 0, 0)
        icon_label.setFixedSize(24, 24)
        return icon_label

    #used to create an icon next to text in a pre-determined fashion
    def create_icon_and_text(self, text, icon=None, temp_tab_spacing=None, vehicle_number=None, icon_pg_type=None):
        """
        Creates a QWidget containing an icon (optional) and a text label, arranged horizontally.
        Used for displaying status with icons and text in the GUI.

        Parameters:
            text (str): The text to display next to the icon.
            icon (QStyle.StandardPixmap, optional): The standard Qt icon to display. If None, no icon is shown.
            temp_tab_spacing (int, optional): Left margin for the layout, used for tab alignment.
            vehicle_number(int, optional): used to name the icon labels
            icon_pg_type(int (0,1), optional):  used to name the icon labels. 0-> general 1->specific

        Returns:
            QWidget: A container widget with the icon and text label.
        """
        # Create a container widget and a horizontal layout for icon and text
        temp_container = QWidget()
        temp_layout = QHBoxLayout(temp_container)
        # If a tab spacing value is provided, set the left margin accordingly
        if temp_tab_spacing: 
            temp_layout.setContentsMargins(temp_tab_spacing, 0, 0, 0)
        temp_layout.setSpacing(20)  # Space between icon and text
        # If an icon is provided, create a QLabel for it and add to the layout
        if icon:
            icon_label = self.make_icon_label(icon, text, vehicle_number, icon_pg_type)
            temp_layout.addWidget(icon_label, alignment=Qt.AlignmentFlag.AlignVCenter)
        # Create the text label and add to the layout
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setStyleSheet(f"color: {self.text_color};")
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)

        # Return the container widget with icon and text
        return temp_container

    #used to create the title labels throughout the window
    def create_title_label(self, text):
        """
        Creates and styles a QLabel for section titles throughout the window.
        Used for headers in columns and sections.
        """
        temp_label = QLabel(text)
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setStyleSheet(f"color: {self.text_color};")
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        return temp_label

    #used to create the section header labels inside status cards (e.g. "Connections", "Sensors", "Status")
    def create_section_header_label(self, text):
        """
        Creates a bold section header label with an accent underline, used to visually
        separate groups of related status information within a status card.
        """
        temp_label = QLabel(text)
        temp_label.setObjectName("sectionHeaderLabel")
        temp_label.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        temp_label.setStyleSheet(
            f"color: {self.text_color}; border-bottom: 2px solid {self.border_outline}; padding-bottom: 6px;"
        )
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        return temp_label

    #used to create the small muted sub-group headings inside a status card (e.g. "Mission", "Position")
    def create_group_label(self, text):
        """
        Creates a small, accent-colored uppercase sub-heading used to label one group of
        related fields (e.g. "Mission", "Position", "Telemetry") within a status card.
        """
        temp_label = QLabel(text.upper())
        temp_label.setObjectName("groupLabel")
        temp_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        temp_label.setStyleSheet(f"color: {self.border_outline};")
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        return temp_label

    #used to lay out a group of status labels as a compact two-column grid
    def create_status_grid(self, rows):
        """
        Builds a two-column QGridLayout of status labels from `rows`, where each row is
        a list of one or two (text, name) pairs passed straight through to
        create_normal_label. A row with a single pair spans both columns.
        Returns the QGridLayout.
        """
        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(10)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        for row_idx, pair in enumerate(rows):
            if len(pair) == 2:
                (text_a, name_a), (text_b, name_b) = pair
                grid.addWidget(self.create_normal_label(text_a, name_a), row_idx, 0, alignment=Qt.AlignmentFlag.AlignVCenter)
                grid.addWidget(self.create_normal_label(text_b, name_b), row_idx, 1, alignment=Qt.AlignmentFlag.AlignVCenter)
            else:
                (text_a, name_a), = pair
                grid.addWidget(self.create_normal_label(text_a, name_a), row_idx, 0, 1, 2, alignment=Qt.AlignmentFlag.AlignVCenter)
        return grid

    #used to wrap a status section (Connections/Sensors/Status) in a rounded card for visual grouping
    def create_status_card(self, inner_layout):
        """
        Wraps the given layout in a bordered, rounded QFrame "card" so related status
        information (connections, sensors, mission status, etc.) reads as one visual group.
        Returns the QFrame; `inner_layout` is set as its layout, so widgets can keep being
        added to `inner_layout` exactly as before.

        Note: the border is scoped with an ID selector (#statusCard) rather than a bare
        declaration or a "QFrame { ... }" type selector. Both of those leak: a bare
        declaration set via setStyleSheet() behaves like a "*" (universal) selector and
        paints every descendant that doesn't set its own border, and a "QFrame" type
        selector matches QLabel too since QLabel is itself a QFrame subclass in Qt. Either
        way every plain status label inside picked up its own little rounded box. An ID
        selector only ever matches the one widget carrying that objectName, so it can't
        cascade onto children no matter what they are.
        """
        card = QFrame()
        card.setObjectName("statusCard")
        card.setStyleSheet(
            f"QFrame#statusCard {{ border: 2px solid {self.border_outline}; border-radius: 10px; }}"
        )
        inner_layout.setContentsMargins(14, 12, 14, 14)
        card.setLayout(inner_layout)
        return card

    #used to create the buttons column on the specific vehicle pages
    def create_vehicle_buttons_column(self, vehicle_number):
        """
        Creates the button column for a specific Vehicle tab, including mission control and emergency buttons,
        as well as labels for seconds since last connection.
        Arranges buttons in two sub-columns and adds status labels at the top.
        Returns the vertical container widget holding all buttons and labels for the Vehicle.
        """
        # Create temporary containers and layouts for organizing buttons and labels
        temp_sub_container1 = QWidget()
        temp_layout1 = QVBoxLayout(temp_sub_container1)
        temp_layout1.setAlignment(Qt.AlignmentFlag.AlignTop)

        # Temp container for the second column of buttons
        temp_sub_container2 = QWidget()
        temp_layout2 = QVBoxLayout(temp_sub_container2)
        temp_layout2.setAlignment(Qt.AlignmentFlag.AlignTop)

        # Temp container for the button columns put together horizontally
        temp_container = QWidget()
        temp_layout = QHBoxLayout(temp_container)

        # Temp container for the entire column, the buttons and the last connected labels
        temp_V_container = QWidget()
        temp_V_layout = QVBoxLayout(temp_V_container)
        setattr(self, f"vehicle{vehicle_number}_buttons_column_widget", temp_V_container)
        setattr(self, f"vehicle{vehicle_number}_buttons_column_layout", temp_V_layout)

        # Load mission (normal button)
        self.create_vehicle_button(vehicle_number, "load_mission", "Load Mission", lambda: self.spec_load_missions_button(vehicle_number))
        # Start mission (normal button)
        self.create_vehicle_button(vehicle_number, "start_mission", "Start Mission", lambda: self.spec_start_missions_button(vehicle_number))
        # Start mission (normal button)
        self.create_vehicle_button(vehicle_number, "copy_bag", "Copy Bag to Base Station", lambda: self.spec_copy_bags(vehicle_number))
        # Sync vehicle (normal button)
        self.create_vehicle_button(vehicle_number, "sync", "Calibrate Vehicle (BUGGY)", lambda: self.run_calibrate_script(vehicle_number))
        # Relay / strobe control (normal button)
        self.create_vehicle_button(vehicle_number, "hardware_control", "Relay / Strobe Control", lambda: self.hardware_control_button(vehicle_number))

        # Emergency surface (danger button)
        self.create_vehicle_button(vehicle_number, "emergency_surface", "Emergency Surface", lambda: self.emergency_surface_button(vehicle_number), danger=True)
        # Publish the shared origin to the vehicles
        self.create_vehicle_button(vehicle_number, "publish_origin", "Publish Origin", self.publish_origin_command)
        # Emergency shutdown (danger button)
        self.create_vehicle_button(vehicle_number, "emergency_shutdown", "Emergency Shutdown", lambda: self.emergency_shutdown_button(vehicle_number), danger=True)
        # Clear console (danger button)
        self.create_vehicle_button(vehicle_number, "clear_console", "Clear Console", lambda: self.clear_console(vehicle_number), danger=True)

        temp_spacing = 20
        # Add buttons to the first and second sub-columns with spacing
        temp_layout1.addWidget(getattr(self, f"load_mission_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"start_mission_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"copy_bag_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"sync_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"hardware_control_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_surface_vehicle{vehicle_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"publish_origin_vehicle{vehicle_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_shutdown_vehicle{vehicle_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"clear_console_vehicle{vehicle_number}_button"))

        # Add the two button columns to the main horizontal layout
        temp_layout.addWidget(temp_sub_container1)
        temp_layout.addWidget(temp_sub_container2)

        # Add a title label and connection time labels to the vertical layout
        temp_V_layout.addWidget(self.create_title_label("Seconds since last connected"))
        self.insert_label(temp_V_layout, "Wifi: xxx", vehicle_number, 2)
        self.insert_label(temp_V_layout, "Radio: xxx", vehicle_number, 1)
        self.insert_label(temp_V_layout, "Acoustics: xxx", vehicle_number, 0)

        temp_V_layout.addWidget(self.make_hline())
        temp_V_layout.addWidget(temp_container)
        
        # Return the vertical container holding all buttons and labels
        return temp_V_container

    def insert_label(self, temp_layout, text, vehicle_number, conn_type):
        """
        Inserts a QLabel into the given layout for displaying the seconds since last connection
        for either radio or modem, and stores it as an attribute for later access.
        """
        text_label = QLabel(text)
        if conn_type == 1:
            name = f"vehicle{vehicle_number}_radio_seconds_widget"
        elif conn_type == 2:
            name = f"vehicle{vehicle_number}_wifi_seconds_widget"
        elif conn_type == 0:
            name = f"vehicle{vehicle_number}_modem_seconds_widget"
        setattr(self, name, text_label)
        text_label.setObjectName(name)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setStyleSheet(f"color: {self.text_color};")
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)

    def create_seconds_label(self, conn_type, seconds):
        """
        Creates a QLabel displaying the seconds since last connection for either radio or modem.
        Used for status display in the vehicle button column.
        """
        if conn_type:
            text = f"Radio: {seconds}"
        else:
            text = f"Acoustics: {seconds}"
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setStyleSheet(f"color: {self.text_color};")
        return text_label

    #Dynamically creates a QPushButton with the given properties and stores it as an attribute.
    def create_vehicle_button(self, vehicle_number, name, text, callback, danger=False):
        """
        Dynamically creates a QPushButton with the given properties and stores it as an attribute.
        Used for mission control and emergency actions in the vehicle button column.
        """
        button = QPushButton(text)
        button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        button.clicked.connect(callback)
        button.setStyleSheet(self.danger_button_style_sheet) if danger else button.setStyleSheet(self.normal_button_style_sheet)
        attr_name = f"{name}_vehicle{vehicle_number}_button"
        setattr(self, attr_name, button)

    def create_specific_vehicle_column0(self, vehicle_number):
        """
        Creates the first column for a specific Vehicle tab, displaying connection and sensor status icons.
        Arranges icons vertically for Wifi, Radio, Modem, DVL, GPS, and IMU inside a bordered status card,
        with the Connections and Sensors groups visually separated for easier scanning.
        Returns the container widget for the column.
        """
        # Create a vertical layout to hold the card's contents and store it as an attribute
        temp_layout = QVBoxLayout()
        setattr(self, f"vehicle{vehicle_number}_column0_layout", temp_layout)
        temp_layout.setSpacing(0)

        # Wrap the section in a rounded card so it reads as one visual group
        card = self.create_status_card(temp_layout)

        # Create the container widget for this column and store it as an attribute
        temp_container = QWidget()
        setattr(self, f"vehicle{vehicle_number}_column0_widget", temp_container)
        container_layout = QVBoxLayout(temp_container)
        container_layout.setContentsMargins(0, 0, 10, 0)

        # Add the Vehicle title label above the card
        container_layout.addWidget(self.create_title_label(f"Vehicle {vehicle_number}"), alignment=Qt.AlignmentFlag.AlignTop)
        container_layout.addSpacing(10)
        container_layout.addWidget(card)

        # Section: Connections
        temp_layout.addWidget(self.create_section_header_label("Connections"))
        temp_layout.addSpacing(12)

        # Add connection status icons (Wifi, Radio, Modem)
        for label_text in ("Wifi", "Radio", "Modem"):
            temp_layout.addWidget(self.create_icon_and_text(label_text, self.icons_dict[self.feedback_dict[label_text][vehicle_number]], 0, vehicle_number, 1))
            temp_layout.addSpacing(10)

        temp_layout.addSpacing(4)
        temp_layout.addWidget(self.make_hline())
        temp_layout.addSpacing(16)

        # Section: Sensors
        temp_layout.addWidget(self.create_section_header_label("Sensors"))
        temp_layout.addSpacing(12)

        # Add sensor status icons (DVL, GPS, IMU)
        for label_text in ("DVL", "GPS", "IMU"):
            temp_layout.addWidget(self.create_icon_and_text(label_text, self.icons_dict[self.feedback_dict[label_text][vehicle_number]], 0, vehicle_number, 1))
            temp_layout.addSpacing(10)

        # Return the container widget holding all status icons
        return temp_container

    #create the second sub-column in the first column of the specific vehiclear pages (starts with "Nodes")
    def create_specific_vehicle_column01(self, vehicle_number):
        """
        Creates the second sub-column in the first column of the specific Vehicle pages.
        This column displays the mission section and the status widgets for the given Vehicle,
        grouped inside a bordered status card (position, mission progress, telemetry) for readability.
        Returns the container widget for the column.
        """
        # Create a vertical layout to hold the card's contents
        temp_layout = QVBoxLayout()
        temp_layout.setSpacing(0)

        # Wrap the section in a rounded card so it reads as one visual group
        card = self.create_status_card(temp_layout)

        # Create the container widget for this column and store it as an attribute
        temp_container = QWidget()
        setattr(self, f"vehicle{vehicle_number}_column01_layout", temp_layout)
        setattr(self, f"vehicle{vehicle_number}_column01_widget", temp_container)
        container_layout = QVBoxLayout(temp_container)
        container_layout.setContentsMargins(0, 0, 0, 0)

        # Add a blank title label at the top so this card's top edge lines up with the
        # Connections/Sensors card in the previous column (which has a real title above it)
        container_layout.addWidget(self.create_title_label(""), alignment=Qt.AlignmentFlag.AlignTop)
        container_layout.addSpacing(10)
        container_layout.addWidget(card)

        # Status widgets section, arranged as a compact two-column grid of related-field
        # pairs and grouped Mission -> Position -> Telemetry (most operationally relevant,
        # i.e. what the mission is doing right now, first).
        status_spacing = 10
        temp_layout.addWidget(self.create_section_header_label("Status"))
        temp_layout.addSpacing(status_spacing + 4)

        # Mission progress: state paired with elapsed time, waypoint id paired with its
        # state, and distance-to-go spans the row on its own since it has no natural pair.
        temp_layout.addWidget(self.create_group_label("Mission"))
        temp_layout.addSpacing(6)
        temp_layout.addLayout(self.create_status_grid([
            [("Mission State: Idle", f"Mission_state{vehicle_number}"), ("Mission Time: 0.0 s", f"Mission_time{vehicle_number}")],
            [("Current Waypoint: w", f"Waypoint{vehicle_number}"), ("Waypoint State: Idle", f"Waypoint_state{vehicle_number}")],
            [("Distance to Next WP (m): x", f"Waypoint_distance{vehicle_number}")],
        ]))
        temp_layout.addSpacing(status_spacing + 4)
        temp_layout.addWidget(self.make_hline())
        temp_layout.addSpacing(status_spacing + 4)

        # Position & heading: x/y coordinates paired, depth/heading paired.
        temp_layout.addWidget(self.create_group_label("Position"))
        temp_layout.addSpacing(6)
        temp_layout.addLayout(self.create_status_grid([
            [("x (m): x", f"XPos{vehicle_number}"), ("y (m): y", f"YPos{vehicle_number}")],
            [("Depth (m): d", f"Depth{vehicle_number}"), ("Heading (deg): h", f"Heading{vehicle_number}")],
        ]))
        temp_layout.addSpacing(status_spacing + 4)
        temp_layout.addWidget(self.make_hline())
        temp_layout.addSpacing(status_spacing + 4)

        # Telemetry / vehicle health: battery and pressure paired, DVL velocity on its own.
        temp_layout.addWidget(self.create_group_label("Telemetry"))
        temp_layout.addSpacing(6)
        temp_layout.addLayout(self.create_status_grid([
            [("Battery (V): b", f"Battery{vehicle_number}"), ("Pressure (Pa):<br>p", f"Pressure{vehicle_number}")],
            [("DVL Velocity <br>(m/s): v", f"DVL_vel{vehicle_number}")],
        ]))

        # Return the container widget holding the mission and status widgets
        return temp_container

    def create_normal_label(self, text, name): 
        """
        Creates a QLabel with the given text and object name, sets its font and margins, 
        and stores it as an attribute for later access.

        Parameters:
            text (str): The text to display in the label.
            name (str): The object name and attribute name for the label.

        Returns:
            QLabel: The created label.
        """
        text_label = QLabel(text)
        text_label.setTextFormat(Qt.TextFormat.RichText)
        setattr(self, name, text_label)
        text_label.setObjectName(name) 
        text_label.setFont(QFont("Arial", 13))
        text_label.setStyleSheet(f"color: {self.text_color};")
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setWordWrap(True)  # Allow text to wrap if it's long
        text_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return text_label

    def recieve_safety_status_message(self, vehicle_number, safety_message):
        """
        Receives a safety status message from ROS and emits a signal to update the GUI.
        Used to update status widgets for GPS, DVL, IMU, and emergency status.
        """
        self.safety_status_signal.emit(vehicle_number, safety_message)

    def _update_safety_status_information(self, vehicle_number, safety_message):
        """
        Updates the GUI to reflect the latest safety status information for each Vehicle.
        This includes GPS, DVL, IMU status, and emergency status messages.

        Parameters:
            vehicle_number: The vehicle number (index) for which to update the status.
            safety_message: The safety status message object containing various status fields.
        """

        #logic is opposite, switch 0 and 1
        if safety_message.gps_status.data: gps_data = 0
        else: gps_data = 1
        self.feedback_dict["GPS"][vehicle_number] = gps_data

        if safety_message.dvl_status.data: dvl_data = 0
        else: dvl_data = 1
        self.feedback_dict["DVL"][vehicle_number] = dvl_data 
        
        if safety_message.imu_published.data: imu_data = 1
        else: imu_data = 0
        self.feedback_dict["IMU"][vehicle_number] = imu_data
        
        #replace general page widgets
        self.replace_general_page_icon_widget(vehicle_number, "GPS")
        self.replace_general_page_icon_widget(vehicle_number, "DVL")
        # self.replace_general_page_icon_widget(vehicle_number, "Wifi")
        self.replace_general_page_icon_widget(vehicle_number, "IMU")

        #replace specific page widgets
        self.replace_specific_icon_widget(vehicle_number, "GPS")
        self.replace_specific_icon_widget(vehicle_number, "DVL")
        # self.replace_specific_icon_widget(vehicle_number, "Wifi")
        self.replace_specific_icon_widget(vehicle_number, "IMU")

        #replace emergency status label
        if self.feedback_dict["Status_messages"][vehicle_number] != safety_message.emergency_status.data:
            self.feedback_dict["Status_messages"][vehicle_number] = safety_message.emergency_status.data
            layout = self.general_page_vehicle_layouts.get(vehicle_number)
            widget = self.general_page_vehicle_widgets.get(vehicle_number)
            new_status_label = self.get_status_label(vehicle_number, self.feedback_dict["Status_messages"][vehicle_number])
            existing_label = widget.findChild(QLabel, f"Status_messages{vehicle_number}")
            if existing_label: existing_label.setText(new_status_label.text())

    def recieve_dvl_velocity(self, vehicle_number, msg):
        """
        Receives a DVL velocity message from ROS and emits a signal to update the GUI.
        Used to update the DVL velocity widget for the vehicle.
        """
        self.dvl_velocity_signal.emit(vehicle_number, msg)
    
    def _update_dvl_velocity(self, vehicle_number, msg):
        """
        Updates the DVL velocity widget for the specified vehicle based on the received message.
        """
        #gets linear velocity from the x,y,and z components of the velocity vector
        self.feedback_dict["DVL_vel"][vehicle_number] = round(math.sqrt(msg.velocity.x**2 + msg.velocity.y**2 + msg.velocity.z**2), 2)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "DVL_vel")

    def recieve_state_estimate_message(self, vehicle_number, msg):
        """
        Receives a state estimate message from ROS and emits a signal to update the GUI.
        Used to update x, y, depth, and heading widgets.
        """
        self.smoothed_output_signal.emit(vehicle_number, msg)

    def _update_gui_smoothed_output(self, vehicle_number, msg):
        """
        Updates the GUI widgets for x, y, depth, and heading based on nav_msgs/Odometry state_estimate.
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x = position.x
        y = position.y
        depth = -position.z
        heading = math.degrees(
            math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
        )

        self.feedback_dict["XPos"][vehicle_number] = round(x, 5)
        self.feedback_dict["YPos"][vehicle_number] = round(y, 5)
        self.feedback_dict["Depth"][vehicle_number] = round(depth, 5)
        self.feedback_dict["Heading"][vehicle_number] = round(heading, 3)

        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "XPos")
        self.replace_specific_status_widget(vehicle_number, "YPos")
        self.replace_specific_status_widget(vehicle_number, "Depth")
        self.replace_specific_status_widget(vehicle_number, "Heading")

    def recieve_depth_data_message(self, vehicle_number, msg):
        """
        Receives a depth data message from ROS and emits a signal to update the GUI.
        Used to update the depth widget for the vehicle.
        """
        self.depth_data_signal.emit(vehicle_number, msg)

    def update_depth_data(self, vehicle_number, msg):
        """
        Updates the depth widget for the specified vehicle based on the received message.
        """
        #update feedback dict 
        self.feedback_dict["Depth"][vehicle_number] = round(msg.pose.pose.position.z, 2)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "Depth")

    def recieve_pressure_data_message(self, vehicle_number, msg):
        """
        Receives a pressure data message from ROS and emits a signal to update the GUI.
        Used to update the pressure widget for the vehicle.
        """
        self.pressure_data_signal.emit(vehicle_number, msg)

    def update_pressure_data(self, vehicle_number, msg):
        """
        Updates the pressure widget for the specified vehicle based on the received message.
        """
        #update feedback dict 
        self.feedback_dict["Pressure"][vehicle_number] = round(msg.fluid_pressure, 2)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "Pressure")

    def recieve_battery_data_message(self, vehicle_number, msg):
        """
        Receives a battery data message from ROS and emits a signal to update the GUI.
        Used to update the battery widget for the vehicle.
        """
        self.battery_data_signal.emit(vehicle_number, msg)
        
    def update_battery_data(self, vehicle_number, msg):
        """
        Updates the battery widget for the specified vehicle based on the received message.
        """
        #update feedback dict 
        self.feedback_dict["Battery"][vehicle_number] = round(msg.voltage, 1)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "Battery")

    def recieve_mission_feedback(self, vehicle_number, msg):
        """
        Receives mission feedback from the vehicle navigation stack.
        """
        self.mission_feedback_signal.emit(vehicle_number, msg)

    def recieve_waypoint_feedback(self, vehicle_number, msg):
        """
        Receives current waypoint feedback from the vehicle navigation stack.
        """
        self.waypoint_feedback_signal.emit(vehicle_number, msg)

    def _update_mission_feedback(self, vehicle_number, msg):
        state_labels = {
            0: "Idle",
            1: "Running",
            2: "Paused",
            3: "Complete",
            4: "Aborted",
        }

        state_value = diagnostic_level_value(msg.state)
        state = state_labels.get(state_value, f"Unknown ({state_value})")
        total = int(msg.waypoints_total)
        completed = int(msg.waypoints_completed)
        current = completed if state == "Complete" else completed + 1
        current = min(current, total) if total else 0

        self.feedback_dict["Mission_state"][vehicle_number] = state
        self.feedback_dict["Mission_time"][vehicle_number] = f"{float(msg.elapsed_time):.1f} s"
        self.feedback_dict["Waypoint"][vehicle_number] = f"{current} / {total}"

        distance = getattr(getattr(msg, "current", None), "horizontal_distance_error", None)
        if distance is not None:
            self.feedback_dict["Waypoint_distance"][vehicle_number] = f"{float(distance):.1f}"

        for key in ("Mission_state", "Mission_time", "Waypoint", "Waypoint_distance"):
            self.replace_specific_status_widget(vehicle_number, key)

    def _update_waypoint_feedback(self, vehicle_number, msg):
        waypoint_state_labels = {
            0: "Idle",
            1: "Transiting",
            2: "Arrived",
            3: "Parking",
            4: "Skipped",
        }
        waypoint_state_value = diagnostic_level_value(msg.state)
        waypoint_state = waypoint_state_labels.get(waypoint_state_value, f"Unknown ({waypoint_state_value})")

        self.feedback_dict["Waypoint_distance"][vehicle_number] = f"{float(msg.horizontal_distance_error):.1f}"
        self.feedback_dict["Waypoint_state"][vehicle_number] = waypoint_state

        self.replace_specific_status_widget(vehicle_number, "Waypoint_distance")
        self.replace_specific_status_widget(vehicle_number, "Waypoint_state")

    def recieve_kill_confirmation_message(self, kill_message):
        """
        Slot to receive a kill confirmation message from the ROS topic.
        Emits a signal to update the GUI with the received message.
        
        Parameters:
            kill_message: The message object received from the 'confirm_e_kill' topic (std_msgs/Bool).
        """
        self.kill_confirm_signal.emit(kill_message)

    def _update_kill_confirmation_gui(self, kill_message): 
        """
        Slot connected to kill_confirm_signal.
        Updates the GUI console with a confirmation or failure message for all Vehicles,
        depending on the value of the kill_message.
        
        Parameters:
            kill_message: The message object received from the 'confirm_e_kill' topic (std_msgs/Bool).
        """
        value = kill_message.data if hasattr(kill_message, 'data') else kill_message
        if value: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Kill Command Confirmed", i)
        else: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Kill Command Failed", i)

    def recieve_surface_confirmation_message(self, surf_message): 
        """
        Slot to receive a surface confirmation message from the ROS topic.
        Emits a signal to update the GUI with the received message.
        
        Parameters:
            surf_message: The message object received from the 'confirm_e_surface' topic (std_msgs/Bool).
        """
        self.surface_confirm_signal.emit(surf_message)

    def _update_surf_confirmation_gui(self, surf_message):
        """
        Slot connected to surface_confirm_signal.
        Updates the GUI console with a confirmation or failure message for all Vehicles,
        depending on the value of the surf_message.
        
        Parameters:
            surf_message: The message object received from the 'confirm_e_surface' topic (std_msgs/Bool).
        """
        value = surf_message.data if hasattr(surf_message, 'data') else surf_message
        if value: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Surface Command Confirmed", i)
        else: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Surface Command Failed", i)

    def recieve_connections(self, conn_message):
        """
        Slot to receive a Connections message and emit a signal to update the GUI.

        Parameters:
            conn_message: The Connections message object.
        """
        self.update_connections_signal.emit(conn_message)

    def recieve_link_status(self, vehicle_number, status_message):
        """
        Slot to receive a per-vehicle DiagnosticStatus link update.
        """
        self.update_connections_signal.emit((vehicle_number, status_message))

    def _update_connections_gui(self, conn_message):
        """
        Updates the GUI to reflect the latest connection status and ping times for each Vehicle.

        Parameters:
            conn_message: Either a (vehicle_number, DiagnosticStatus) tuple or the legacy
            Connections message object containing connection_type, connections, and last_ping.
        """
        try:
            if isinstance(conn_message, tuple):
                vehicle_number, status_message = conn_message
                self._update_link_status_gui(vehicle_number, status_message)
                return

            if conn_message.connection_type == 1:
                feedback_key = "Radio"
                feedback_key_seconds = "Radio_seconds"
                conn_type = 1
            elif conn_message.connection_type == 0:
                feedback_key = "Modem"
                feedback_key_seconds = "Modem_seconds"
                conn_type = 0
            elif conn_message.connection_type == 2:
                feedback_key = "Wifi"
                feedback_key_seconds = "Wifi_seconds"
                conn_type = 2

            # Update connection status icons for each Vehicle
            for i, vehicle_number in enumerate(conn_message.vehicle_ids):
                try:
                    if vehicle_number not in self.feedback_dict[feedback_key]:
                        continue
                    
                    # Use the index i instead of vehicle_number-1
                    status = 1 if conn_message.connections[i] else 0
                    self.feedback_dict[feedback_key][vehicle_number] = status
                    # prefix = feedback_key.split("_")[0]
                    
                    # Update general page
                    layout = self.general_page_vehicle_layouts.get(vehicle_number)
                    widget = self.general_page_vehicle_widgets.get(vehicle_number)
                    if layout and widget:
                        self.replace_general_page_icon_widget(vehicle_number, feedback_key)

                    # Update specific page
                    layout = getattr(self, f"vehicle{vehicle_number}_column0_layout", None)
                    widget = getattr(self, f"vehicle{vehicle_number}_column0_widget", None)
                    if layout and widget:
                        self.replace_specific_icon_widget(vehicle_number, feedback_key)
                        
                except Exception as e:
                    print(f"Exception updating connection status for vehicle {vehicle_number}: {e}")

            # Update seconds since last ping for each Vehicle
            for i, vehicle_number in enumerate(conn_message.vehicle_ids):
                try:
                    if vehicle_number not in self.selected_vehicles:
                        continue
                    
                    # Use the index i instead of count
                    ping = conn_message.last_ping[i]
                    self.feedback_dict[feedback_key_seconds][vehicle_number] = ping
                    
                    layout = getattr(self, f"vehicle{vehicle_number}_buttons_column_layout", None)
                    widget = getattr(self, f"vehicle{vehicle_number}_buttons_column_widget", None)
                    
                    if layout and widget:
                        if conn_type == 1:
                            old_label = f"vehicle{vehicle_number}_radio_seconds_widget"
                            existing_label = widget.findChild(QLabel, old_label)
                            new_text = f"Radio: {ping}"
                            if existing_label:
                                existing_label.setText(new_text)
                        elif conn_type == 2:
                            old_label = f"vehicle{vehicle_number}_wifi_seconds_widget"
                            existing_label = widget.findChild(QLabel, old_label)
                            new_text = f"Wifi: {ping}"
                            if existing_label:
                                existing_label.setText(new_text)
                        else:
                            old_label = f"vehicle{vehicle_number}_modem_seconds_widget"
                            existing_label = widget.findChild(QLabel, old_label)
                            new_text = f"Acoustics: {ping}"
                            if existing_label:
                                existing_label.setText(new_text)
                                
                except Exception as e:
                    print(f"Exception updating ping time for vehicle {vehicle_number}: {e}")

        except Exception as e:
            print("Exception in update_connections_gui:", e)
            for i in self.selected_vehicles:
                self.recieve_console_update(f"Exception in update_connections_gui: {e}", i)

    def _update_link_status_gui(self, vehicle_number, status_message):
        hardware_id = getattr(status_message, "hardware_id", "").lower()
        link_info = {
            "radio": ("Radio", "Radio_seconds", "Radio"),
            "wifi": ("Wifi", "Wifi_seconds", "Wifi"),
            "modem": ("Modem", "Modem_seconds", "Acoustics"),
        }.get(hardware_id)

        if link_info is None or vehicle_number not in self.selected_vehicles:
            return

        feedback_key, feedback_key_seconds, label_prefix = link_info
        status = 1 if diagnostic_level_value(getattr(status_message, "level", 2)) == 0 else 0
        self.feedback_dict[feedback_key][vehicle_number] = status

        layout = self.general_page_vehicle_layouts.get(vehicle_number)
        widget = self.general_page_vehicle_widgets.get(vehicle_number)
        if layout and widget:
            self.replace_general_page_icon_widget(vehicle_number, feedback_key)

        layout = getattr(self, f"vehicle{vehicle_number}_column0_layout", None)
        widget = getattr(self, f"vehicle{vehicle_number}_column0_widget", None)
        if layout and widget:
            self.replace_specific_icon_widget(vehicle_number, feedback_key)

        seconds = self._seconds_from_link_status(status_message, feedback_key_seconds, vehicle_number)
        self.feedback_dict[feedback_key_seconds][vehicle_number] = seconds

        buttons_widget = getattr(self, f"vehicle{vehicle_number}_buttons_column_widget", None)
        if buttons_widget:
            label_name = f"vehicle{vehicle_number}_{hardware_id}_seconds_widget"
            existing_label = buttons_widget.findChild(QLabel, label_name)
            if existing_label:
                existing_label.setText(f"{label_prefix}: {seconds}")

    def _seconds_from_link_status(self, status_message, fallback_key, vehicle_number):
        values = {item.key: item.value for item in getattr(status_message, "values", [])}

        if "last_ping_seconds" in values:
            try:
                return int(float(values["last_ping_seconds"]))
            except (TypeError, ValueError):
                pass

        if "last_message_time" in values:
            try:
                timestamp = float(values["last_message_time"])
                if timestamp > 0:
                    return max(0, int(time.time() - timestamp))
            except (TypeError, ValueError):
                pass

        return self.feedback_dict[fallback_key].get(vehicle_number, 0)
            
    def get_status_label(self, vehicle_number, status_message):
        """
        Returns a QLabel for the emergency status message for the given vehicle.
        Used to display status such as "Good", "EMERGENCY", "Surfaced/Disarmed", or "No Data Received".
        """
        if not status_message: message_text = "Good"
        elif status_message == 1: message_text = "EMERGENCY: <br>Recall Vehicle"
        elif status_message == 2: message_text = "Surfaced/Disarmed"
        else: message_text = "No Data Received"

        temp_label = QLabel(f"{message_text}", font=QFont("Arial", 13), alignment=Qt.AlignmentFlag.AlignTop)
        temp_label.setTextFormat(Qt.TextFormat.RichText)
        return temp_label

    def recieve_console_update(self, console_message, vehicle_number):
        """
        Slot to receive a console message and emit a signal to update the GUI.

        Parameters:
            console_message: The console message object.
        """
        self.update_console_signal.emit(console_message, vehicle_number)
    
    def _update_console_gui(self, console_message, vehicle_number):
        """
        Appends a new console message to the specific Vehicle's console log label.
        If vehicle_number is 0, send to all selected Vehicles.
        """
        # Determine which Vehicles to update
        if vehicle_number == 0:
            vehicle_numbers = self.selected_vehicles
        else:
            vehicle_numbers = [vehicle_number]

        for vehicle in vehicle_numbers:
            try:
                label = self.findChild(QLabel, f"Console_messages{vehicle}")
                if label:
                    current_text = label.text()
                    updated_text = f"{current_text}\n{console_message}" if current_text else console_message
                    label.setText(updated_text)
                    label.setStyleSheet(f"color: {self.text_color};")
                    # Scroll to the bottom of the scroll area only if user was already at the bottom
                    scroll_area = getattr(self, f"vehicle{vehicle}_console_scroll_area", None)
                    if scroll_area:
                        vbar = scroll_area.verticalScrollBar()
                        at_bottom = vbar.value() >= vbar.maximum() - 2  # Allow for rounding
                        def maybe_scroll():
                            if at_bottom:
                                vbar.setValue(vbar.maximum())
                        QTimer.singleShot(50, maybe_scroll)
                else:
                    print(f"Console log label not found for Vehicle {vehicle}")
            except Exception as e:
                print(f"Exception in _update_console_gui for Vehicle {vehicle}: {e}")

    def replace_general_page_icon_widget(self, vehicle_number, prefix):
        """
        Replaces the icon widget for a connection or sensor on the general page for the specified vehicle.
        Updates the icon based on the current status in the feedback_dict.
        """
        layout = self.general_page_vehicle_layouts.get(vehicle_number)
        widget = self.general_page_vehicle_widgets.get(vehicle_number)
        status = self.feedback_dict[prefix][vehicle_number]
        icon_type = self.icons_dict[status]
        existing_label = widget.findChild(QLabel, f"icon_{prefix}{vehicle_number}0")
        if existing_label: self.replace_icon_widget(existing_label, icon_type)
        else: print(f"icon_{prefix}{vehicle_number}0 label does not exist")
    
    def replace_specific_icon_widget(self, vehicle_number, prefix):
        """
        Replaces the icon widget for a connection or sensor on the specific vehicle tab.
        Updates the icon based on the current status in the feedback_dict.
        """
        layout = getattr(self, f"vehicle{vehicle_number}_column0_layout")
        widget = getattr(self, f"vehicle{vehicle_number}_column0_widget")
        status = self.feedback_dict[prefix][vehicle_number]
        icon_type = self.icons_dict[status]
        existing_label = widget.findChild(QLabel, f"icon_{prefix}{vehicle_number}1")
        if existing_label: self.replace_icon_widget(existing_label, icon_type)
        else: print(f"icon_{prefix}{vehicle_number}1 label does not exist")

    def replace_icon_widget(self, icon_label, icon_type):
        """
        Updates the icon label's pixmap and type to reflect the new status.
        Used for both general and specific vehicle tab icons.
        """
        if icon_label: 
            icon_label._icon_type = icon_type
            # Update the original icon pixmap to the new icon
            icon_pixmap = self.style().standardIcon(icon_type).pixmap(16, 16)
            icon_label._original_icon_pixmap = icon_pixmap
            self.repaint_icon(icon_label)

    def replace_specific_status_widget(self, vehicle_number, prefix):
        """
        Updates the status widget (label) for a specific vehicle tab with the latest value.
        Used for position, depth, heading, velocity, battery, and pressure.
        """
        layout = getattr(self, f"vehicle{vehicle_number}_column01_layout")
        widget = getattr(self, f"vehicle{vehicle_number}_column01_widget")
        new_text = self.key_to_text_dict[prefix] + str(self.feedback_dict[prefix][vehicle_number])
        existing_label = widget.findChild(QLabel, f"{prefix}{vehicle_number}")
        if existing_label: existing_label.setText(new_text)
        else: print(f"label with name {prefix}{vehicle_number} does not exist")

#used by ros to open a window. Needed in order to start PyQt on a different thread than ros
def OpenWindow(ros_node, selected_vehicles, borders=False):
    """
    Launches the main GUI window for the base station application.
    Handles splash screen display and main window instantiation.
    Returns the QApplication instance and a result dict containing the window.

    Parameters:
        ros_node: The ROS node to pass to the MainWindow.
        selected_vehicles (list): List of vehicle numbers to create tabs for.
        borders (bool): If True, applies a red border to all widgets for debugging layout.

    Steps:
        1. Create QApplication and set window size.
        2. Load and display splash image (with dark mode inversion).
        3. Center splash on the screen.
        4. Show splash message and build main window after a delay.
        5. Return app and result dict.
    """
    app = QApplication(sys.argv)
    window_width, window_height = 1200, 800

    # Prepare splash image
    img_path = media_directory

    pixmap = QPixmap(img_path)
    pixmap = pixmap.toImage()
    pixmap.invertPixels() #This turns the Splash image from dark to light for dark mode
    pixmap = QPixmap.fromImage(pixmap)
    if pixmap.isNull():
        print(f"Warning: ⚠️ Could not load splash image '{img_path}'. Using solid color instead.")
        pixmap = QPixmap(window_width, window_height)
        pixmap.fill(QColor(("#0F1C37")))
    else:
        background = QPixmap(window_width, window_height)
        background.fill(QColor("#0F1C37"))
        painter = QPainter(background)
        x = (window_width - pixmap.width()) // 2
        y = (window_height - pixmap.height()) // 2
        painter.drawPixmap(x, y, pixmap)
        painter.end()
        pixmap = background

    pixmap = pixmap.scaled(window_width, window_height, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
    splash = CustomSplash(pixmap)
    splash.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, False)
    splash.show()

    # Move splash to the center of the screen
    screen = app.primaryScreen()
    if QApplication.screens():
        mouse_pos = QCursor.pos()
        for scr in QApplication.screens():
            if scr.geometry().contains(mouse_pos):
                screen = scr
                break
    screen_geometry = screen.geometry()
    x = screen_geometry.x() + (screen_geometry.width() - window_width) // 2
    y = screen_geometry.y() + (screen_geometry.height() - window_height) // 2
    splash.move(x, y)

    app.processEvents()

    # Display loading message on splash
    splash.showMessage(f"Loading main window for vehicles: {selected_vehicles}...")

    if borders:
        app.setStyleSheet("""*{border: 1px solid red;}""")

    result = {}

    def build_main_window():
        window = MainWindow(ros_node, selected_vehicles)
        window.resize(window_width, window_height)
        window.move(x, y)
        result['window'] = window
        QTimer.singleShot(3000, lambda: (
            window.show(),
            window.activateWindow(),
            splash.close()
        ))

    QTimer.singleShot(500, build_main_window)

    #return the app and the result
    return app, result

class CustomSplash(QWidget):
    """
    Custom splash screen widget for the application.
    Displays a splash image and an optional message label.
    Used to show branding and loading status before the main window appears.
    """
    def __init__(self, pixmap, parent=None):
        super().__init__(parent)
        # Set window flags for splash appearance
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        # Main image label
        self.label = QLabel(self)
        self.label.setPixmap(pixmap)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.resize(pixmap.size())
        # Message label for status text
        self.message_label = QLabel("", self)
        self.message_label.setAlignment(Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter)
        self.message_label.setStyleSheet("color: white; font-size: 18pt; font-weight: bold;")
        self.message_label.setGeometry(0, pixmap.height() - 60, pixmap.width(), 60)

    def showMessage(self, text):
        """
        Sets the splash message text.
        """
        self.message_label.setText(text)

class ConfirmationDialog(QDialog):
    """
    Custom dialog for confirming or aborting mission-related actions (e.g., shutdown, recall).
    Presents a message and Accept/Decline buttons.

    Parameters:
        window_title (str): The title of the dialog window.
        message_text (str): The message to display in the dialog.
        parent (QWidget, optional): The parent widget.
        background_color (str): Background color for the dialog.
        text_color (str): Text color for the dialog.
        pop_up_window_style (str): Custom stylesheet for the dialog.
    """
    def __init__(self, window_title, message_text, parent=None, background_color="white", text_color="black", pop_up_window_style=None):
        super().__init__(parent)

        self.setWindowTitle(window_title)
        self.setStyleSheet(pop_up_window_style)

        QBtn = (
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Change button labels
        ok_button = self.buttonBox.button(QDialogButtonBox.StandardButton.Ok)
        ok_button.setText("Accept")

        cancel_button = self.buttonBox.button(QDialogButtonBox.StandardButton.Cancel)
        cancel_button.setText("Decline")

        layout = QVBoxLayout()
        message = QLabel(message_text)
        layout.addWidget(message)
        layout.addWidget(self.buttonBox)
        self.setLayout(layout)

def load_origin_presets():
    """
    Load named origin presets from cougars_mapviz's mapviz_origins.yaml, so mapviz
    and this GUI share a single list of sites. Returns [] if the file or package
    can't be found (e.g. cougars_mapviz isn't installed on this machine).
    """
    try:
        mapviz_share = get_package_share_directory('cougars_mapviz')
    except Exception:
        return []

    origins_path = os.path.join(mapviz_share, 'mapviz', 'mapviz_origins.yaml')
    try:
        with open(origins_path, 'r') as f:
            entries = yaml.safe_load(f) or []
    except Exception:
        return []

    presets = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        if 'name' not in entry or 'latitude' not in entry or 'longitude' not in entry:
            continue
        presets.append({
            'name': entry['name'],
            'latitude': float(entry['latitude']),
            'longitude': float(entry['longitude']),
            'altitude': float(entry.get('altitude', 0.0)),
        })
    return presets


class HardwareControlDialog(QDialog):
    """
    Dialog for turning the DVL/modem relay and nav light strobe on/off/auto over radio.
    Each button sends its command to the vehicle immediately; the dialog stays open
    so both the relay and the strobe can be controlled independently.
    """
    MODES = [("Auto", "AUTO"), ("On", "ON"), ("Off", "OFF")]

    def __init__(self, vehicle_number, ros_node, parent=None, background_color="white", text_color="black", pop_up_window_style=None):
        super().__init__(parent)
        self.vehicle_number = vehicle_number
        self.ros_node = ros_node
        self.setWindowTitle(f"Relay / Strobe Control - Coug {vehicle_number}")
        self.setStyleSheet(pop_up_window_style)
        self.text_color = text_color
        self.background_color = background_color

        layout = QVBoxLayout()
        layout.addWidget(self._build_device_row("Relay", "RELAY"))
        layout.addWidget(self._build_device_row("Strobe", "STROBE"))

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.setLayout(layout)

    def _build_device_row(self, label_text, device_key):
        row_widget = QWidget()
        row = QHBoxLayout(row_widget)
        row.setContentsMargins(0, 0, 0, 0)

        label = QLabel(f"{label_text}:")
        label.setStyleSheet(f"color: {self.text_color};")
        row.addWidget(label)

        for mode_text, mode_key in self.MODES:
            button = QPushButton(mode_text)
            button.setStyleSheet(
                f"background-color: {self.background_color}; color: {self.text_color}; border: 1px solid {self.text_color}; padding: 4px;"
            )
            button.clicked.connect(lambda _, d=device_key, m=mode_key: self.send_command(d, m))
            row.addWidget(button)

        return row_widget

    def send_command(self, device_key, mode_key):
        self.ros_node.publish_hardware_control(self.vehicle_number, device_key, mode_key)


class OriginDialog(QDialog):
    """
    Dialog for publishing the shared WGS84 origin.
    """
    def __init__(self, origin_values, parent=None, background_color="white", text_color="black", pop_up_window_style=None):
        super().__init__(parent)
        self.setWindowTitle("Publish Origin")
        self.setStyleSheet(pop_up_window_style)

        latitude, longitude, altitude = origin_values

        layout = QGridLayout()

        self.presets = load_origin_presets()
        preset_label = QLabel("Preset:")
        preset_label.setStyleSheet(f"color: {text_color};")
        self.preset_combo = QComboBox()
        self.preset_combo.addItem("Manual")
        for preset in self.presets:
            self.preset_combo.addItem(preset['name'])
        self.preset_combo.currentIndexChanged.connect(self.apply_preset)
        layout.addWidget(preset_label, 0, 0)
        layout.addWidget(self.preset_combo, 0, 1)

        self.latitude_spin = self.create_spin_box(latitude, -90.0, 90.0, 8)
        self.longitude_spin = self.create_spin_box(longitude, -180.0, 180.0, 8)
        self.altitude_spin = self.create_spin_box(altitude, -10000.0, 10000.0, 2)

        fields = [
            ("Latitude:", self.latitude_spin),
            ("Longitude:", self.longitude_spin),
            ("Altitude:", self.altitude_spin),
        ]
        for row, (label_text, spin_box) in enumerate(fields, start=1):
            label = QLabel(label_text)
            label.setStyleSheet(f"color: {text_color};")
            layout.addWidget(label, row, 0)
            layout.addWidget(spin_box, row, 1)

        button_box = QDialogButtonBox()
        publish_button = button_box.addButton("Publish Origin", QDialogButtonBox.ButtonRole.AcceptRole)
        button_box.addButton(QDialogButtonBox.StandardButton.Cancel)
        publish_button.setStyleSheet(
            f"background-color: {background_color}; color: {text_color}; border: 1px solid {text_color}; padding: 4px;"
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box, len(fields) + 1, 0, 1, 2)
        self.setLayout(layout)

    def apply_preset(self, index):
        preset_index = index - 1
        if preset_index < 0 or preset_index >= len(self.presets):
            return
        preset = self.presets[preset_index]
        self.latitude_spin.setValue(preset['latitude'])
        self.longitude_spin.setValue(preset['longitude'])
        self.altitude_spin.setValue(preset['altitude'])

    def create_spin_box(self, value, minimum, maximum, decimals):
        spin_box = QDoubleSpinBox()
        spin_box.setRange(minimum, maximum)
        spin_box.setDecimals(decimals)
        spin_box.setSingleStep(0.000001 if decimals > 2 else 1.0)
        spin_box.setValue(float(value))
        return spin_box

    def get_origin(self):
        return (
            self.latitude_spin.value(),
            self.longitude_spin.value(),
            self.altitude_spin.value(),
        )

class LoadMissionsDialog(QDialog):
    """
    Custom dialog for loading the vehicle missions.
    Allows the user to select mission files for each vehicle (multi-tab) or a single vehicle.
    Handles file browsing, display, and validation before accepting.

    Parameters:
        parent (QWidget, optional): Parent widget.
        vehicle (int): If 0, multi-vehicle mode; otherwise, single vehicle mode.
        background_color (str): Background color for the dialog.
        text_color (str): Text color for the dialog.
        pop_up_window_style (str): Custom stylesheet for the dialog.
        selected_vehicles (list): List of selected vehicle numbers.
    """
    def __init__(self, parent=None, vehicle=0, background_color="white", text_color="black", pop_up_window_style=None, selected_vehicles=None):
        """
        Parameters:
            options (list of str): List of checkbox labels.
        """
        super().__init__(parent)
        # Set window title based on mode
        if not vehicle: self.setWindowTitle("Load All Missions")
        else: self.setWindowTitle(f"Load Vehicle{vehicle} Mission")
        self.checkboxes = {}
        layout = QVBoxLayout()

        # Make the dialog not resizable
        self.setFixedSize(300, 200)  # Set to your preferred width and height

        self.setStyleSheet(pop_up_window_style)

        self.file_display_labels = {}  # Store file display labels for each tab

        if not vehicle:
            # Multi-vehicle mode: create tabs for each vehicle
            self.selected_files = {}  # Store files per tab
            #Create the tabs
            self.pop_up_tabs = QTabWidget()
            #Orient the tabs at the tob of the screen
            self.pop_up_tabs.setTabPosition(QTabWidget.TabPosition.North)
            #The tabs' order can't be changed or moved
            self.pop_up_tabs.setMovable(False)
            tab_names = [f"Vehicle {i}" for i in selected_vehicles]
            
            # Create separate content widgets for each tab
            for name in tab_names: 
                content_widget = QWidget()
                content_layout = QVBoxLayout(content_widget)
                
                # Add file selection widgets to each tab
                file_section_label = QLabel("Select Mission File:")
                file_section_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
                content_layout.addWidget(file_section_label)

                # File display for this tab
                file_display_label = QLabel("No file selected")
                file_display_label.setWordWrap(True)
                file_display_label.setStyleSheet(f"border: 1px solid {text_color}; padding: 8px; min-height: 40px; color: {text_color}; background-color: {background_color};")
                content_layout.addWidget(file_display_label)

                # Store the file display label for this tab
                self.file_display_labels[name] = file_display_label
                
                # Browse button for this tab
                browse_button = QPushButton("Browse Files...")
                browse_button.setStyleSheet(f"background-color: {background_color}; color: {text_color}; border: 1px solid {text_color}; padding: 5px;")
                browse_button.clicked.connect(lambda checked, tab=name: self.browse_file(tab))
                content_layout.addWidget(browse_button)
                
                self.pop_up_tabs.addTab(content_widget, name)
            
            # Add the tab widget to the main layout
            layout.addWidget(self.pop_up_tabs)
            
        else:
            # Single vehicle mode: add file selection directly to layout
            self.selected_file = None  # Initialize for single vehicle mode
            file_section_label = QLabel("Select Mission File:")
            file_section_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
            layout.addWidget(file_section_label)

            # File display
            self.file_display_label = QLabel("No file selected")
            self.file_display_label.setWordWrap(True)
            self.file_display_label.setStyleSheet(f"border: 1px solid {text_color}; padding: 8px; min-height: 40px; color: {text_color}; background-color: {background_color};")
            layout.addWidget(self.file_display_label)
            
            # Browse button
            browse_button = QPushButton("Browse Files...")
            browse_button.setStyleSheet(f"background-color: {background_color}; color: {text_color}; border: 1px solid {text_color}; padding: 5px;")
            browse_button.clicked.connect(self.browse_file)
            layout.addWidget(browse_button)

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        button_row = QHBoxLayout()

        if not vehicle:
            # "Apply to All" button for multi-vehicle mode
            applyAllButton = QPushButton("Apply to All")
            button_row.addWidget(applyAllButton)
            applyAllButton.clicked.connect(
                lambda: self.apply_to_all(
                    background_color=background_color,
                    text_color=text_color,
                    pop_up_window_style=pop_up_window_style
                )
            )

        button_row.addWidget(buttonBox)
        layout.addLayout(button_row)
        self.setLayout(layout)
    
    def apply_to_all(self, background_color=None, text_color=None, pop_up_window_style=None):
        """
        Applies the currently selected file in the active tab to all vehicles/tabs.
        Shows a confirmation dialog before overwriting.
        """
        current_tab_index = self.pop_up_tabs.currentIndex()
        current_tab_name = self.pop_up_tabs.tabText(current_tab_index)
        # Check if a file is selected for the current tab
        selected_file = self.selected_files.get(current_tab_name)
        if not selected_file:
            QMessageBox.warning(self, "No File Selected", "Please select a file for the current tab before applying to all.")
            return

        # Confirm with the user
        dlg = ConfirmationDialog(
            "Apply to All?",
            "Are you sure you want to apply this file to all vehicles? This will overwrite any other files you have already selected.",
            self,
            background_color=background_color,
            text_color=text_color,
            pop_up_window_style=pop_up_window_style
        )
        if dlg.exec():
            # Apply the selected file to all tabs (even if not previously selected)
            for tab_name in self.file_display_labels:
                self.selected_files[tab_name] = selected_file
                self.update_file_display(tab_name)

    def browse_file(self, tab_name=None):
        """
        Opens a file dialog to select a mission file for the current tab or single vehicle.
        Updates the display label with the selected file name.
        """
        # Set default directory - you can customize this path
        default_dir = os.path.expanduser("/home/frostlab/base_station/mission_control/missions")
        
        if tab_name:
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                f"Select Mission File for {tab_name}",
                default_dir,
                "Mission Files (*.yaml *.yml *.json);;All Files (*)"
            )
            if file_path:
                self.selected_files[tab_name] = file_path
                self.update_file_display(tab_name)
        else:
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select Mission File",
                default_dir,
                "Mission Files (*.yaml *.yml *.json);;All Files (*)"
            )
            if file_path:
                self.selected_file = file_path
                self.update_file_display()

    def update_file_display(self, tab_name=None):
        """
        Updates the file display label for the selected file in the current tab or single vehicle.
        """
        if tab_name:
            if tab_name in self.selected_files:
                file_name = os.path.basename(self.selected_files[tab_name])
                self.file_display_labels[tab_name].setText(file_name)
            else:
                self.file_display_labels[tab_name].setText("No file selected")
        else:
            if hasattr(self, 'file_display_label') and hasattr(self, 'selected_file'):
                if self.selected_file:
                    file_name = os.path.basename(self.selected_file)
                    self.file_display_label.setText(file_name)
                else:
                    self.file_display_label.setText("No file selected")

    def validate_and_accept(self):
        """
        Validates that all tabs (multi-vehicle) or the single vehicle have a selected file before accepting.
        Shows a warning if any are missing.
        """
        if hasattr(self, 'selected_files'):
            if len(self.selected_files) == len(self.file_display_labels) and all(self.selected_files.values()):
                self.accept()
            else:
                from PyQt6.QtWidgets import QMessageBox
                QMessageBox.warning(self, "Files Required", "Please select a mission file for each Vehicle before continuing.")
        elif hasattr(self, 'selected_file') and self.selected_file:
            self.accept()
        else:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "File Required", "Please select a mission file before continuing.")

    def get_states(self):
        """
        Returns a dict containing the selected file path(s).
        For multi-vehicle: {"selected_files": {tab_name: file_path, ...}}
        For single vehicle: {"selected_file": file_path}
        """
        if hasattr(self, 'selected_files'):
            return {"selected_files": self.selected_files}
        else:
            return {"selected_file": getattr(self, 'selected_file', None)}

class StartMissionsDialog(QDialog):
    """
    Custom dialog for starting the vehicle missions.
    Presents a window with configuration options (checkboxes and text inputs).
    Handles validation of options before accepting.

    Parameters:
        options (list of str): List of option labels.
        parent (QWidget, optional): Parent widget.
        passed_option_map (dict): Mapping from option label to internal key.
        vehicle (int): If 0, multi-vehicle mode; otherwise, single vehicle mode.
        background_color (str): Background color for the dialog.
        text_color (str): Text color for the dialog.
        pop_up_window_style (str): Custom stylesheet for the dialog.
    """
    def __init__(self, options, parent=None, passed_option_map=None, vehicle=0, background_color="white", text_color="black", pop_up_window_style=None):
        """
        Parameters:
            options (list of str): List of checkbox labels.
        """
        super().__init__(parent)
        if not vehicle: self.setWindowTitle("Start All Missions Configuration")
        else: self.setWindowTitle(f"Start Vehicle{vehicle} Mission Configuration")
        self.checkboxes = {}
        self.passed_option_map = passed_option_map
        layout = QVBoxLayout()

        # Make the dialog not resizable
        self.setFixedSize(300, 200)  # Set to your preferred width and height

        self.setStyleSheet(pop_up_window_style)

        self.inputs = {}

        for opt in options:
            if "rosbag prefix" in opt.lower():
                le = QLineEdit()
                le.setPlaceholderText("Enter Rosbag Prefix...")
                self.inputs[opt] = le
                layout.addWidget(le)
            else:
                cb = QCheckBox(opt)
                cb.setChecked(False)
                self.checkboxes[opt] = cb
                layout.addWidget(cb)

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        layout.addWidget(buttonBox)
        self.setLayout(layout)
    
    def validate_and_accept(self):
        """
        Validates that if 'Record rosbag' is checked, a prefix is provided, and vice versa.
        Shows warnings if validation fails.
        """
        states = self.get_states()
        #if record rosbag was chosen, a prefix must be given, as well as the opposite
        if states["record_rosbag"] and not states["rosbag_prefix"]: 
            QMessageBox.warning(self, "Prefix Required to Record Rosbag", "Please enter a rosbag prefix before continuing")
        elif not states["record_rosbag"] and states["rosbag_prefix"]: 
            QMessageBox.warning(self, "Record Rosbag Required to have Prefix", "Please select record rosbag before continuing")
        else:
            self.accept()
            return

    def get_states(self):
        """
        Returns a dict of {easy_key: value} for each option.
        """
        result = {}
        for opt, cb in self.checkboxes.items():
            result[self.passed_option_map[opt]] = cb.isChecked()
        for opt, le in self.inputs.items():
            result[self.passed_option_map[opt]] = le.text()
        return result

class ConfigurationWindow(QDialog):
    """
    Custom configuration dialog for selecting vehicles.
    Allows selection of up to 4 vehicles via checkboxes and custom number inputs.
    Handles validation for duplicates, valid numbers, and maximum count.

    Parameters:
        options (list of str): List of checkbox labels.
        parent (QWidget, optional): Parent widget.
        background_color (str): Background color for the dialog.
        text_color (str): Text color for the dialog.
    """
    def __init__(self, options, parent=None, background_color="white", text_color="black"):
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.checkboxes = {}
        self.custom_inputs = []
        self.custom_plus_buttons = []
        layout = QVBoxLayout()
        self.background_color = background_color
        self.text_color = text_color
        self.MAX_VEHICLES = 4
        self.HIGHEST_VEHICLE_LABEL = 999

        self.setMinimumWidth(300)
        self.resize(300, 200)
        self.setStyleSheet(f"""
            QDialog {{
                background-color: {self.background_color};
                color: {self.text_color};
            }}
            QLabel, QCheckBox {{
                color: {self.text_color};
            }}
            QCheckBox::indicator {{
                width: 13px;
                height: 13px;
            }}
            QCheckBox::indicator:checked {{
                border: 1px solid {self.text_color};
            }}
            QCheckBox::indicator:unchecked {{
                background-color: {self.text_color};
                border: 1px solid {self.text_color};
            }}
            QLineEdit {{
                background-color: {self.background_color};
                color: {self.text_color};
                border: 1px solid {self.text_color};
                padding: 2px;
            }}
        """)

        self.inputs = []

        # Create a checkbox for each option
        for opt in options:
            if "select custom:" not in opt.lower():
                cb = QCheckBox(opt)
                cb.setChecked(False)
                self.checkboxes[opt] = cb
                layout.addWidget(cb)

        # Container for custom Vehicle inputs
        self.custom_container = QVBoxLayout()
        layout.addLayout(self.custom_container)

        # Add the first "+" button
        self.add_custom_plus_button()

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        layout.addWidget(buttonBox)
        self.setLayout(layout)

    def add_custom_plus_button(self):
        """
        Adds a "+" button to allow the user to add a custom vehicle number input.
        """
        custom_input_style = f"background-color: {self.background_color}; color: {self.text_color}; border: 2px solid {self.text_color};"
        plus_btn = QPushButton("+ Add Custom Vehicle")
        plus_btn.setStyleSheet(custom_input_style)
        plus_btn.clicked.connect(lambda: self.add_custom_input(plus_btn))
        self.custom_container.addWidget(plus_btn)
        self.custom_plus_buttons.append(plus_btn)

    def add_custom_input(self, plus_btn):
        """
        Adds a new QLineEdit for custom vehicle number input, up to MAX_VEHICLES.
        Removes the plus button that was clicked and adds a new one below.
        """
        current_count = sum(cb.isChecked() for cb in self.checkboxes.values())
        current_count += len(self.custom_inputs)
        if current_count >= self.MAX_VEHICLES:
            QMessageBox.warning(self, "Limit Reached", f"You cannot add more than {self.MAX_VEHICLES} Vehicles.")
            return
        # Remove the plus button that was clicked
        self.custom_container.removeWidget(plus_btn)
        plus_btn.hide()
        # Add a new QLineEdit for custom input
        le = QLineEdit()
        le.setPlaceholderText("Enter Custom Number...")
        self.custom_inputs.append(le)
        self.custom_container.addWidget(le)
        # Add a new plus button below this input
        self.add_custom_plus_button()
        # Adjust the dialog size to fit new content
        self.adjustSize()
        
    def validate_and_accept(self):
        """
        Validates that at least one vehicle is selected, no duplicates, and all numbers are valid.
        Shows warnings if validation fails.
        """
        states = self.get_states()
        valid_custom = True
        valid_vehicle_number = True
        if not states:
            QMessageBox.warning(self, "Selection Required", "Please select at least one Vehicle before continuing.")
        elif len(states) > self.MAX_VEHICLES:
            QMessageBox.warning(self, "Max Vehicle Limit Reached", "Selection of more than 4 Vehicles not allowed")
        else:
            for value in states:
                try:
                    num = int(value)
                    if num > self.HIGHEST_VEHICLE_LABEL or num < 0: valid_vehicle_number = False
                except:
                    valid_custom = False
            if len(states) != len(set(states)):
                QMessageBox.warning(self, "Duplicate Vehicles", "Please ensure all Vehicle numbers are unique.")
            elif not valid_custom:
                QMessageBox.warning(self, "Invalid Custom", "Please enter a valid integer for custom Vehicle number.")
            elif not valid_vehicle_number:
                QMessageBox.warning(self, "Invalid Vehicle Number", "Please enter an integer from 0-999.")
            else:
                self.accept()

    def get_states(self):
        """
        Returns a list of selected vehicle numbers (from checkboxes and custom inputs).
        """
        selected_vehicles = []
        for opt, cb in self.checkboxes.items():
            if cb.isChecked():
                try:
                    num = int(opt.split()[-1])
                    selected_vehicles.append(num)
                except Exception:
                    pass
        for le in self.custom_inputs:
            value = le.text().strip()
            if value:
                try:
                    selected_vehicles.append(int(value))
                except ValueError:
                    selected_vehicles.append(value)
        return selected_vehicles

class CalibrateFinsDialog(QDialog): 
    """
    Custom dialog for fin calibration.
    Allows the user to adjust fin offsets for each vehicle using sliders.
    Supports publishing changes to ROS and saving to params.
    Tabs for each vehicle, exclusive checkboxes for pub type, and value display.


    Parameters:
        parent (QWidget, optional): Parent widget.
        background_color (str): Background color for the dialog.
        text_color (str): Text color for the dialog.
        pop_up_window_style (str): Custom stylesheet for the dialog.
        selected_vehicles (list): List of selected vehicle numbers.
        passed_ros_node: ROS node for publishing.
        on_slider_change (callable): Callback for slider value changes.
        vehicle_init_params (dict): Initial fin values for each vehicle.
    """

    #template vehicle_init_params
    # {1: {'top_fin_offset': -10.0, 'right_fin_offset': 0.0, 'left_fin_offset': -0.0, }, 
    # 2: {'top_fin_offset': -10.0, 'right_fin_offset': 0.0, 'left_fin_offset': -0.0, }, 
    # 3: {'top_fin_offset': -10.0, 'right_fin_offset': 0.0, 'left_fin_offset': -0.0 }}

    def __init__(self, parent=None, background_color="white", text_color="black", pop_up_window_style=None, selected_vehicles=None, passed_ros_node=None, on_slider_change=None, vehicle_init_params=None):
        super().__init__(parent)
        self.setWindowTitle("Fin Calibration:")
        self.pub_types = {}
        self.fin_sliders = {}
        self.fin_dict = {
            1: "top_fin_offset",
            2: "right_fin_offset",
            3: "left_fin_offset"
        }
        self.fin_dict_to_label = {
            "top_fin_offset": "Top Fin",
            "right_fin_offset": "Right Fin",
            "left_fin_offset": "Left Fin"
        }
        self.on_slider_change = on_slider_change  # <-- store callback
        layout = QVBoxLayout()
        self.setFixedSize(300, 300)
        self.setStyleSheet(pop_up_window_style)
        self.pop_up_tabs = QTabWidget()
        self.pop_up_tabs.setTabPosition(QTabWidget.TabPosition.North)
        self.pop_up_tabs.setMovable(False)
        tab_names = [f"Vehicle {i}" for i in selected_vehicles]

        for idx, name in enumerate(tab_names):
            vehicle_num = selected_vehicles[idx]
            content_widget = QWidget()
            content_widget.setStyleSheet(f"background-color: {background_color};")
            content_layout = QVBoxLayout(content_widget)
            self.fin_sliders[name] = []
            for i in range(1, 4):
                row = QHBoxLayout()
                fin_label = QLabel(f"{self.fin_dict_to_label[self.fin_dict[i]]}: ")
                fin_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
                row.addWidget(fin_label)
                fin_slider = QSlider(Qt.Orientation.Horizontal)
                fin_slider.setMinimum(-180)
                fin_slider.setMaximum(180)
                
                if not vehicle_init_params: fin_slider.setValue(0)
                else: fin_slider.setValue(int(vehicle_init_params[vehicle_num][self.fin_dict[i]]))

                fin_slider.setTickInterval(1)
                # moves one tick with the arrows
                fin_slider.setSingleStep(1)
                # moves one tick with the page up/down buttons
                fin_slider.setPageStep(5)
                fin_slider.setStyleSheet(f"color: {text_color};")
                row.addWidget(fin_slider)
                value_label = QLabel(str(fin_slider.value()))
                value_label.setStyleSheet(f"color: {text_color};")
                fin_slider.valueChanged.connect(lambda val, lbl=value_label: lbl.setText(str(val)))

                if self.on_slider_change:
                    fin_slider.valueChanged.connect(
                        lambda _, vnum=vehicle_num, tab=name: self._handle_slider_change(vnum, tab, self.pub_types[vehicle_num])
                    )
                row.addWidget(value_label)
                content_layout.addLayout(row)
                self.fin_sliders[name].append(fin_slider)

            self.pub_types[vehicle_num] = 1
            # pub_type = 1
            cb = QCheckBox(f"coug{vehicle_num}/kinematics/command")
            cb.setChecked(True)
            # pub_type = 0
            cb2 = QCheckBox(f"coug{vehicle_num}/controls/command")
            cb2.setChecked(False)

            self.make_exclusive(cb, cb2)

            cb.stateChanged.connect(lambda state, vnum=vehicle_num: self.set_pub_type(vnum, 1) if state else None)
            cb2.stateChanged.connect(lambda state, vnum=vehicle_num: self.set_pub_type(vnum, 0) if state else None)

            content_layout.addWidget(cb)
            content_layout.addWidget(cb2)
            self.pop_up_tabs.addTab(content_widget, name)

        note_label = QLabel("(Arrows -> 1, Pg Up/Down -> 5)")
        note_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
        layout.addWidget(note_label)
        layout.addWidget(self.pop_up_tabs)
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        ok_button = buttonBox.button(QDialogButtonBox.StandardButton.Ok)
        ok_button.setText("Save Changes To Params")
        buttonBox.accepted.connect(self.validate_and_accept)
        button_row = QHBoxLayout()
        button_row.addWidget(buttonBox)
        layout.addLayout(button_row)
        self.setLayout(layout)

    def make_exclusive(self, box1, box2):
        """
        Ensures that only one of the two checkboxes is checked at a time (exclusive selection).
        """
        box1.stateChanged.connect(lambda state: box2.setChecked(False) if state else box2.setChecked(True))
        box2.stateChanged.connect(lambda state: box1.setChecked(False) if state else box1.setChecked(True))

    def set_pub_type(self, vehicle_num, value):
        """
        Sets the pub_type value for the given vehicle number.
        """
        self.pub_types[vehicle_num] = value

    def _handle_slider_change(self, vehicle_num, tab_name, pub_type):
        """
        Called whenever a slider changes for a vehicle.
        Invokes the on_slider_change callback with current values.
        """
        if self.on_slider_change:
            # Get current values for this vehicle
            values = [slider.value() for slider in self.fin_sliders[tab_name]]
            self.on_slider_change(vehicle_num, values, pub_type)

    def validate_and_accept(self): 
        """
        Accepts the dialog (no additional validation).
        """
        self.accept()

    def get_states(self):
        """
        Returns a dict mapping each vehicle/tab name to a list of its fin slider values.
        Example: {'Vehicle 1': [val1, val2, val3], ...}
        """
        states = {}
        for tab_name, sliders in self.fin_sliders.items():
            states[tab_name[-1]] = [slider.value() for slider in sliders]
        return states

class CalibrateFinsWorker(QThread):
    """
    Worker thread for loading vehicle kinematics parameters for fin calibration.
    Loads parameters for each selected vehicle, creates new param files if needed,
    and emits a signal with the results.

    Parameters:
        selected_vehicles (list): List of vehicle numbers.
        load_vehicle_kinematics_params (callable): Function to load params.
        create_new_param_file (callable): Function to create new param file.
    """
    finished = pyqtSignal(dict, dict, list, list)
    def __init__(self, selected_vehicles, load_vehicle_kinematics_params, create_new_param_file):
        super().__init__()
        self.selected_vehicles = selected_vehicles
        self.load_vehicle_kinematics_params = load_vehicle_kinematics_params
        self.create_new_param_file = create_new_param_file

    def run(self):
        """
        Loads parameters for each vehicle, creates new param files if missing,
        and emits the finished signal with results.
        """
        vehicle_params_dict = {}
        params_found_dict = {}
        base_params_problems = []
        vehicle_params_problems = []
        for i in self.selected_vehicles:
            vehicle_params, base_params = self.load_vehicle_kinematics_params(i)
            params_found_dict[i] = (vehicle_params, base_params)
            if vehicle_params is not None: 
                vehicle_params_dict[i] = vehicle_params
                if base_params is None: self.create_new_param_file(i)
            elif base_params is not None: vehicle_params_dict[i] = base_params
            else: 
                self.create_new_param_file(i)
                vehicle_params, base_params = self.load_vehicle_kinematics_params(i)
                if base_params: vehicle_params_dict[i] = base_params

        for vehicle_id, params in params_found_dict.items():
            if params[0] is None:
                vehicle_params_problems.append(vehicle_id)
            if params[1] is None:
                base_params_problems.append(vehicle_id)
        self.finished.emit(vehicle_params_dict, params_found_dict, base_params_problems, vehicle_params_problems)

class LoadingDialog(QDialog):
    """
    Simple modal dialog for displaying a loading message.
    Used to indicate that a background operation is in progress.

    Parameters:
        message (str): The message to display.
        parent (QWidget, optional): Parent widget.
        background_color (str): Background color for the dialog.
        text_color (str): Text color for the dialog.
    """
    def __init__(self, message="Loading, please wait...", parent=None, background_color="#222", text_color="#fff"):
        super().__init__(parent)
        self.setWindowTitle("Please Wait")
        self.setModal(True)
        self.setFixedSize(250, 100)
        self.setStyleSheet(f"background-color: {background_color}; color: {text_color};")
        layout = QVBoxLayout()
        label = QLabel(message)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet(f"color: {text_color}; font-size: 12pt;")
        layout.addWidget(label)
        self.setLayout(layout)
