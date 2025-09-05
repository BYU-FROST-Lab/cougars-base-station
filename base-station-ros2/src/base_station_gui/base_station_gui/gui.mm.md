# tabbed_window.py Feature-Centered Mind Map

## General Page

### Buttons

#### Load All Missions (map viz)
- **MainWindow.load_missions_button**
- **LoadMissionsDialog**
  - file browsing: `browse_file`, `update_file_display`, `apply_to_all`, `validate_and_accept`, `get_states`
- **deploy_in_thread** (threaded deploy logic)
- **ROS publish**: `ros_node.publish_origin`, `ros_node.publish_path`

#### Start All Missions
- **MainWindow.start_missions_button**
- **StartMissionsDialog**
  - options: start node, record rosbag, rosbag prefix, arm thruster, start DVL
  - validation: `validate_and_accept`, `get_states`
- **startup_call.publish_system_control**

#### Plot Waypoints
- **MainWindow.load_waypoint_button**
- **WaypointPlannerApp** (external, launched via multiprocessing)
- **origin publishing**: `ros_node.publish_origin`

#### Copy Bags to Base Station
- **MainWindow.copy_bags**
- **MainWindow.run_sync_bags**
- **MainWindow.spec_copy_bags** (per vehicle)

#### Calibrate All Vehicles
- **MainWindow.run_calibrate_script**
- **MainWindow.run_calibrate_script_threaded**
- **calibrate.main** (external)

#### Calibrate Fins
- **MainWindow.calibrate_fins**
- **CalibrateFinsWorker** (threaded param loading)
- **CalibrateFinsDialog**
  - sliders: `make_exclusive`, `set_pub_type`, `_handle_slider_change`, `validate_and_accept`, `get_states`
- **MainWindow.save_param_file**
- **MainWindow.load_vehicle_kinematics_params**
- **MainWindow.create_new_param_file**

#### Recall Vehicles (no signal)
- **MainWindow.recall_vehicles**
- **ConfirmationDialog**

### Icons/Data

#### Confirm/Reject Label
- **MainWindow.replace_confirm_reject_label**
- **self.confirm_reject_labels** (dict)

#### Connections
- **Wifi**
  - `feedback_dict["Wifi"]`
  - **MainWindow.ping_vehicles_via_wifi**
  - **MainWindow.update_wifi_widgets**
  - **MainWindow.replace_general_page_icon_widget**
  - **MainWindow.replace_specific_icon_widget**
- **Radio**
  - `feedback_dict["Radio"]`
  - **MainWindow._update_connections_gui**
  - **MainWindow.replace_general_page_icon_widget**
  - **MainWindow.replace_specific_icon_widget**
- **Modem**
  - `feedback_dict["Modem"]`
  - **MainWindow._update_connections_gui**
  - **MainWindow.replace_general_page_icon_widget**
  - **MainWindow.replace_specific_icon_widget**
  - **MainWindow.modem_shut_off_service**

#### Sensors
- **DVL**
  - `feedback_dict["DVL"]`
  - **MainWindow._update_safety_status_information**
  - **MainWindow.replace_general_page_icon_widget**
  - **MainWindow.replace_specific_icon_widget**
- **GPS**
  - `feedback_dict["GPS"]`
  - **MainWindow._update_safety_status_information**
  - **MainWindow.replace_general_page_icon_widget**
  - **MainWindow.replace_specific_icon_widget**
- **IMU**
  - `feedback_dict["IMU"]`
  - **MainWindow._update_safety_status_information**
  - **MainWindow.replace_general_page_icon_widget**
  - **MainWindow.replace_specific_icon_widget**

#### Emergency Status
- **Status_messages**
  - `feedback_dict["Status_messages"]`
  - **MainWindow._update_safety_status_information**
  - **MainWindow.get_status_label**

---

## Specific Page(s)

### Confirm/Reject Label
- **MainWindow.replace_confirm_reject_label**
- **self.confirm_reject_labels**

### Buttons

#### Load Mission (no map viz data)
- **MainWindow.spec_load_missions_button**
- **LoadMissionsDialog**

#### Start Mission
- **MainWindow.spec_start_missions_button**
- **StartMissionsDialog**

#### Copy Bag to Base Station
- **MainWindow.spec_copy_bags**
- **MainWindow.run_sync_bags**

#### Calibrate Vehicle
- **MainWindow.run_calibrate_script**
- **MainWindow.run_calibrate_script_threaded**

#### Emergency Surface
- **MainWindow.emergency_surface_button**
- **ConfirmationDialog**

#### Recall Vehicle
- **MainWindow.recall_spec_vehicle**
- **ConfirmationDialog**

#### Emergency Shutdown
- **MainWindow.emergency_shutdown_button**
- **ConfirmationDialog**

#### Clear Console
- **MainWindow.clear_console**
- **MainWindow._update_console_gui**

### Message Log
- **MainWindow.create_specific_vehicle_console_log**
- **MainWindow.recieve_console_update**
- **MainWindow._update_console_gui**

### Icons/Data

#### Status
- **X**
  - `feedback_dict["XPos"]`
  - **MainWindow._update_gui_smoothed_output**
  - **MainWindow.replace_specific_status_widget**
- **Y**
  - `feedback_dict["YPos"]`
  - **MainWindow._update_gui_smoothed_output**
  - **MainWindow.replace_specific_status_widget**
- **Depth**
  - `feedback_dict["Depth"]`
  - **MainWindow.update_depth_data**
  - **MainWindow.replace_specific_status_widget**
- **Heading**
  - `feedback_dict["Heading"]`
  - **MainWindow._update_gui_smoothed_output**
  - **MainWindow.replace_specific_status_widget**
- **Current Waypoint**
  - `feedback_dict["Waypoint"]`
  - (Set via mission loading, not directly updated in code)
- **DVL Velocity**
  - `feedback_dict["DVL_vel"]`
  - **MainWindow._update_gui_smoothed_output**
  - **MainWindow.replace_specific_status_widget**
- **Angular Velocity**
  - `feedback_dict["Angular_vel"]`
  - **MainWindow._update_gui_smoothed_output**
  - **MainWindow.replace_specific_status_widget**
- **Battery**
  - `feedback_dict["Battery"]`
  - **MainWindow.update_battery_data**
  - **MainWindow.replace_specific_status_widget**
- **Pressure**
  - `feedback_dict["Pressure"]`
  - **MainWindow.update_pressure_data**
  - **MainWindow.replace_specific_status_widget**

### Seconds Since Last Connected
- **Radio**
  - `feedback_dict["Radio_seconds"]`
  - **MainWindow._update_connections_gui**
  - **MainWindow.insert_label**
- **Acoustics**
  - `feedback_dict["Modem_seconds"]`
  - **MainWindow._update_connections_gui**
  - **MainWindow.insert_label**

---