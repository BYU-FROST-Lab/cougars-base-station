#!/usr/bin/env python3

import os
import re
import sys
import yaml
import threading
import json
import time
from pathlib import Path
import paramiko
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node
global ros_node

# SSH configuration
SSH_KEY_PATH = str(Path.home()) + "/.ssh/id_ed25519_cougs"
DEPLOY_CONFIG_PATH = str(Path.home()) + "/base_station/mission_control/deploy_config.json"

#Confirmed: This exists
VEHICLE_PARAMS_PATH = os.path.expanduser("~/config/vehicle_params.yaml")
#Confirmed: This exists
PYTHON_SCRIPT = os.path.expanduser("~/ros2_ws/update_yaml.py")
#Confirmed: This exists
DVL_DIR = os.path.expanduser("~/ros2_ws/dvl_tools")
NODE_NAME = "depth_convertor"
ROS_PARAM_NAME = "fluid_pressure_atm"

def get_vehicle_config(vehicle_number):
    """Get vehicle configuration from deploy config file."""
    try:
        with open(DEPLOY_CONFIG_PATH, "r") as f:
            config = json.load(f)
        vehicles = config["vehicles"]
        vehicle_info = vehicles.get(str(vehicle_number))
        if vehicle_info:
            return vehicle_info
        else:
            ros_node.publish_console_log(f"[ERROR] Vehicle {vehicle_number} not found in config", vehicle_number)
            return None
    except Exception as e:
        ros_node.publish_console_log(f"[ERROR] Failed to load config: {e}", vehicle_number)
        return None

def create_ssh_connection(vehicle_number):
    """Create SSH connection to vehicle using key authentication."""
    vehicle_config = get_vehicle_config(vehicle_number)
    if not vehicle_config:
        return None
        
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        private_key = paramiko.Ed25519Key.from_private_key_file(SSH_KEY_PATH)

        ssh.connect(
            hostname=vehicle_config["remote_host"],
            username=vehicle_config["remote_user"],
            pkey=private_key,
            timeout=10
        )
        
        return ssh
    except Exception as e:
        ros_node.publish_console_log(f"[ERROR] Failed to connect to vehicle {vehicle_number}: {e}", vehicle_number)
        return None

def run_service_call(service_name, srv_type, args, vehicle):
    """Run ROS2 service call on remote vehicle via SSH."""
    ssh = create_ssh_connection(vehicle)
    if not ssh:
        return
        
    try:
        command = f"ros2 service call {service_name} {srv_type} '{args}'"
        stdin, stdout, stderr = ssh.exec_command(command, timeout=10)
        
        output = stdout.read().decode() + stderr.read().decode()
        
        if "success=True" in output:
            match = re.search(r'message=\s*(.*)', output)
            ros_node.publish_console_log(f"[INFO] Service call to {service_name} succeeded. {match.group(1) if match else ''}", vehicle)
        elif "success=False" in output:
            match = re.search(r'message=\s*(.*)', output)
            ros_node.publish_console_log(f"[WARNING] Service call to {service_name} failed with message: {match.group(1) if match else ''}", vehicle)
        else:
            ros_node.publish_console_log(f"[ERROR] Unexpected response from service {service_name}: {output}", vehicle)
            
    except Exception as e:
        ros_node.publish_console_log(f"[ERROR] Service call to {service_name} failed: {e}", vehicle)
    finally:
        ssh.close()

def run_script(script_path, *args, vehicle):
    """Run script on remote vehicle via SSH with real-time output."""
    ssh = create_ssh_connection(vehicle)
    if not ssh:
        return
        
    try:
        # Build command with arguments
        command = f"bash {script_path}"
        if args:
            command += " " + " ".join(str(arg) for arg in args)
            
        ros_node.publish_console_log(f"[INFO] Executing: {command}", vehicle)
        
        # Execute command with real-time output
        transport = ssh.get_transport()
        channel = transport.open_session()
        channel.settimeout(30)  # 30 second timeout
        
        channel.exec_command(command)
        
        # ANSI color code pattern
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        
        def read_output():
            try:
                while True:
                    if channel.recv_ready():
                        data = channel.recv(1024).decode('utf-8')
                        if not data:
                            break
                        
                        # Process line by line
                        for line in data.splitlines():
                            line = line.rstrip()
                            if line:
                                # Skip curl progress lines
                                if any(skip_text in line for skip_text in [
                                    '% Total', '% Received', '% Xferd', 'Dload', 'Upload', 
                                    '--:--:--', 'Current', 'Speed', 'Time'
                                ]):
                                    continue
                                
                                clean_line = ansi_escape.sub('', line)
                                ros_node.publish_console_log(clean_line, vehicle)
                    
                    if channel.exit_status_ready():
                        break
                        
                    time.sleep(0.1)  # Small delay to prevent busy waiting
                        
            except Exception as e:
                ros_node.publish_console_log(f"[ERROR] Error reading output: {e}", vehicle)
        
        # Start reading output in a thread
        output_thread = threading.Thread(target=read_output, daemon=True)
        output_thread.start()
        
        # Wait for command to complete
        exit_status = channel.recv_exit_status()
        
        # Wait for output thread to finish
        output_thread.join(timeout=2)
        
        if exit_status != 0:
            ros_node.publish_console_log(f"[ERROR] Script {script_path} failed with exit code {exit_status}", vehicle)
        else:
            ros_node.publish_console_log(f"[INFO] Script {script_path} completed successfully", vehicle)
            
        channel.close()
        
    except Exception as e:
        ros_node.publish_console_log(f"[ERROR] Failed to run script {script_path}: {e}", vehicle)
    finally:
        ssh.close()
             
def get_ros_param(namespace, vehicle):
    """Get ROS parameter from remote vehicle via SSH."""
    ssh = create_ssh_connection(vehicle)
    if not ssh:
        return None
        
    full_node = f"/{namespace}/{NODE_NAME}".replace("//", "/")
    
    try:
        command = f"ros2 param get {full_node} {ROS_PARAM_NAME}"
        stdin, stdout, stderr = ssh.exec_command(command, timeout=10)
        
        output = stdout.read().decode().strip()
        error_output = stderr.read().decode().strip()
        
        if error_output:
            ros_node.publish_console_log(f"[ERROR] Error getting parameter: {error_output}", vehicle)
            return None
            
        match = re.search(r"[-+]?\d*\.\d+|\d+", output)
        if match:
            return match.group()
        elif "Parameter not set" in output:
            ros_node.publish_console_log(f"[ERROR] Parameter '{ROS_PARAM_NAME}' is not set on node '{NODE_NAME}'.", vehicle)
        else:
            ros_node.publish_console_log(f"[ERROR] Failed to retrieve parameter: {output}", vehicle)
            
    except Exception as e:
        ros_node.publish_console_log(f"[ERROR] Exception while getting ROS param: {e}", vehicle)
    finally:
        ssh.close()
        
    return None

def update_yaml_param(file_path, param_name, value, node_name, namespace, vehicle):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    if namespace not in data:
        data[namespace] = {}
    if node_name not in data[namespace]:
        data[namespace][node_name] = {}
    
    data[namespace][node_name][param_name] = float(value)

    with open(file_path, 'w') as f:
        yaml.dump(data, f)

    ros_node.publish_console_log(f"[INFO] Updated {file_path} with {param_name} = {value} under {namespace}/{node_name}", vehicle)

# ----------------------------
# MAIN SCRIPT LOGIC
# ----------------------------
def main(passed_ros_node, vehicle_numbers):
    global ros_node
    ros_node = passed_ros_node

    for vehicle in vehicle_numbers:
        namespace = f"coug{vehicle}"

        ros_node.publish_console_log(f"Running run_service_call...", vehicle)
        #run calibrate depth service
        run_service_call(f"{namespace}/calibrate_depth", "std_srvs/srv/Trigger", "{}", vehicle=vehicle)

        ros_node.publish_console_log(f"Running calibrate_gyro.sh...", vehicle)
        #run calibrate gyro script (remote DVL directory)
        remote_dvl_dir = "~/ros2_ws/dvl_tools"
        run_script(f"{remote_dvl_dir}/calibrate_gyro.sh", vehicle=vehicle)

        ros_node.publish_console_log(f"Running set_ntp...", vehicle)
        # run ntp script (remote DVL directory) 
        run_script(f"{remote_dvl_dir}/set_ntp.sh", vehicle=vehicle)

        ros_node.publish_console_log(f"Running get_ros_param...", vehicle)
        # get fluid_pressure_atm param value
        param_value = get_ros_param(namespace, vehicle=vehicle)
        if param_value:
            update_yaml_param(VEHICLE_PARAMS_PATH, ROS_PARAM_NAME, param_value, NODE_NAME, namespace, vehicle=vehicle)
        ros_node.publish_console_log(f"Calibrate.py finished", vehicle)

if __name__ == "__main__":
    main()
