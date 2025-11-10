#!/usr/bin/env python3

import os
import re
import sys
import yaml
import threading
import json
import time
import subprocess
from pathlib import Path
import paramiko
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node
global ros_node

# SSH configuration
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
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Vehicle {vehicle_number} not found in config", vehicle_number=vehicle_number))
            return None
    except Exception as e:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Failed to load config: {e}", vehicle_number=vehicle_number))
        return None

def get_ssh_connection(ip_address, remote_user, vehicle_number):
    """Get SSH connection using the same method as deploy.py"""
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(ip_address, username=remote_user)
        return ssh
    except (paramiko.AuthenticationException, paramiko.SSHException):
        # Run ssh-copy-id to add the key
        ros_node.console_publisher.publish(ConsoleLog(message=f"SSH Authentication failed for {ip_address}. Attempting to copy SSH key. Enter password in the terminal", vehicle_number=0))
        try:
            if ensure_ssh_key():
                subprocess.run(["ssh-copy-id", f"{remote_user}@{ip_address}"], check=True)
                ros_node.console_publisher.publish(ConsoleLog(message=f"SSH key copied successfully to {ip_address}.", vehicle_number=0))
                return get_ssh_connection(ip_address, remote_user, vehicle_number)
        except subprocess.CalledProcessError as e:
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Failed to copy SSH key to {ip_address}: {e}", vehicle_number=0))
    except Exception as e:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Failed to connect to {ip_address}: {e}", vehicle_number=0))
        return None

def ensure_ssh_key(key_path="~/.ssh/id_rsa"):
    """Ensure SSH key exists, same as deploy.py"""
    key_path = os.path.expanduser(key_path)
    pub_key_path = key_path + ".pub"
    # Check if both private and public key exist
    if os.path.exists(key_path) and os.path.exists(pub_key_path):
        return True  # SSH key exists
    else:
        # Generate a new SSH key with ssh-keygen
        subprocess.run(["ssh-keygen", "-t", "rsa", "-b", "4096", "-f", key_path, "-N", ""], check=True)
        return os.path.exists(key_path) and os.path.exists(pub_key_path)

def create_ssh_connection(vehicle_number):
    """Create SSH connection to vehicle using the same method as deploy.py"""
    ros_node.console_publisher.publish(ConsoleLog(message=f"🔍 Getting vehicle config for vehicle {vehicle_number}...", vehicle_number=vehicle_number))
    vehicle_config = get_vehicle_config(vehicle_number)
    if not vehicle_config:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ No vehicle config found for vehicle {vehicle_number}", vehicle_number=vehicle_number))
        return None
        
    ros_node.console_publisher.publish(ConsoleLog(message=f"🔗 Connecting to {vehicle_config['remote_host']} as {vehicle_config['remote_user']}...", vehicle_number=vehicle_number))
    return get_ssh_connection(
        vehicle_config["remote_host"],
        vehicle_config["remote_user"],
        vehicle_number
    )

def run_service_call(service_name, srv_type, args, vehicle):
    """Run ROS2 service call on remote vehicle via SSH."""
    ros_node.console_publisher.publish(ConsoleLog(message=f"🔗 Attempting SSH connection to vehicle {vehicle}...", vehicle_number=vehicle))
    ssh = create_ssh_connection(vehicle)
    if not ssh:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Failed to establish SSH connection to vehicle {vehicle}", vehicle_number=vehicle))
        return
        
    try:
        command = f"ros2 service call {service_name} {srv_type} '{args}'"
        ros_node.console_publisher.publish(ConsoleLog(message=f"🔧 Executing command: {command}", vehicle_number=vehicle))
        stdin, stdout, stderr = ssh.exec_command(command, timeout=10)
        
        output = stdout.read().decode() + stderr.read().decode()
        ros_node.console_publisher.publish(ConsoleLog(message=f"📄 Command output: {output}", vehicle_number=vehicle))
        
        if "success=True" in output:
            match = re.search(r'message=\s*(.*)', output)
            ros_node.console_publisher.publish(ConsoleLog(message=f"✅ Service call to {service_name} succeeded. {match.group(1) if match else ''}", vehicle_number=vehicle))
        elif "success=False" in output:
            match = re.search(r'message=\s*(.*)', output)
            ros_node.console_publisher.publish(ConsoleLog(message=f"⚠️ Service call to {service_name} failed with message: {match.group(1) if match else ''}", vehicle_number=vehicle))
        else:
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Unexpected response from service {service_name}: {output}", vehicle_number=vehicle))
            
    except Exception as e:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Service call to {service_name} failed: {e}", vehicle_number=vehicle))
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
            
        ros_node.console_publisher.publish(ConsoleLog(message=f"🔧 Executing: {command}", vehicle_number=vehicle))
        
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
                                ros_node.console_publisher.publish(ConsoleLog(message=clean_line, vehicle_number=vehicle))
                    
                    if channel.exit_status_ready():
                        break
                        
                    time.sleep(0.1)  # Small delay to prevent busy waiting
                        
            except Exception as e:
                ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Error reading output: {e}", vehicle_number=vehicle))
        
        # Start reading output in a thread
        output_thread = threading.Thread(target=read_output, daemon=True)
        output_thread.start()
        
        # Wait for command to complete
        exit_status = channel.recv_exit_status()
        
        # Wait for output thread to finish
        output_thread.join(timeout=2)
        
        if exit_status != 0:
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Script {script_path} failed with exit code {exit_status}", vehicle_number=vehicle))
        else:
            ros_node.console_publisher.publish(ConsoleLog(message=f"✅ Script {script_path} completed successfully", vehicle_number=vehicle))
            
        channel.close()
        
    except Exception as e:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Failed to run script {script_path}: {e}", vehicle_number=vehicle))
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
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Error getting parameter: {error_output}", vehicle_number=vehicle))
            return None
            
        match = re.search(r"[-+]?\d*\.\d+|\d+", output)
        if match:
            return match.group()
        elif "Parameter not set" in output:
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Parameter '{ROS_PARAM_NAME}' is not set on node '{NODE_NAME}'.", vehicle_number=vehicle))
        else:
            ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Failed to retrieve parameter: {output}", vehicle_number=vehicle))
            
    except Exception as e:
        ros_node.console_publisher.publish(ConsoleLog(message=f"❌ Exception while getting ROS param: {e}", vehicle_number=vehicle))
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

    ros_node.console_publisher.publish(ConsoleLog(message=f"✅ Updated {file_path} with {param_name} = {value} under {namespace}/{node_name}", vehicle_number=vehicle))

# ----------------------------
# MAIN SCRIPT LOGIC
# ----------------------------
def main(passed_ros_node, vehicle_numbers):
    global ros_node
    ros_node = passed_ros_node

    for vehicle in vehicle_numbers:
        namespace = f"coug{vehicle}"

        ros_node.console_publisher.publish(ConsoleLog(message=f"🔧 Running calibrate depth service...", vehicle_number=vehicle))
        #run calibrate depth service
        run_service_call(f"{namespace}/calibrate_depth", "std_srvs/srv/Trigger", "{}", vehicle=vehicle)

        ros_node.console_publisher.publish(ConsoleLog(message=f"🔧 Running calibrate_gyro.sh...", vehicle_number=vehicle))
        #run calibrate gyro script (remote DVL directory)
        remote_dvl_dir = "~/ros2_ws/dvl_tools"
        run_script(f"{remote_dvl_dir}/calibrate_gyro.sh", vehicle=vehicle)

        ros_node.console_publisher.publish(ConsoleLog(message=f"🔧 Running set_ntp...", vehicle_number=vehicle))
        # run ntp script (remote DVL directory) 
        run_script(f"{remote_dvl_dir}/set_ntp.sh", vehicle=vehicle)

        ros_node.console_publisher.publish(ConsoleLog(message=f"🔧 Getting ROS parameter...", vehicle_number=vehicle))
        # get fluid_pressure_atm param value
        param_value = get_ros_param(namespace, vehicle=vehicle)
        if param_value:
            update_yaml_param(VEHICLE_PARAMS_PATH, ROS_PARAM_NAME, param_value, NODE_NAME, namespace, vehicle=vehicle)
        ros_node.console_publisher.publish(ConsoleLog(message=f"✅ Calibrate.py finished for Vehicle {vehicle}", vehicle_number=vehicle))

if __name__ == "__main__":
    main()
