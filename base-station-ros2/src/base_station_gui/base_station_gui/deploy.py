import os
import json
from datetime import datetime
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node
from pathlib import Path
import paramiko
import subprocess

global ros_node

# SSH configuration
PARAM_DIR = os.environ.get(
    "BASE_STATION_PARAM_DIR",
    os.path.expanduser("~/base_station/mission_control/params")
)
DEPLOY_HISTORY_DIR = "/home/frostlab/bag/deployment_history"
CONFIG_FILE = str(Path.home()) + "/base_station/mission_control/deploy_config.json"

os.makedirs(DEPLOY_HISTORY_DIR, exist_ok=True)

def load_config(sel_vehicles):
    with open(CONFIG_FILE, "r") as f:
        config = json.load(f)
        vehicles = config["vehicles"]
    result = []
    for num in sel_vehicles:
        if str(num) in vehicles:
            result.append(vehicles[str(num)])
        else:
            ros_node.console_log.publish(ConsoleLog(message=f"❌ Vehicle {num} not found in config, consider adding (skipping)", vehicle_number=num))
    return result

def sftp_file(file_path, remote_user, remote_host, remote_path, remote_filename, vehicle_num):
    """Deletes existing file then copies a new one via SFTP using SSH key authentication."""
    try:
        ros_node.console_log.publish(ConsoleLog(message=f"🗑️ Deleting {remote_filename} on {remote_host}...", vehicle_number=vehicle_num))
        ssh = get_ssh_connection(remote_host, remote_user)

        if ssh is None:
            ros_node.console_log.publish(ConsoleLog(message=f"❌ Failed to establish SSH connection to {remote_host}", vehicle_number=vehicle_num))
            return False

        sftp = ssh.open_sftp()
        remote_full_path = os.path.join(remote_path, remote_filename)
        
        sftp = ssh.open_sftp()
        remote_full_path = os.path.join(remote_path, remote_filename)
        # Try to delete the remote file if it exists
        try:
            sftp.remove(remote_full_path)
        except FileNotFoundError:
            pass  # It's OK if the file doesn't exist
        ros_node.console_log.publish(ConsoleLog(message=f"📤 Copying {file_path} to {remote_user}@{remote_host}:{remote_full_path}...", vehicle_number=vehicle_num))
        sftp.put(file_path, remote_full_path)
        sftp.close()
        ssh.close()
        return True
    except Exception as e:
        ros_node.console_log.publish(ConsoleLog(message=f"❌ SFTP error: {e}", vehicle_number=vehicle_num))
        return False

def get_ssh_connection(ip_address, remote_user):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip_address, username=remote_user)
            return ssh
        except (paramiko.AuthenticationException, paramiko.SSHException):
            # Run ssh-copy-id to add the key
            ros_node.console_log.publish(ConsoleLog(message=f"SSH Authentication failed for {ip_address}. Attempting to copy SSH key. Enter password in the terminal", vehicle_number=0))
            try:
                if ensure_ssh_key():
                    subprocess.run(["ssh-copy-id", f"{remote_user}@{ip_address}"], check=True)
                    ros_node.console_log.publish(ConsoleLog(message=f"SSH key copied successfully to {ip_address}.", vehicle_number=0))
                    return get_ssh_connection(ip_address, remote_user)
            except subprocess.CalledProcessError as e:
                ros_node.console_log.publish(ConsoleLog(message=f"Failed to copy SSH key to {ip_address}: {e}", vehicle_number=0))
        except Exception as e:
            ros_node.console_log.publish(ConsoleLog(message=f"Failed to connect to {ip_address}: {e}", vehicle_number=0))
            return None

def ensure_ssh_key(key_path="~/.ssh/id_rsa"):
    key_path = os.path.expanduser(key_path)
    pub_key_path = key_path + ".pub"
    # Check if both private and public key exist
    if os.path.exists(key_path) and os.path.exists(pub_key_path):
        return True  # SSH key exists
    else:
        # Generate a new SSH key with ssh-keygen
        subprocess.run(["ssh-keygen", "-t", "rsa", "-b", "4096", "-f", key_path, "-N", ""], check=True)
        return os.path.exists(key_path) and os.path.exists(pub_key_path)

def log_deployment(vehicle, files_sent, vehicle_number):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(DEPLOY_HISTORY_DIR, f"{vehicle['name']}_deployment_{timestamp}.txt")
    with open(log_file, "w") as f:
        f.write(f"Deployment for {vehicle['name']} at {timestamp}\n")
        f.write(f"Host: {vehicle['remote_host']}\n")
        for label, path in files_sent:
            f.write(f"{label}: {path}\n")
    ros_node.console_log.publish(ConsoleLog(message=f"✅ Deployment logged: {log_file}\n", vehicle_number=vehicle_number))

def main(passed_ros_node, sel_vehicles, passed_file_paths=[]): #selected vehicles
    global ros_node
    ros_node = passed_ros_node

    vehicles = load_config(sel_vehicles)
    for i, vehicle in enumerate(vehicles):
        load_success = True
        files_sent = []
        mission_path = passed_file_paths[i]
        param_path = os.path.join(PARAM_DIR, vehicle["param_file"])
        fleet_path = os.path.join(PARAM_DIR, vehicle["fleet_param_file"])
        vehicle_number = int(vehicle["name"][-1])
        ros_node.console_log.publish(ConsoleLog(message=f"Mission path for coug{vehicle_number} was set as: {mission_path}", vehicle_number=vehicle_number))

        for label, file_path, remote_filename in [
            ("Mission File", mission_path, "mission.yaml"),
            ("Vehicle Params", param_path, os.path.basename(param_path)),
            ("Fleet Params", fleet_path, os.path.basename(fleet_path)),
        ]:
            if os.path.exists(file_path):
                success = sftp_file(
                    file_path,
                    vehicle["remote_user"],
                    vehicle["remote_host"],
                    vehicle["remote_path"],
                    remote_filename,
                    vehicle_number
                )
                if success:
                    files_sent.append((label, file_path))
                else:
                    ros_node.console_log.publish(ConsoleLog(message=f"❌ Failed to deploy {label} to {vehicle['name']}", vehicle_number=vehicle_number))
                    load_success = False
            else:
                ros_node.console_log.publish(ConsoleLog(message=f"⚠️ File not found: {file_path} (skipping)", vehicle_number=vehicle_number))
                load_success = False

        log_deployment(vehicle, files_sent, vehicle_number)
        message = f"Coug{vehicle_number} mission loading finished with no errors." if load_success else f"Coug{vehicle_number} mission loading failed."
        ros_node.console_log.publish(ConsoleLog(message=message, vehicle_number=vehicle_number))