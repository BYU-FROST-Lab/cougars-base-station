
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"
#include "base_station_interfaces/srv/load_mission.hpp"
#include "base_station_interfaces/msg/status.hpp"
#include "base_station_interfaces/msg/connections.hpp"
#include "base_station_interfaces/msg/u_command_base.hpp"
#include "cougars_interfaces/msg/system_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "base_station_interfaces/srv/init.hpp"
#include "cougars_interfaces/msg/u_command.hpp"
#include "base_station_interfaces/msg/console_log.hpp"



#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"


#include <iostream>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>
#include <sstream>
#include <cstring>


using namespace std::literals::chrono_literals;
using namespace cougars_coms;
using namespace narval::seatrac;


using std::placeholders::_1;
using std::placeholders::_2;


class ComsNode : public rclcpp::Node {
public:
    ComsNode() : Node("base_station_coms") {

        // Frequency of status requests to vehicles in mission, one at a time
        this->declare_parameter<double>("status_request_frequency_seconds", 1);
        this->status_request_frequency = this->get_parameter("status_request_frequency_seconds").as_double();

        // list of beacon ids of vehicles in mission
        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();

        this->declare_parameter<bool>("wifi_enabled", true);
        this->wifi_enabled_ = this->get_parameter("wifi_enabled").as_bool();

        // When true the node will periodically request status from vehicles in mission
        bool request_status = this->declare_parameter<bool>("request_status", true);

        rf_transmit_pub_ = this->create_publisher<std_msgs::msg::String>(
            "rf_transmit",
            10);
        
        modem_transmit_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "modem_transmit",
            10);
        
        // client for the base_station_wifi node. Requests that the wifi sends an emergency kill command to a specific coug
        wifi_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "wifi_e_kill"
        );



        // client for the base_station_wifi node. Requests that an init command is sent over wifi
        wifi_init_client_ = this->create_client<base_station_interfaces::srv::Init>(
            "wifi_init"
        );

        // client for the base_station_wifi node. Requests that a load mission command is sent over wifi
        wifi_load_mission_client_ = this->create_client<base_station_interfaces::srv::LoadMission>(
            "wifi_load_mission"
        );

        // client for the base_station_radio node. Requests that a load mission command is sent over radio
        radio_load_mission_client_ = this->create_client<base_station_interfaces::srv::LoadMission>(
            "radio_load_mission"
        );

        // service for sending e_kill message. Decides whether to send over radio or modem
        emergency_kill_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "e_kill_service",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );

        // service for sending e_surface message. Decides whether to send over radio or modem
        emergency_surface_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "e_surface_service",
            std::bind(&ComsNode::emergency_surface_callback, this, _1, _2)
        );

        // service for sending init message. Decides whether to send over radio or modem
        init_service_ = this->create_service<base_station_interfaces::srv::Init>(
            "init_service",
            std::bind(&ComsNode::init_callback, this, _1, _2)
        );

        // service for sending load mission message. Decides whether to send over radio or wifi
        load_mission_service_ = this->create_service<base_station_interfaces::srv::LoadMission>(
            "load_mission_service",
            std::bind(&ComsNode::load_mission_callback, this, _1, _2)
        );

        // subscriber to the status topic published by the modem and radio nodes
        status_subscriber_ = this->create_subscription<base_station_interfaces::msg::Status>(
            "status", 10,
            std::bind(&ComsNode::publish_status_callback, this, _1)
        );

        // subscriber to the connections topic of the modem and radio nodes, keeps track of which vehicles are connected via radio or modem
        connections_subscriber_ = this->create_subscription<base_station_interfaces::msg::Connections>(
            "connections", 10,
            std::bind(&ComsNode::listen_to_connections, this, _1)
        );

        // subscriber to the keyboard controls
        keyboard_controls_subscriber_ = this->create_subscription<base_station_interfaces::msg::UCommandBase>(
            "keyboard_controls", 10,
            std::bind(&ComsNode::keyboard_controls_callback, this, _1)
        );

        // publisher for wifi keyboard controls
        wifi_key_publisher_ = this->create_publisher<base_station_interfaces::msg::UCommandBase>(
            "wifi_keyboard_controls", 10
        );

                // publisher for the confirmation of the emergency kill command
        print_to_gui_pub = this->create_publisher<base_station_interfaces::msg::ConsoleLog>("console_log", 10);

        // RCLCPP_INFO(this->get_logger(), "request status: %d", request_status);
        if (request_status) {
           // timer that periodically requests status from vehicles in mission
            RCLCPP_INFO(this->get_logger(), "requesting status");
            timer_ = this->create_wall_timer(std::chrono::duration<double>(status_request_frequency), std::bind(&ComsNode::request_status_callback, this));


           // Status publishers for each vehicle in the mission modelling the topics on each vehicle
            for (int vehicle_id : vehicles_in_mission_) {
                std::string ros_namespace = "/coug" + std::to_string(vehicle_id);
                safety_status_publishers_[vehicle_id] = this->create_publisher<cougars_interfaces::msg::SystemStatus>(
                    ros_namespace + "/safety_status", 10);
                dvl_publishers_[vehicle_id] = this->create_publisher<dvl_msgs::msg::DVLDR>(
                    ros_namespace + "/dvl/position", 10);
                dvl_vel_publishers_[vehicle_id] = this->create_publisher<dvl_msgs::msg::DVL>(
                    ros_namespace + "/dvl/data", 10);
                battery_publishers_[vehicle_id] = this->create_publisher<sensor_msgs::msg::BatteryState>(
                    ros_namespace + "/battery/data", 10);
                depth_publishers_[vehicle_id] = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    ros_namespace + "/depth_data", 10);
                pressure_publishers_[vehicle_id] = this->create_publisher<sensor_msgs::msg::FluidPressure>(
                    ros_namespace + "/pressure/data", 10);
            }
        }

       // Send to all is always true
       modem_connection[0] = true;
       radio_connection[0] = true;

       modem_status_buffer = 4;
       last_modem_status = this->get_clock()->now();
       // Initialize lists of connections statuses for each vehicle in mission
       for(int64_t i: vehicles_in_mission_){
           modem_connection[i] = false;
           radio_connection[i] = false;
           wifi_connection[i] = false;
       }
       std::ostringstream ss;
       ss << "Vehicle ids in mission: ";
       for(int64_t i: vehicles_in_mission_) ss << i << ", ";
       RCLCPP_INFO(this->get_logger(), "base station started");
       RCLCPP_INFO(this->get_logger(), ss.str().c_str());
   }


    // Callback for the connections topic, updates the connections for each vehicle in mission
    // The connections are stored in three maps, one for radio, one for modem, and one for wifi
    void listen_to_connections(const base_station_interfaces::msg::Connections::SharedPtr msg) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Updating connections for cougs");

        size_t n = std::min(vehicles_in_mission_.size(), msg->connections.size());
        for (size_t i = 0; i < n; ++i) {
            int beacon_id = vehicles_in_mission_[i];
            if (msg->connection_type == 1) {
                radio_connection[beacon_id] = msg->connections[i];
            } else if (msg->connection_type == 0) {
                modem_connection[beacon_id] = msg->connections[i];
            } else if (msg->connection_type == 2 && wifi_enabled_) {
                wifi_connection[beacon_id] = msg->connections[i];
            }
        }
    }

    // Callback for the keyboard controls topic, updates the keyboard controls for each vehicle in mission
    void keyboard_controls_callback(const base_station_interfaces::msg::UCommandBase::SharedPtr msg) {
        int vehicle_id = msg->vehicle_id;
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received keyboard controls for vehicle %d", vehicle_id);

        // if wifi connection, send to wifi keyboard controls
        if (wifi_connection[vehicle_id]) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending keyboard controls for coug %d over wifi", vehicle_id);
            wifi_key_publisher_->publish(*msg);
            return;
        } else if (radio_connection[vehicle_id]) {
                
                // Build JSON message matching Python format
                std::ostringstream json_stream;
                json_stream << "{"
                            << "\"vehicle_id\":" << vehicle_id << ","
                            << "\"message\":\"KEY_CONTROL\","
                            << "\"command\":{"
                            << "\"fin\":[" << msg->ucommand.fin[0] << "," 
                            << msg->ucommand.fin[1] << "," 
                            << msg->ucommand.fin[2] << "," 
                            << msg->ucommand.fin[3] << "],"
                            << "\"throttle\":" << static_cast<int>(msg->ucommand.thruster) << ","
                            << "\"enable\":" << (msg->thruster_enabled ? "true" : "false")
                            << "}"
                            << "}";
                
                std_msgs::msg::String key_control_msg = std_msgs::msg::String();
                key_control_msg.data = json_stream.str();
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing JSON: %s", key_control_msg.data.c_str());
                rf_transmit_pub_->publish(key_control_msg);
                
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Key control command sent to Coug %d through radio", vehicle_id);
            return;
        } else {    
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No connection to coug %d, cannot send keyboard controls", vehicle_id);
            return;
        }
    }
    
    // Callback for the status request service, requests the status of a specific vehicle in mission
    // If the vehicle is connected via radio, it requests the status through the radio node
    // If not it attempts to request the status through the modem node
    void request_status_callback(){

        // If there are no vehicles in mission, skip the status request
        if (vehicles_in_mission_.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No vehicles in mission — skipping status request.");
            return;
        } 

        // Does not make status request if the vehicle is connected via wifi
        if (wifi_connection[vehicles_in_mission_[vehicle_id_index]]) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Vehicle %li is connected via wifi, skipping status request.", vehicles_in_mission_[vehicle_id_index]);
            return;
        }

                // Cycle through vehicles in mission to request status one by one
        vehicle_id_index += 1;
        if (vehicle_id_index>=vehicles_in_mission_.size()){
            vehicle_id_index = 0;
        }
        // Create the status request
        auto request = std::make_shared<base_station_interfaces::srv::BeaconId::Request>();
        int beacon_id = vehicles_in_mission_[vehicle_id_index];
        request->beacon_id = beacon_id;

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting Status: Coug %i connections: radio - %d, modem - %d", beacon_id, radio_connection[beacon_id], modem_connection[beacon_id]);
        // Attempts to request through radio first, if the vehicle is connected via radio
        // If the vehicle is not connected via radio, it attempts to request through modem
        if (radio_connection[beacon_id]) {
            std::ostringstream json_stream;
            json_stream << "{"
                        << "\"vehicle_id\":" << beacon_id << ","
                        << "\"message\":\"STATUS\""
                        << "}";
            std_msgs::msg::String status_msg = std_msgs::msg::String();
            status_msg.data = json_stream.str();
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through radio", beacon_id);
            rf_transmit_pub_->publish(status_msg);

        } else if ((this->get_clock()->now() - last_modem_status).seconds() > this->modem_status_buffer) {

            if (!modem_connection[beacon_id]){
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Status request may not work because there is no connection.");
            }
            
            // Create RequestStatus message and send via modem_transmit topic
            RequestStatus request_status_msg;
            std_msgs::msg::UInt8MultiArray msg;
            msg.data.push_back(beacon_id);  // First byte: target vehicle ID
            msg.data.push_back(request_status_msg.msg_id);  // Message payload: REQUEST_STATUS
            
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through modem", beacon_id);
            modem_transmit_pub_->publish(msg);
            
            // gets time since last status request
            this->last_modem_status = this->get_clock()->now();
        } else {
            vehicle_id_index -= 1;
        }


    }

    // Callback for the emergency kill service, sends an emergency kill command to a specific vehicle in mission
    // If the vehicle is connected via radio, it sends the command through the radio node
    // If not it attempts to send the command through the modem node
    void emergency_kill_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response){
        int beacon_id = request->beacon_id;
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), beacon_id) != vehicles_in_mission_.end() || beacon_id == BEACON_ALL){
            if (wifi_connection[beacon_id]) {

                wifi_e_kill_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success) {
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through wifi", beacon_id);
                            base_station_interfaces::msg::ConsoleLog log_msg;
                            log_msg.message = "Emergency kill command was sent through wifi for Coug " + std::to_string(beacon_id);
                            log_msg.vehicle_number = beacon_id;
                            print_to_gui_pub->publish(log_msg);
                        } else {
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through wifi failed", beacon_id);
                        }
                    });
                response->success = true;
            
            } else if (radio_connection[beacon_id]){

                std_msgs::msg::String e_kill_msg = std_msgs::msg::String();
                e_kill_msg.data = "E_KILL";
                rf_transmit_pub_->publish(e_kill_msg);
                response->success = true;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending E_KILL to Coug %i through radio", beacon_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Emergency kill command was sent through radio for Coug " + std::to_string(beacon_id);
                log_msg.vehicle_number = beacon_id;
                print_to_gui_pub->publish(log_msg);
            } else if (modem_connection[beacon_id]) {
                
                EmergencyKill e_kill_msg;
                std_msgs::msg::UInt8MultiArray msg;
                msg.data.push_back(beacon_id);  // First byte: target vehicle ID
                msg.data.push_back(e_kill_msg.msg_id);  // Message payload: EMERGENCY_KILL
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending E_KILL to Coug %i through modem", beacon_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Emergency kill command was sent through modem for Coug " + std::to_string(beacon_id);
                log_msg.vehicle_number = beacon_id;
                print_to_gui_pub->publish(log_msg);
                modem_transmit_pub_->publish(msg);
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send e_kill command to Coug %i because there is no connection", beacon_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Failed to send emergency kill command to Coug " + std::to_string(beacon_id) + " because there is no connection";
                log_msg.vehicle_number = beacon_id;
                print_to_gui_pub->publish(log_msg);
                response->success = false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send kill command. There is no Vehicle with ID %i", beacon_id);
            response->success = false;
        }
    }

    // Callback for the emergency surface service, sends an emergency surface command to a specific vehicle in mission
    // Only sent through the modem node, as the radio node does not support this command
    void emergency_surface_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response){
        int beacon_id = request->beacon_id;
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), beacon_id) != vehicles_in_mission_.end() || beacon_id == BEACON_ALL){
            if (modem_connection[beacon_id]){

                EmergencySurface e_surface_msg;
                std_msgs::msg::UInt8MultiArray msg;
                msg.data.push_back(beacon_id);  // First byte: target vehicle ID
                msg.data.push_back(e_surface_msg.msg);  // Message payload: EMERGENCY_SURFACE
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending E_SURFACE to Coug %i through modem", beacon_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Emergency surface command was sent through modem for Coug " + std::to_string(beacon_id);
                log_msg.vehicle_number = beacon_id;
                print_to_gui_pub->publish(log_msg);
                modem_transmit_pub_->publish(msg);
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send surface command to Coug %i because there is no modem connection", beacon_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Failed to send emergency surface command to Coug " + std::to_string(beacon_id) + " because there is no modem connection";
                log_msg.vehicle_number = beacon_id;
                print_to_gui_pub->publish(log_msg);
                response->success = false;
            }
        } else{
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send surface command. There is no Vehicle with ID %i", beacon_id);
            response->success = false;
        }
    }


    void init_callback(const std::shared_ptr<base_station_interfaces::srv::Init::Request> request,
                       std::shared_ptr<base_station_interfaces::srv::Init::Response> response) {
        int vehicle_id = request->vehicle_id;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending init command to %i", vehicle_id);
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), vehicle_id) != vehicles_in_mission_.end()) {
            if (wifi_connection[vehicle_id]) {
                wifi_init_client_->async_send_request(request,
                    [this, vehicle_id](rclcpp::Client<base_station_interfaces::srv::Init>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success) {
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through wifi", vehicle_id);
                            base_station_interfaces::msg::ConsoleLog log_msg;
                            log_msg.message = "Init command was sent through wifi for Coug " + std::to_string(vehicle_id);
                            log_msg.vehicle_number = vehicle_id;
                            print_to_gui_pub->publish(log_msg);
                        } else {
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through wifi failed", vehicle_id);
                            base_station_interfaces::msg::ConsoleLog log_msg;
                            log_msg.message = "Failed to send init command to Coug " + std::to_string(vehicle_id) + " through wifi";
                            log_msg.vehicle_number = vehicle_id;
                            print_to_gui_pub->publish(log_msg);
                        }
                    });
                response->success = true;
            } else if (radio_connection[vehicle_id]) {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received initialization request for Coug %i", vehicle_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Init command was sent through radio for Coug " + std::to_string(vehicle_id);
                log_msg.vehicle_number = vehicle_id;
                print_to_gui_pub->publish(log_msg);
                std::ostringstream json_stream;
                json_stream << "{"
                           << "\"vehicle_id\":" << vehicle_id << ","
                           << "\"message\":\"INIT\","
                           << "\"start\":" << (request->start.data ? "true" : "false") << ","
                           << "\"rosbag_flag\":" << (request->rosbag_flag.data ? "true" : "false") << ","
                           << "\"rosbag_prefix\":\"" << request->rosbag_prefix << "\","
                           << "\"thruster_arm\":" << (request->thruster_arm.data ? "true" : "false") << ","
                           << "\"dvl_acoustics\":" << (request->dvl_acoustics.data ? "true" : "false")
                           << "}";
                
                std_msgs::msg::String init_msg;
                init_msg.data = json_stream.str();
                rf_transmit_pub_->publish(init_msg);
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through radio", vehicle_id);
                response->success = true;
            } else if (modem_connection[vehicle_id]) {

                // Build Init message with bitmask
                Init init_msg;
                init_msg.init_bitmask = 0;
                if (request->start.data) init_msg.init_bitmask |= 0x01;
                if (request->rosbag_flag.data) init_msg.init_bitmask |= 0x02;
                if (request->thruster_arm.data) init_msg.init_bitmask |= 0x04;
                if (request->dvl_acoustics.data) init_msg.init_bitmask |= 0x08;
                
                // Copy rosbag prefix (max 28 chars)
                std::strncpy(init_msg.rosbag_prefix, request->rosbag_prefix.c_str(), 
                             sizeof(init_msg.rosbag_prefix) - 1);
                init_msg.rosbag_prefix[sizeof(init_msg.rosbag_prefix) - 1] = '\0';
                
                // Create MultiArray message
                std_msgs::msg::UInt8MultiArray msg;
                msg.data.push_back(vehicle_id);  // First byte: target vehicle ID
                
                // Add init message bytes (msg_id + bitmask + rosbag_prefix)
                uint8_t* init_bytes = (uint8_t*)&init_msg;
                for (size_t i = 0; i < sizeof(init_msg); ++i) {
                    msg.data.push_back(init_bytes[i]);
                }
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through modem", vehicle_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Init command was sent through modem for Coug " + std::to_string(vehicle_id);
                log_msg.vehicle_number = vehicle_id;
                print_to_gui_pub->publish(log_msg);
                modem_transmit_pub_->publish(msg);
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send init command to Coug %i because there is no connection", vehicle_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Failed to send init command to Coug " + std::to_string(vehicle_id) + " because there is no connection";
                log_msg.vehicle_number = vehicle_id;
                print_to_gui_pub->publish(log_msg);
                response->success = false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send init command. There is no Vehicle with ID %i", vehicle_id);
            response->success = false;
        }
    }

    // Callback for loading the params and mission files to a specific vehicle in mission
    // If the vehicle is connected via wifi, it sends the command through the wifi node
    // If not it attempts to send the command through the radio node
    void load_mission_callback(const std::shared_ptr<base_station_interfaces::srv::LoadMission::Request> request,
                               std::shared_ptr<base_station_interfaces::srv::LoadMission::Response> response) {
        int64_t vehicle_id = request->vehicle_id;

        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), vehicle_id) != vehicles_in_mission_.end()) {
            // Load the mission for the specified vehicle
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading mission for Coug %li", vehicle_id);
            if (wifi_connection[vehicle_id]) {
                wifi_load_mission_client_->async_send_request(request,
                    [this, vehicle_id](rclcpp::Client<base_station_interfaces::srv::LoadMission>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success) {
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading mission to Coug %li through wifi", vehicle_id);
                            base_station_interfaces::msg::ConsoleLog log_msg;
                            log_msg.message = "Loading mission through wifi for Coug " + std::to_string(vehicle_id);
                            log_msg.vehicle_number = vehicle_id;
                            print_to_gui_pub->publish(log_msg);
                        } else {
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Loading mission to Coug %li through wifi failed", vehicle_id);
                        }
                    });
                response->success = true;
            } else if (radio_connection[vehicle_id]) {
                radio_load_mission_client_->async_send_request(request,
                    [this, vehicle_id](rclcpp::Client<base_station_interfaces::srv::LoadMission>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success) {
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading mission to Coug %li through radio", vehicle_id);
                            base_station_interfaces::msg::ConsoleLog log_msg;
                            log_msg.message = "Loading mission through radio for Coug " + std::to_string(vehicle_id);
                            log_msg.vehicle_number = vehicle_id;
                            print_to_gui_pub->publish(log_msg);
                        } else {
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Loading mission to Coug %li through radio failed", vehicle_id);
                        }
                    });
                response->success = true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot load mission command. No connection to Coug %li", vehicle_id);
                base_station_interfaces::msg::ConsoleLog log_msg;
                log_msg.message = "Failed to load mission to Coug " + std::to_string(vehicle_id) + " because there is no connection";
                log_msg.vehicle_number = vehicle_id;
                print_to_gui_pub->publish(log_msg);
                response->success = false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot load mission. There is no Vehicle with ID %li", vehicle_id);
            response->success = false;
        }
    }


    // Callback for the status subscriber, publishes the status of a specific vehicle in mission
    // If the vehicle is in the mission, it publishes the status to the appropriate topics
    void publish_status_callback(const std::shared_ptr<base_station_interfaces::msg::Status> msg) {
        int64_t vehicle_id = msg->vehicle_id;
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), vehicle_id) != vehicles_in_mission_.end()) {
            // Publish the status to the appropriate topic
            cougars_interfaces::msg::SystemStatus safety_status = msg->safety_status;
            dvl_msgs::msg::DVLDR dvl_pos = msg->dvl_pos;
            dvl_msgs::msg::DVL dvl_vel;
            sensor_msgs::msg::BatteryState battery_state = msg->battery_state;
            geometry_msgs::msg::PoseWithCovarianceStamped depth_status = msg->depth_data;
            sensor_msgs::msg::FluidPressure pressure_status = msg->pressure;

            safety_status_publishers_[vehicle_id]->publish(safety_status);
            dvl_publishers_[vehicle_id]->publish(dvl_pos);
            dvl_vel_publishers_[vehicle_id]->publish(dvl_vel);
            battery_publishers_[vehicle_id]->publish(battery_state);
            depth_publishers_[vehicle_id]->publish(depth_status);
            pressure_publishers_[vehicle_id]->publish(pressure_status);

        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot publish status. There is no Vehicle with ID %li", vehicle_id);
        }
    }


private:


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<base_station_interfaces::msg::Connections>::SharedPtr connections_subscriber_;

    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr wifi_e_kill_client_;
    rclcpp::Client<base_station_interfaces::srv::Init>::SharedPtr wifi_init_client_;
    rclcpp::Client<base_station_interfaces::srv::LoadMission>::SharedPtr wifi_load_mission_client_;
    rclcpp::Client<base_station_interfaces::srv::LoadMission>::SharedPtr radio_load_mission_client_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rf_transmit_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr modem_transmit_pub_;


    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_kill_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_surface_service_;
    rclcpp::Service<base_station_interfaces::srv::Init>::SharedPtr init_service_;
    rclcpp::Service<base_station_interfaces::srv::LoadMission>::SharedPtr load_mission_service_;


    rclcpp::Subscription<base_station_interfaces::msg::Status>::SharedPtr status_subscriber_;
    rclcpp::Subscription<base_station_interfaces::msg::UCommandBase>::SharedPtr keyboard_controls_subscriber_;
    std::unordered_map<int64_t, rclcpp::Publisher<cougars_interfaces::msg::SystemStatus>::SharedPtr> safety_status_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<dvl_msgs::msg::DVLDR>::SharedPtr> dvl_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<dvl_msgs::msg::DVL>::SharedPtr> dvl_vel_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr> battery_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> depth_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr> pressure_publishers_;

    std::unordered_map<int,bool> radio_connection;
    std::unordered_map<int,bool> modem_connection;
    std::unordered_map<int,bool> wifi_connection;

    rclcpp::Publisher<base_station_interfaces::msg::UCommandBase>::SharedPtr wifi_key_publisher_;
    rclcpp::Publisher<base_station_interfaces::msg::ConsoleLog>::SharedPtr print_to_gui_pub;


    std::vector<int64_t> vehicles_in_mission_;

    size_t vehicle_id_index = 0;

    double status_request_frequency;
    double modem_status_buffer;
    bool wifi_enabled_;
    rclcpp::Time last_modem_status;

};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}