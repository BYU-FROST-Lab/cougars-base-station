
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"
#include "base_station_interfaces/msg/status.hpp"
#include "base_station_interfaces/msg/connections.hpp"
#include "frost_interfaces/msg/system_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "base_station_interfaces/srv/init.hpp"

#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"


#include <iostream>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>


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

        // When true the node will periodically request status from vehicles in mission
        bool request_status = this->declare_parameter<bool>("request_status", true);


        // client for the base_station_radio node. Requests that the radio sends an emergency kill command to a specific coug
        radio_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "radio_e_kill"
        );
        
        // client for the base_station_modem node. Requests that the modem sends an emergency kill command to a specific coug
        modem_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_e_kill"
        );

        wifi_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "wifi_e_kill"
        );

        // client for the base_station_modem node. Requests that the modem sends an emergency surface command to a specific coug
        modem_e_surface_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_e_surface"
        );

        // client for the base_station_modem node. Requests that the modem gets the status of a coug
        modem_status_request_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_status_request"
        );

        // client for the base_station_radio node. Requests that the radio gets the status of a coug
        radio_status_request_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "radio_status_request"
        );

        wifi_init_client_ = this->create_client<base_station_interfaces::srv::Init>(
            "wifi_init"
        );

        modem_init_client_ = this->create_client<base_station_interfaces::srv::Init>(
            "modem_init"
        );

        radio_init_client_ = this->create_client<base_station_interfaces::srv::Init>(
            "radio_init"
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

        init_service_ = this->create_service<base_station_interfaces::srv::Init>(
            "init_service",
            std::bind(&ComsNode::init_callback, this, _1, _2)
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

        // RCLCPP_INFO(this->get_logger(), "request status: %d", request_status);
        if (request_status) {
           // timer that periodically requests status from vehicles in mission
            RCLCPP_INFO(this->get_logger(), "requesting status");
            timer_ = this->create_wall_timer(std::chrono::duration<double>(status_request_frequency), std::bind(&ComsNode::request_status_callback, this));


           // Status publishers for each vehicle in the mission modelling the topics on each vehicle
            for (int vehicle_id : vehicles_in_mission_) {
                std::string ros_namespace = "/coug" + std::to_string(vehicle_id);
                safety_status_publishers_[vehicle_id] = this->create_publisher<frost_interfaces::msg::SystemStatus>(
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
    // The connections are stored in two maps, one for radio and one for modem connections
    void listen_to_connections(const base_station_interfaces::msg::Connections::SharedPtr msg) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Updating connections for cougs");

        size_t n = std::min(vehicles_in_mission_.size(), msg->connections.size());
        for (size_t i = 0; i < n; ++i) {
            int beacon_id = vehicles_in_mission_[i];
            if (msg->connection_type == 1) {
                radio_connection[beacon_id] = msg->connections[i];
            } else if (msg->connection_type == 0) {
                modem_connection[beacon_id] = msg->connections[i];
            } else if (msg->connection_type == 2) {
                wifi_connection[beacon_id] = msg->connections[i];
            }
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

            if (!radio_status_request_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Radio status request service not available for Coug %i. Skipping request.", beacon_id);
                return;
            }
            auto result_future = radio_status_request_client_->async_send_request(request,
                [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through radio", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Status request from Coug %i through radio failed", beacon_id);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception while requesting status from Coug %i: %s", beacon_id, e.what());
                    }
                });
        } else if ((this->get_clock()->now() - last_modem_status).seconds() > this->modem_status_buffer) {

            if (!modem_connection[beacon_id]){
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Status request may not work because there is no connection.");
            }
            if (!modem_status_request_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Modem status request service not available for Coug %i. Skipping request.", beacon_id);
            }
            else {
                auto result_future = modem_status_request_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        try {
                            auto response = future.get();
                            if (response->success)
                            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through modem", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Status request from Coug %i through modem failed", beacon_id);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception while requesting status from Coug %i: %s", beacon_id, e.what());
                    }
                });
                // gets time since last status request
                this->last_modem_status = this->get_clock()->now();

            }
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
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through wifi", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through wifi failed", beacon_id);
                    });
                response->success = true;
            
            } else if (radio_connection[beacon_id]){

                radio_e_kill_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through radio", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through radio failed", beacon_id);
                    });
                response->success = true;

            } else if (modem_connection[beacon_id]) {
                
                modem_e_kill_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through modem", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through modem failed", beacon_id);
                    });
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send e_kill command to Coug %i because there is no connection", beacon_id);
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

                modem_e_surface_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surface command sent to Coug %i through modem", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Surface command sent to Coug %i through modem", beacon_id);
                    });
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send surface command to Coug %i because there is no modem connection", beacon_id);
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
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through wifi", vehicle_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through wifi failed", vehicle_id);
                    });
                response->success = true;
            } else if (radio_connection[vehicle_id]) {
                radio_init_client_->async_send_request(request,
                    [this, vehicle_id](rclcpp::Client<base_station_interfaces::srv::Init>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through radio", vehicle_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through radio failed", vehicle_id);
                    });
                response->success = true;
            } else if (modem_connection[vehicle_id]) {

                modem_init_client_->async_send_request(request,
                    [this, vehicle_id](rclcpp::Client<base_station_interfaces::srv::Init>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through modem", vehicle_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Init command sent to Coug %i through modem failed", vehicle_id);
                    });
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send init command to Coug %i because there is no connection", vehicle_id);
                response->success = false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send init command. There is no Vehicle with ID %i", vehicle_id);
            response->success = false;
        }
    }

    // Callback for the status subscriber, publishes the status of a specific vehicle in mission
    // If the vehicle is in the mission, it publishes the status to the appropriate topics
    void publish_status_callback(const std::shared_ptr<base_station_interfaces::msg::Status> msg) {
        int64_t vehicle_id = msg->vehicle_id;
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), vehicle_id) != vehicles_in_mission_.end()) {
            // Publish the status to the appropriate topic
            frost_interfaces::msg::SystemStatus safety_status = msg->safety_status;
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
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr radio_e_kill_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr modem_e_kill_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr modem_e_surface_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr radio_status_request_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr modem_status_request_client_;
    rclcpp::Client<base_station_interfaces::srv::Init>::SharedPtr wifi_init_client_;
    rclcpp::Client<base_station_interfaces::srv::Init>::SharedPtr modem_init_client_;
    rclcpp::Client<base_station_interfaces::srv::Init>::SharedPtr radio_init_client_;

    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_kill_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_surface_service_;
    rclcpp::Service<base_station_interfaces::srv::Init>::SharedPtr init_service_;

    rclcpp::Subscription<base_station_interfaces::msg::Status>::SharedPtr status_subscriber_;
    std::unordered_map<int64_t, rclcpp::Publisher<frost_interfaces::msg::SystemStatus>::SharedPtr> safety_status_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<dvl_msgs::msg::DVLDR>::SharedPtr> dvl_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<dvl_msgs::msg::DVL>::SharedPtr> dvl_vel_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr> battery_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> depth_publishers_;
    std::unordered_map<int64_t, rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr> pressure_publishers_;

    std::unordered_map<int,bool> radio_connection;
    std::unordered_map<int,bool> modem_connection;
    std::unordered_map<int,bool> wifi_connection;

    std::vector<int64_t> vehicles_in_mission_;

    size_t vehicle_id_index = 0;

    double status_request_frequency;
    double modem_status_buffer;
    rclcpp::Time last_modem_status;

};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}