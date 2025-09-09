
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"
#include "base_station_interfaces/msg/status.hpp"
#include "base_station_interfaces/msg/connections.hpp"
#include "std_msgs/msg/bool.hpp"
#include "base_station_interfaces/msg/console_log.hpp"
#include "base_station_interfaces/srv/init.hpp"



#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"


#include <iostream>
#include <chrono>
#include <memory>


using namespace std::literals::chrono_literals;
using namespace cougars_coms;
using namespace narval::seatrac;


using std::placeholders::_1;
using std::placeholders::_2;


class ModemComs : public rclcpp::Node {
public:
    ModemComs() : Node("base_station_modem") {

        // id of the base station beacon, used to identify the base station in the network
        this->declare_parameter<int>("base_station_beacon_id", 15);
        this->base_station_beacon_id_ = this->get_parameter("base_station_beacon_id").as_int();

        // lsit of beacon ids of vehicles in mission
        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();

        //subscriber to ModemRec messages, which are received from the cougs
        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ModemComs::listen_to_modem, this, _1)
        );
        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);

        // service that requests the status of a vehicle specified in the request
        request_status_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "modem_status_request",
            std::bind(&ModemComs::status_request_callback, this, _1, _2)
        );

        // service that sends an emergency kill command to a vehicle specified in the request
        emergency_kill_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "modem_e_kill",
            std::bind(&ModemComs::emergency_kill_callback, this, _1, _2)
        );

        // service that sends an emergency surface command to a vehicle specified in the request
        emergency_surface_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "modem_e_surface",
            std::bind(&ModemComs::emergency_surface_callback, this, _1, _2)
        );

        // service that sends an init command to a vehicle specified in the request
        init_service_ = this->create_service<base_station_interfaces::srv::Init>(
            "modem_init",
            std::bind(&ModemComs::init_callback, this, _1, _2)
        );

        // publisher for the status of the cougs, published to the status topic
        this->status_publisher_ = this->create_publisher<base_station_interfaces::msg::Status>("status", 10);

        // publisher for the connections of the vehicles in the mission via modem
        this->modem_connections_publisher_ = this->create_publisher<base_station_interfaces::msg::Connections>("connections", 10);

        // publisher for the confirmation of the emergency kill command
        this->print_to_gui_pub = this->create_publisher<base_station_interfaces::msg::ConsoleLog>("console_log", 10);

        RCLCPP_INFO(this->get_logger(), "base station coms node started");

        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&ModemComs::check_modem_connections, this)
        );

        for (int vehicle : vehicles_in_mission_) {
            this->modem_connection[vehicle] = true;
            this->messages_missed_[vehicle] = this->max_missed_messages-1;
            this->last_message_time_[vehicle] = this->now();
        }

    }

    // listens to ModemRec message and processes msg according to the msg id
    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        last_message_time_[id] = this->now();

        switch(id) {
            default: break;
            case EMPTY: break;
            case VEHICLE_STATUS:{
                recieve_status(&msg);
            } break;
            case CONFIRM_EMERGENCY_KILL: {
                emergency_kill_confirmed(&msg);
            } break;
            case CONFIRM_EMERGENCY_SURFACE: {
                emergency_surface_confirmed(&msg);
            } break;
        }
    }

    

    // Sends emergency kill signal to coug specified in request
    void emergency_kill_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)
    {
        EmergencyKill e_kill_msg;
        send_acoustic_message(request->beacon_id, sizeof(e_kill_msg), (uint8_t*)&e_kill_msg, MSG_OWAY);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Kill Signal Sent to Coug %i", request->beacon_id);
        response->success = true;
    }


    // Sends emergency surface signal to coug specified in request
    void emergency_surface_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)      
    {
        EmergencySurface e_surface_msg;
        send_acoustic_message(request->beacon_id, sizeof(e_surface_msg), (uint8_t*)&e_surface_msg, MSG_OWAY);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Surface Signal Sent to Coug %i", request->beacon_id);
        response->success = true;
    }

    // requests status of coug specified in request
    void status_request_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting Status of Coug %i", request->beacon_id);
        this->messages_missed_[request->beacon_id]++;

        RequestStatus request_status_msg;
        send_acoustic_message(request->beacon_id, sizeof(request_status_msg), (uint8_t*)&request_status_msg, MSG_OWAY);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting Status of Coug %i", request->beacon_id);
        response->success = true;
    }

    void init_callback(const std::shared_ptr<base_station_interfaces::srv::Init::Request> request,
                        std::shared_ptr<base_station_interfaces::srv::Init::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init command received for Coug %i", request->vehicle_id);

        Init init_msg;
        init_msg.init_bitmask = (request->start.data ? 0x01 : 0x00) | (request->rosbag_flag.data ? 0x02 : 0x00) | (request->thruster_arm.data ? 0x04 : 0x00) | (request->dvl_acoustics.data ? 0x08 : 0x00);
        strncpy(init_msg.rosbag_prefix, request->rosbag_prefix.c_str(), sizeof(init_msg.rosbag_prefix) - 1);
        init_msg.rosbag_prefix[sizeof(init_msg.rosbag_prefix) - 1] = '\0';  // Ensure null termination

        send_acoustic_message(request->vehicle_id, sizeof(init_msg), (uint8_t*)&init_msg, MSG_OWAY);
        response->success = true;
    }

    // publishes the status recieved through the modem
    void recieve_status(seatrac_interfaces::msg::ModemRec* msg) {

        this->messages_missed_[msg->src_id] = 0;
        this->last_message_time_[msg->src_id] = this->now();

        const VehicleStatus* status = reinterpret_cast<const VehicleStatus*>(msg->packet_data.data());
        auto status_msg = base_station_interfaces::msg::Status();

        // Fill in the status message from the data received
        status_msg.vehicle_id = msg->src_id;
        status_msg.safety_status.depth_status.data = (status->safety_mask & 0x01) != 0;
        status_msg.safety_status.gps_status.data = (status->safety_mask & 0x02) != 0;
        status_msg.safety_status.modem_status.data = (status->safety_mask & 0x04) != 0;
        status_msg.safety_status.dvl_status.data = (status->safety_mask & 0x08) != 0;
        status_msg.safety_status.emergency_status.data = (status->safety_mask & 0x10) != 0;
        status_msg.dvl_pos.position.x = status->x;
        status_msg.dvl_pos.position.y = status->y;
        status_msg.dvl_pos.position.z = status->depth;
        status_msg.dvl_pos.roll = status->roll;
        status_msg.dvl_pos.pitch = status->pitch;
        status_msg.dvl_pos.yaw = status->yaw;
        status_msg.dvl_vel.velocity.x = status->x_vel;
        status_msg.dvl_vel.velocity.y = status->y_vel;
        status_msg.dvl_vel.velocity.z = status->z_vel;
        status_msg.battery_state.voltage = status->battery_voltage;
        status_msg.battery_state.percentage = status->battery_percentage;
        status_msg.depth_data.pose.pose.position.z = status->depth;
        status_msg.pressure.fluid_pressure = status->pressure;

        this->status_publisher_->publish(status_msg);

    RCLCPP_INFO(this->get_logger(), "position (x, y, z): (%.2f, %.2f, %.2f)", 
        static_cast<double>(status->x), 
        static_cast<double>(status->y), 
        static_cast<double>(status->depth));
    RCLCPP_INFO(this->get_logger(), "velocity (x, y): (%.2f, %.2f)", 
        static_cast<double>(status->x_vel), 
        static_cast<double>(status->y_vel));
    RCLCPP_INFO(this->get_logger(), "battery voltage: %.2f", static_cast<double>(status->battery_voltage));
    RCLCPP_INFO(this->get_logger(), "battery percentage: %.2f", static_cast<double>(status->battery_percentage));
    RCLCPP_INFO(this->get_logger(), "pressure: %.2f", static_cast<double>(status->pressure));

    }
 
    // publishes succes or failure of emergency kill command
    void emergency_kill_confirmed(seatrac_interfaces::msg::ModemRec* msg){
        std::string message;

        if (msg->packet_data[1]){
            message = "Emergency kill command was successful for Coug " + std::to_string(msg->src_id);
        } else {
            message = "Emergency kill command failed for Coug " + std::to_string(msg->src_id);
        }
        base_station_interfaces::msg::ConsoleLog log_msg;
        log_msg.message = message;
        log_msg.vehicle_number = msg->src_id;
        this->print_to_gui_pub->publish(log_msg);
    }

    // publishes success or failure of emergency surface command
    void emergency_surface_confirmed(seatrac_interfaces::msg::ModemRec* msg){
        std::string message;

        if (msg->packet_data[1]){
            message = "Emergency surface command was successful for Coug " + std::to_string(msg->src_id);
        } else {
            message = "Emergency surface command failed for Coug " + std::to_string(msg->src_id);
        }
        base_station_interfaces::msg::ConsoleLog log_msg;
        log_msg.message = message;
        log_msg.vehicle_number = msg->src_id;
        this->print_to_gui_pub->publish(log_msg);
    }

   // checks the connections of the vehicles in the mission and publishes the connections
    void check_modem_connections() {
        rclcpp::Time now = this->now();
        std::vector<uint64_t> last_ping;


        base_station_interfaces::msg::Connections msg;
        msg.connection_type = 0; // 0 for acoustic modem
        for (auto id : this->vehicles_in_mission_) {
            if (this->messages_missed_[id] >= this->max_missed_messages) {
                if (this->modem_connection[id]) {
                    RCLCPP_WARN(this->get_logger(), "Coug %i has missed %i or more messages, marking as disconnected", id, this->max_missed_messages);
                    this->modem_connection[id] = false;
                }
                msg.connections.push_back(false);
            } else {
                msg.connections.push_back(true);
                this->modem_connection[id] = true;
            }
            msg.last_ping.push_back(static_cast<uint64_t>(this->now().seconds() - last_message_time_[id].seconds()));
        }
        msg.vehicle_ids = this->vehicles_in_mission_;

        modem_connections_publisher_->publish(msg);
   }

   
    //used by the service callback functions to publish messages to the cougs
    void send_acoustic_message(int target_id, int message_len, uint8_t* message, AMSGTYPE_E msg_type) {


        auto request = seatrac_interfaces::msg::ModemSend();
        request.msg_id = CID_DAT_SEND;
        request.dest_id = (uint8_t)target_id;
        request.msg_type = msg_type;
        request.packet_len = (uint8_t)std::min(message_len, 31);
        // request.insert_timestamp = true;
        std::memcpy(&request.packet_data, message, request.packet_len);
       
        this->modem_publisher_->publish(request);
    }




private:


    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;

    rclcpp::Publisher<base_station_interfaces::msg::Status>::SharedPtr status_publisher_;
    rclcpp::Publisher<base_station_interfaces::msg::Connections>::SharedPtr modem_connections_publisher_;
    rclcpp::Publisher<base_station_interfaces::msg::ConsoleLog>::SharedPtr print_to_gui_pub;

    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_surface_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_kill_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr request_status_service_;
    rclcpp::Service<base_station_interfaces::srv::Init>::SharedPtr init_service_;


    rclcpp::TimerBase::SharedPtr timer_;


    std::vector<int64_t> vehicles_in_mission_;

    std::unordered_map<int, rclcpp::Time> last_message_time_;



    size_t modem_coms_schedule_turn_index = 0;  


    int status_request_frequency;


    int base_station_beacon_id_;

    int max_missed_messages = 2;

    std::unordered_map<int,bool> modem_connection;

    std::unordered_map<int, int> messages_missed_; // keeps track of how many messages have been missed for each coug




};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ModemComs>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}
