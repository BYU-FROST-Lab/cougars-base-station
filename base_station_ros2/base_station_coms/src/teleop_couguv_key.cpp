#include <rclcpp/rclcpp.hpp>
#include <base_station_interfaces/msg/u_command_radio.hpp>
#include <base_station_interfaces/msg/console_log.hpp>
#include <std_msgs/msg/string.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <thread>

#define KEYCODE_SPACE 0x20
#define KEYCODE_DOT 0x2E

class TeleopCommand : public rclcpp::Node
{
public:
  TeleopCommand();
  void keyLoop();

private:
  void switchVehicle();
  void keyPressCallback(const std_msgs::msg::String::SharedPtr msg);
  void processKey(char key);
  void publishCommand();
  void publishConsoleLog(const std::string& message, int vehicle_id = 0);

  double fin1_, fin2_, fin3_, max_fin_value_; 
  int thruster_value_;
  std::string vehicle_name_;
  int vehicle_id_;
  std::vector<int64_t> vehicles_in_mission_;
  size_t current_vehicle_index_;
  base_station_interfaces::msg::UCommandRadio last_command_msg_;
  rclcpp::Publisher<base_station_interfaces::msg::UCommandRadio>::SharedPtr command_pub_;
  rclcpp::Publisher<base_station_interfaces::msg::ConsoleLog>::SharedPtr console_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keypress_sub_;
};

TeleopCommand::TeleopCommand() :
  Node("teleop_command"),
  fin1_(0.0), fin2_(0.0), fin3_(0.0), current_vehicle_index_(0)
{
  declare_parameter("max_fin_value", 35.0);
  declare_parameter("thruster_value", 800);
  declare_parameter("vehicles_in_mission", std::vector<int64_t>{1, 2, 5});

  get_parameter("max_fin_value", max_fin_value_);
  get_parameter("thruster_value", thruster_value_);
  get_parameter("vehicles_in_mission", vehicles_in_mission_);

  // Initialize with first vehicle in the list
  if (!vehicles_in_mission_.empty()) {
    vehicle_id_ = vehicles_in_mission_[current_vehicle_index_];
  } else {
    vehicle_id_ = 1; // Default fallback
    vehicles_in_mission_ = {1}; // Ensure we have at least one vehicle
  }

  command_pub_ = create_publisher<base_station_interfaces::msg::UCommandRadio>("/keyboard_controls", 10);

  // Publisher for console log messages to GUI
  console_pub_ = create_publisher<base_station_interfaces::msg::ConsoleLog>("console_log", 10);

  // Subscribe to GUI key press events
  keypress_sub_ = create_subscription<std_msgs::msg::String>(
    "/gui_keypress", 10,
    std::bind(&TeleopCommand::keyPressCallback, this, std::placeholders::_1)
  );

  // Initialize last command message
  last_command_msg_.vehicle_id = vehicle_id_;
  last_command_msg_.ucommand.header.stamp = get_clock()->now();
  last_command_msg_.ucommand.header.frame_id = vehicle_name_;
  last_command_msg_.vehicle_id = vehicle_id_;
  last_command_msg_.ucommand.fin = {
    -fin1_,  // Send degrees directly (negated for proper direction)
     fin2_,  // Send degrees directly
    -fin3_,  // Send degrees directly (negated for proper direction)
     0.0     // 4th fin (if needed)
  };
  last_command_msg_.ucommand.thruster = static_cast<double>(thruster_value_);

  RCLCPP_INFO(get_logger(), "Initialized teleop for vehicle %d", vehicle_id_);
  
  // Send initial console message to GUI
  publishConsoleLog("Teleop node initialized for vehicle " + std::to_string(vehicle_id_));
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto teleop_command = std::make_shared<TeleopCommand>();

  signal(SIGINT, quit);

  // Start keyboard loop in a separate thread
  std::thread keyboard_thread([teleop_command]() {
    teleop_command->keyLoop();
  });

  // Spin ROS to handle callbacks (including keypress subscriber)
  rclcpp::spin(teleop_command);
  
  // Clean up
  keyboard_thread.join();
  rclcpp::shutdown();
  return 0;
}

void TeleopCommand::keyLoop()
{
  char c;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys OR WASD to control fins");
  puts("W/Up: Fins up, S/Down: Fins down");
  puts("A/Left: Turn left, D/Right: Turn right");
  puts("Spacebar: +thruster, R/Period/Comma: -thruster");
  puts("E: Switch vehicle");
  puts("Press 'q' to quit");
  puts("NOTE: This node also accepts key presses from the GUI via ROS messages");
  
  RCLCPP_INFO(get_logger(), "Currently controlling vehicle %d", vehicle_id_);

  for(;;)
  {
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    // Handle escape sequences for arrow keys
    if (c == 0x1B) // ESC
    {
      char seq[2];
      if (read(kfd, &seq[0], 1) != 1) continue;
      if (read(kfd, &seq[1], 1) != 1) continue;
      
      if (seq[0] == '[')
      {
        switch(seq[1])
        {
          case 'A': // Up arrow
            RCLCPP_INFO(get_logger(), "UP - Fins up");
            fin2_ = std::min(max_fin_value_, fin2_ + 2.0);
            fin3_ = std::min(max_fin_value_, fin3_ + 2.0);
            publishCommand();
            break;
          case 'B': // Down arrow
            RCLCPP_INFO(get_logger(), "DOWN - Fins down");
            fin2_ = std::max(-max_fin_value_, fin2_ - 2.0);
            fin3_ = std::max(-max_fin_value_, fin3_ - 2.0);
            publishCommand();
            break;
          case 'C': // Right arrow
            RCLCPP_INFO(get_logger(), "RIGHT - Turn right");
            fin1_ = std::min(max_fin_value_, fin1_ + 2.0);
            publishCommand();
            break;
          case 'D': // Left arrow
            RCLCPP_INFO(get_logger(), "LEFT - Turn left");
            fin1_ = std::max(-max_fin_value_, fin1_ - 2.0);
            publishCommand();
            break;
        }
      }
    }
    else
    {
      // Handle regular keys
      if (c == 'q' || c == 'Q') {
        RCLCPP_INFO(get_logger(), "Quit requested");
        quit(0);
      } else {
        // Use the common key processing function
        processKey(c);
      }
    }
  }
}

void TeleopCommand::switchVehicle()
{
  current_vehicle_index_ = (current_vehicle_index_ + 1) % vehicles_in_mission_.size();
  vehicle_id_ = vehicles_in_mission_[current_vehicle_index_];
  last_command_msg_.vehicle_id = vehicle_id_;
  
  RCLCPP_INFO(get_logger(), "Switched to controlling vehicle %d", vehicle_id_);
}

void TeleopCommand::publishCommand()
{
  last_command_msg_.ucommand.header.stamp = get_clock()->now();
  last_command_msg_.ucommand.header.frame_id = vehicle_name_;
  last_command_msg_.vehicle_id = vehicle_id_;
  last_command_msg_.ucommand.fin = {
    -fin1_,  // Send degrees directly (negated for proper direction)
     fin2_,  // Send degrees directly
    -fin3_   // Send degrees directly (negated for proper direction)
  };
  last_command_msg_.ucommand.thruster = static_cast<double>(thruster_value_);

  command_pub_->publish(last_command_msg_);
  
  RCLCPP_INFO(get_logger(), "Vehicle %d - Fins: [%.1f, %.1f, %.1f] deg, Thruster: %d", 
              vehicle_id_, fin1_, fin2_, fin3_, thruster_value_);
  
  // Send console log to GUI
  publishConsoleLog("Vehicle " + std::to_string(vehicle_id_) + 
                   " - Fins: [" + std::to_string(fin1_) + ", " + 
                   std::to_string(fin2_) + ", " + std::to_string(fin3_) + 
                   "] deg, Thruster: " + std::to_string(thruster_value_), vehicle_id_);
}

void TeleopCommand::keyPressCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Only process single character messages from ROS
  if (!msg->data.empty() && msg->data.length() == 1) {
    char key = msg->data[0];
    RCLCPP_INFO(get_logger(), "Received key press from GUI: '%c' (0x%02X)", key, static_cast<unsigned char>(key));
    processKey(key);
  } else if (!msg->data.empty()) {
    RCLCPP_WARN(get_logger(), "Received multi-character string from GUI: '%s' - ignoring", msg->data.c_str());
  }
}

void TeleopCommand::processKey(char key)
{
  bool dirty = false;
  
  switch(key) {
    case 'w':
    case 'W':
      RCLCPP_INFO(get_logger(), "W - Fins up");
      fin2_ = std::min(max_fin_value_, fin2_ + 2.0);
      fin3_ = std::min(max_fin_value_, fin3_ + 2.0);
      dirty = true;
      break;
    case 's':
    case 'S':
      RCLCPP_INFO(get_logger(), "S - Fins down");
      fin2_ = std::max(-max_fin_value_, fin2_ - 2.0);
      fin3_ = std::max(-max_fin_value_, fin3_ - 2.0);
      dirty = true;
      break;
    case 'a':
    case 'A':
      RCLCPP_INFO(get_logger(), "A - Turn left");
      fin1_ = std::max(-max_fin_value_, fin1_ - 2.0);
      dirty = true;
      break;
    case 'd':
    case 'D':
      RCLCPP_INFO(get_logger(), "D - Turn right");
      fin1_ = std::min(max_fin_value_, fin1_ + 2.0);
      dirty = true;
      break;
    case ' ': // Space key
      thruster_value_ = std::min(2000, thruster_value_ + 50);
      RCLCPP_INFO(get_logger(), "Spacebar - Increased thruster to %d", thruster_value_);
      dirty = true;
      break;
    case 'r':
    case 'R':
      thruster_value_ = std::max(0, thruster_value_ - 50);
      RCLCPP_INFO(get_logger(), "R - Decreased thruster to %d", thruster_value_);
      dirty = true;
      break;
    case '.':
    case ',':
    case '-':
    case '_':
      thruster_value_ = std::max(0, thruster_value_ - 50);
      RCLCPP_INFO(get_logger(), "Decrease key - Decreased thruster to %d", thruster_value_);
      dirty = true;
      break;
    case 'e':
    case 'E':
      switchVehicle();
      publishConsoleLog("Switched to controlling vehicle " + std::to_string(vehicle_id_));
      break;
    default:
      // Ignore unknown keys
      break;
  }
  
  if (dirty) {
    publishCommand();
  }
}

void TeleopCommand::publishConsoleLog(const std::string& message, int vehicle_id)
{
  auto console_msg = base_station_interfaces::msg::ConsoleLog();
  console_msg.message = message;
  console_msg.vehicle_number = vehicle_id;
  console_pub_->publish(console_msg);
}
