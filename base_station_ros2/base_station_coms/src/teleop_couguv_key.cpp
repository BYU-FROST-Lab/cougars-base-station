#include <rclcpp/rclcpp.hpp>
#include <cougars_interfaces/msg/u_command.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <string>

#define KEYCODE_SPACE 0x20
#define KEYCODE_DOT 0x2E

class TeleopCommand : public rclcpp::Node
{
public:
  TeleopCommand();
  void keyLoop();

private:
  void timerCallback();

  double fin1_, fin2_, fin3_, max_fin_value_; 
  int thruster_value_;
  std::string vehicle_name_;
  cougars_interfaces::msg::UCommand last_command_msg_;
  rclcpp::Publisher<cougars_interfaces::msg::UCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

TeleopCommand::TeleopCommand() :
  Node("teleop_command"),
  fin1_(0.0), fin2_(0.0), fin3_(0.0)
{
  declare_parameter("max_fin_value", 35.0);
  declare_parameter("thruster_value", 800);
  declare_parameter("holoocean_vehicle", "auv0");

  get_parameter("max_fin_value", max_fin_value_);
  get_parameter("thruster_value", thruster_value_);
  get_parameter("holoocean_vehicle", vehicle_name_);

  command_pub_ = create_publisher<cougars_interfaces::msg::UCommand>("/coug2/controls/command", 10);

  // Initialize last command message
  last_command_msg_.header.stamp = get_clock()->now();
  last_command_msg_.header.frame_id = vehicle_name_;
  last_command_msg_.fin = {
    -fin1_,  // Send degrees directly (negated for proper direction)
     fin2_,  // Send degrees directly
    -fin3_,  // Send degrees directly (negated for proper direction)
     0.0     // 4th fin (if needed)
  };
  last_command_msg_.thruster = static_cast<double>(thruster_value_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(1500),
    std::bind(&TeleopCommand::timerCallback, this)
  );
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

  teleop_command->keyLoop();
  
  rclcpp::spin(teleop_command);
  rclcpp::shutdown();
  return 0;
}

void TeleopCommand::keyLoop()
{
  char c;
  bool dirty = false;

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
  puts("Press 'q' to quit");

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
            dirty = true;
            break;
          case 'B': // Down arrow
            RCLCPP_INFO(get_logger(), "DOWN - Fins down");
            fin2_ = std::max(-max_fin_value_, fin2_ - 2.0);
            fin3_ = std::max(-max_fin_value_, fin3_ - 2.0);
            dirty = true;
            break;
          case 'C': // Right arrow
            RCLCPP_INFO(get_logger(), "RIGHT - Turn right");
            fin1_ = std::min(max_fin_value_, fin1_ + 2.0);
            dirty = true;
            break;
          case 'D': // Left arrow
            RCLCPP_INFO(get_logger(), "LEFT - Turn left");
            fin1_ = std::max(-max_fin_value_, fin1_ - 2.0);
            dirty = true;
            break;
        }
      }
    }
    else
    {
      // Handle regular keys
      switch(c)
      {
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
        case KEYCODE_SPACE:
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
        case KEYCODE_DOT:
        case ',':  // Comma key (easier to reach than period)
        case '-':  // Minus key 
        case '_':  // Underscore (Shift + minus)
          thruster_value_ = std::max(0, thruster_value_ - 50);
          RCLCPP_INFO(get_logger(), "Decrease key - Decreased thruster to %d", thruster_value_);
          dirty = true;
          break;
        case 'q':
        case 'Q':
          RCLCPP_INFO(get_logger(), "Quit requested");
          quit(0);
          break;
        default:
          // Ignore other keys
          break;
      }
    }

    if (dirty)
    {
      last_command_msg_.header.stamp = get_clock()->now();
      last_command_msg_.header.frame_id = vehicle_name_;
      last_command_msg_.fin = {
        -fin1_,  // Send degrees directly (negated for proper direction)
         fin2_,  // Send degrees directly
        -fin3_  // Send degrees directly (negated for proper direction)
      };
      last_command_msg_.thruster = static_cast<double>(thruster_value_);

      command_pub_->publish(last_command_msg_);
      
      RCLCPP_INFO(get_logger(), "Fins: [%.1f, %.1f, %.1f] deg, Thruster: %d", 
                  fin1_, fin2_, fin3_, thruster_value_);
      
      dirty = false;
    }
  }
}

void TeleopCommand::timerCallback()
{
  last_command_msg_.header.stamp = get_clock()->now();
  command_pub_->publish(last_command_msg_);
}
