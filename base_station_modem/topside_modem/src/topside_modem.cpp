#include <chrono>
#include <iostream>
#include <stdio.h>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "topside_modem/coms_protocol.hpp"

using namespace narval::seatrac;
using namespace std::chrono;

class MyDriver : public SeatracDriver {
public:
  MyDriver(const std::string &serialPort = "/dev/ttyUSB0")
      : SeatracDriver(serialPort) {}

  void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU) {
    messages::PingSend::Request req;
    req.target = target;
    req.pingType = pingType;

    this->send(sizeof(req), (const uint8_t *)&req);
  }

  // this method is called on any message returned by the beacon.
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {}
};

void send_kill_command(MyDriver &seatrac, int target_id) {
  if (target_id <= 0 || target_id > 16) {
    cougars_coms::EmergencyKill message;
    command::data_send(seatrac, BEACON_ALL, MSG_OWAY, sizeof(message),
                       (uint8_t *)&message);
    std::cout << "Kill command sent to all vehicles" << std::endl;
    std::cout << "Waiting for confirmation responses" << std::endl;
    messages::DataReceive rec;
    auto start_time = steady_clock::now();
    int elapsed_seconds = 0;
    do {
      if (seatrac.wait_for_message(CID_DAT_RECEIVE, &rec, 500) &&
          rec.packetLen == 1 &&
          rec.packetData[0] == cougars_coms::CONFIRM_EMERGENCY_KILL) {
        std::cout << "Thruster kill confirmed from id " << rec.acoFix.srcId
                  << std::endl;
      }
      auto current_time = steady_clock::now();
      elapsed_seconds =
          duration_cast<seconds>(current_time - start_time).count();
    } while (elapsed_seconds <= 4);
  } else {
    cougars_coms::EmergencyKill message;
    command::data_send(seatrac, (BID_E)target_id, MSG_OWAY, sizeof(message),
                       (uint8_t *)&message);
    std::cout << "Kill command sent to id " << target_id << std::endl;
    messages::DataReceive rec;

    auto start_time = steady_clock::now();
    int elapsed_seconds = 0;
    do {
      if (seatrac.wait_for_message(CID_DAT_RECEIVE, &rec, 500) &&
          rec.acoFix.srcId == target_id && rec.packetLen == 1 &&
          rec.packetData[0] == cougars_coms::CONFIRM_EMERGENCY_KILL) {
        std::cout << "Thruster kill confirmed from id " << target_id
                  << std::endl;
        return;
      }
      auto current_time = steady_clock::now();
      elapsed_seconds =
          duration_cast<seconds>(current_time - start_time).count();
    } while (elapsed_seconds <= 4);
    std::cout << "Timed out waiting for thruster kill confirmation from id "
              << target_id << std::endl;
  }
}

int main(int argc, char *argv[]) {

  // TODO add code here to run specific commands

  std::string serial_port;
  if (argc == 1)
    serial_port = "/dev/ttyUSB0";
  else
    serial_port = argv[1];

  MyDriver seatrac(serial_port);

  // second argument is command
  if (argc >= 3 && argv[2][0] == '-' && strlen(argv[2]) == 2) {
    switch (argv[2][1]) {
    default: {
      getchar();
    } break;

    case 'k': {
      int target_id = (argc >= 4) ? std::stoi(argv[3]) : 0;
      send_kill_command(seatrac, target_id);
    } break;
    }
  }

  return 0;
}
