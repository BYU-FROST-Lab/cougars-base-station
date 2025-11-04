#include <iostream>

#include <seatrac_driver/Calibration.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "toml.hpp"

using namespace std::chrono_literals;
using namespace narval::seatrac;

class MyDriver : public SeatracDriver {
public:
  MyDriver(const std::string &serialPort = "/dev/ttyUSB0")
      : SeatracDriver(serialPort) {}

  // this method is called on any message returned by the beacon.
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {
    switch (msgId) {
    default:
      // std::cout << "Got message : " << msgId << std::endl << std::flush;
      break;

    case CID_STATUS: {
      messages::Status status;
      status = data;
      calibration::printCalFeedback(std::cout, status);
    } break;
    }
  }
};

void skip_cin_line() {
  while (std::cin.get() != '\n')
    ;
}
bool yn_answer() {
  while (true) {
    char yorn;
    if (scanf("%c", &yorn)) {
      if (yorn == 'y') {
        skip_cin_line();
        return true;
      }
      if (yorn == 'n') {
        skip_cin_line();
        return false;
      }
    }
    skip_cin_line();
    std::cout << "Invalid response. Please enter 'y' or 'n': ";
  }
}
void upload_config_settings(
    SeatracDriver &seatrac, SETTINGS_T &settings,
    std::string config_file_path = "./seatrac_config.toml") {

  std::cout << "starting upload config settings to seatrac beacon" << std::endl;

  auto config = toml::parse_file(config_file_path);
  auto seatrac_config = config["SeatracConfig"];

  switch ((int)(seatrac_config["status_report_frequency_hertz"].value_or(0.0) *
                10)) {
  case 0:
    settings.statusFlags = STATUS_MODE_MANUAL;
    break;
  case 10:
    settings.statusFlags = STATUS_MODE_1HZ;
    break;
  case 25:
    settings.statusFlags = STATUS_MODE_2HZ5;
    break;
  case 50:
    settings.statusFlags = STATUS_MODE_5HZ;
    break;
  case 100:
    settings.statusFlags = STATUS_MODE_10HZ;
    break;
  case 250:
    settings.statusFlags = STATUS_MODE_25HZ;
    break;
  default:
    throw std::runtime_error("Seatrac Config Error: value of "
                             "status_report_frequency_hertz is invalid");
  };
  settings.status_output =
      (STATUS_BITS_E)(ENVIRONMENT *
                          seatrac_config
                              ["status_include_temp_pressure_depth_vos"]
                                  .value_or(false) |
                      ATTITUDE * seatrac_config["status_include_yaw_pitch_roll"]
                                     .value_or(false) |
                      MAG_CAL * seatrac_config["status_include_mag_cal_data"]
                                    .value_or(false) |
                      ACC_CAL * seatrac_config["status_include_accel_cal_data"]
                                    .value_or(false) |
                      AHRS_RAW_DATA *
                          seatrac_config
                              ["status_include_uncompensated_accel_mag_gyro"]
                                  .value_or(false) |
                      AHRS_COMP_DATA *
                          seatrac_config
                              ["status_include_compensated_accel_mag_gyro"]
                                  .value_or(false));

  settings.envFlags =
      (ENV_FLAGS_E)(AUTO_VOS *
                        seatrac_config["auto_calc_velocity_of_sound"].value_or(
                            true) |
                    AUTO_PRESSURE_OFS *
                        seatrac_config["auto_calc_pressure_offset"].value_or(
                            true));
  // settings.envPressureOfs = 0; //value will be overwritten if
  // auto_calc_pressure_offset is true settings.envVos = 0; //value will be
  // overwritten if auto_calc_velocity_of_sound is true

  settings.envSalinity =
      (uint8_t)(10 * seatrac_config["env_salinity_ppt"].value_or(0.0));

  settings.ahrsFlags =
      (AHRS_FLAGS_E)seatrac_config["automatic_mag_calibration"].value_or(false);

  settings.ahrsYawOfs = 0;
  settings.ahrsPitchOfs = 0;
  settings.ahrsRollOfs = 0;

  XCVR_TXMSGCTRL_E msgctrl =
      seatrac_config["transceiver_block_send_all"].value_or(false)
          ? XCVR_TXMSG_BLOCK_ALL
          : (seatrac_config["transceiver_block_send_response"].value_or(false)
                 ? XCVR_TXMSG_BLOCK_RESP
                 : XCVR_TXMSG_ALLOW_ALL);

  settings.xcvrFlags =
      (XCVR_FLAGS_E)(USBL_USE_AHRS //* seatrac_config["usbl_use_AHRS"].value_or(true)
                     | XCVR_POSFLT_ENABLE *
                           seatrac_config["position_filter_enabled"].value_or(
                               true) |
                     XCVR_USBL_MSGS *
                         seatrac_config["report_transceiver_usbl_msgs"]
                             .value_or(false) |
                     XCVR_FIX_MSGS *
                         seatrac_config["report_transceiver_fix_msgs"].value_or(
                             false) |
                     XCVR_DIAG_MSGS *
                         seatrac_config["report_transceiver_msg_diagnostics"]
                             .value_or(false) |
                     (msgctrl << 3) // XCVR_TXMSGCTRL_E takes up the 3rd and 4th
                                    // bits of xcvrFlags
      );

  settings.xcvrBeaconId = (BID_E)seatrac_config["beacon_id"].value_or(1);
  if (settings.xcvrBeaconId < 1 || settings.xcvrBeaconId > 15)
    throw std::runtime_error(
        "Seatrac Config Error: beacon_id must be between 1 and 15");

  settings.xcvrRangeTmo =
      (uint16_t)seatrac_config["transceiver_range_timeout_meters"].value_or(
          1000);
  if (settings.xcvrRangeTmo > 3000 || settings.xcvrRangeTmo < 1000)
    throw std::runtime_error(
        "Seatrac Config Error: transceiver_range_timeout_meters must be "
        "between 1000 and 3000 m");
  settings.xcvrRespTime =
      (uint16_t)seatrac_config["transceiver_response_delay_milliseconds"]
          .value_or(10);
  if (settings.xcvrRespTime > 1000 || settings.xcvrRespTime < 10)
    throw std::runtime_error(
        "Seatrac Config Error: transceiver_response_delay_milliseconds must be "
        "between 10 and 1000 ms");

  settings.xcvrPosfltVel =
      (uint8_t)(seatrac_config["pos_filter_velocity_limit_meters_per_sec"]
                    .value_or(3));
  settings.xcvrPosfltAng =
      (uint8_t)(seatrac_config["pos_filter_angle_limit_degrees"].value_or(10));
  settings.xcvrPosfltTmo =
      (uint8_t)(seatrac_config["pos_filter_timeout_seconds"].value_or(60));

  // std::cout << settings << std::endl;

  std::cout << "uploading settings to beacon" << std::endl;
  messages::SettingsSet resp = command::settings_set(seatrac, settings);
  if (resp.statusCode != CST_OK)
    throw std::runtime_error(
        "Seatrac Config Error: Error saving settings to seatrac beacon");
  std::cout << "Automatic Config Settings upload complete." << std::endl
            << std::endl;
  return;
}
void manual_set_settings(MyDriver &seatrac, SETTINGS_T &settings) {
  // Change beacon id
  std::cout << "Current Beacon Id: " << (int)settings.xcvrBeaconId << std::endl
            << "Change Beacon Id (y/n)? ";
  if (yn_answer()) {
    int bid;
    while (true) {
      std::cout << "Enter New Beacon Id (integer between 1 and 15 inclusive): ";
      if (scanf("%d", &bid) && bid <= 15 && bid >= 1)
        break;
      skip_cin_line();
      std::cout << "Invalid Beacon Id. Id should be an integer between 1 and "
                   "15 inclusive."
                << std::endl;
    }
    skip_cin_line();
    std::cout << "Setting Beacon Id to " << bid << "... ";
    settings.xcvrBeaconId = (BID_E)bid;
    command::settings_set(seatrac, settings);
    std::cout << "done" << std::endl << std::endl;
  }

  // Change Env Salinity Settings
  std::cout
      << "Current Water Salinity Setting: " << settings.envSalinity / 10.0
      << " ppt" << std::endl
      << "Fresh water has salinity of 0 ppt. Salt water has salinity of 35 ppt."
      << std::endl
      << "Change Water Salinity Setting (y/n)? ";
  if (yn_answer()) {
    float sal;
    while (true) {
      std::cout << "Enter New Salinity (float): ";
      if (scanf("%f", &sal))
        break;
      skip_cin_line();
      std::cout << "Invalid Salinity. Salinity should be a float." << std::endl;
    }
    skip_cin_line();
    std::cout << "Setting Salinity to " << sal << " ppt... ";
    settings.envSalinity = (int)(sal * 10);
    command::settings_set(seatrac, settings);
    std::cout << "done" << std::endl << std::endl;
  }

  // Change Status Report Settings
  std::cout << "View and modify serial report settings (y/n)? ";
  if (yn_answer()) {
    std::cout << "Current Status Report Frequency: " << settings.statusFlags
              << std::endl
              << "Change Status Report frequency (y/n)? ";
    if (yn_answer()) {
      int val;
      while (true) {
        std::cout << "Select Status frequency from list:" << std::endl
                  << "\t1) Manual - 0 Hz" << std::endl
                  << "\t2) 1 Hz" << std::endl
                  << "\t3) 2.5 Hz" << std::endl
                  << "\t4) 5 Hz" << std::endl
                  << "\t5) 10 Hz" << std::endl
                  << "\t6) 25 Hz" << std::endl
                  << "Enter a number from 1 to 6: ";
        if (scanf("%d", &val) && val <= 6 && val >= 1)
          break;
        skip_cin_line();
        std::cout << "Invalid Selection. Options are from 1 to 6." << std::endl;
      }
      skip_cin_line();
      settings.statusFlags = (STATUSMODE_E)(val - 1);
    }
    std::cout << "Current fields included in status output: "
              << settings.status_output << std::endl
              << "Modify status output (y/n)? ";
    if (yn_answer()) {
      settings.status_output = (STATUS_BITS_E)0x00;
      std::cout << "Include temperature, pressure, depth and velocity in "
                   "status report (y/n)? ";
      settings.status_output =
          settings.status_output | (STATUS_BITS_E)(yn_answer() * ENVIRONMENT);
      std::cout << "Include yaw pitch and roll in status report (y/n)? ";
      settings.status_output =
          settings.status_output | (STATUS_BITS_E)(yn_answer() * ATTITUDE);
      std::cout << "Include accelerometer, magnetometer, and gyroscope sensor "
                   "values in status report (y/n)? ";
      settings.status_output = settings.status_output |
                               (STATUS_BITS_E)(yn_answer() * AHRS_COMP_DATA);
    }

    std::cout << "Transciever serial reports: " << std::endl
              << "  - transceiver usbl messages: "
              << (bool)(settings.xcvrFlags & XCVR_USBL_MSGS) << std::endl
              << "  - transceiver fix messages: "
              << (bool)(settings.xcvrFlags & XCVR_FIX_MSGS) << std::endl
              << "  - transceiver diagnostic messages: "
              << (bool)(settings.xcvrFlags & XCVR_DIAG_MSGS) << std::endl
              << "If true, these reports will be sent in addition to the "
                 "higher protocol reports such as PING or DAT."
              << std::endl
              << "Modify transceiver serial report settings (y/n)? ";
    if (yn_answer()) {
      settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags & 0x1F);
      std::cout << "report transceiver usbl messages (y/n)? ";
      if (yn_answer())
        settings.xcvrFlags =
            (XCVR_FLAGS_E)(settings.xcvrFlags | XCVR_USBL_MSGS);
      std::cout << "report transceiver fix messages (y/n)? ";
      if (yn_answer())
        settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags | XCVR_FIX_MSGS);
      std::cout << "report transceiver diagnostic messages (y/n)? ";
      if (yn_answer())
        settings.xcvrFlags =
            (XCVR_FLAGS_E)(settings.xcvrFlags | XCVR_DIAG_MSGS);
    }

    std::cout << "Saving serial report settings... ";
    command::settings_set(seatrac, settings);
    std::cout << "done" << std::endl;
  }

  std::cout << "View and modify transciever and sensor settings (y/n)? ";
  if (yn_answer()) {
    std::cout << "Use position filter: "
              << ((settings.xcvrFlags & XCVR_POSFLT_ENABLE) ? "true" : "false")
              << std::endl
              << "Use ahrs for usbl position: "
              << ((settings.xcvrFlags & USBL_USE_AHRS) ? "true" : "false")
              << std::endl
              << "Automatic pressure offset calculation: "
              << ((settings.envFlags & AUTO_PRESSURE_OFS) ? "true" : "false")
              << std::endl
              << "Automatic velocity of sound calculation: "
              << ((settings.envFlags & AUTO_VOS) ? "true" : "false")
              << std::endl
              << "automatic magnetometer calibration: "
              << (settings.ahrsFlags ? "true" : "false") << std::endl;
    std::cout << "Modify any of these settings (y/n)? ";
    if (yn_answer()) {
      settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags & 0xE0);
      std::cout << "Use position filter (y/n)? ";
      settings.xcvrFlags =
          (XCVR_FLAGS_E)(settings.xcvrFlags | yn_answer() * XCVR_POSFLT_ENABLE);
      std::cout << "Use Mag, Accel and Gyro (arhs) to calculate usbl position "
                   "(y/n)? ";
      settings.xcvrFlags =
          (XCVR_FLAGS_E)(settings.xcvrFlags | yn_answer() * USBL_USE_AHRS);
      std::cout << "Automatic pressure offset calculation (y/n)? ";
      bool auto_p_ofs = yn_answer();
      std::cout << "Automatic velocity of sound calculation (y/n)? ";
      settings.envFlags = (ENV_FLAGS_E)(yn_answer() * AUTO_VOS |
                                        auto_p_ofs * AUTO_PRESSURE_OFS);
      std::cout << "Automatic Mag Calibration (y/n)? ";
      settings.ahrsFlags = (AHRS_FLAGS_E)yn_answer();
    }
    std::cout << "Saving transciever and sensor settings... ";
    command::settings_set(seatrac, settings);
    std::cout << "done" << std::endl << std::endl;
  }

  // TODO: these settings can be easy to mess up and really shouldn't change at
  // all
  //  Not sure if I should give the user control over this section or not.
  settings.xcvrRangeTmo = 1000;
  settings.xcvrRespTime = 10;
  settings.xcvrPosfltVel = 3;
  settings.xcvrPosfltAng = 10;
  settings.xcvrPosfltTmo = 60;

  command::settings_set(seatrac, settings);
  std::cout << "Manual Settings upload complete." << std::endl << std::endl;
}

int main(int argc, char *argv[]) {

  std::cout << "=== Seatrac Beacon Setup Tool ===" << std::endl << std::endl;

  bool cont = true;
  while (cont) {
    std::cout << "Enter Serial Port (or blank for default '/dev/ttyUSB0'): ";
    char serial_port[30];
    fgets(serial_port, sizeof(serial_port), stdin);
    serial_port[strlen(serial_port) - 1] = 0x00;
    if (strlen(serial_port) == 0)
      strcpy(serial_port, "/dev/ttyUSB0");

    {
      std::cout << "Connecting to Beacon... ";
      MyDriver seatrac(serial_port);
      SETTINGS_T origional_settings = command::settings_get(seatrac).settings;
      SETTINGS_T settings = origional_settings;
      command::status_config_set(seatrac, (STATUS_BITS_E)0x0);
      std::cout << "Done" << std::endl;

      std::cout << "View current settings (y/n)? ";
      if (yn_answer())
        std::cout << settings << std::endl << std::endl;

      std::cout << "How would you like to enter beacon settings?" << std::endl
                << "  'm': manual,  'c': config file,  's': skip to "
                   "calibration  (m/c/s)? ";
      while (true) {
        char ans;
        if (scanf("%c", &ans)) {
          if (ans == 'm') {
            skip_cin_line();
            manual_set_settings(seatrac, settings);
            break;
          }
          if (ans == 'c') {
            skip_cin_line();
            std::cout << "Enter config file path (or blank for default "
                         "'./seatrac_config.toml'): ";
            char config_path[100];
            fgets(config_path, sizeof(config_path), stdin);
            config_path[strlen(config_path) - 1] = 0x00;
            if (strlen(config_path) == 0)
              strcpy(config_path, "./seatrac_config.toml");
            upload_config_settings(seatrac, settings, config_path);
            // Change beacon id
            std::cout << "Current Beacon Id: " << (int)settings.xcvrBeaconId
                      << std::endl
                      << "Change Beacon Id (y/n)? ";
            if (yn_answer()) {
              int bid;
              while (true) {
                std::cout << "Enter New Beacon Id (integer between 1 and 15 "
                             "inclusive): ";
                if (scanf("%d", &bid) && bid <= 15 && bid >= 1)
                  break;
                skip_cin_line();
                std::cout << "Invalid Beacon Id. Id should be an integer "
                             "between 1 and 15 inclusive."
                          << std::endl;
              }
              skip_cin_line();
              std::cout << "Setting Beacon Id to " << bid << "... ";
              settings.xcvrBeaconId = (BID_E)bid;
              command::settings_set(seatrac, settings);
              std::cout << "done" << std::endl << std::endl;
            }
            break;
          }
          if (ans == 's') {
            skip_cin_line();
            std::cout << "skipping to calibration" << std::endl;
            break;
          }
        }
        skip_cin_line();
        std::cout << "Invalid response. Please enter 'm', 'c', or 's': ";
      }

      std::cout << "Calibrate Magnetometer (y/n)? ";
      if (yn_answer()) {
        calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, false);
      }
      std::cout << "Calibrate Accelerometer (y/n)? ";
      if (yn_answer()) {
        calibration::calibrateAccelerometer(seatrac, std::cout, std::cin,
                                            false);
      }

      std::cout << "Beacon setup complete" << std::endl
                << "Review changes to settings (y/n)? ";
      if (yn_answer()) {
        std::cout << std::endl
                  << "== Origional Settings ==" << std::endl
                  << origional_settings << std::endl
                  << std::endl
                  << "== New Settings ==" << std::endl
                  << command::settings_get(seatrac).settings << std::endl
                  << std::endl;
      }

      std::cout << "Save Settings to permanent EEPROM memory? " << std::endl
                << "If you chose not to, settings will still be saved in "
                   "beacon ram. (y/n)? ";
      if (yn_answer()) {
        std::cout << "Saving Settings... ";
        command::settings_save(seatrac);
        std::cout << "done" << std::endl;
      }

      std::cout << std::endl << "Beacon setup complete" << std::endl;
    }

    std::cout << std::endl << "Setup another beacon (y/n)? ";
    cont = yn_answer();
  }
}
