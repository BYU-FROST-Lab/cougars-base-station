# GUI Sources

## cougars_interfaces
### msg

#### UCommand (name: /coug{coug_number}/controls/command)
- fin calibration

#### SystemControl (name: /coug{coug_number}/system/status)
- start
  - GUI Start Mission Buttons
- rosbag_flag
  - GUI Start Mission Buttons
- rosbag_prefix
  - GUI Start Mission Buttons
- thruster_arm
  - GUI Start Mission Buttons
- dvl_acoustics
  - GUI Start Mission Buttons

#### SystemStatus (name: coug{coug_number}/safety_status)
- std_msgs/Header header
- std_msgs/Int8 depth_status
- std_msgs/Int8 gps_status
  - GUI GPS Icon
- std_msgs/Int8 modem_status
- std_msgs/Int8 dvl_status
  - GUI DVL Icon
- std_msgs/Int8 emergency_status
  - GUI Emergency Status Label
- std_msgs/Int8 sender_id
  - GUI wifi Icon
- std_msgs/Bool imu_published
  - GUI IMU Icon
- std_msgs/Int8 emergency_status
  - GUI Emergency Status Message

## base_station_interfaces
### msg

#### Connections (name: connections)
- connection_type
  - GUI Radio Icon
  - GUI Modem Icon
  - GUI Radio Seconds
  - GUI Modem Seconds
- connections
  - GUI Radio Icon
  - GUI Modem Icon
- last_ping
  - GUI Radio Seconds
  - GUI Modem Seconds
#### ConsoleLog (name:console_log)
- GUI Console Logs (Individual or together)

### srv
##### BeaconId (name: e_surface_service, name: e_kill_service)
- uint8 beacon_id
  - GUI Emergency Shutdown Button
  - GUI Emergency Surface Button
- bool success
  - GUI Emergency Kill Confirmation
  - GUI Emergency Surface Confirmation

## nav_msgs.msg
### Odometry (name: coug{coug_number}/smoothed_ouput)
- twist
  - twist
    - linear
      - GUI DVL Velocity
    - angular
      - GUI angular Velocity
- pose
  - pose
    - orientation
      - w
      - x
      - y
      - z
        - GUI heading
    - position
      - x
        - GUI x position
      - y
        - GUI y position

### Path (name: /coug{coug_number}/map_viz_path)
- mapviz path data

## rcl_interfaces.srv
### SetParameters (name: /coug{coug_number}/coug_kinematics)
- fin calibration

## rcl_interfaces.msg
### Parameter (name: coug{coug_number}_kinematics_client)
- fin calibration set parameters (TODO)

## geometry_msgs.msg
### PoseWithCovarianceStamped (name: coug{coug_number}/depth_data)
- pose
  - pose
    - position
      - z
        - GUI depth

## sensor_msgs.msg
### FluidPressure (name: coug{coug_number}/pressure/data)
- fluid_pressure
  - GUI pressure

### BatteryState (name: coug{coug_number}/battery/data)
- voltage
  - GUI battery voltage
### NavSatFix (name: /map_viz_origin)
- mapviz origin 

## Waypoint (TODO)