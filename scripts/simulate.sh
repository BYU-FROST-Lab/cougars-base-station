# !/bin/bash

echo you need to fix the path in this script for sourcing the setup file
source install/setup.sh

ros2 topic pub /coug7/safety_status cougars_interfaces/msg/SystemStatus "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  depth_status: {data: 0},
  imu_published: {data: 2},
  gps_status: {data: 0},
  modem_status: {data: 0},
  dvl_status: {data: 0},
  emergency_status: {data: 1},
  sender_id: {data: 1}
}" --once 

ros2 topic pub /coug7/smoothed_output nav_msgs/msg/Odometry "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: 'odom'
  },
  child_frame_id: 'base_link',
  pose: {
    pose: {
      position: { x: 1.0, y: 2.0, z: 0.0 },
      orientation: { x: 0.418, y: 0.502, z: 0.752, w: 0.084 }
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  twist: {
    twist: {
      linear: { x: 0.5, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.1 }
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}" --once

ros2 topic pub /coug7/depth_data geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  pose: {
    pose: {
      position: { x: 1.0, y: 2.0, z: 3.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    },
    covariance: [
      0.1, 0, 0, 0, 0, 0,
      0, 0.1, 0, 0, 0, 0,
      0, 0, 0.1, 0, 0, 0,
      0, 0, 0, 0.1, 0, 0,
      0, 0, 0, 0, 0.1, 0,
      0, 0, 0, 0, 0, 0.1
    ]
  }
}" --once

ros2 topic pub /coug7/pressure/data sensor_msgs/msg/FluidPressure "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: 'pressure_sensor'
  },
  fluid_pressure: 101325.0,
  variance: 0.5
}" --once

# Example: Publishes battery at 75% (0.75)
ros2 topic pub /coug7/battery/data sensor_msgs/msg/BatteryState "{
  percentage: 0.75
}" --once

c1=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
c2=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
c3=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
c4=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
p1=$((RANDOM % 31))
p2=$((RANDOM % 31))
p3=$((RANDOM % 31))
p4=$((RANDOM % 31))
ct=$((RANDOM % 2))

ros2 topic pub -1 /connections base_station_interfaces/msg/Connections "{
  connection_type: 1,
  vehicle_ids: [1, 2, 3, 5],
  connections: [$c1, $c2, $c3, true],
  last_ping: [$p1, $p2, $p3, $p4]
}"

