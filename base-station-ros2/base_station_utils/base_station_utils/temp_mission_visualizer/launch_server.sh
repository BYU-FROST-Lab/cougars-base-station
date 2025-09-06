#!/bin/bash
cd "/home/frostlab/base_station/base-station-ros2/src/base_station_utils/base_station_utils/temp_mission_visualizer/mapproxy"
mapproxy-util serve-develop -b 0.0.0.0:8080 mapproxy.yaml