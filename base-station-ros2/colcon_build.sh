
# USE THIS instead of "colcon build" to build the base-station

# base-station-ros2 uses the seatrac and seatrac_interfaces 
# packages from cougars-ros2. So to build the package, we need 
# to overlay the build in cougars-ros2 onto base-station-ros2. 
# This script is designed to run in the docker container and
# assumes ros is already sourced.

# build required packages in cougars-ros2 and source
cd ~/ros2_ws
colcon build --packages-select seatrac seatrac_interfaces frost_interfaces dvl_msgs
source install/setup.bash

# Build packages in base station. "--merge-install" tells 
# colcon to include the packages from cougars-ros2 into
# base-station-ros2
cd ~/base_station/base-station-ros2
colcon build --merge-install
source install/setup.bash
