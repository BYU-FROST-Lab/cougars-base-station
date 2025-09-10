
source install/setup.bash

echo "Enter a folder name for the rosbag: "
read folder
echo ""

folder=$folder-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/basestation/$folder -a --include-unpublished-topics

# Copy base station params to bag folder
cp ~/base_station/base-station-ros2/base_station_params.yaml ~/bag/basestation/$folder/
