
# Cougars Base Station

This repository contains the ROS 2 packages and scripts used on the base station computer to control used by the BYU FRoSt lab's **Cooperative Underwater Group of Autonomous Robots (CoUGARs)** project.

# High-Level Overview of Modules

* **base_station_ros2:**  ROS 2 workspace for the base station computer

    * **base_station_coms:**  Core nodes to communicate with CoUGARs vehicles over wifi, radio, and acoustic beacon.

    * **base_station_gui:**  Opens a Graphical User Interface from which operators can deploy and monitor CoUGARs on a multiagent mission

* **sim:**  Code to deploy a CoUGARs vehicle in HoloOcean sim