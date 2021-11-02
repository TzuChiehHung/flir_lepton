#!/bin/bash
source /home/pi/ros2_foxy/install/setup.bash
source /home/pi/perception_ws/install/setup.bash
# ros2 launch flir_lepton_data_processor test.launch.py
# ros2 launch image_publisher image_publisher_file.launch.py
ros2 launch rfid_reader test.launch.py
