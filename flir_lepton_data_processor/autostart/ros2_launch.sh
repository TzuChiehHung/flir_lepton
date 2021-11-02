#!/bin/bash

source /home/pi/perception_ws/install/setup.bash
ros2 launch flir_lepton_data_processor test.launch.py &
