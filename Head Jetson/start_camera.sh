#!/bin/bash
source ~/.bashrc
# export DISPLAY=:0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed2i namespace:=zed2i
