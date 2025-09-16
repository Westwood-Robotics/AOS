# run_ros2.sh
#!/bin/bash

export PYTHONPATH=$PYTHONPATH:/home/nvidia/miniconda3/envs/groundedsam/lib/python3.10/site-packages
source /opt/ros/humble/setup.bash
pkill -f zhuanhuan_entry.py
# source /home/nvidia/ros2_detection/install/setup.bash
echo "Starting click_to_map_converter.py ..."
python3 ~/ros2_detection/src/click_to_map_converter_entry.py 
