#!/bin/bash
<< 'COMMENT'
__author__    = "Westwoodrobotics"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "February 12, 2025"
__update__    = "Aug 26, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT

sleep 1s
#start zedxm
screen -S themis_navigation -p body_zedxm -X stuff "sshpass -p nvidia ssh nvidia@body007.local './restart_zedxm.sh'\n"

sleep 5s

#restart zed2i

screen -S themis_navigation -p head_zed2i -X stuff "sshpass -p nvidia ssh nvidia@head007.local './restart_zed2i.sh'\n"

sleep 10s

#start themis_navigation handles
screen -S themis_navigation -p themis_handle -X stuff "source ~/.bashrc && ros2 launch themis_navigation themis_handle.launch.py\n"








