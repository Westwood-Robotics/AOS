#!/bin/bash
<< 'COMMENT'
__author__    = "Westwoodrobotics"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "April 9, 2025"
__update__    = "Aug 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT

sleep 1s

# go to folder
cd /home/themis/THEMIS/THEMIS

python3 -m Play.Navigation.memory_manager

screen -dmS themis_navigation

echo '====== Shared Memory Created ======'
echo ''

screen -S themis_navigation -X screen -t themis_handle

screen -S themis_navigation -X screen -t navigation_auto

screen -S themis_navigation -X screen -t navigation_apf

screen -S themis_navigation -X screen -t head_zed2i

screen -S themis_navigation -X screen -t body_zedxm


sleep 1s
