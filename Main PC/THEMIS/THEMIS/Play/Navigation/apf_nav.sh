#!/bin/bash
<< 'COMMENT'
__author__    = "Westwoodrobotics"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "Aug 11, 2025"
__update__    = "Aug 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT

sleep 1s

screen -S themis_navigation -p navigation_auto -X stuff '^C'

sleep 10s

screen -S themis_navigation -p navigation_apf -X stuff "source ~/.bashrc && ros2 launch navigation navigation_apf.launch.py\n"
