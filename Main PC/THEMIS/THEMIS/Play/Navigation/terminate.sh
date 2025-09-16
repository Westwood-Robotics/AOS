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

# terminate threads
for i in $(seq 1 10)
do
  screen -S themis_navigation -p tf_node       -X stuff '^C'
  screen -S themis_navigation -p gamepad_node  -X stuff '^C'
  screen -S themis_navigation -p zed_node      -X stuff '^C'
  screen -S themis_navigation -p navigation    -X stuff '^C'
  screen -S themis_navigation -p update_height -X stuff '^C'
  screen -S themis_navigation -p goal_filter   -X stuff '^C'
  sleep 0.1s
done
