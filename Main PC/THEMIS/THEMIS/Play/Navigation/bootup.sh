#!/bin/bash
<< 'COMMENT'
__author__    = "Westwoodrobotics"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "April 10, 2025"
__update__    = "Aug 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT


# terminate threads
for i in $(seq 1 10)
do
  screen -S themis_navigation -p themis_handle   -X stuff '^C'
  screen -S themis_navigation -p navigation_auto -X stuff '^C'
  screen -S themis_navigation -p navigation_apf  -X stuff '^C'
  screen -S themis_navigation -p head_zed2i      -X stuff '^C'
  screen -S themis_navigation -p body_zedxm      -X stuff '^C'

  
  screen -S themis -p top_level -X stuff '^C'
  
  sleep 0.1s
done

screen -S themis -p top_level -X stuff 'python3 -m Play.Navigation.top_level^M'
sleep 1s
echo '====== Navigation Top-Level Control Online ======'
echo ''





