#!/bin/bash
<< COMMENT
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 29, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT


# nvidia
sshpass -p nvidia ssh nvidia@head007.local 'echo nvidia | sudo -S shutdown -h now'
sshpass -p nvidia ssh nvidia@body007.local 'echo nvidia | sudo -S shutdown -h now'

# terminate threads
for i in $(seq 1 10)
do
  screen -S themis -p bear_right_leg -X stuff '^C'
  screen -S themis -p bear_left_leg  -X stuff '^C'
  screen -S themis -p bear_right_arm -X stuff '^C'
  screen -S themis -p bear_left_arm  -X stuff '^C'
  screen -S themis -p bear_head      -X stuff '^C'
  screen -S themis -p dxl_right_hand -X stuff '^C'
  screen -S themis -p dxl_left_hand  -X stuff '^C'
  screen -S themis -p sense          -X stuff '^C'
  screen -S themis -p estimation     -X stuff '^C'
  screen -S themis -p low_level      -X stuff '^C'
  screen -S themis -p high_level     -X stuff '^C'
  screen -S themis -p top_level      -X stuff '^C'
  screen -S themis -p battery        -X stuff '^C'
  screen -S themis -p auxiliary      -X stuff '^C'
  sleep 0.1s
done

# ser7
echo themis | sudo -S shutdown -h now