#!/bin/bash
<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 29, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT

# read config
cd /home/themis/THEMIS/THEMIS
output=`python3 -m Play.config`
config=(${output// / })
simulation=${config[0]}
if [ $simulation = "True" ]
then
  echo $'\e[31mERROR: WRONG SETTING OF Play/config.py!\e[0m'
  exit
fi

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
  screen -S themis -p auxiliary      -X stuff '^C'
  screen -S themis -p zed            -X stuff '^C'
  sleep 0.1s
done

# reset thread status
screen -S themis -p sense -X stuff 'python3 -m Startup.reset_memory^M'

sleep 1s

screen -S themis -p zed -X stuff 'source ~/.bashrc && ros2 run shared_memory_publisher shared_memory_publisher_node^M'

sleep 3s

# run_bear
echo $'\e[31mATTENTION: Press ENTER to start BEAR control! BEAR LED will be on!\e[0m'
kill -19 $(pgrep bootup)
screen -S themis -p bear_right_leg  -X stuff 'python3 -m Startup.run_bear^M'
screen -S themis -p bear_left_leg   -X stuff 'python3 -m Startup.run_bear^M'
screen -S themis -p bear_right_arm  -X stuff 'python3 -m Startup.run_bear^M'
screen -S themis -p bear_left_arm   -X stuff 'python3 -m Startup.run_bear^M'
screen -S themis -p bear_head       -X stuff 'python3 -m Startup.run_bear^M'
screen -S themis -p dxl_right_hand  -X stuff 'python3 -m Startup.run_dxl^M'
screen -S themis -p dxl_left_hand   -X stuff 'python3 -m Startup.run_dxl^M'
sleep 2s
screen -S themis -p bear_right_leg  -X stuff 'rl^M'
screen -S themis -p bear_left_leg   -X stuff 'll^M'
screen -S themis -p bear_right_arm  -X stuff 'ra^M'
screen -S themis -p bear_left_arm   -X stuff 'la^M'
screen -S themis -p bear_head       -X stuff 'h^M'
screen -S themis -p dxl_right_hand  -X stuff 'rh^M'
screen -S themis -p dxl_left_hand   -X stuff 'lh^M'
sleep 1s
echo '====== Actuator Control Online ======'
echo ''

# initialize
echo $'\e[31mATTENTION: Press START to initialize THEMIS! Limbs will move!\e[0m'
kill -19 $(pgrep bootup)
screen -S themis -p auxiliary -X stuff 'python3 -m Play.Others.initialize_from_chair^M'
sleep 2s
echo '====== THEMIS Initialized ======'
echo ''

# run_sense
echo $'\e[31mATTENTION: Place THEMIS on the ground! Press ENTER to start THEMIS sense!\e[0m'
kill -19 $(pgrep bootup)
screen -S themis -p sense -X stuff 'python3 -m Startup.run_sense^M'
sleep 1s
echo '====== THEMIS Sense Online ======'
echo ''

# run_estimation
echo $'\e[31mATTENTION: Press Enter to start estimation! Wait until arms move!\e[0m'
# kill -19 $(pgrep bootup)
screen -S themis -p estimation -X stuff 'python3 -m Startup.run_estimation^M'
sleep 2s
echo '====== State Estimation Online ======'
echo ''

# low_level
# screen -S themis -p low_level -X stuff 'python3 -m Play.Locomotion.low_level^M'
screen -S themis -p low_level -X stuff 'Play/Locomotion/low_level^M'
sleep 1s
echo $'\e[31mATTENTION: Press ENTER to start low-level control! THEMIS may twitch a bit!\e[0m'
kill -19 $(pgrep bootup)
sleep 1s
screen -S themis -p low_level -X stuff 'y^M'
echo '====== Low-Level Control Online ======'
echo ''

# high_level
echo $'\e[31mATTENTION: Press ENTER to start high-level control! THEMIS will rise a bit!\e[0m'
kill -19 $(pgrep bootup)
# screen -S themis -p high_level -X stuff 'python3 -m Play.Locomotion.high_level^M'
screen -S themis -p high_level -X stuff 'Play/Locomotion/high_level^M'
sleep 1s
echo '====== High-Level Control Online ======'
echo ''

# top_level
echo $'\e[31mATTENTION: Press ENTER to enter cockpit!\e[0m'
kill -19 $(pgrep bootup)
screen -S themis -p top_level -X stuff 'python3 -m Play.Locomotion.top_level^M'
sleep 1s
echo '====== Top-Level Control Online ======'
echo ''