#!/bin/bash
<< COMMENT
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 28, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT

# read config
output=`python3 -m Play.config`
config=(${output// / })
simulation=${config[0]}
gamepad=${config[1]}
estimation=${config[2]}
if [ $simulation != "True" ]
then
  echo $'\e[31mERROR: WRONG SETTING OF Play/config.py!\e[0m'
  exit
fi

# reset shared memory
python3 -m Startup.reset_memory
echo '====== Shared Memory Reset ======'
echo ''
sleep 1s

# delete all the existing screens if any
pkill screen

# create a background screen named 'themis'
screen -d -m -S themis

# create windows
screen -S themis -X screen -t gamepad
screen -S themis -X screen -t simulation
screen -S themis -X screen -t estimation
screen -S themis -X screen -t low_level
screen -S themis -X screen -t high_level
screen -S themis -X screen -t top_level
# sleep 1s

# run gazebo
screen -S themis -p simulation -X stuff 'python3 -m Startup.run_simulation^M'
sleep 3s
echo "====== Gazebo Online ======"
echo ""

# run estimation
if [ $estimation = "True" ]
then
  screen -S themis -p estimation -X stuff 'python3 -m Startup.run_estimation^M'
  sleep 1s
  echo "====== Estimation Online ======"
  echo ""
fi

# run low-level
# screen -S themis -p low_level -X stuff 'python3 -m Play.Locomotion.low_level^M'
screen -S themis -p low_level -X stuff 'Play/Locomotion/low_level^M'
sleep 1s
echo "====== Low-Level Controller Online ======"
echo ""

# run high-level
sleep 3s
# screen -S themis -p high_level -X stuff 'python3 -m Play.Locomotion.high_level^M'
screen -S themis -p high_level -X stuff 'Play/Locomotion/high_level^M'
sleep 1s
echo "====== High-Level Controller Online ======"
echo ""

# run top-level
screen -S themis -p top_level -X stuff 'python3 -m Play.Locomotion.top_level^M'
sleep 1s
echo "====== Top-Level Controller Online ======"
echo ""

# run gamepad
if [ $gamepad = "True" ]
then
  screen -S themis -p gamepad -X stuff 'python3 -m Startup.run_gamepad^M'
  # sleep 1s
  echo "====== Gamepad Online ======"
  echo ""
fi

input="n"
while [ $input != "y" ]
do
  echo -ne "Entering Cockpit .\033[0K\r"
  sleep 0.3s
  echo -ne "Entering Cockpit ..\033[0K\r"
  sleep 0.3s
  echo -ne "Entering Cockpit ...\033[0K\r"
  sleep 0.3s
  screen -r themis -p top_level
  sleep 0.2s
  read -p "Exit? (y/n)" input
done

# terminate
screen -S themis -p gamepad    -X stuff '^C'
screen -S themis -p simulation -X stuff '^C'
screen -S themis -p estimation -X stuff '^C'
screen -S themis -p low_level  -X stuff '^C'
screen -S themis -p high_level -X stuff '^C'
screen -S themis -p top_level  -X stuff '^C'
sleep 1s

# delete the screen
pkill screen