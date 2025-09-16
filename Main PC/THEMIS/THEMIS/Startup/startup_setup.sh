#!/bin/bash
<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 18, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT


sleep 1s

# virtual display
export DISPLAY=:0

# go to folder
cd /home/themis/THEMIS/THEMIS

# read config
output=`python3 -m Play.config`
config=(${output// / })
simulation=${config[0]}
gamepad=${config[1]}

# USB low latency setup
Startup/usb_latency_setup.sh
echo '====== USB Low Latency Setup Finished ======'
echo ''

# lcm
readonly PASSWORD=themis
echo $PASSWORD | sudo -S ifconfig enp1s0 multicast
echo $PASSWORD | sudo -S route add -net 224.0.0.0 netmask 240.0.0.0 dev enp1s0

# create shared memory
python3 -m Startup.memory_manager
echo '====== Shared Memory Created ======'
echo ''

# create a background screen named 'themis'
pkill screen
sleep 0.1s
screen -dmS themis

# create a window named 'gamepad'
if [ $gamepad = "True" ]
then
  screen -S themis -X screen -t gamepad
  screen -S themis -p gamepad -X stuff 'python3 -m Startup.run_gamepad^M'
  echo '====== THEMIS Gamepad Online ======'
  echo ''
fi

# create a window named 'bear_right_leg'
screen -S themis -X screen -t bear_right_leg

# create a window named 'bear_left_leg'
screen -S themis -X screen -t bear_left_leg

# create a window named 'bear_right_arm'
screen -S themis -X screen -t bear_right_arm

# create a window named 'bear_left_arm'
screen -S themis -X screen -t bear_left_arm

# create a window named 'bear_head'
screen -S themis -X screen -t bear_head

# create a window named 'dxl_right_hand'
screen -S themis -X screen -t dxl_right_hand

# create a window named 'dxl_left_hand'
screen -S themis -X screen -t dxl_left_hand

# create a window named 'sense'
screen -S themis -X screen -t sense

# create a window named 'estimation'
screen -S themis -X screen -t estimation

# create a window named 'low_level'
screen -S themis -X screen -t low_level

# create a window named 'high_level'
screen -S themis -X screen -t high_level

# create a window named 'top_level'
screen -S themis -X screen -t top_level

# create a window named 'battery'
screen -S themis -X screen -t battery
screen -S themis -p battery -X stuff 'python3 -m Startup.run_battery^M'
echo '====== THEMIS Battery Online ======'
echo ''

# create a window named 'auxiliary'
screen -S themis -X screen -t auxiliary

# navigation startup
Play/Navigation/startup_setup.sh

sleep 1s