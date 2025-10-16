#!/bin/bash

sshpass -p nvidia ssh nvidia@192.168.0.72 'echo nvidia | ./ros2_detection/src/start_click_to_map.sh'

