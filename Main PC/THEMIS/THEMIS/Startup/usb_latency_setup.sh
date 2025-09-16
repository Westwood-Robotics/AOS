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

# set password
readonly PASSWORD=themis

# USB low latency setup
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer"
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB2/latency_timer"
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB3/latency_timer"
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB4/latency_timer"
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB5/latency_timer"
echo $PASSWORD | sudo -S sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB6/latency_timer"