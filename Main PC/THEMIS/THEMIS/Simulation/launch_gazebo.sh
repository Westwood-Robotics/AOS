#!/bin/bash
<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "June 4, 2024"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"
COMMENT

# read config
fixed=`python3 -m Simulation.config`

if [ $fixed = "True" ]
then
  gazebo --verbose Simulation/worlds/themis_fixed.world
else
  gazebo --verbose Simulation/worlds/themis.world
fi