#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 3, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script that holds useful locomotion macros
'''


# ==================================================
# ==================================================
# LOCOMOTION STATE
# ==================================================
# ==================================================

# ------------------------------
# MODE
# ------------------------------
BALANCE = 0
WALK    = 1

# ------------------------------
# PHASE
# ------------------------------
STANCE_DOUBLE = 0
STANCE_RIGHT  = 1
STANCE_LEFT   = 2

# ------------------------------
# LEG
# ------------------------------
RIGHT = +1
LEFT  = -1