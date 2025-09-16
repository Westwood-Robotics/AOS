#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "September 12, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Set BEAR ID
'''

import time
from Setting.Macros.bear_macros import *
from Library.BEAR_ACTUATOR import Manager as bear_actuator_manager


if __name__ == '__main__':
    print("====== THEMIS BEAR ID SETTING ======")
    chain = input("Communicate right leg (rl) or left leg (ll) or right arm (ra) or left arm (la) or head (h) chain? ")
    if chain == 'rl':
        bam = bear_actuator_manager.BearActuatorManager(port=LEG_R_BEAR_PORT, baudrate=BEAR_BAUDRATE)
    elif chain == 'll':
        bam = bear_actuator_manager.BearActuatorManager(port=LEG_L_BEAR_PORT, baudrate=BEAR_BAUDRATE)
    elif chain == 'ra':
        bam = bear_actuator_manager.BearActuatorManager(port=ARM_R_BEAR_PORT, baudrate=BEAR_BAUDRATE)
    elif chain == 'la':
        bam = bear_actuator_manager.BearActuatorManager(port=ARM_L_BEAR_PORT, baudrate=BEAR_BAUDRATE)
    elif chain == 'h':
        bam = bear_actuator_manager.BearActuatorManager(port=HEAD_BEAR_PORT,  baudrate=BEAR_BAUDRATE)
    else:
        print("Error input!")
        exit()

    while True:
        m_id = int(input("Enter the present ID and press enter: "))
        print("Present ID entered is %02d." % m_id)
        if bam.ping([m_id])[0][0] is not None:
            print(bam.ping([m_id]))
            print("BEAR %02d is connected." % m_id)
            m_id_new = int(input("Enter the new ID and press enter: "))
            if m_id_new == m_id:
                print("Please enter a different ID.")
            else:
                bam.set_ID([m_id], [m_id_new])
                for _ in range(5):  # make sure it is saved!!!
                    bam.save_config([m_id_new])
                    time.sleep(0.1)
                if bam.ping([m_id_new])[0][0] is not None:
                    print("BEAR ID has been changed from %02d to %02d." % (m_id, m_id_new))
                else:
                    print("BEAR ID change is unsuccessful. Please try again.")
        else:
            print("Seems like that BEAR is offline. Please double check your entry and connection.")
        print()