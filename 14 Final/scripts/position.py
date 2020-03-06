import numpy as np
import os
import EnvSetupVar

# Set modules you want.
EnvSetupVar.Need_Gripper_L = True
EnvSetupVar.Need_PA10_L = True
EnvSetupVar.SubProcess = os.fork()

from EnvSetup import *
from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC


# Define consts.
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

Gripper_L.open(200)

ARM_L.StandbyARM()
raw_input("Press ENTER key to continue!")

base_xyzabc = [0.65, 0, 0.18, 0, 0, 0]
div_d = 0.05
div_ang = 20 * toRad

print "move xyz"

j = base_xyzabc[:]
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")


raw_input("Move 1st")
j[0]=0.35
ARM_L.move_rmrc(j,DEBUG_MODE = True)

raw_input("Move 2nd")
j[0]=0.55
ARM_L.move_rmrc(j,DEBUG_MODE = True)

raw_input("Move 3rd")
j[0]= 0.75
ARM_L.move_rmrc(j,DEBUG_MODE = True)

raw_input("Move 4th")
j[1]= -0.4
ARM_L.move_rmrc(j,DEBUG_MODE = True)

raw_input("Move 5th")
j[0]= 0.55
ARM_L.move_rmrc(j,DEBUG_MODE = True)

raw_input("Move 6th")
j[0]= 0.35
ARM_L.move_rmrc(j,DEBUG_MODE = True)



#0.015
#-0.02


