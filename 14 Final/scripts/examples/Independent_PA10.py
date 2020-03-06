import numpy as np
import os


# ### Import the environment setup variables
import EnvSetupVar
# ### Set modules you want.
# EnvSetupVar.Need_PA10_L = True
# EnvSetupVar.Need_PA10_R = True
EnvSetupVar.Need_PA10_BOTH = True
EnvSetupVar.SubProcess = os.fork()
# ### Setup the environment.
from EnvSetup import *


# Define consts.
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

raw_input("Start program!")


##################################################################
# 1. Initial arm and move it to mechanical-zero position, and set to "rmrc" mode.
##################################################################
ARM_L.StandbyARM()
ARM_R.StandbyARM()
raw_input("Press ENTER key to continue!")


##################################################################
# 2. Move everywhere you want.
##################################################################
# Prepare move coordinates.
base_xyzabc = [0.65, 0, 0.2, 0, 0, 0]
div_d = 0.05
div_ang = 20 * toRad

# ### Change mode to rmrc mode. This step had been done by InitialARM() function.
# ### But you should know when you want to move arm in rmrc mode, you must change to rmrc mode first.
# print "mode rmrc"
# ARM_L.mode_rmrc()
print "move xyz"
# ### Move to start.
j = base_xyzabc[:]
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

j[1] -= div_d
ARM_L.move_rmrc(j)
j[1] += div_d
raw_input("Press ENTER key to continue!")

j[0] += div_d
ARM_L.move_rmrc(j)
j[1] += div_d
raw_input("Press ENTER key to continue!")

j[0] -= div_d
ARM_L.move_rmrc(j)
j = base_xyzabc[:]
raw_input("Press ENTER key to continue!")


# move abc
print "move abc"
ARM_L.move_rmrc(j)
j[3] += div_ang
ARM_L.move_rmrc(j)
j[3] -= div_ang
raw_input("Press ENTER key to continue!")

j[4] += div_ang
ARM_L.move_rmrc(j)
j[3] -= div_ang
raw_input("Press ENTER key to continue!")

j[4] -= div_ang
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

j = base_xyzabc[:]
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")


##################################################################
# 2-2. Set tool offset.
##################################################################
ARM_L.set_tool_xyz(0, 0, 0.07)
raw_input("Press ENTER key to continue!")

ARM_R.set_tool_xyz(0, 0.05, 0)
raw_input("Press ENTER key to continue!")
