import numpy as np
import os


import EnvSetupVar

EnvSetupVar.Need_PA10_L = True
EnvSetupVar.SubProcess = os.fork()
from EnvSetup import *

toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

raw_input("Start program!")

ARM_L.StandbyARM()
raw_input("Press ENTER key to continue!")

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

j[2] = 0.25
j[0] = 0.5
j[1] = -0.2
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")
