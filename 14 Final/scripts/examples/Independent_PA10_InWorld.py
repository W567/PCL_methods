##################################################################
# Please read "independent_PA10.py", before you read this example.
##################################################################
import numpy as np
import os


# ### Import the environment setup variables
import EnvSetupVar
# ### Set modules you want.
# EnvSetupVar.Need_PA10_L = True
# EnvSetupVar.Need_PA10_R = True
EnvSetupVar.Need_PA10_BOTH = True
EnvSetupVar.Need_PA10_IN_WORLD = True
EnvSetupVar.SubProcess = os.fork()
# ### Setup the Left ARM to world transform matrix.
EnvSetupVar.Tl2world = [0, 0, 0, 0, 0, 0]
# ### Setup the environment.
from EnvSetup import *


# Define consts.
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0


##################################################################
# 1. Initial arm and move it to mechanical-zero position, and set to "rmrc" mode.
##################################################################
ARM_L.StandbyARM()
ARM_R.StandbyARM()
raw_input("Start program!")


##################################################################
# 2. Move everywhere in world coordinate you want.
##################################################################
# Prepare move coordinates.
base_xyzabc = [0.65, 0, 0.2, 0, 0, 0]

ARM_L.move_rmrc(base_xyzabc)
raw_input("Start program!")
ARM_R.move_rmrc([0.65, -0.7, 0.2, 0, 0, 0])
