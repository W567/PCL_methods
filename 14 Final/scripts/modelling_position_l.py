import numpy as np
import os


# ### Import the environment setup variables
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
base_xyzabc = [0.65, 0, 0.24, 0, 0, 0]
div_d = 0.05
div_ang = 20 * toRad

# ARM_L.mode_rmrc()
j = base_xyzabc[:]
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")


j[2]= 0.25
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

j[2]=0.26
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

#raw_input("move to the scene_capture position")
j[2]=0.27
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

j[2]=0.28
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

j[2]=0.29
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

j[2]=0.3
ARM_L.move_rmrc(j)
raw_input("Press ENTER key to continue!")

#0.4
#0.7


