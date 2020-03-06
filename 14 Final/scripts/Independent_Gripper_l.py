import os


import EnvSetupVar
# Set modules you want.
EnvSetupVar.Need_Gripper_L = True
EnvSetupVar.SubProcess = os.fork()
# Setup the environment.
from EnvSetup import *

Gripper_L.self_test()
raw_input("Press ENTER key to continue!")

Gripper_L.free()
raw_input("Press ENTER key to continue!")

exit(0)
