import os


import EnvSetupVar
# Set modules you want.
EnvSetupVar.Need_Gripper_BOTH = True
EnvSetupVar.SubProcess = os.fork()
# Setup the environment.
from EnvSetup import *


Gripper_L.get_position()
Gripper_R.get_position()
raw_input("Press ENTER key to continue!")

Gripper_L.self_test()
Gripper_R.self_test()
raw_input("Press ENTER key to continue!")

Gripper_L.open(250)
Gripper_R.open(250)
raw_input("Press ENTER key to continue!")

Gripper_L.close(250)
Gripper_R.close(250)
raw_input("Press ENTER key to continue!")

Gripper_L.free()
Gripper_R.free()
raw_input("Press ENTER key to continue!")

exit(0)
