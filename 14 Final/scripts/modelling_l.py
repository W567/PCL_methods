import numpy as np
import os
import EnvSetupVar

# Set modules you want.
EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_Gripper_L = True
EnvSetupVar.Need_PA10_L = True
EnvSetupVar.SubProcess = os.fork()

from EnvSetup import *
from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC


# Define consts.
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

raw_input("open camera")
RealSense_L.open(RS_DEPTH | RS_PC)
RealSense_L.ControlProcess.ImportAdvanceSettings("/home/wu/Workspace/wu/data/mysetting.json")
RealSense_L.ControlProcess.EnablePostProcess(True)
RealSense_L.ControlProcess.StartStream()
raw_input("Press ENTER key to continue!")

Gripper_L.open(100)

ARM_L.StandbyARM()
raw_input("Press ENTER key to continue!")

base_xyzabc = [0.65, 0, 0.25, 0, 0, 0]
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

while True:
	temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
	flag = PointCloudProcess.single_plate_modelling(temp)
        print flag
	raw_input("Press ENTER key to continue!")

#0.015
#-0.02


