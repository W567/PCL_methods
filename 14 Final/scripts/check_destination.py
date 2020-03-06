import numpy as np
import os
import EnvSetupVar

# Set modules you want.
# EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_BOTH = True

EnvSetupVar.Need_PA10_R = True
EnvSetupVar.SubProcess = os.fork()

from EnvSetup import *
from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC


# Define consts.
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

raw_input("Start program!")
RealSense_R.open(RS_DEPTH | RS_PC)
RealSense_R.ControlProcess.ImportAdvanceSettings("/home/wu/Workspace/wu/data/mysetting.json")
RealSense_R.ControlProcess.EnablePostProcess(True)
raw_input("Press ENTER key to continue!")
RealSense_R.ControlProcess.StartStream()
raw_input("Press ENTER key to continue!")


##################################################################
# 1. Initial arm and move it to mechanical-zero position, and set to "rmrc" mode.
##################################################################
ARM_R.StandbyARM()
# ARM_R.StandbyARM()
raw_input("Press ENTER key to continue!")

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
ARM_R.move_rmrc(j)
raw_input("Press ENTER key to continue!")

plate_position = input("input the plate position: x and y")

j[0] += plate_position[0]
j[1] += plate_position[1]
j[2] += 0.1
raw_input("move to %f %f %f ?" %(j[0],j[1],j[2]))
ARM_R.move_rmrc(j)
raw_input("Press ENTER key to continue!")


while True:
	temp = (ARM_R.h_FRAME_now * ARM_R.Th2c).tolist()
	z = PointCloudProcess.check_destination(temp,0.09);
        print z
	raw_input("Press ENTER key to continue!")

#0.015
#-0.02


