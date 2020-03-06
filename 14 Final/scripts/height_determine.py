import numpy as np
import os
import EnvSetupVar

# Set modules you want.
# EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_BOTH = True

EnvSetupVar.SubProcess = os.fork()

from EnvSetup import *
from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC

raw_input("Start program!")
RealSense_R.open(RS_DEPTH | RS_PC)
RealSense_R.ControlProcess.ImportAdvanceSettings("/home/wu/Workspace/wu/data/mysetting.json")
RealSense_R.ControlProcess.EnablePostProcess(True)
raw_input("Press ENTER key to continue!")
RealSense_R.ControlProcess.StartStream()
raw_input("Press ENTER key to continue!")

while True:
	height = PointCloudProcess.height_determine(0.1,0.1)
	print height
	raw_input("continue?")
