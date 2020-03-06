import numpy as np
import os


# Import the environment setup variables
import EnvSetupVar
# Set modules you want.
# EnvSetupVar.Need_RealSense_L = True
# EnvSetupVar.Need_RealSense_R = True
EnvSetupVar.Need_RealSense_BOTH = True
EnvSetupVar.SubProcess = os.fork()
# Setup the environment.
from EnvSetup import *

from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC


##################################################################
# 1. Open camera with any streams and settings that you want.
##################################################################
raw_input("Start program!")
RealSense_L.open(0x13)  # default is RS_COLOR|RS_DEPTH
RealSense_R.open(0x13)
# RealSense_L.open(RS_DEPTH | RS_IRL)
# RealSense_L.open(RS_COLOR | RS_DEPTH | RS_PC)
# ### Setup your own settings
# from MyEnvSetup.YourOwnRealsenseSetting import ExampleSetting
# RealSense_L.open(RS_IRL | RS_IRR, Settings=ExampleSetting)

##################################################################
# 1-1. (Optional)Import advanced setting to RealSense D435.
##################################################################
# raw_input("Start program!")
# ### File in "share/data/RealSense_D435"
# RealSense_L.ControlProcess.ImportAdvanceSettings("Default_CookTeam.json")
# ### Load your own advance settings.
# ### Current directory of load json file is ~/Workspace/[UserName]/rtc
# RealSense_R.ControlProcess.ImportAdvanceSettings("/home/cook-team/Documents/MySettings.json")

##################################################################
# 1-2. (Optional)Enable filters process if you need.
##################################################################
# raw_input("Start program!")
RealSense_L.ControlProcess.EnablePostProcess(True)
RealSense_R.ControlProcess.EnablePostProcess(True)

##################################################################
# 1-3. (Optional)You can get some information about camera. This part of code is not complete, do not use.
##################################################################
# raw_input("Start program!")
# print np.array(RealSense_L.GetInfoProcess.Get_IR_LeftToDepth_ExtrinsicParameters())
# print np.array(RealSense_L.GetInfoProcess.Get_IR_LeftToDepth_ExtrinsicParameters())


##################################################################
# 2. Open the stream. Realsense start to send data.
##################################################################
raw_input("Press ENTER key to continue!")
RealSense_L.ControlProcess.StartStream()
RealSense_R.ControlProcess.StartStream()

raw_input("Press ENTER key to continue!")
while True:
	##################################################################
	# 3. Save data by channels and to anywhere you want.
	##################################################################
	ImageProcess.save_colorImage(RS_CH1 | RS_CH2, "color.png")
	# ImageProcess.save_colorImage(RS_CH1 | RS_CH2, "/home/cook-team/Workspace/color.png")
	#
	ImageProcess.save_depthImage(RS_CH1 | RS_CH2, "depth.png")
	# ImageProcess.save_irImage(RS_CH1, "left_ir.png")
	# ImageProcess.save_ir_rightImage(RS_CH1, "right_ir.png")
	PointCloudProcess.save_pointCloud(RS_CH1 | RS_CH2, "test.pcd")
	# raw_input("Start program!")

	##################################################################
	# 3. Or get data by channels.
	##################################################################

	raw_input("Press ENTER key to continue!")
