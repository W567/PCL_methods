##################################################################
# Please read all of independent examples, before you read this example.
##################################################################
import numpy as np
import os


# ### Import the environment setup variables
import EnvSetupVar
# ### Set modules you want.
EnvSetupVar.Need_Gripper_BOTH = True
EnvSetupVar.Need_RealSense_BOTH = True
EnvSetupVar.Need_PA10_BOTH = True
EnvSetupVar.SubProcess = os.fork()
# ### Setup the environment.
from EnvSetup import *
from MyEnvSetup.YourOwnRealsenseSetting import ExampleSetting

# Define consts.
from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0


##################################################################
# 1-1. Initial arm
##################################################################
ARM_L.StandbyARM()
ARM_R.StandbyARM()
raw_input("Start program!")

##################################################################
# 1-2. Initial RealSense
##################################################################
ARM_L.RealSense.open(RS_COLOR | RS_DEPTH | RS_PC)
ARM_R.RealSense.open(RS_COLOR | RS_DEPTH | RS_PC)

##################################################################
# 1-3. Import advanced setting to RealSense D435.
##################################################################
ARM_L.RealSense.ControlProcess.ImportAdvanceSettings("Default_CookTeam.json")
ARM_R.RealSense.ControlProcess.ImportAdvanceSettings("Default_CookTeam.json")

##################################################################
# 1-3. (Optional) Gripper self test.
##################################################################
ARM_L.Gripper.self_test()
ARM_R.Gripper.self_test()

##################################################################
# 2-1. Start Realsense stream.
##################################################################
ARM_L.RealSense.ControlProcess.StartStream()
ARM_R.RealSense.ControlProcess.StartStream()

##################################################################
# 2-2. Move hand to capture point.
##################################################################
# Prepare move coordinates.
base_xyzabc = [0.65, 0, 0.2, 0, 0, 0]
LeftCapturePoint = [0.65, -0.3, 0.35, 0, 0, 0]
RightCapturePoint = [0.65, 0.3, 0.35, 0, 0, 0]

ARM_L.move_rmrc(LeftCapturePoint)
ARM_R.move_rmrc(RightCapturePoint)

##################################################################
# 2-3. Move hand to capture point.
##################################################################
# ### Save color point cloud by dual-arm.
# ARM_L.RealSense.ImageProcess.save_pointcloud(RS_CH1 | RS_CH2, "SyntheticalExample.pcd")
PointCloudProcess.save_pointcloud(RS_CH1 | RS_CH2, "SyntheticalExample.pcd")


