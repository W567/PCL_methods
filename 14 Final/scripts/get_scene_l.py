import numpy as np
import os


# Import the environment setup variables
import EnvSetupVar
# Set modules you want.
EnvSetupVar.Need_RealSense_L = True
# EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_BOTH = True

EnvSetupVar.Need_PA10_L = True
EnvSetupVar.SubProcess = os.fork()
# Setup the environment.
from EnvSetup import *
from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC

# Define consts.
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

raw_input("Start!")
print "Open Camera"
RealSense_L.open(RS_DEPTH | RS_PC)

print "Import Advanced Settings"
RealSense_L.ControlProcess.ImportAdvanceSettings("/home/wu/Workspace/wu/data/mysetting.json")

print "Enable Post Process"
RealSense_L.ControlProcess.EnablePostProcess(True)

print "Start Camera Stream"
RealSense_L.ControlProcess.StartStream()

raw_input("Camera setting finished")

raw_input("move arm(standby)")
ARM_L.StandbyARM()

raw_input("Move arm to the initial position?")
base_xyzabc = [0.65, 0, 0.24, 0, 0, 0]
div_d = 0.05
div_ang = 20 * toRad
# ARM_L.mode_rmrc()
j = base_xyzabc[:]
ARM_L.move_rmrc(j)
print "Arm has been moved to the initial position."

raw_input("move to the scene_capture position")


j[0]=0.4
ARM_L.move_rmrc(j)
raw_input("continue")
temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
PointCloudProcess.add_PointCloud(temp)
raw_input("Press ENTER key to continue!")

j[0]=0.7
ARM_L.move_rmrc(j)
raw_input("continue")
temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
PointCloudProcess.add_PointCloud(temp)
raw_input("Press ENTER key to continue!")


j[1]= - 0.4
ARM_L.move_rmrc(j)
raw_input("continue")
temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
PointCloudProcess.add_PointCloud(temp)
raw_input("Press ENTER key to continue!")


j[0]=0.4
ARM_L.move_rmrc(j)
raw_input("continue")
temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
PointCloudProcess.add_PointCloud(temp)
raw_input("Press ENTER key to continue!")










