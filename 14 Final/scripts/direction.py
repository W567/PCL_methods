import numpy as np
import os


# Import the environment setup variables
import EnvSetupVar
# Set modules you want.
EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_BOTH = True

EnvSetupVar.Need_PA10_R = True
EnvSetupVar.Need_Gripper_R = True
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
RealSense_R.open(RS_DEPTH | RS_PC)

print "Import Advanced Settings"
RealSense_R.ControlProcess.ImportAdvanceSettings("/home/wu/Workspace/wu/data/mysetting.json")

print "Enable Post Process"
RealSense_R.ControlProcess.EnablePostProcess(True)

print "Start Camera Stream"
RealSense_R.ControlProcess.StartStream()

raw_input("Camera setting finished\n Open gripper?")
Gripper_R.open(100)

raw_input("move arm(standby)")
ARM_R.StandbyARM()

raw_input("Move arm to the initial position?")
base_xyzabc = [0.65, 0, 0.3, 0, 0, 0]
div_d = 0.05
div_ang = 20 * toRad
# ARM_L.mode_rmrc()
j = base_xyzabc[:]
ARM_R.move_rmrc(j, DEBUG_MODE = True)
print "Arm has been moved to the initial position."

while True:
	final_position_x = input("input the destination: x:  ")
	final_position_y = input("input the destination: y:  ")
	if final_position_x < 0.2:
		break
	if final_position_x > 0.8:
		break
	if final_position_y < -0.1:
		break
	if final_position_y > 0.45:
		break

	final = [final_position_x,final_position_y,0.25,0,0,0]
	put = final[:]
	get = final[:]
	
	temp = (ARM_R.h_FRAME_now * ARM_R.Th2c).tolist()
	height = PointCloudProcess.add_PointCloud(temp)
	print height

	raw_input("plate_recog")
	numberOfPlates, plate_information = PointCloudProcess.plate_recog(temp)
	print numberOfPlates
	if numberOfPlates > 0:
		print "check destination"
		PointCloudProcess.check_destination(plate_information[0][3],final_position_x,final_position_y)    #check if the destination is occupied with the max radius of dishes.
		temp_z = height   # /4		
		print temp_z
		accu_z = 0
	        for i in range(numberOfPlates):
			if plate_information[i][4] == 0 :              # if no food
				h_tool = PointCloudProcess.height_determine(plate_information[i][2],plate_information[i][3]) - 0.014
				
				print("move to the %d th plate" %(i+1))
				get[0] = plate_information[i][0]
				get[1] = plate_information[i][1]
				get[2] = plate_information[i][2] + h_tool + plate_information[i][5]  #calculate the height for catching the dish: h_desktop + h_tool + h_dish
				get[5] = plate_information[i][7]
				if get[5] > 0:
					get[5] = get[5] - np.pi/2
				else:
					get[5] = get[5] + np.pi/2
				print get

				if get[5] > 2:
					print "warning!"

				raw_input("move or not?")
				ARM_R.move_rmrc(get,DEBUG_MODE = True)
				raw_input("close gripper?")
				Gripper_R.close(150)
				raw_input("pick up?")
				get[2] += 0.05
				ARM_R.move_rmrc(get,DEBUG_MODE = True)

				raw_input("move to the destination?")
				ARM_R.move_rmrc(final,DEBUG_MODE = True)
				
				
				
				put[2] = temp_z + h_tool + accu_z +plate_information[i][5]
				accu_z += plate_information[i][6]    #add the thickness of the dish before and after it is placed on the destination
				print accu_z
				print put
				raw_input("put down?")
				ARM_R.move_rmrc(put,DEBUG_MODE = True)
				raw_input("open gripper?")
				Gripper_R.open(150)
				raw_input("move upper?")
				ARM_R.move_rmrc(final,DEBUG_MODE = True)
				raw_input("next move?")
			else:
				raw_input("this plate have food inside, continue?")
	j = [0.65, 0, 0.3, 0, 0, 0]
	ARM_R.move_rmrc(j,DEBUG_MODE = True)
	Gripper_R.free()
	raw_input("Try again?")






