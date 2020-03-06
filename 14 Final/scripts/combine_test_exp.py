import numpy as np
import os
import time

import EnvSetupVar
EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_PA10_L = True
EnvSetupVar.Need_Gripper_L = True
EnvSetupVar.SubProcess = os.fork()

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

#raw_input("Camera setting finished\n Open gripper?")
Gripper_L.open(100)

#raw_input("move arm(standby)")
ARM_L.StandbyARM()

#raw_input("Move arm to the initial position?")
base_xyzabc = [0.65, -0.2, 0.22, 0, 0, 0]
div_d = 0.05
div_ang = 20 * toRad
# ARM_L.mode_rmrc()
j = base_xyzabc[:]
#ARM_L.move_rmrc(j, DEBUG_MODE = True)
ARM_L.move_rmrc(j)
print "Arm has been moved to the initial position."

while True:
	final_position_x = input("input the destination: x:  ")
	final_position_y = input("input the destination: y:  ")
	if final_position_x < 0.25:
		break
	if final_position_x > 0.8:
		break
	if final_position_y > 0.1:
		break
	if final_position_y < -0.55:
		break

	final = [final_position_x,final_position_y,0.25,0,0,0]
	put = final[:]
	get = final[:]
	move = final[:]	

	raw_input("begin")
	#raw_input("Move 1st")
	j[0]=0.36
	#ARM_L.move_rmrc(j,DEBUG_MODE = True)
	ARM_L.move_rmrc(j)
	#raw_input("Capture 1st Point Cloud")
	temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
	height = PointCloudProcess.get_FirstCloud(temp)
	print height

	#raw_input("Move 2nd")
	j[0]=0.55
	#ARM_L.move_rmrc(j,DEBUG_MODE = True)
	ARM_L.move_rmrc(j)	
	#raw_input("Capture 2nd Point Cloud")
	temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
	height += PointCloudProcess.add_PointCloud(temp)
	print height

	#raw_input("Move 3rd")
	j[0]= 0.74
	#ARM_L.move_rmrc(j,DEBUG_MODE = True)
	ARM_L.move_rmrc(j)
	#raw_input("Capture 3rd Point Cloud")
	temp = (ARM_L.h_FRAME_now * ARM_L.Th2c).tolist()
	height += PointCloudProcess.add_PointCloud(temp)
	print height


	#raw_input("plate_recog")
	numberOfPlates, plate_information, r_max = PointCloudProcess.plate_recog(temp)
	print numberOfPlates

	if numberOfPlates > 0:
		print "check destination"
		print r_max
		print final_position_x
		print final_position_y
		occupy = PointCloudProcess.check_destination(r_max, final_position_x, final_position_y)    # check if the destination is occupied with the max radius of dishes.
		if occupy == 0 :
			raw_input("the destination is occupied by other stuff! Continue?")
		temp_z = height/3
		print temp_z
		accu_z = 0

	        for i in range(numberOfPlates):
			h_tool = PointCloudProcess.height_determine(plate_information[i][2]) - 0.014
		
			print("move to the %d th plate" %(i+1))
			get[0] = plate_information[i][0]
			get[1] = plate_information[i][1]
			get[2] = plate_information[i][4] + h_tool -0.017  # calculate the height for catching the dish: bottom2deskC + h_tool 0.003
			get[5] = plate_information[i][7]  # angle
			if get[5] > 0:
				get[5] = get[5] - np.pi/2
			else:
				get[5] = get[5] + np.pi/2
			print get

			if get[5] > 2:
				print "warning!"

			#raw_input("move or not?")
			#ARM_L.move_rmrc(get,DEBUG_MODE = True)
			ARM_L.move_rmrc(get)
			#raw_input("close gripper?")
			Gripper_L.close(150)
			time.sleep(1)
			#raw_input("pick up?")
			get[2] += 0.055
			#ARM_L.move_rmrc(get,DEBUG_MODE = True) 
			ARM_L.move_rmrc(get)  
			# finish picking up the plate

			#raw_input("move to the target?")
			
			target = int(plate_information[i][8])
			if target == -1:
				move = final[:]
				#move[5] = get[5]
				put[0] = final[0]
				put[1] = final[1]
				put[2] = temp_z + h_tool + plate_information[i][9] + accu_z
				put[5] = 0
				accu_z = accu_z + plate_information[i][6]
				plate_information[i][0] = final[0]
				plate_information[i][1] = final[1]
				plate_information[i][5] = accu_z + temp_z
				plate_information[i][7] = np.pi/2
			else : 
				move[0] = plate_information[target][0]   # x
				move[1] = plate_information[target][1]   # y 
				#move[5] = get[5]
				put[0] = plate_information[target][0]	 # x		
				put[1] = plate_information[target][1]    # y
				put[2] = plate_information[target][5] + h_tool + plate_information[i][9]  # z = bottom2desk + topself + h_tool
				if plate_information[target][7] > 0:
					put_angle = plate_information[target][7] - np.pi/2
			        else:
					put_angle = plate_information[target][7] + np.pi/2
				put[5] = put_angle

				# accumulate bottom2desk += bottomself
				plate_information[target][6] += plate_information[i][6]
				plate_information[target][5] = plate_information[target][5] + plate_information[i][6] 
				plate_information[i][0] = plate_information[target][0]  # x
				plate_information[i][1] = plate_information[target][1]  # y 
				plate_information[i][5] = plate_information[target][5]  # bottom2desk
				plate_information[i][8] = plate_information[target][8]  # target
				plate_information[i][7] = plate_information[target][7]  # angle
			

			for k in range(numberOfPlates) : 
				if plate_information[k][8] == target :
					plate_information[k][0] = plate_information[i][0]
					plate_information[k][1] = plate_information[i][1]
					plate_information[k][5] = plate_information[i][5]
					plate_information[k][8] = plate_information[i][8]
					plate_information[k][7] = plate_information[i][7]

			print move			
			#ARM_L.move_rmrc(move,DEBUG_MODE = True)
			ARM_L.move_rmrc(move)

			#put[2] += 0.01
			print put
			#raw_input("put down?")
			#ARM_L.move_rmrc(put,DEBUG_MODE = True)
			ARM_L.move_rmrc(put)
			#raw_input("open gripper?")
			Gripper_L.open(150)
			time.sleep(1)
			
			if i < numberOfPlates - 1 and plate_information[i+1][8] == plate_information[i][8] : 
				continue
				#raw_input("the position of the arm is the position of the next target, next move?")
			else: 
				#raw_input("move upper?")
				#ARM_L.move_rmrc(move,DEBUG_MODE = True)
				ARM_L.move_rmrc(move)
				#raw_input("next move?")


	j = [0.65, 0, 0.24, 0, 0, 0]
	#ARM_L.move_rmrc(j,DEBUG_MODE = True)
	ARM_L.move_rmrc(j)
	Gripper_L.free()
	raw_input("Try again?")






