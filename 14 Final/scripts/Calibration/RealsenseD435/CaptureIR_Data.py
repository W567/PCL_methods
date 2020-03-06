import pyrealsense2 as rs
import os
import cv2
import time
from CalibrationActions import *
from myPyUtil import json_read, json_write, calc_extrinsic
import sys
import threading
from multiprocessing import Process, Manager, Value, Pipe
import numpy as np
from PIL import Image, ImageTk
import Tkinter as tk

# import EnvSetupVar
# # EnvSetupVar.Need_PA10_L
# from EnvSetup import *


CHESS_SIZE = (13 - 1, 9 - 1)
CHESS_SQUARE_SIZE = 25  # [mm]


def InitialTKWindow(arg="-l"):
	global window, IMG_L, IMG_R
	window = tk.Tk()
	window.wm_title("RealSense D435 Capture Window")
	if arg == "-l":
		IMG_L = tk.Label(window)
		IMG_L.pack(side=tk.LEFT, padx=10, pady=10)
	elif arg == "-r":
		IMG_R = tk.Label(window)
		IMG_R.pack(side=tk.RIGHT, padx=10, pady=10)
	elif arg == "-b":
		IMG_L = tk.Label(window)
		IMG_L.pack(side=tk.LEFT, padx=10, pady=10)
		IMG_R = tk.Label(window)
		IMG_R.pack(side=tk.RIGHT, padx=10, pady=10)


def cap_l(frame):
	global Trigger, parent_conn, Count, IMG_L, window
	depth_image = np.asanyarray(frame.get_data())
	depth_image_Display = cv2.resize(depth_image, (depth_image.shape[1]/2, depth_image.shape[0]/2))

	pilImg = Image.fromarray(depth_image_Display)
	imageLeft = ImageTk.PhotoImage(pilImg)

	IMG_L.configure(image=imageLeft)
	IMG_L.image = imageLeft
	cv2.waitKey(25)

	if parent_conn.poll() and parent_conn.recv() == 1:
		Dir = "../../../data/CalibrationImage/Left_" + str(Count) + ".png"
		cv2.imwrite(Dir, depth_image)
		print "Save image to ", Dir
		Count += 1


def cap_r(frame):
	global Trigger, parent_conn, Count, IMG_R, window
	depth_image = np.asanyarray(frame.get_data())
	depth_image_Display = cv2.resize(depth_image, (depth_image.shape[1] / 2, depth_image.shape[0] / 2))

	pilImg = Image.fromarray(depth_image_Display)
	imageRight = ImageTk.PhotoImage(pilImg)

	IMG_R.configure(image=imageRight)
	IMG_R.image = imageRight
	cv2.waitKey(25)

	if parent_conn.poll() and parent_conn.recv() == 1:
		Dir = "../../../data/CalibrationImage/Right_" + str(Count) + ".png"
		cv2.imwrite(Dir, depth_image)
		print "Save image to ", Dir
		Count += 1


def Capture_ThreadProcess(sensor, profile, callback):
	sensor.open(profile)
	sensor.start(callback)
	print "Start"


def SetupCamera(camera="-l"):
	ctx = rs.context()
	if len(ctx.devices) == 0:
		print "No Realsense connect, exit."
		exit(-1)

	for d in ctx.devices:
		sensors = d.query_sensors()
		for se in sensors:
			if se.get_info(rs.camera_info.name) == "Stereo Module":
				for p in se.profiles:
					p = p.as_video_stream_profile()
					if p.width() == 1280 and p.height() == 720 and p.fps() == 30 and p.format() == rs.format.y8 and p.stream_name() == "Infrared 1":
						if camera == "-l" and d.get_info(rs.camera_info.serial_number) == EnvSetupVar.RS_SerialNumber_L:
							profile = p.as_stream_profile()
							print p.width(), p.height(), p.fps(), p.format()
							sensor = se
						elif camera == "-r" and d.get_info(
								rs.camera_info.serial_number) == EnvSetupVar.RS_SerialNumber_R:
							profile = p.as_stream_profile()
							print "Right:", p.width(), p.height(), p.fps(), p.format()
							sensor = se

	sensor.set_option(rs.option.laser_power, 0)
	return sensor, profile


def StartCapture(sensor, profile, camera="-l"):
	if camera == "-l":
		Thread = threading.Thread(target=Capture_ThreadProcess, args=[sensor, profile, cap_l])
	elif camera == "-r":
		print "Right:"
		Thread = threading.Thread(target=Capture_ThreadProcess, args=[sensor, profile, cap_r])
	Thread.setDaemon(True)
	Thread.start()
	time.sleep(0.5)  # Without delay, program will get error...

	return Thread  # , profile_l, profile_r


def move_calib(arm, camera="-l", PicturePath="../../../data/CalibrationImage/"):
	global Trigger

	arm.StandbyARM()
	arm.set_tool_xyz(-Th2tool_Cali.vec[0], Th2tool_Cali.vec[1], -Th2tool_Cali.vec[2], "hand")
	ToolPos = []
	print("Start capture!")
	cv2.waitKey()
	for i in range(len(LeftARM_Actions)):
		arm.move_rmrc(LeftARM_Actions[i])
		time.sleep(3)
		Trigger = Value('b', 1)
		print Trigger.value
		ToolPos.append(arm.t_xyzabc_now)

	print("Capture finished. \nPress Enter to back to ready position")
	cv2.waitKey()

	arm.StandbyARM()

	json_data = {"ToolPos": ToolPos}
	json_write(json_data, PicturePath + "ToolPosition.json")


def GetIntrinsic(profile):
	pv = profile.as_video_stream_profile()
	Intrinsic = np.eye(3)
	Intrinsic[0, 0] = pv.intrinsics.fx
	Intrinsic[1, 1] = pv.intrinsics.fy
	Intrinsic[0, 2] = pv.intrinsics.ppx
	Intrinsic[1, 2] = pv.intrinsics.ppy
	Distortion = np.array([0, 0, 0, 0, 0])
	return Intrinsic, Distortion


def make_jsonData(camParam, distortion, camera="-l", jsonDataName="ir.json", PicturePath="../../../data/CalibrationImage/"):
	# Read tools position data.
	jdata = json_read(PicturePath + "ToolPosition.json")
	ToolsPosition_j = jdata["ToolPos"]

	# Read image into list.
	list_img = []
	for i in range(len(ToolsPosition_j)):
		if camera == "-l":
			title = PicturePath + "Left_" + str(i) + ".png"
		elif camera == "-r":
			title = PicturePath + "Right_" + str(i) + ".png"
		list_img.append(cv2.imread(title))

	# Calculate extrinsics.
	list_Tf, isAccepted = calc_extrinsic(CHESS_SIZE, CHESS_SQUARE_SIZE, list_img, camParam, distortion)

	print np.sum(isAccepted), " images are accepted."
	print "list_Tf : ", list_Tf

	ToolsPosition = []
	listTf_Tc2cb = []
	for i, ret in enumerate(isAccepted):
		if (ret == True):
			ToolsPosition.append(ToolsPosition_j[i])
			listTf_Tc2cb.append(list_Tf[i])

	jdata = {"ToolsPosition": ToolsPosition,
				"listTf_Tc2cb": listTf_Tc2cb,
				"Th2tool_Cali": Th2tool_Cali.xyzabc(),
				"Th2c_init": Th2c_init.xyzabc()}

	json_write(jdata, jsonDataName)


def ARM_Move_subProcess(stdin, arg, child_conn, PicturePath="../../../data/CalibrationImage/"):
	import sys
	sys.stdin = os.fdopen(stdin)
	import EnvSetupVar
	if arg == "-l":
		EnvSetupVar.Need_PA10_L = True
	elif arg == "-r":
		EnvSetupVar.Need_PA10_R = True
	elif arg == "-b":
		EnvSetupVar.Need_PA10_BOTH = True
	from EnvSetup import *
	PicturePath = "../../../data/CalibrationImage/"
	if arg == "-l":
		arm = ARM_L
	elif arg == "-r":
		arm = ARM_R
	arm.StandbyARM()
	arm.set_tool_xyz(-Th2tool_Cali.vec[0], Th2tool_Cali.vec[1], -Th2tool_Cali.vec[2], "hand")
	ToolPos = []
	raw_input("Start capture!")
	for i in range(len(LeftARM_Actions)):
		if arg == "-l":
			arm.move_rmrc(LeftARM_Actions[i], DEBUG_MODE=True)
		elif arg == "-r":
			arm.move_rmrc(RightARM_Actions[i], DEBUG_MODE=True)
		time.sleep(1)
		#raw_input("Next?")
		child_conn.send(1)
		ToolPos.append(arm.t_xyzabc_now)
		time.sleep(1)

	raw_input("Capture finished. \nPress Enter to back to ready position")

	arm.StandbyARM()

	json_data = {"ToolPos": ToolPos}
	json_write(json_data, PicturePath + "ToolPosition.json")


parent_conn = 0
Count = 0
window = 0
IMG_L = 0
IMG_R = 0


if __name__ == "__main__":

	import EnvSetupVar
	if len(sys.argv) == 1:
		sys.argv.append("-l")
		sys.argv.append("-f")
	if len(sys.argv) == 3 or len(sys.argv) == 4:
		if sys.argv[1] == "-l":
			if len(sys.argv) == 4:
				PicturePath = sys.argv[3]
			sensor_l, profile_l = SetupCamera("-l")
			make_jsonData(profile_l, sys.argv[1])
		elif sys.argv[1] == "-r":
			if len(sys.argv) == 4:
				PicturePath = sys.argv[3]
			sensor_r, profile_r = SetupCamera("-r")
			make_jsonData(profile_r, sys.argv[1])
		exit()

	SysInputStream = sys.stdin.fileno()
	parent_conn, child_conn = Pipe()
	p = Process(target=ARM_Move_subProcess, args=(SysInputStream, sys.argv[1], child_conn))
	p.start()

	InitialTKWindow(sys.argv[1])
	Left_camPara = 0
	Right_camPara = 0
	Left_Distortion = 0
	Right_Distortion = 0
	if sys.argv[1] == "-l":
		sensor_l, profile_l = SetupCamera("-l")
		Left_camPara, Left_Distortion = GetIntrinsic(profile_l)
		print Left_camPara
		if len(sys.argv) < 3:
			Capture_Thread_l = StartCapture(sensor_l, profile_l, "-l")
	elif sys.argv[1] == "-r":
		sensor_r, profile_r = SetupCamera("-r")
		Right_camPara, Right_Distortion = GetIntrinsic(profile_r)
		print Right_camPara
		if len(sys.argv) < 3:
			Capture_Thread_r = StartCapture(sensor_r, profile_r, "-r")
	elif sys.argv[1] == "-b":
		sensor_l, profile_l = SetupCamera("-l")
		Left_camPara, Left_Distortion = GetIntrinsic(profile_l)
		if len(sys.argv) < 3:
			Capture_Thread_l = StartCapture(sensor_l, profile_l, "-l")
		sensor_r, profile_r = SetupCamera("-r")
		Right_camPara, Right_Distortion = GetIntrinsic(profile_r)
		if len(sys.argv) < 3:
			Capture_Thread_r = StartCapture(sensor_r, profile_r, "-r")

	window.mainloop()
	
	if sys.argv[1] == "-l":
		sensor_l.stop()
		make_jsonData(Left_camPara, Left_Distortion, sys.argv[1])
	elif sys.argv[1] == "-r":
		sensor_r.stop()
		make_jsonData(Right_camPara, Right_Distortion, sys.argv[1])
	

	exit()

