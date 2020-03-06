import sys
import os
import time
import datetime
import cv2
import json

import numpy as np

# import world path ===================
import WorkspaceDirInfo as shareDir

sys.path.append(shareDir.WorkspaceDir)

import share.tools.geo.geo as geo


def json_read(filePath):
	f = open(filePath, "r")
	json_data = json.load(f)
	f.close()

	return json_data


def json_write(dict, filePath):
	f = open(filePath, "w")
	json.dump(dict, f, indent=4)
	f.close()



def getDateString():
	date = datetime.datetime.today()
	str_date = "%s_%s_%s__%s_%s_%s_%s" % (date.year, date.month, date.day,
										date.hour, date.minute, date.second, date.microsecond)
	return str_date


def CompareDateString(str1, str2):
	date1 = datetime.datetime.strptime(str1, "%Y_%m_%d__%H_%M_%S_%f")
	date2 = datetime.datetime.strptime(str2, "%Y_%m_%d__%H_%M_%S_%f")
	return date1 > date2


def cvt_FRAME2npmat(frm):
	npmat = np.matrix([
		[frm.mat[0][0], frm.mat[0][1], frm.mat[0][2], frm.vec[0]],
		[frm.mat[1][0], frm.mat[1][1], frm.mat[1][2], frm.vec[1]],
		[frm.mat[2][0], frm.mat[2][1], frm.mat[2][2], frm.vec[2]],
		[0.0, 0.0, 0.0, 1.0]
	])
	return npmat


def cvt_npmat2FRAME(npmat):
	frm = geo.FRAME(
		mat=[
			[npmat[0, 0], npmat[0, 1], npmat[0, 2]],
			[npmat[1, 0], npmat[1, 1], npmat[1, 2]],
			[npmat[2, 0], npmat[2, 1], npmat[2, 2]]
		],
		vec=[npmat[0, 3], npmat[1, 3], npmat[2, 3]])
	return frm


def calc_extrinsic(tuple_chessSize, chessSquareSize, list_img, camParam, distortion):
	# create object points
	objPoints = []
	for i in range(0, tuple_chessSize[1]):
		for j in range(0, tuple_chessSize[0]):
			objPoints.append((i * chessSquareSize, j * chessSquareSize, 0))
	objPoints = np.array(objPoints)

	list_Tf = []
	list_accept = []
	counter = 0
	# process each images
	for img in list_img:
		print "img : %d" % counter
		# cvt img BGR to GRAY
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		# find chessboard corners
		found, corners = cv2.findChessboardCorners(gray, tuple_chessSize)
		if found:
			# find corner sub pix
			cv2.cornerSubPix(
				image=gray,
				corners=corners,
				winSize=(5, 5),
				zeroZone=(-1, -1),
				criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

			# calc extrinsic
			print 'distortion'
			objPoints = objPoints.astype(np.float64)
			found, r_vec, t_vec = cv2.solvePnP(
				objectPoints=objPoints,
				imagePoints=corners,
				cameraMatrix=camParam,
				distCoeffs=distortion)

			r_mtx, scratch = cv2.Rodrigues(r_vec)
			Tf = np.matrix([
				[r_mtx[0, 0], r_mtx[0, 1], r_mtx[0, 2], t_vec[0, 0] * 0.001],
				[r_mtx[1, 0], r_mtx[1, 1], r_mtx[1, 2], t_vec[1, 0] * 0.001],
				[r_mtx[2, 0], r_mtx[2, 1], r_mtx[2, 2], t_vec[2, 0] * 0.001],
				[0., 0., 0., 1.],  # t_vec is converted [mm]->[m]
			])

			list_Tf.append(Tf.tolist())
			list_accept.append(True)
			print "Transform"
			print Tf

		else:
			list_Tf.append(None)
			list_accept.append(False)
			print "FAILED to fined corner"

		print ""
		cv2.drawChessboardCorners(img, tuple_chessSize, corners, found)
		cv2.imshow("ChessBoard Corners", img)
		cv2.waitKey()

		counter += 1

	cv2.destroyWindow("ChessBoard Corners")
	return list_Tf, list_accept

