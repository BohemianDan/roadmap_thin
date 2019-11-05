from __future__ import division 
import pybullet as p
import numpy as np
import pybullet_data
import IPython
import math
import time
import random

from scipy import spatial
import cPickle as pickle

def rotationMatrixToQuaternion(T):
	### first compute the trace
	tr = T[0][0] + T[1][1] + T[2][2]

	if (tr > 0):
		S = math.sqrt(tr+1.0) * 2
		qw = 0.25 * S
		qx = (T[2][1] - T[1][2]) / S
		qy = (T[0][2] - T[2][0]) / S
		qz = (T[1][0] - T[0][1]) / S
	elif ( (T[0][0]>T[1][1]) and (T[0][0]>T[2][2]) ):
		S = math.sqrt(1.0+T[0][0]-T[1][1]-T[2][2]) * 2
		qw = (T[2][1] - T[1][2]) / S
		qx = 0.25 * S
		qy = (T[0][1] + T[1][0]) / S
		qz = (T[0][2] + T[2][0]) / S
	elif (T[1][1] > T[2][2]):
		S = math.sqrt(1.0+T[1][1]-T[0][0]-T[2][2]) * 2
		qw = (T[0][2] - T[2][0]) / S
		qx = (T[0][1] + T[1][0]) / S
		qy = 0.25 * S
		qz = (T[1][2] + T[2][1]) / S
	else:
		S = math.sqrt(1.0+T[2][2]-T[0][0]-T[1][1]) * 2
		qw = (T[1][0] - T[0][1]) / S
		qx = (T[0][2] + T[2][0]) / S
		qy = (T[1][2] + T[2][1]) / S
		qz = 0.25 * S


	return [qx, qy, qz, qw]


