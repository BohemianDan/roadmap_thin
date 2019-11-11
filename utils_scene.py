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

class Mesh:
	def __init__(self, m, objName, hypoIdx, pos, quat, prob):
		self.m = m
		self.objName = objName
		self.hypoIdx = hypoIdx
		self.pos = pos
		self.quat = quat
		self.prob = prob


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

def createHypotheses(currentlabelIdx, object_name, meshFile_and_existProb, img_index, camera_extrinsic, clientID):
	### starting label index for the current obs (currentlabelIdx)
	### with the mesh file
	### create the collision and visual shape of the object
	hypotheses = []

	### first specify the collision (they are identical within the same object)
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshFile_and_existProb[0], meshScale=[1,1,1], physicsClientId=clientID)
	largest_prob = 0.0
	largest_prob_idx = currentlabelIdx

	### deal with scores first
	score_path = "../model_matching/examples/ycb/pose_candidates/" + img_index + "/" + object_name + "/best_pose_scores.txt"
	temp_scores = []
	f_scores = open(score_path)
	for line in f_scores:
		line = line.split()
		temp_scores.append(float(format(float(line[0]), '3f')))
	### convert the scores to probabilities based on existence probability
	score_sum = sum(temp_scores)
	for i in xrange(len(temp_scores)):
		temp_scores[i] = temp_scores[i] * meshFile_and_existProb[1] / score_sum

	### Now deal with transforms (position+orientation)
	counter = 0
	scene_path = "../model_matching/examples/ycb/pose_candidates/" + img_index + "/" + object_name + "/best_pose_candidates.txt"
	f_transforms = open(scene_path)
	for line in f_transforms:
		temp_matrix = np.zeros(shape=(4,4))
		line = line.split()
		for i in xrange(len(line)):
			temp_matrix[i // 4][i % 4] = line[i]
		temp_matrix[3][3] = 1
		temp_matrix = np.dot(camera_extrinsic, temp_matrix)
		### get position(x,y,z) and orientation(quaternion:qx,qy,qz,qw)
		temp_pos = [ temp_matrix[0][3], temp_matrix[1][3], temp_matrix[2][3] ]
		temp_quat = rotationMatrixToQuaternion(temp_matrix)

		### create visual shape (level of transparency reflects probability/confidence)
		_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=meshFile_and_existProb[0], meshScale=[1,1,1], 
									rgbaColor=[1.0, 1.0, 1.0, temp_scores[counter]], physicsClientId=clientID)
		### create multibody of the current hypothesis
		_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
			basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=clientID)		

		### update the most promosing hypothesis for the current object
		if temp_scores[counter] > largest_prob:
			largest_prob = temp_scores[counter]
			largest_prob_idx = currentlabelIdx + counter
		### add this hypothesis
		hypotheses.append( Mesh(_m, object_name, currentlabelIdx+counter, temp_pos, temp_quat, temp_scores[counter]) )

		counter += 1

	return hypotheses, largest_prob_idx


def printPoses(meshes):
	for mesh in meshes:
		print("hypo " + str(mesh.hypoIdx) + " " + str(mesh.pos) + " " + str(mesh.quat) + " " + \
				str(mesh.prob) + "\tfor object " + mesh.objName )

	print "--------------------------------------\n"



def planScene_generation(Objects, img_index, camera_extrinsic, clientID):
	hypotheses = []
	mostPromisingHypoIdxes = []
	currentlabelIdx = 0
	nObjectInPlanning = 0
	for object_name in Objects:
		if Objects[object_name][1] <= 0.35:
			continue
		mm, pp = createHypotheses(currentlabelIdx, object_name, Objects[object_name], img_index, camera_extrinsic, clientID)
		hypotheses += mm
		mostPromisingHypoIdxes.append(pp)
		nObjectInPlanning += 1
		currentlabelIdx = len(hypotheses)
	print "Number of Objects in the planning scene: " + str(nObjectInPlanning)
	if clientID == 0:
		printPoses(hypotheses)
	return hypotheses, mostPromisingHypoIdxes, nObjectInPlanning





