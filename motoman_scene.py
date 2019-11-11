from __future__ import division
import pybullet as p
import pybullet_data
import utils_scene

import math
import random
import time
import numpy as np

import sys
import os
import subprocess

from scipy import spatial
import cPickle as pickle

import IPython
from collections import OrderedDict

### create two servers ###
### One for planning, the other executing (ground truth) ###
planningServer = p.connect(p.GUI)
executingServer = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# print pybullet_data.getDataPath()
### set the real-time physics simulation ###
# p.setGravity(0.0, 0.0, -9.8, executingServer)
# p.setRealTimeSimulation(1, executingServer)
known_geometries_planning = []
known_geometries_executing = []

### Introduce Motoman arm ###
motomanID_p = p.loadURDF("motoman.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=planningServer)
motomanID_e = p.loadURDF("motoman.urdf", useFixedBase=True, physicsClientId=executingServer)
known_geometries_planning.append(motomanID_p)
known_geometries_executing.append(motomanID_e)

### preserve the following five lines for test purposes ###
# print "Motoman Robot: " + str(motomanID_p)
# num_joints = p.getNumJoints(motomanID_p, planningServer)
# print "Num of joints: " + str(num_joints)
# for i in range(num_joints):
# 	print(p.getJointInfo(motomanID_p, i, planningServer))

########## information related to Motoman ###########
motoman_ee_idx = 10
### There is a torso joint which connects the lower and upper body (-2.957 ~ 2.957)
### But so far we decide to make that torso joint fixed
### For each arm, there are 10 joints and 7 of them are revolute joints
### There are total 14 revolute joints for each arm
### lower limits for null space
ll = [-3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13]
### upper limits for null space
ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, -2.95, 2.36, 3.13, 1.90, 3.13]
### joint ranges for null space
jr = [6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
### restposes for null space
rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


################################# table scene #####################################################
print "---------Enter to table scene!----------"
### reset the base of motoman
motomanBasePosition = [0, 0, 0]
motomanBaseOrientation = [0, 0, 0, 1]
p.resetBasePositionAndOrientation(motomanID_p, motomanBasePosition, 
								motomanBaseOrientation, physicsClientId=planningServer)
p.resetBasePositionAndOrientation(motomanID_e, motomanBasePosition, 
								motomanBaseOrientation, physicsClientId=executingServer)
### set motoman home configuration
home_configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

### create the known geometries - standingBase  ###
standingBase_dim = np.array([0.915, 0.62, 0.19])
standingBasePosition = [motomanBasePosition[0], motomanBasePosition[1], motomanBasePosition[2]-standingBase_dim[2]/2-0.005]
standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
							halfExtents=standingBase_dim/2, physicsClientId=planningServer)
standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, 
							halfExtents=standingBase_dim/2, physicsClientId=planningServer)
standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_p, baseVisualShapeIndex=standingBase_v_p,
							basePosition=standingBasePosition, physicsClientId=planningServer)
standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
							halfExtents=standingBase_dim/2, physicsClientId=executingServer)
standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, 
							halfExtents=standingBase_dim/2, physicsClientId=executingServer)
standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_e, baseVisualShapeIndex=standingBase_v_e,
							basePosition=standingBasePosition, physicsClientId=executingServer)
known_geometries_planning.append(standingBaseM_p)
known_geometries_executing.append(standingBaseM_e)
print "standing base: " + str(standingBaseM_e)
### create the known geometries - table ###
table_dim = np.array([0.555, 1.11, 0.59])
# tablePosition = [motomanBasePosition[0]+standingBase_dim[0]/2+table_dim[0]/2, motomanBasePosition[1], 
				# motomanBasePosition[2]+(table_dim[2]/2-standingBase_dim[2]-0.005)]
tablePosition = [motomanBasePosition[0]+standingBase_dim[0]/2+table_dim[0]/2, motomanBasePosition[1], 
				motomanBasePosition[2]+(table_dim[2]/2-standingBase_dim[2]-0.005)]
table_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=table_dim/2, physicsClientId=planningServer)
table_v_p = p.createVisualShape(shapeType=p.GEOM_BOX,
						halfExtents=table_dim/2, physicsClientId=planningServer)
tableM_p = p.createMultiBody(baseCollisionShapeIndex=table_c_p, baseVisualShapeIndex=table_v_p,
									basePosition=tablePosition, physicsClientId=planningServer)
table_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=table_dim/2, physicsClientId=executingServer)
table_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
						halfExtents=table_dim/2, physicsClientId=executingServer)
tableM_e = p.createMultiBody(baseCollisionShapeIndex=table_c_e, baseVisualShapeIndex=table_v_e,
									basePosition=tablePosition, physicsClientId=executingServer)
known_geometries_planning.append(tableM_p)
known_geometries_executing.append(tableM_e)
print "table: " + str(tableM_e)
####################################### end of table scene setup ##########################################


print("---------camera information---------")
camera_extrinsic = np.array(
	[[-0.0129781, -0.7283973,  0.6850321, 0.330336], 
	 [-0.9996760,  0.0244548,  0.0070638, -0.0345459],
	 [-0.0218976, -0.6847184, -0.7284787, 1.24119], 
	 [0.0, 0.0, 0.0, 1.0]])

img_index = sys.argv[1]
# nsamples = int(sys.argv[2]) ### choose from (1000-5000)

Objects = OrderedDict() ### key: Object names / value: (1) mesh file (2) existence probability
prob_stat_file = "../Object_Existence_Network/data/probability_statistics/" + img_index + "_probs.txt"
f_prob_stat = open(prob_stat_file)
for line in f_prob_stat:
	line = line.split()
	Objects[line[0]] = ["mesh/"+line[0]+"/google_16k/textured.obj", float( format(float(line[1]), '.3f') )]


hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = \
	utils_scene.planScene_generation(Objects, img_index, camera_extrinsic, planningServer)




time.sleep(10000)


# startTime = time.clock()
# for object_name in object_names:
# 	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="mesh/" + object_name + "/google_16k/textured.obj", 
# 			meshScale=[1,1,1], physicsClientId=planningServer)
# 	_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="mesh/" + object_name + "/google_16k/textured.obj", 
# 			meshScale=[1, 1, 1], rgbaColor=[1.0, 1.0, 1.0, 1.0], physicsClientId=planningServer)
# 	_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
# 		basePosition=[tablePosition[0]+0.2, tablePosition[1]+0.4, tablePosition[2]+table_dim[2]/2+0.4], 
# 		baseOrientation=[0,0,0,1], physicsClientId=planningServer)
# print("Time elapse to load a 006_mustard_bottle: " + str(time.clock()-startTime))



