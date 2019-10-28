from __future__ import division
import pybullet as p
import pybullet_data
import utils_motoman

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

### create two servers ###
### One for planning, the other executing (ground truth) ###
planningServer = p.connect(p.GUI)
executingServer = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
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


### set up several parameter before generating the benchmark ###
bt = sys.argv[1] # choose "table" or "shelf"
scene = sys.argv[2] # choose "1" or "2" or "3"
nHypos = int(sys.argv[3]) # choose from (1-7)
noiseLevel = int(sys.argv[4]) # choose from (1-7)
transErrors = [0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035]
orientErrors = [5, 10, 15, 20, 25, 30, 35]
orientErrors = [float(format(ii * math.pi/180, '.3f')) for ii in orientErrors]

### generate the static geometries ###
########################################################################################################################
if bt == "table":
	table_path = "newChapter/table"
	try:
		os.mkdir(table_path)
	except OSError:
		print "Creation of the directory %s falied\n" % table_path
	else:
		pass
	print "---------Enter to table scene!----------"
	### create the known geometries - table ###
	table_dim = np.array([0.555, 1.11, 0.59])
	tablePosition = [0, 0, table_dim[2]/2]
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
	### create the known geometries - standingBase  ###
	standingBase_dim = np.array([0.915, 0.62, 0.19])
	standingBasePosition = [tablePosition[0]-table_dim[0]/2-standingBase_dim[0]/2-0.01, 
																tablePosition[1], standingBase_dim[2]/2]
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
	### reset the base of motoman
	motomanBasePosition = [standingBasePosition[0], standingBasePosition[1], 
													standingBasePosition[2]+standingBase_dim[2]/2+0.005]
	motomanBaseOrientation = [0, 0, 0, 1]
	p.resetBasePositionAndOrientation(motomanID_p, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(motomanID_e, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=executingServer)
	### set motoman home configuration
	home_configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

else:
	shelf_path = "newChapter/shelf"
	try:
		os.mkdir(shelf_path)
	except OSError:
		print "Creation of the directory %s falied\n" % shelf_path
	else:
		pass
	print "---------Enter to shelf scene!----------"
	### create the known geometries - shelf ###
	### shelfbase
	shelfbase_dim = np.array([0.7, 1.3, 0.3])
	shelfbasePosition = [0.0, 0.0, shelfbase_dim[2]/2]
	shelfbase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
							halfExtents=shelfbase_dim/2, physicsClientId=planningServer)
	shelfbase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim/2, 
								rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
	shelfbaseM_p = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c_p,
							baseVisualShapeIndex=shelfbase_v_p,
								basePosition=shelfbasePosition, physicsClientId=planningServer)
	shelfbase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
							halfExtents=shelfbase_dim/2, physicsClientId=executingServer)
	shelfbase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim/2, 
								rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
	shelfbaseM_e = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c_e,
							baseVisualShapeIndex=shelfbase_v_e,
								basePosition=shelfbasePosition, physicsClientId=executingServer)
	known_geometries_planning.append(shelfbaseM_p)
	known_geometries_executing.append(shelfbaseM_e)
	print "shelf base: " + str(shelfbaseM_e)
	### the shape and visual of the flank
	flank_dim = np.array([shelfbase_dim[0], 0.06, 1.2])
	flank_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=flank_dim/2, physicsClientId=planningServer)
	flank_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2, 
							rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
	flank_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=flank_dim/2, physicsClientId=executingServer)
	flank_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2, 
							rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
	### left flank
	leftflankPosition = [0.0, shelfbase_dim[1]/2-flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	leftflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p, 
								baseVisualShapeIndex=flank_v_p, 
									basePosition=leftflankPosition, physicsClientId=planningServer)
	leftflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e, 
								baseVisualShapeIndex=flank_v_e, 
									basePosition=leftflankPosition, physicsClientId=executingServer)
	known_geometries_planning.append(leftflankM_p)
	known_geometries_executing.append(leftflankM_e)
	print "left flank: " + str(leftflankM_e)
	### right flank
	rightflankPosition = [0.0, -shelfbase_dim[1]/2+flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	rightflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
								baseVisualShapeIndex=flank_v_p, 
									basePosition=rightflankPosition, physicsClientId=planningServer)
	rightflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
								baseVisualShapeIndex=flank_v_e, 
									basePosition=rightflankPosition, physicsClientId=executingServer)
	known_geometries_planning.append(rightflankM_p)
	known_geometries_executing.append(rightflankM_e)
	print "right flank: " + str(rightflankM_e)
	### middle flank
	middleflankPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
							baseVisualShapeIndex=flank_v_p, 
								basePosition=middleflankPosition, physicsClientId=planningServer)
	middleflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
							baseVisualShapeIndex=flank_v_e, 
								basePosition=middleflankPosition, physicsClientId=executingServer)
	known_geometries_planning.append(middleflankM_p)
	known_geometries_executing.append(middleflankM_e)
	print "middle flank: " + str(middleflankM_e)
	### the shape and visual of the flat
	flat_dim = np.array([shelfbase_dim[0], shelfbase_dim[1], 0.06])
	flat_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
									halfExtents=flat_dim/2, physicsClientId=planningServer)
	flat_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2, 
							rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
	flat_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
									halfExtents=flat_dim/2, physicsClientId=executingServer)
	flat_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2, 
							rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
	### middle flat
	middleflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
						baseVisualShapeIndex=flat_v_p, 
							basePosition=middleflatPosition, physicsClientId=planningServer)
	middleflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
						baseVisualShapeIndex=flat_v_e, 
							basePosition=middleflatPosition, physicsClientId=executingServer)
	known_geometries_planning.append(middleflatM_p)
	known_geometries_executing.append(middleflatM_e)
	print "middle flat: " + str(middleflatM_e)
	### top flat
	topflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]+flat_dim[2]/2]
	topflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
						baseVisualShapeIndex=flat_v_p, 
							basePosition=topflatPosition, physicsClientId=planningServer)
	topflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
						baseVisualShapeIndex=flat_v_e, 
							basePosition=topflatPosition, physicsClientId=executingServer)
	known_geometries_planning.append(topflatM_p)
	known_geometries_executing.append(topflatM_e)
	print "top flat: " + str(topflatM_e)
	### back
	shelfback_dim = np.array([0.02, shelfbase_dim[1], flank_dim[2]+flat_dim[2]])
	shelfbackPosition = [shelfbase_dim[0]/2-shelfback_dim[0]/2, 0.0, 
														shelfbase_dim[2]+shelfback_dim[2]/2]
	shelfback_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
																physicsClientId=planningServer)
	shelfback_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
								rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
	shelfback_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
																physicsClientId=executingServer)
	shelfback_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
								rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
	shelfbackM_p = p.createMultiBody(baseCollisionShapeIndex=shelfback_c_p,
							baseVisualShapeIndex=shelfback_v_p, 
								basePosition=shelfbackPosition, physicsClientId=planningServer)
	shelfbackM_e = p.createMultiBody(baseCollisionShapeIndex=shelfback_c_e,
							baseVisualShapeIndex=shelfback_v_e, 
								basePosition=shelfbackPosition, physicsClientId=executingServer)
	known_geometries_planning.append(shelfbackM_p)
	known_geometries_executing.append(shelfbackM_e)
	print "shelf back: " + str(shelfbackM_e)
	### reset the base of Motoman
	motomanBasePosition = [shelfbasePosition[0]-shelfbase_dim[0]/2-0.8, shelfbasePosition[1], 0.0]
	motomanBaseOrientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
	p.resetBasePositionAndOrientation(motomanID_p, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(motomanID_e, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=executingServer)
	### set motoman home configuration
	home_configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

########################################################################################################################

### specify the objects in a chosen scene ###
if bt == "table" and scene == "1":
	path = table_path + "/scenario1"
	print "--------Welcome to " + bt + " sceneario " + scene + "--------"
	try:
		os.mkdir(path)
	except OSError:
		print "Creation of the directory %s falied\n" % path
	else:
		pass

	### Try different YCB objects in the scene to see the time for importing
	###################################################################################################################
	Objects = dict()
	### objIdx, meshFile, objectName, objectRole, scale, true_pos, 
	### true_angles(radians), mass, nHypos, prob(not in the scene)
	Objects[0] = [0, "/mesh/009_gelatin_box/gelatin_box_512_reduced.obj", "gelatin box", 
		"target", 1, [0.0, 0.29, 0.002+table_dim[2]], [0, 0, math.pi/7], 1.4, nHypos, 0.08]
	# Objects[0] = [0, "/mesh/006_mustard_bottle_64/mustard_bottle_64.obj", "mustard bottle",
	# 	"target", 1, [0.2, 0.2, 0.1+table_dim[2]], [0, 0, 0], 2.1, nHypos, 0.16]
	Objects[1] = [1, "/mesh/006_mustard_bottle_512/mustard_bottle_512_reduced.obj", "mustard bottle",
		"normal", 1, [-0.145, 0.37, 0.005+table_dim[2]], [0, 0, math.pi/4], 2.1, nHypos, 0.16]
	# Objects[0] = [0, "/mesh/006_mustard_bottle_512/mustard_bottle_512.obj", "mustard bottle",
	# 	"normal", 1, [0.4, 0.4, 0.1+table_dim[2]], [0, 0, 0], 2.1, nHypos, 0.16]
	Objects[2] = [2, "/mesh/021_bleach_cleanser/bleach_cleanser_512_reduced.obj", "bleach cleanser",
		"normal", 1, [0.01, 0.37, 0.004+table_dim[2]], [0, 0, 0], 4.2, nHypos, 0.11]
	Objects[3] = [3, "/mesh/003_cracker_box/cracker_box_512_reduced.obj", "cracker box",
		"normal", 1, [-0.02, 0.18, 0.005+table_dim[2]], [0, 0, math.pi/2], 4.8, nHypos, 0.10]
	Objects[4] = [4, "/mesh/008_pudding_box/pudding_box_512_reduced.obj", "pudding box",
		"phantom", 1, [0.05, 0.33, 0.002+table_dim[2]], [0, 0, -3*math.pi/4], 1.4, nHypos, 0.86]

	### pick goal offset
	goalPos_offset = [0.0, 0.0, 0.05]
	goalEuler = [0.0, math.pi, 0.0] ### overhan grasps
	x_ll = motomanBasePosition[0] - 0.35
	x_ul = tablePosition[0] + table_dim[0]/2
	y_ll = -table_dim[1]/2
	y_ul = table_dim[1]/2
	z_ll = table_dim[2]
	z_ul = table_dim[2] + 0.9


	####################################################################################################################

### At here we finish selecting the specific scene for specific benchmark type
### Now let's generate ground truth of the specific scene and specific benchmark
startTime = time.clock()
truePoses, nObjectInExecuting = utils_motoman.trueScene_generation(bt, scene, Objects, executingServer)
print "Time elapsed to load the objects as the ground truth: ", time.clock() - startTime
utils_motoman.planScene_generation(Objects, bt, 
			known_geometries_planning, transErrors[noiseLevel-1], orientErrors[noiseLevel-1], executingServer)
startTime = time.clock()
hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = utils_motoman.planScene_generation(Objects, bt, 
			known_geometries_planning, transErrors[noiseLevel-1], orientErrors[noiseLevel-1], planningServer)
print "Time elapsed to load the objects in the planning scene: ", time.clock() - startTime
print "most promisings: " + str(mostPromisingHypoIdxes)




###################################test reachability of the robot arm##################################################
### specify q_start and set of q_goal first
q_start = home_configuration
### generate goal configurations
goalSet = []
goalHypos = []
MaxGoalsPerPose = 5
MaxTrialsPerPose = 7
MaxTrialsPerEE = 7

### for each target hypotheses
for t_hp in xrange(Objects[0][8]):
	print "***********For Hypo " + str(t_hp) + "***************"
	temp_goalsForThatHypo = []
	temp_survivalForThatHypo = []
	### specify the position of the goal pose for that particular target hypothsis
	goal_pose_pos = []
	for i in xrange(len(goalPos_offset)):
		goal_pose_pos.append(hypotheses[t_hp].pos[i] + goalPos_offset[i])
	print "goal_pose_pos: " + str(goal_pose_pos)
	temp_trials_pose = 0
	while temp_trials_pose < MaxTrialsPerPose:
		print "-----A new pose-----"
		for j in range(1, 8):
			result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=planningServer)
		for j in range(11, 18):
			result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=planningServer)
		### specify the quaternion of that particular goal pose
		temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
													goalEuler[2]+random.uniform(-math.pi, math.pi)])
		goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
		### now start to exploring the IKs
		temp_trials_ee = 0
		while temp_trials_ee < MaxTrialsPerEE:
			q_goal = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx, goal_pose_pos, goal_pose_quat, 
																			ll, ul, jr, physicsClientId=planningServer)
			for j in range(1, 8):
				result_p = p.resetJointState(motomanID_p, j, q_goal[j-1], physicsClientId=planningServer)
			for j in range(11, 18):
				result_p = p.resetJointState(motomanID_p, j, q_goal[j-4], physicsClientId=planningServer)
			p.stepSimulation(planningServer)
			### check collision for robot self and known obstacles
			isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
			isCollisionKnownObs = utils_motoman.collisionCheck_knownObs(motomanID_p, known_geometries_planning, 
																									planningServer)
			if isCollisionSelf or isCollisionKnownObs:
				print "Collision with robot itself or known obstacles, ee +2!"
				temp_trials_ee += 2 ## This may be a bad pose, so let's check it in a quicker way
				raw_input("Press Enter to continue")
				continue
			else:
				### check collision condition with all hypos of objects
				### (The opposite of the collision probability)
				collidedHypos = utils_motoman.collisionCheck_hypos(motomanID_p, hypotheses, planningServer)
				print "Collide with Hypos: " + str(collidedHypos)
				### compute the survivability
				if t_hp in collidedHypos:
					### the end effector collides with the pose it deems as the target pose
					temp_survival = 0.0
					temp_trials_ee += 1
					print "Survival: " + str(temp_survival)
					raw_input("Press Enter to continue")
					continue
				else:
					temp_survival = 1.0
					collisionPerObj = [0.0] * nObjectInPlanning
					for ch in collidedHypos:
						if hypotheses[ch].objIdx != 0:
							collisionPerObj[hypotheses[ch].objIdx] += hypotheses[ch].prob
					for cpobs in collisionPerObj:
						temp_survival *= (1 - cpobs)
					print "Survival: " + str(temp_survival)
					raw_input("Press Enter to continue")
					### add the q_goals and its corresponding survivability
					temp_goalsForThatHypo.append(q_goal)
					temp_survivalForThatHypo.append(temp_survival)
					temp_trials_ee += 1
		### finish the current pose
		temp_trials_pose += 1

	### You are here since you finish generating poses for a particular hypothesis
	### sort temp_survivalForThatHypo and pick top (MaxGoalsPerPose) ones
	idx_rank = sorted( range(len(temp_survivalForThatHypo)), key=lambda k: temp_survivalForThatHypo[k], reverse=True )
	if len(idx_rank) >= MaxGoalsPerPose:
		### add top (MaxGoalsPerPose) ones
		for mm in xrange(MaxGoalsPerPose):
			goalSet.append(temp_goalsForThatHypo[idx_rank[mm]])
			goalHypos.append(t_hp)
print "goalHypos:" + str(goalHypos)




time.sleep(10000)




















################## has to go back and solve self collision ############################
# # goal_pose_pos = [motomanBasePosition[0]+0.05, motomanBasePosition[1], 1.1]
# goal_pose_pos = [0.20, 0.15, 0.4]
# temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
# 											goalEuler[2]+random.uniform(-math.pi, math.pi)])
# goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
# q_goal = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx, goal_pose_pos, goal_pose_quat, 
# 																	ll, ul, jr, physicsClientId=planningServer)
# for j in range(1, 8):
# 	result_p = p.resetJointState(motomanID_p, j, q_goal[j-1], physicsClientId=planningServer)
# for j in range(11, 18):
# 	result_p = p.resetJointState(motomanID_p, j, q_goal[j-4], physicsClientId=planningServer)
# p.stepSimulation(planningServer)
# isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
# if isCollisionSelf:
# 	print "self collision occurs!"
# raw_input("put the arm back to home configuration by pressing enter...")
# for j in range(1, 8):
# 	result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=planningServer)
# for j in range(11, 18):
# 	result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=planningServer)
# p.stepSimulation(planningServer)
# isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
# if isCollisionSelf:
# 	print "self collision occurs!"