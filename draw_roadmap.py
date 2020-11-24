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
import pickle as pickle
import graph_logic


# import IPython

def get_position(body, joint_num):
    return p.getLinkState(body, joint_num, computeForwardKinematics=1)[0]
    """starting_point, parent = p.getJointInfo(body, joint_num)[14::2]

    if parent == -1:
        parent_frame = p.getBasePositionAndOrientation(body)[0]
    else:
        parent_frame = get_position(body, parent)
    return [x + y for x, y in zip(parent_frame, starting_point)]"""


def main(*args):
    ### create two servers ###
    ### One for planning, the other executing (ground truth) ###
    planningServer = p.connect(p.DIRECT)
    executingServer = p.connect(p.GUI)  # TODO: FIX
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print(pybullet_data.getDataPath())
    ### set the real-time physics simulation ###
    # p.setGravity(0.0, 0.0, -9.8, executingServer)
    # p.setRealTimeSimulation(1, executingServer)
    known_geometries_planning = []
    known_geometries_executing = []

    ### Introduce Motoman arm ###
    motomanID_p = p.loadURDF("fix.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION,
                             physicsClientId=planningServer)
    motomanID_e = p.loadURDF("fix.urdf", useFixedBase=True, physicsClientId=executingServer)
    known_geometries_planning.append(motomanID_p)
    known_geometries_executing.append(motomanID_e)

    ### preserve the following five lines for test purposes ###
    print("Motoman Robot: " + str(motomanID_p))
    num_joints = p.getNumJoints(motomanID_p, planningServer)
    print("Num of joints: " + str(num_joints))
    for i in range(num_joints):
        print((p.getJointInfo(motomanID_p, i, planningServer)))

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
    bt = args[1]  # choose "table" or "shelf"
    scene = args[2]  # choose "1" or "2" or "3"
    nHypos = int(args[3])  # choose from (1-7)
    noiseLevel = int(args[4])  # choose from (1-7)
    transErrors = [0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035]
    orientErrors = [5, 10, 15, 20, 25, 30, 35]
    orientErrors = [float(format(ii * math.pi / 180, '.3f')) for ii in orientErrors]
    nsamples = int(args[5])  # choose from (1000-5000), if the sampling is heuristic, 1000-2000 may be enough

    ### generate the static geometries ###
    ########################################################################################################################
    if bt == "table":
        table_path = "newChapter/table"
        try:
            os.mkdir(table_path)
        except OSError:
            print("Creation of the directory %s falied\n" % table_path)
        else:
            pass
        print("---------Enter to table scene!----------")
        ### create the known geometries - table ###
        table_dim = np.array([0.555, 1.11, 0.59])
        tablePosition = [0, 0, table_dim[2] / 2]
        table_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                           halfExtents=table_dim / 2, physicsClientId=planningServer)
        table_v_p = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        halfExtents=table_dim / 2, physicsClientId=planningServer)
        tableM_p = p.createMultiBody(baseCollisionShapeIndex=table_c_p, baseVisualShapeIndex=table_v_p,
                                     basePosition=tablePosition, physicsClientId=planningServer)
        table_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                           halfExtents=table_dim / 2, physicsClientId=executingServer)
        table_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        halfExtents=table_dim / 2, physicsClientId=executingServer)
        tableM_e = p.createMultiBody(baseCollisionShapeIndex=table_c_e, baseVisualShapeIndex=table_v_e,
                                     basePosition=tablePosition, physicsClientId=executingServer)
        known_geometries_planning.append(tableM_p)
        known_geometries_executing.append(tableM_e)
        print("table: " + str(tableM_e))
        ### create the known geometries - standingBase  ###
        standingBase_dim = np.array([0.915, 0.62, 0.19])
        standingBasePosition = [tablePosition[0] - table_dim[0] / 2 - standingBase_dim[0] / 2 - 0.01,
                                tablePosition[1], standingBase_dim[2] / 2]
        standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                  halfExtents=standingBase_dim / 2, physicsClientId=planningServer)
        standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX,
                                               halfExtents=standingBase_dim / 2, physicsClientId=planningServer)
        standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_p,
                                            baseVisualShapeIndex=standingBase_v_p,
                                            basePosition=standingBasePosition, physicsClientId=planningServer)
        standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                  halfExtents=standingBase_dim / 2, physicsClientId=executingServer)
        standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
                                               halfExtents=standingBase_dim / 2, physicsClientId=executingServer)
        standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_e,
                                            baseVisualShapeIndex=standingBase_v_e,
                                            basePosition=standingBasePosition, physicsClientId=executingServer)
        known_geometries_planning.append(standingBaseM_p)
        known_geometries_executing.append(standingBaseM_e)
        print("standing base: " + str(standingBaseM_e))
        ### reset the base of motoman
        motomanBasePosition = [standingBasePosition[0], standingBasePosition[1],
                               standingBasePosition[2] + standingBase_dim[2] / 2 + 0.005]
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
            print("Creation of the directory %s falied\n" % shelf_path)
        else:
            pass
        print("---------Enter to shelf scene!----------")
        ### create the known geometries - shelf ###
        ### shelfbase
        shelfbase_dim = np.array([0.7, 1.3, 0.3])
        shelfbasePosition = [0.0, 0.0, shelfbase_dim[2] / 2]
        shelfbase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                               halfExtents=shelfbase_dim / 2, physicsClientId=planningServer)
        shelfbase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim / 2,
                                            rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
        shelfbaseM_p = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c_p,
                                         baseVisualShapeIndex=shelfbase_v_p,
                                         basePosition=shelfbasePosition, physicsClientId=planningServer)
        shelfbase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                               halfExtents=shelfbase_dim / 2, physicsClientId=executingServer)
        shelfbase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim / 2,
                                            rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
        shelfbaseM_e = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c_e,
                                         baseVisualShapeIndex=shelfbase_v_e,
                                         basePosition=shelfbasePosition, physicsClientId=executingServer)
        known_geometries_planning.append(shelfbaseM_p)
        known_geometries_executing.append(shelfbaseM_e)
        print("shelf base: " + str(shelfbaseM_e))
        ### the shape and visual of the flank
        flank_dim = np.array([shelfbase_dim[0], 0.06, 1.2])
        flank_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                           halfExtents=flank_dim / 2, physicsClientId=planningServer)
        flank_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim / 2,
                                        rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
        flank_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                           halfExtents=flank_dim / 2, physicsClientId=executingServer)
        flank_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim / 2,
                                        rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
        ### left flank
        leftflankPosition = [0.0, shelfbase_dim[1] / 2 - flank_dim[1] / 2, shelfbase_dim[2] + flank_dim[2] / 2]
        leftflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
                                         baseVisualShapeIndex=flank_v_p,
                                         basePosition=leftflankPosition, physicsClientId=planningServer)
        leftflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
                                         baseVisualShapeIndex=flank_v_e,
                                         basePosition=leftflankPosition, physicsClientId=executingServer)
        known_geometries_planning.append(leftflankM_p)
        known_geometries_executing.append(leftflankM_e)
        print("left flank: " + str(leftflankM_e))
        ### right flank
        rightflankPosition = [0.0, -shelfbase_dim[1] / 2 + flank_dim[1] / 2, shelfbase_dim[2] + flank_dim[2] / 2]
        rightflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
                                          baseVisualShapeIndex=flank_v_p,
                                          basePosition=rightflankPosition, physicsClientId=planningServer)
        rightflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
                                          baseVisualShapeIndex=flank_v_e,
                                          basePosition=rightflankPosition, physicsClientId=executingServer)
        known_geometries_planning.append(rightflankM_p)
        known_geometries_executing.append(rightflankM_e)
        print("right flank: " + str(rightflankM_e))
        ### middle flank
        middleflankPosition = [0.0, 0.0, shelfbase_dim[2] + flank_dim[2] / 2]
        middleflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
                                           baseVisualShapeIndex=flank_v_p,
                                           basePosition=middleflankPosition, physicsClientId=planningServer)
        middleflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
                                           baseVisualShapeIndex=flank_v_e,
                                           basePosition=middleflankPosition, physicsClientId=executingServer)
        known_geometries_planning.append(middleflankM_p)
        known_geometries_executing.append(middleflankM_e)
        print("middle flank: " + str(middleflankM_e))
        ### the shape and visual of the flat
        flat_dim = np.array([shelfbase_dim[0], shelfbase_dim[1], 0.06])
        flat_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                          halfExtents=flat_dim / 2, physicsClientId=planningServer)
        flat_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim / 2,
                                       rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
        flat_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                          halfExtents=flat_dim / 2, physicsClientId=executingServer)
        flat_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim / 2,
                                       rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
        ### middle flat
        middleflatPosition = [0.0, 0.0, shelfbase_dim[2] + flank_dim[2] / 2]
        middleflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
                                          baseVisualShapeIndex=flat_v_p,
                                          basePosition=middleflatPosition, physicsClientId=planningServer)
        middleflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
                                          baseVisualShapeIndex=flat_v_e,
                                          basePosition=middleflatPosition, physicsClientId=executingServer)
        known_geometries_planning.append(middleflatM_p)
        known_geometries_executing.append(middleflatM_e)
        print("middle flat: " + str(middleflatM_e))
        ### top flat
        topflatPosition = [0.0, 0.0, shelfbase_dim[2] + flank_dim[2] + flat_dim[2] / 2]
        topflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
                                       baseVisualShapeIndex=flat_v_p,
                                       basePosition=topflatPosition, physicsClientId=planningServer)
        topflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
                                       baseVisualShapeIndex=flat_v_e,
                                       basePosition=topflatPosition, physicsClientId=executingServer)
        known_geometries_planning.append(topflatM_p)
        known_geometries_executing.append(topflatM_e)
        print("top flat: " + str(topflatM_e))
        ### back
        shelfback_dim = np.array([0.02, shelfbase_dim[1], flank_dim[2] + flat_dim[2]])
        shelfbackPosition = [shelfbase_dim[0] / 2 - shelfback_dim[0] / 2, 0.0,
                             shelfbase_dim[2] + shelfback_dim[2] / 2]
        shelfback_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim / 2,
                                               physicsClientId=planningServer)
        shelfback_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim / 2,
                                            rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
        shelfback_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim / 2,
                                               physicsClientId=executingServer)
        shelfback_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim / 2,
                                            rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
        shelfbackM_p = p.createMultiBody(baseCollisionShapeIndex=shelfback_c_p,
                                         baseVisualShapeIndex=shelfback_v_p,
                                         basePosition=shelfbackPosition, physicsClientId=planningServer)
        shelfbackM_e = p.createMultiBody(baseCollisionShapeIndex=shelfback_c_e,
                                         baseVisualShapeIndex=shelfback_v_e,
                                         basePosition=shelfbackPosition, physicsClientId=executingServer)
        known_geometries_planning.append(shelfbackM_p)
        known_geometries_executing.append(shelfbackM_e)
        print("shelf back: " + str(shelfbackM_e))
        ### reset the base of Motoman
        motomanBasePosition = [shelfbasePosition[0] - shelfbase_dim[0] / 2 - 0.8, shelfbasePosition[1], 0.0]
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
        print("--------Welcome to " + bt + " sceneario " + scene + "--------")
        try:
            os.mkdir(path)
        except OSError:
            # os.rmdir(path)
            print("Creation of the directory %s falied\n" % path)
        else:
            pass

        ### Try different YCB objects in the scene to see the time for importing
        ###################################################################################################################
        Objects = dict()
        ### objIdx, meshFile, objectName, objectRole, scale, true_pos,
        ### true_angles(radians), mass, nHypos, prob(not in the scene)
        Objects[0] = [0, "/mesh/009_gelatin_box/google_16k/textured_reduced.obj", "gelatin box",
                      "target", 1, [0.0, 0.29, 0.002 + table_dim[2]], [0, 0, math.pi / 7], 1.4, nHypos, 0.08]
        # Objects[0] = [0, "/mesh/006_mustard_bottle_64/mustard_bottle_64.obj", "mustard bottle",
        # 	"target", 1, [0.2, 0.2, 0.1+table_dim[2]], [0, 0, 0], 2.1, nHypos, 0.16]
        Objects[1] = [1, "/mesh/006_mustard_bottle/google_16k/textured_reduced.obj", "mustard bottle",
                      "normal", 1, [-0.145, 0.37, 0.005 + table_dim[2]], [0, 0, math.pi / 4], 2.1, nHypos, 0.16]
        # Objects[0] = [0, "/mesh/006_mustard_bottle_512/mustard_bottle_512.obj", "mustard bottle",
        # 	"normal", 1, [0.4, 0.4, 0.1+table_dim[2]], [0, 0, 0], 2.1, nHypos, 0.16]
        Objects[2] = [2, "/mesh/021_bleach_cleanser/google_16k/textured_reduced.obj", "bleach cleanser",
                      "normal", 1, [0.01, 0.37, 0.004 + table_dim[2]], [0, 0, 0], 4.2, nHypos, 0.11]
        Objects[3] = [3, "/mesh/003_cracker_box/google_16k/textured_reduced.obj", "cracker box",
                      "normal", 1, [-0.02, 0.18, 0.005 + table_dim[2]], [0, 0, math.pi / 2], 4.8, nHypos, 0.10]
        Objects[4] = [4, "/mesh/008_pudding_box/google_16k/textured_reduced.obj", "pudding box",
                      "phantom", 1, [0.05, 0.33, 0.002 + table_dim[2]], [0, 0, -3 * math.pi / 4], 1.4, nHypos, 0.86]

        ### pick goal offset
        goalPos_offset = [0.0, 0.0, 0.05]
        goalEuler = [0.0, math.pi, 0.0]  ### overhan grasps
        x_ll = motomanBasePosition[0] - 0.35
        x_ul = tablePosition[0] + table_dim[0] / 2
        y_ll = -table_dim[1] / 2
        y_ul = table_dim[1] / 2
        z_ll = table_dim[2]
        z_ul = table_dim[2] + 0.9

    ####################################################################################################################

    ### At here we finish selecting the specific scene for specific benchmark type
    ### Now let's generate ground truth of the specific scene and specific benchmark
    startTime = time.time()
    truePoses, nObjectInExecuting = utils_motoman.trueScene_generation(bt, scene, Objects, executingServer)
    utils_motoman.planScene_generation(Objects, bt,
                                       known_geometries_planning, transErrors[noiseLevel - 1],
                                       orientErrors[noiseLevel - 1], executingServer)
    print("Time elapsed to load the objects as the ground truth: ", time.time() - startTime)
    startTime = time.time()
    hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = utils_motoman.planScene_generation(Objects, bt,
                                                                                               known_geometries_planning,
                                                                                               transErrors[
                                                                                                   noiseLevel - 1],
                                                                                               orientErrors[
                                                                                                   noiseLevel - 1],
                                                                                               planningServer)
    print("Time elapsed to load the objects in the planning scene: ", time.time() - startTime)
    print("most promisings: " + str(mostPromisingHypoIdxes))

    ### Now we can generate "labelWeights.txt" file
    currentlabelWeightFile = path + "/labelWeights.txt"
    f_labelWeights = open(currentlabelWeightFile, "w")
    for hypo in hypotheses:
        f_labelWeights.write(str(hypo.hypoIdx) + " " + str(hypo.objIdx) + " " + str(hypo.prob) + "\n")
    f_labelWeights.close()
    ### Now we can generate "mostPromisingLabels.txt" file
    currentMostPromisingLabelsFile = path + "/mostPromisingLabels.txt"
    f_mostPromisingLabels = open(currentMostPromisingLabelsFile, "w")
    for mphi in mostPromisingHypoIdxes:
        f_mostPromisingLabels.write(str(mphi) + " ")
    f_mostPromisingLabels.write("\n")
    f_mostPromisingLabels.close()

    ##############################################roadmap generation########################################################
    startTime = time.time()
    ### specify q_start and set of q_goal first
    q_start = home_configuration
    starting_point = get_position(motomanID_p, motoman_ee_idx)

    ### generate goal configurations
    goalSet = []
    goalPoints = []
    goalHypos = []
    MaxGoalsPerPose = 5
    MaxTrialsPerPose = 7
    MaxTrialsPerEE = 7

    ### for each target hypotheses
    for t_hp in range(Objects[0][8]):
        # print "***********For Hypo " + str(t_hp) + "***************"
        temp_goalsForThatHypo = []
        temp_pointsForThatHypo = []
        temp_survivalForThatHypo = []
        ### specify the position of the goal pose for that particular target hypothsis
        goal_pose_pos = []
        for i in range(len(goalPos_offset)):
            goal_pose_pos.append(hypotheses[t_hp].pos[i] + goalPos_offset[i])
        # print "goal_pose_pos: " + str(goal_pose_pos)
        temp_trials_pose = 0
        while temp_trials_pose < MaxTrialsPerPose:
            # print "-----A new pose-----"
            for j in range(1, 8):
                result_p = p.resetJointState(motomanID_p, j, home_configuration[j - 1], physicsClientId=planningServer)
            for j in range(11, 18):
                result_p = p.resetJointState(motomanID_p, j, home_configuration[j - 4], physicsClientId=planningServer)
            ### specify the quaternion of that particular goal pose
            temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1],
                                                            goalEuler[2] + random.uniform(-math.pi, math.pi)])
            goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3],
                              temp_goal_pose_quat[2]]
            ### now start to exploring the IKs
            temp_trials_ee = 0
            while temp_trials_ee < MaxTrialsPerEE:
                q_goal = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx, goal_pose_pos, goal_pose_quat,
                                                      ll, ul, jr, physicsClientId=planningServer)
                for j in range(1, 8):
                    result_p = p.resetJointState(motomanID_p, j, q_goal[j - 1], physicsClientId=planningServer)
                for j in range(11, 18):
                    result_p = p.resetJointState(motomanID_p, j, q_goal[j - 4], physicsClientId=planningServer)
                p.stepSimulation(planningServer)
                ### check collision for robot self and known obstacles
                isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
                isCollisionKnownObs = utils_motoman.collisionCheck_knownObs(motomanID_p, known_geometries_planning,
                                                                            planningServer)
                if isCollisionSelf or isCollisionKnownObs:
                    # print "Collision with robot itself or known obstacles, ee +2!"
                    temp_trials_ee += 2  ## This may be a bad pose, so let's check it in a quicker way
                    # raw_input("Press Enter to continue")
                    continue
                else:
                    ### check collision condition with all hypos of objects
                    ### (The opposite of the collision probability)
                    collidedHypos = utils_motoman.collisionCheck_hypos(motomanID_p, hypotheses, planningServer)
                    # print "Collide with Hypos: " + str(collidedHypos)
                    ### compute the survivability
                    if t_hp in collidedHypos:
                        ### the end effector collides with the pose it deems as the target pose
                        temp_survival = 0.0
                        temp_trials_ee += 1
                        # print "Survival: " + str(temp_survival)
                        # raw_input("Press Enter to continue")
                        continue
                    else:
                        temp_survival = 1.0
                        collisionPerObj = [0.0] * nObjectInPlanning
                        for ch in collidedHypos:
                            if hypotheses[ch].objIdx != 0:
                                collisionPerObj[hypotheses[ch].objIdx] += hypotheses[ch].prob
                        for cpobs in collisionPerObj:
                            temp_survival *= (1 - cpobs)
                        # print "Survival: " + str(temp_survival)
                        # raw_input("Press Enter to continue")
                        ### add the q_goals and its corresponding survivability
                        temp_goalsForThatHypo.append(q_goal)
                        temp_pointsForThatHypo.append(goal_pose_pos)
                        temp_survivalForThatHypo.append(temp_survival)
                        temp_trials_ee += 1
            ### finish the current pose
            temp_trials_pose += 1

        ### You are here since you finish generating poses for a particular hypothesis
        ### sort temp_survivalForThatHypo and pick top (MaxGoalsPerPose) ones
        idx_rank = sorted(list(range(len(temp_survivalForThatHypo))), key=lambda k: temp_survivalForThatHypo[k],
                          reverse=True)
        if len(idx_rank) >= MaxGoalsPerPose:
            ### add top (MaxGoalsPerPose) ones
            for mm in range(MaxGoalsPerPose):
                goalSet.append(temp_goalsForThatHypo[idx_rank[mm]])
                goalPoints.append(temp_pointsForThatHypo[idx_rank[mm]])
                goalHypos.append(t_hp)
    print("goalHypos:" + str(goalHypos))

    ############# start sampling ##############
    currentSamplesFile = path + "/samples.txt"
    location_file_str = path + "/locations.txt"
    f_samples = open(currentSamplesFile, "w")
    f_locations = open(location_file_str, "w")
    nodes = []
    temp_counter = 0
    locations = []

    while temp_counter < nsamples:
        ### sample a cartesian ee pose and calculate the IK solution
        temp_x = float(format(random.uniform(x_ll, x_ul), '.2f'))
        temp_y = float(format(random.uniform(y_ll, y_ul), '.2f'))
        temp_z = float(format(random.uniform(z_ll, z_ul), '.2f'))
        location = [temp_x, temp_y, temp_z]
        ikSolution = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx,
                                                  location, ll, ul, jr, physicsClientId=planningServer)
        for j in range(1, 8):
            result_p = p.resetJointState(motomanID_p, j, ikSolution[j - 1], physicsClientId=planningServer)
        for j in range(11, 18):
            result_p = p.resetJointState(motomanID_p, j, ikSolution[j - 4], physicsClientId=planningServer)
        p.stepSimulation(planningServer)
        ### check collision for robot self and known obstacles
        isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
        # if isCollisionSelf:
        # 	print "self collision occurs during the sampling"
        isCollisionKnownObs = utils_motoman.collisionCheck_knownObs(motomanID_p, known_geometries_planning,
                                                                    planningServer)
        if (not isCollisionSelf) and (not isCollisionKnownObs):
            nodes.append(ikSolution)
            locations.append(tuple(location))
            ### write it into a sample file
            f_samples.write(str(temp_counter) + " " + str(ikSolution[0]) + " " + str(ikSolution[1]) + " " \
                            + str(ikSolution[2]) + " " + str(ikSolution[3]) + " " + str(ikSolution[4]) + " " \
                            + str(ikSolution[5]) + " " + str(ikSolution[6]) + "\n")
            f_locations.write(
                str(temp_counter) + " " + str(location[0]) + " " + str(location[1]) + " " + str(location[2]) + "\n")
            temp_counter += 1

    ############## connect neighbors to build roadmaps #############
    startTime = time.time()
    connectivity = np.zeros((nsamples, nsamples))
    # tree = spatial.KDTree(nodes)
    tree = spatial.KDTree(locations)
    # TODO: Play with 1.5
    neighbors_const = 2.5 * math.e * (1 + 1 / (len(home_configuration) / 2))
    num_neighbors = int(neighbors_const * math.log(nsamples))
    if num_neighbors >= nsamples:
        num_neighbors = nsamples - 1
    print("num_neighbors: " + str(num_neighbors))
    currentRoadmapFile = path + "/roadmap.txt"
    f_roadmap = open(currentRoadmapFile, "w")
    edges = []
    ### for each node
    for i in range(len(nodes)):
        queryNode = nodes[i]
        query_location = locations[i]

        knn = tree.query(query_location, k=num_neighbors, p=2)
        ### for each neighbor
        for j in range(len(knn[1])):
            if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
                ### if the neighbor is the query node itself
                ### or the connectivity has been checked before
                ### then skip the edge checking procedure
                continue
                ### Otherwise, check the edge validity (in terms of collision with robot itself and known obstacles)
            ### between the query node and the the current neighbor
            neighbor = nodes[knn[1][j]]
            # isEdgeValid = True
            isEdgeValid = utils_motoman.checkEdgeValidity(queryNode, neighbor, motomanID_p,
                                                          known_geometries_planning, planningServer)
            if isEdgeValid:
                ### write this edge information with their cost and labels into the txt file
                ### It is a valid edge in terms of no collision with robot itself and known obstacles
                ### Let's check the collision status for each hypothesis for the purpose of labeling
                temp_labels = utils_motoman.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
                f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " ")
                edges.append((i, knn[1][j], knn[0][j]))
                for tl in temp_labels:
                    f_roadmap.write(str(tl) + " ")
                f_roadmap.write("\n")
                ### update connectivity information
                connectivity[i][knn[1][j]] = 1
                connectivity[knn[1][j]][i] = 1

        if i % 100 == 99:
            print("finish labeling and connecting neighbors for node " + str(i))
    print("finish all the neighbors")
    print("Time elapsed: ", time.time() - startTime)
    ########################################################

    ############## add the start node and goal nodes to the roadmap ##########
    nodes.append(q_start)
    locations.append(starting_point)
    start_index = len(nodes) - 1
    f_samples.write(str(temp_counter) + " " + str(q_start[0]) + " " + str(q_start[1]) + " " \
                    + str(q_start[2]) + " " + str(q_start[3]) + " " + str(q_start[4]) + " " + \
                    str(q_start[5]) + " " + str(q_start[6]) + "\n")
    f_locations.write(
        str(temp_counter) + " " + str(starting_point[0]) + " " + str(starting_point[1]) + " " + str(starting_point[2]) + "\n")
    temp_counter += 1
    ### connect the start to the roadmap
    # tree = spatial.KDTree(nodes)
    tree = spatial.KDTree(locations)

    queryNode = nodes[temp_counter - 1]
    query_location = locations[temp_counter - 1]

    knn = tree.query(query_location, k=num_neighbors, p=2)
    highestSurvival_neighborIdx = -1
    highestSurvivalSofar = -1.0
    highestSurvival_labels = []
    highestSurvival_cost = -1.0

    ### for each neighbor
    for j in range(len(knn[1])):
        if knn[1][j] == (temp_counter - 1):
            continue
        else:
            ### check collision
            neighbor = nodes[knn[1][j]]
            # isEdgeValid = True
            isEdgeValid = utils_motoman.checkEdgeValidity(queryNode, neighbor, motomanID_p,
                                                          known_geometries_planning, planningServer)
            if isEdgeValid:
                ### examine the survivability of the edge
                ### first get the labels
                temp_labels = utils_motoman.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
                ### compute the survivability
                temp_survival = 1.0
                collisionPerObj = [0.0] * nObjectInPlanning
                for tl in temp_labels:
                    collisionPerObj[hypotheses[tl].objIdx] += hypotheses[tl].prob
                for cpobs in collisionPerObj:
                    temp_survival *= (1 - cpobs)
                # if temp_survival > highestSurvivalSofar:
                highestSurvivalSofar = temp_survival
                highestSurvival_neighborIdx = knn[1][j]
                highestSurvival_labels = temp_labels
                highestSurvival_cost = knn[0][j]
                ### finish all the survival computation for all k nearest neighbors
                ### Now connect the start to the one neighbor with the largest survival (lower cost for tie breaker)
                # if (highestSurvival_neighborIdx != -1):
                f_roadmap.write(str(temp_counter - 1) + " " + str(highestSurvival_neighborIdx) + " " \
                                + format(highestSurvival_cost, '.4f') + " ")
                edges.append((temp_counter - 1, highestSurvival_neighborIdx, highestSurvival_cost))

    for hl in highestSurvival_labels:
        f_roadmap.write(str(hl) + " ")
    f_roadmap.write("\n")
    print("successfully connect the start to the roadmap\n")

    ### loop through goalSet
    goalConnectSuccess = False
    for q_goal, hypo in zip(goalSet, goalHypos):
        nodes.append(q_goal)
        location = goalPoints.pop(0)
        locations.append(location)

        f_samples.write(str(temp_counter) + " " + str(q_goal[0]) + " " + str(q_goal[1]) + " " \
                        + str(q_goal[2]) + " " + str(q_goal[3]) + " " + str(q_goal[4]) + " " + \
                        str(q_goal[5]) + " " + str(q_goal[6]) + " " + str(hypo) + "\n")
        f_locations.write(str(temp_counter) + " " + str(location[0]) + " " + str(location[1]) + " " + str(location[2]) + "\n")
        temp_counter += 1
        ### connect the goal to the roadmap
        # tree = spatial.KDTree(nodes)
        tree = spatial.KDTree(locations)
        queryNode = nodes[temp_counter - 1]
        query_location = locations[temp_counter - 1]
        knn = tree.query(query_location, k=num_neighbors, p=2)
        ### for each neighbor
        for j in range(len(knn[1])):
            if knn[1][j] == (temp_counter - 1):
                continue
            else:
                ### check collision
                neighbor = nodes[knn[1][j]]
                # isEdgeValid = True
                isEdgeValid = utils_motoman.checkEdgeValidity(queryNode, neighbor, motomanID_p,
                                                              known_geometries_planning, planningServer)
                if isEdgeValid:
                    ### examine the survivability of the edge
                    ### first get the labels
                    temp_labels = utils_motoman.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses,
                                                               planningServer)
                    f_roadmap.write(str(temp_counter - 1) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " ")
                    edges.append((temp_counter - 1, knn[1][j], knn[0][j]))

                    for temp_l in temp_labels:
                        f_roadmap.write(str(temp_l) + " ")
                    f_roadmap.write("\n")
                    goalConnectSuccess = True

    if goalConnectSuccess:
        print("successfully connect the goal to the roadmap\n")
    f_samples.close()
    f_locations.close()
    f_roadmap.close()
    print("roadmap generated with " + str(len(nodes)) + " nodes in " + str(time.time() - startTime) + " second.")
    print(len(nodes))
    print(len(locations))
    vertices = []
    edge_dict = {}
    for a, b, dist in edges:
        edge_dict.setdefault(a, {})[b] = dist
        edge_dict.setdefault(b, {})[a] = dist
    edges = edge_dict

    roadmap = graph_logic.Graph(nodes, locations, edges, start_index)
    components = roadmap.get_components()
    first_nodes = roadmap.get_neighbors(start_index)

    paths = roadmap.ucs()
    path = roadmap.astar(len(nodes) - 1)
    path = [] if path is None else path

    for i, location in enumerate(locations):
        size = .01
        color = [1, 0, 0, 1]
        # if location == starting_point:
        #    size = .05
        #    color = [0, 1, 0, 1]
        if i in path:
            color = [0, 1, 0, 1]
            size = .05
        elif i in paths:
            color = [0, 0, 1, 1]
            size = .05
        vertice_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=size, rgbaColor=color,
                                        physicsClientId=executingServer)
        vertice = p.createMultiBody(baseVisualShapeIndex=vertice_v, basePosition=location,
                                    physicsClientId=executingServer)

        vertices.append(vertice)

    print(first_nodes)


if __name__ == "__main__":
    main("", "table", "1", 1, 1, 5000)