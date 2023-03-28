# Author: Alexander Schperberg, aschperb@gmail.com
# Date: 2023-03-27
#!/usr/bin/env python
import pybullet as p
import pybullet_data
import numpy as np
import math
import time
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import copy
from scaler_kin.SCALAR_kinematics import scalar_k as ScalerKinematics
from utils.settings import INITIAL_PARAMS

def postion_quat_to_matrix(inpt):
    #input -> [postion, quation]
    #return matrix form
    rot = R.from_quat(inpt[1])
    res = np.eye(4, dtype=np.float32)
    res[0:3,0:3] = rot.as_matrix()
    res[0:3,3] = np.array(inpt[0])
    return res

class pyb_sim(object):
    N_leg = 4
    connectedLinkNameList = [['C_Link00', 'C_Link01'], ['C_Link10', 'C_Link11'], ['C_Link20', 'C_Link21'], ['C_Link30', 'C_Link31']]

    passiveJointNameList = ['B_Joint0', 'E_Joint0', 'B_Joint1', 'E_Joint1', 'B_Joint2', 'E_Joint2', 'B_Joint3', 'E_Joint3']
    passiveJointNum = 8

    bodyJointNameList = ['BodyTopleft_Joint','BodyRight_Joint', 'BodyBot_Joint']
    bodyJointNum = 3

    cameraJointNameList = ['above_camera_joint', 'body_camera_joint']
    cameraJointNum = 2

    maxForce = 300
    passiveJointFriction = 0

    def __init__(self, urdf_filename, urdf_filename_wall, bodyFixed=True, RobotStartPos=[0, 0, 0.55],
                 RobotStartOrientation=[0, 0, 0], delta_t=None, render=True):

        self.bodyFixed = bodyFixed
        self.urdf_filename = urdf_filename
        # define the wall and initial bouldering holds here
        self.urdf_filename_wall = urdf_filename_wall
        self.RobotStartPos = RobotStartPos
        self.RobotStartOrientation = p.getQuaternionFromEuler(RobotStartOrientation)
        self.delta_t = delta_t
        self.dt_cmd = INITIAL_PARAMS.DT_CMD
        self.worldTbody = np.eye(4, dtype=np.float32)
        self.worldTbody[0:3, 3] = np.array(self.RobotStartPos)
        self.worldTbody[0:3, 0:3] = np.array(p.getMatrixFromQuaternion(self.RobotStartOrientation)).reshape((3, 3))
        self.bodyTworld = np.linalg.inv(self.worldTbody)
        self.render = render
        self.kin = ScalerKinematics()

        self.motorJointNameList = ['Shoulder_Joint0', 'A_Joint0', 'F_Joint0','wrist1_Joint0', 'wrist2_Joint0', 'wrist3_Joint0',
                                   'Shoulder_Joint1', 'A_Joint1', 'F_Joint1','wrist1_Joint1', 'wrist2_Joint1', 'wrist3_Joint1',
                                   'Shoulder_Joint2', 'A_Joint2', 'F_Joint2','wrist1_Joint2', 'wrist2_Joint2', 'wrist3_Joint2',
                                   'Shoulder_Joint3', 'A_Joint3', 'F_Joint3','wrist1_Joint3', 'wrist2_Joint3', 'wrist3_Joint3',
                                   'base_joint_gripper_left0','base_joint_gripper_right0','base_joint_gripper_left1','base_joint_gripper_right1',
                                   'base_joint_gripper_left2','base_joint_gripper_right2', 'base_joint_gripper_left3','base_joint_gripper_right3']
        self.motorJointNum = 32
        #self.HOME_POSE = [0.0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, -0.055,0.055, -0.055,0.055, -0.055,0.055, -0.055,0.055]  #The first one is the body motor
        self.HOME_POSE = [0.0, 0, 0, 2.0137870320351823, 0, 0, 0, 0, 0, 2.0137870320351823, 0, 0, 0, 0, 0,
                          2.0137870320351823, 0, 0, 0, 0, 0, 2.0137870320351823, 0, 0, 0, -0.055, 0.055, -0.055,
                          0.055, -0.055, 0.055, -0.055, 0.055]
        self.DoFnum = 6

        # Define the bouldering holds (first 4 are the starting bouldering hold positions)
        self.bouldering_hold_leg_index = [1,2,3,0]
        #self.bouldering_holds_list_pos_str = ["-0.12 0.45 0.07", "0.12 0.32 0.07", "0.12 -0.32 0.07", "-0.12 -0.32 0.07", "-0.12 0.4 0.07"]
        self.bouldering_holds_list_pos_str = ["-0.12 0.32 0.07", "0.12 0.42 0.07", "0.12 -0.32 0.07", "-0.12 -0.32 0.07", "-0.12 0.5 0.07"]
        self.bouldering_holds_list_pos = [list(map(float, s.split())) for s in self.bouldering_holds_list_pos_str]
        self.bouldering_holds_list_height = [0.15, 0.15, 0.15, 0.15, 0.0]
        self.bouldering_hold_rotation = [[0.999,0.0,0.0,0],[0.999,0.0,0.0,0.0],[0.999,0.0,0.0,0.0],[0.999, 0.0, 0.0, 0]]

        self.moving_bouldering_holds_pos = copy.deepcopy(self.bouldering_holds_list_pos[0])
        self.moving_bouldering_holds_rotation = self.bouldering_hold_rotation[0]

        # FK ID link leg
        self.fk_ID_link = [51,67,33,17]

        # defining joint limits for the 6 DoF case
        self.body_lim = [-0.8, 0.8]
        self.shoulder_lim = [-0.1, 1.0]
        self.A_joint_lim = [-1.0, 0.6]
        self.F_joint_lim = [0.97, 2.37]
        self.wrist1_lim = [-0.2, 0.3]
        self.wrist2_lim = [-0.9, 0.9]
        self.wrist3_lim = [-1.6, 1.6]
        self.finger_limit = [0.11]

        self.x_limit = [-241.66, -74.59]
        self.y_limit = [250.0, 400.0]
        self.z_limit = [-260.6, -120.99]

        # threshold for distance from gripper to obstacle to be considered graspable
        self.grasp_threshold = 0.05

        #which direction for x and for y to correct leg is dependent on leg number
        self.index_correct = [[1,0,0,1],[1,0,1,0],[0,1,1,0],[0,1,0,1]]
        self.task_space_limits_list = [self.x_limit, self.y_limit, self.z_limit]
        self.gripper_limit_list = [0.0,0.11]
        self.joint_limits_list = [self.wrist1_lim, self.wrist2_lim, self.wrist3_lim]
        self.joint_limits_list2 = [self.shoulder_lim, self.A_joint_lim, self.F_joint_lim, self.wrist1_lim, self.wrist2_lim, self.wrist3_lim]

        if self.render:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        self.setup_environment()

    def matrix_to_quaternion(self, R_matrix):
        r = R.from_matrix(R_matrix)
        q = r.as_quat()

        return q

    def quaternion_to_matrix(self, q):
        r = R.from_quat(q)
        matrix = r.as_matrix()

        return matrix

    def distance_to_goal_quat(self, q_cur, q_goal):
        dot_product = np.dot(q_cur, q_goal)
        angle = 2 * np.arccos(dot_product)

        return angle

    def task_space_limits(self, cur_pose):
        # make it invarient to leg number (all will follow sign of leg 3)
        cur_poseCorrected = copy.deepcopy(cur_pose)
        if np.sign(cur_pose[0]) == 1.0:
            cur_poseCorrected[0] = cur_pose[0]*-1.0
        if np.sign(cur_pose[1]) == -1.0:
            cur_poseCorrected[1] = cur_pose[1]*-1.0

        self.task_space_limit_reached = [[False,False]]*3
        for i in range(len(cur_poseCorrected)):
            if cur_poseCorrected[i] < self.task_space_limits_list[i][0]:
                cur_list = copy.deepcopy(self.task_space_limit_reached[i])
                cur_list[0] = True
                self.task_space_limit_reached[i] = cur_list

            elif cur_poseCorrected[i] > self.task_space_limits_list[i][1]:
                cur_list = copy.deepcopy(self.task_space_limit_reached[i])
                cur_list[1] = True
                self.task_space_limit_reached[i] = cur_list

        return self.task_space_limit_reached

    def gripper_limits(self, cur_pose):
        distance_finger = np.abs(cur_pose[0] - cur_pose[1])
        self.gripper_limit_reached = [False, False]

        if distance_finger < self.gripper_limit_list[0]:
           self.gripper_limit_reached[0] = True

        if distance_finger > self.gripper_limit_list[1]:
            self.gripper_limit_reached[1] = True

        return self.gripper_limit_reached

    def body_motor_limits(self, cur_pose):
        self.body_motor_limit_reached = [False, False]
        if cur_pose < self.body_lim[0]:
           self.body_motor_limit_reached[0] = True

        if cur_pose > self.body_lim[1]:
            self.body_motor_limit_reached[1] = True

        return self.body_motor_limit_reached

    def joint_limits(self, cur_joints):
        self.joint_space_limit_reached = [[False, False]] * 3
        for i in range(len(cur_joints)):
            if cur_joints[i] < self.joint_limits_list[i][0]:
                cur_list = copy.deepcopy(self.joint_space_limit_reached[i])
                cur_list[0] = True
                self.joint_space_limit_reached[i] = cur_list

            elif cur_joints[i] > self.joint_limits_list[i][1]:
                cur_list = copy.deepcopy(self.joint_space_limit_reached[i])
                cur_list[1] = True
                self.joint_space_limit_reached[i] = cur_list

        return self.joint_space_limit_reached

    def correct_cmd_body(self, pose_body_motor, cmd_vel_body_motor):
        # impose constraints on body motor
        self.body_motor_limit_reached = self.body_motor_limits(pose_body_motor)
        if cmd_vel_body_motor < 0.0:
            if not self.body_motor_limit_reached[0]:
                pose_body_motor = pose_body_motor + cmd_vel_body_motor * self.dt_cmd

        if cmd_vel_body_motor > 0.0:
            if not self.body_motor_limit_reached[1]:
                pose_body_motor = pose_body_motor + cmd_vel_body_motor * self.dt_cmd

        return pose_body_motor

    def correct_cmd_body_vel(self, pose_T_list, cmd_vel, cur_leg):
        for leg in range(4):
            if leg != cur_leg:
                pose_T = copy.deepcopy(pose_T_list[leg])
                pose = pose_T[0:3, 3]
                # impose constraints on limb workspace
                task_space_limit_reached = copy.deepcopy(self.task_space_limits(pose))
                # get the correction index based on the leg
                correct_index = copy.deepcopy(self.index_correct[leg])
                # correct in x
                if cmd_vel[0] > 0.0:
                    if not task_space_limit_reached[0][correct_index[0]]:
                        pose[0] = pose[0] - cmd_vel[0] * self.dt_cmd

                if cmd_vel[0] < 0.0:
                    if not task_space_limit_reached[0][correct_index[1]]:
                        pose[0] = pose[0] - cmd_vel[0] * self.dt_cmd

                # correct in y
                if cmd_vel[1] > 0.0:
                    if not task_space_limit_reached[1][correct_index[2]]:
                        pose[1] = pose[1] - cmd_vel[1] * self.dt_cmd

                if cmd_vel[1] < 0.0:
                    if not task_space_limit_reached[1][correct_index[3]]:
                        pose[1] = pose[1] - cmd_vel[1] * self.dt_cmd

                pose_T[0:3, 3] = pose
                pose_T_list[leg] = pose_T

        return pose_T_list

    def correct_cmd_limb(self, leg, pose, cmd_vel, pose_gripper, cmd_vel_gripper, wrist_joints, cmd_wrist_joints):
        self.gripper_limit_reached = self.gripper_limits(pose_gripper)
        if cmd_vel_gripper < 0.0:
            if not self.gripper_limit_reached[0]:
                distance_finger = np.abs(pose_gripper[0] - pose_gripper[1])
                distance_finger = distance_finger + cmd_vel_gripper * self.dt_cmd
                finger1_pos = -distance_finger / 2.0
                finger2_pos = distance_finger / 2.0
                pose_gripper[0] = finger1_pos
                pose_gripper[1] = finger2_pos

        if cmd_vel_gripper > 0.0:
            if not self.gripper_limit_reached[1]:
                distance_finger = np.abs(pose_gripper[0] - pose_gripper[1])
                distance_finger = distance_finger + cmd_vel_gripper * self.dt_cmd
                finger1_pos = -distance_finger / 2.0
                finger2_pos = distance_finger / 2.0
                pose_gripper[0] = finger1_pos
                pose_gripper[1] = finger2_pos

        # impose constraints on limb workspace
        self.task_space_limit_reached = self.task_space_limits(pose)
        # get the correction index based on the leg
        correct_index = self.index_correct[leg]

        # correct in x
        if cmd_vel[0] < 0.0:
            if not self.task_space_limit_reached[0][correct_index[0]]:
                pose[0] = pose[0] + cmd_vel[0] * self.dt_cmd

        if cmd_vel[0] > 0.0:
            if not self.task_space_limit_reached[0][correct_index[1]]:
                pose[0] = pose[0] + cmd_vel[0] * self.dt_cmd

        # correct in y
        if cmd_vel[1] < 0.0:
            if not self.task_space_limit_reached[1][correct_index[2]]:
                pose[1] = pose[1] + cmd_vel[1] * self.dt_cmd

        if cmd_vel[1] > 0.0:
            if not self.task_space_limit_reached[1][correct_index[3]]:
                pose[1] = pose[1] + cmd_vel[1] * self.dt_cmd

        # correct in z
        if cmd_vel[2] < 0.0:
            if not self.task_space_limit_reached[2][0]:
                pose[2] = pose[2] + cmd_vel[2] * self.dt_cmd

        if cmd_vel[2] > 0.0:
            if not self.task_space_limit_reached[2][1]:
                pose[2] = pose[2] + cmd_vel[2] * self.dt_cmd

        # impose constraints on orientation of the gripper
        self.joint_space_limit_reached = self.joint_limits(wrist_joints)
        # correct in x
        if cmd_wrist_joints[0] < 0.0:
            if not self.joint_space_limit_reached[0][0]:
                wrist_joints[0] = wrist_joints[0] + cmd_wrist_joints[0] * self.dt_cmd

        if cmd_wrist_joints[0] > 0.0:
            if not self.joint_space_limit_reached[0][1]:
                wrist_joints[0] = wrist_joints[0] + cmd_wrist_joints[0] * self.dt_cmd

        # correct in y
        if cmd_wrist_joints[1] < 0.0:
            if not self.joint_space_limit_reached[1][0]:
                wrist_joints[1] = wrist_joints[1] + cmd_wrist_joints[1] * self.dt_cmd

        if cmd_wrist_joints[1] > 0.0:
            if not self.joint_space_limit_reached[1][1]:
                wrist_joints[1] = wrist_joints[1] + cmd_wrist_joints[1] * self.dt_cmd

        # correct in z
        if cmd_wrist_joints[2] < 0.0:
            if not self.joint_space_limit_reached[2][0]:
                wrist_joints[2] = wrist_joints[2] + cmd_wrist_joints[2] * self.dt_cmd

        if cmd_wrist_joints[2] > 0.0:
            if not self.joint_space_limit_reached[2][1]:
                wrist_joints[2] = wrist_joints[2] + cmd_wrist_joints[2] * self.dt_cmd

        return pose, pose_gripper, wrist_joints


    def buildJointNameToIdDict(self):
        nJoints = p.getNumJoints(self.RobotId)
        self.jointNameToId = {}
        self.linkNameToId = {}
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.RobotId, i)
            self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
            self.linkNameToId[jointInfo[12].decode('UTF-8')] = jointInfo[0]

    def buildWallJointNameToIdDict(self):
        nJoints = p.getNumJoints(self.wall)
        self.wallJointNameToId = {}
        self.wallLinkNameToId = {}
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.wall, i)
            self.wallJointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
            self.wallLinkNameToId[jointInfo[12].decode('UTF-8')] = jointInfo[0]

    def buildBodyMotorIdList(self):
        self.BodyMotorIdList = []
        for motorJointName in self.bodyJointNameList:
            self.BodyMotorIdList.append(self.jointNameToId[motorJointName])

    def buildMotorIdList(self):
        self.MotorIdList = []
        for motorJointName in self.motorJointNameList:
            self.MotorIdList.append(self.jointNameToId[motorJointName])

    def connectLinks(self):
        for i in range(self.N_leg):
            cid = p.createConstraint(self.RobotId, self.linkNameToId[self.connectedLinkNameList[i][0]],
                                     self.RobotId, self.linkNameToId[self.connectedLinkNameList[i][1]],
                                     p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 0])
            p.changeConstraint(cid, maxForce=self.maxForce * 1000)

    def setPassiveJoints(self):
        for i in range(self.passiveJointNum):
            p.setJointMotorControl2(bodyUniqueId=self.RobotId,
                                    jointIndex=self.jointNameToId[self.passiveJointNameList[i]],
                                    controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=self.passiveJointFriction)

    def resettoPose(self, targetPose):
        newtargetPose = np.array(targetPose)
        for i in range(self.N_leg):
            newtargetPose[i * self.DoFnum + 2] = newtargetPose[i * self.DoFnum + 2] - np.pi / 2
        for i in range(self.motorJointNum):
            p.resetJointState(self.RobotId, self.jointNameToId[self.motorJointNameList[i]], newtargetPose[i])

    def movetoPose(self, targetPose):
        # This function use position control to move the robot to the target pose
        # Input: targetPose -> bodymotor + [shoulder angle, q11, q21, wrist1, wrist2, wrist3]*4 legs, totatally 25 values if 6 DOf
        #                      bodymotor + [shoulder angle, q11, q21]*4 legs, totatally 13 values if 6 DOf
        newtargetPose = np.array(targetPose).reshape(-1)
        bodyAngle = newtargetPose[0]
        newtargetPose = newtargetPose[1:]

        bodyPose = [bodyAngle, -bodyAngle, bodyAngle]
        p.setJointMotorControlArray(self.RobotId, self.BodyMotorIdList, p.POSITION_CONTROL, bodyPose,
                                    forces=[self.maxForce] * self.bodyJointNum)

        for i in range(self.N_leg):
            newtargetPose[i * self.DoFnum + 2] = newtargetPose[i * self.DoFnum + 2] - np.pi / 2
        p.setJointMotorControlArray(self.RobotId, self.MotorIdList, p.POSITION_CONTROL, newtargetPose,
                                    forces=[self.maxForce] * self.motorJointNum)

    def setup_environment(self):
        # Start pybullet and load the plane
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.spawnHolds(holds = self.bouldering_holds_list_pos_str,
                                            heights = self.bouldering_holds_list_height)

        # Add the scandots to the simulation
        # p.addUserDebugPoints([[1,1,1],[2,2,2]],[[1,1,1],[1,1,1]], 10)
        self.planeId = p.loadURDF("plane.urdf")
        # Specify the position and orientation of the wall
        position = [0, 0, 0]
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        print('\n \033[93m' + "Loading Environment..." + '\033[0m')
        self.wall = p.loadURDF(self.urdf_filename_wall,
                               basePosition=position, baseOrientation=orientation)
        # load the robot from urdf
        self.RobotId = p.loadURDF(self.urdf_filename, self.RobotStartPos, self.RobotStartOrientation,
                                  useFixedBase=self.bodyFixed)

        self.buildJointNameToIdDict()
        self.buildWallJointNameToIdDict()
        self.buildBodyMotorIdList()
        self.buildMotorIdList()
        self.connectLinks()
        self.setPassiveJoints()

        if self.delta_t is None:
            p.setRealTimeSimulation(True)
        else:
            self.setTimestep(self.delta_t)

        if not self.bodyFixed:
            print('\n \033[93m' + "Initializing Constraints" + '\033[0m')
            self.disable_collisions()
            # If body is not fixed, we first put the robot in a configuration compatible for the initial constraints # TODO: Consider finding this ideal configuration automatically based on initial climbing hold positions
            self.movetoPose([0.0, -0.04307892918586731, -0.29589801382778946, 2.188847118880749, 0, 0.26100000388920075, 0, -0.05838123336434364, -0.2855537499710605, 2.1837214914171503, 0, 0.30000000447034725, 0, 0.08952036499977112, -0.2675979216947099, 2.1746518954402414, 0, 0.30000000447034725, 0, -0.054661452770233154, -0.24390613211111314, 2.162350166661688, 0, 0.30000000447034725, 0, -0.055, 0.055, -0.055, 0.055, -0.055, 0.055, -0.055, 0.055])
            self.step()
            time.sleep(0.1)
            self.constraint_list = [0,0,0,0]
            for leg in range(4):
                if leg != 0:
                    const = self.create_constraint_init(self.bouldering_holds_list_pos[self.bouldering_hold_leg_index[leg]],leg)
                    self.constraint_list[leg] = const
            time.sleep(1.0)
            self.enable_collisions()
            cur_joints = self.getJointStates()
            cur_joints_pos = cur_joints[0]
            self.HOME_POSE = cur_joints_pos

    def spawnHolds(self, holds, heights):
        # holds should be array of 4 strings holding xyz positions of each hold
        wall_urdf_path = self.urdf_filename_wall
        tree = ET.parse(wall_urdf_path)
        root = tree.getroot()
        i = 0
        for child in root.iter("origin"):
            if child.get("rpy") == "3.14 0 0.0":
                child.set("xyz", holds[i])
                i += 1
        # search through hold links and adjust heights
        i = 0
        for child in root.iter("link"):
            linkName = child.get("name")
            if linkName[:4] == "hold":
                for box in child.iter("box"):
                    size = box.get("size")
                    new_string = size.rsplit(' ', 1)[0]
                    new_string += " " + str(heights[i])
                    box.set("size", new_string)
                i += 1

        tree.write(wall_urdf_path)

    def body_to_world_frame(self, pose_T):
        # input is the T_pose matrix (4x4) of the footstep position relative to body frame, note, we assume T_pose is in mm, as IK/FK assumes mm
        # get current body state (in m, as pybullet assumes meters)
        body_pose_T = self.getBodyState()
        body_pose_T[3,3] = body_pose_T[3,3]
        # convert to mm for pose
        body_pose_T[0:3,3] = body_pose_T[0:3,3]
        # convert footstep position from body to world frame
        pose_T_world = np.matmul(body_pose_T, pose_T)

        return pose_T_world

    def world_to_body_frame(self, pose_T):
        # input is the T_pose matrix (4x4) of the footstep position relative to world frame, we assume meters as input to pose_T
        # get current body state (in m, as pybullet assumes meters)
        body_pose_T = self.getBodyState()
        # convert footstep position from body to world frame
        pose_T_body = np.matmul(np.linalg.inv(body_pose_T), pose_T)

        return pose_T_body

    def get_finger_contact_normalOffset(self):
        # Retrieve whether contact occurred or not, returns a boolean to indicate contact happened, and also the angle between normal force and surface normal
        try:
            contact_points = p.getContactPoints(self.RobotId, self.wall, 13, 1)
            #contact_flag, bodyid_a, bodyid_b, link_a, link_b, pos_a, pos_b, contact_normal_b, contact_distance, normal_force, \
            #lateralfriction1,lateralfrictiondir1,lateralfriction2,lateralfrictiondir2 = contact_points[0]
            contact_normal_b, normal_force = contact_points[0][7], contact_points[0][9]
            # Compute the dot product of the surface normal and the normal force
            dot_product = sum(a * normal_force for a in contact_normal_b)
            # Compute the magnitudes of the surface normal and the normal force
            magnitude_normal = math.sqrt(sum(a ** 2 for a in contact_normal_b))
            magnitude_force = abs(normal_force)
            # Compute the angle between the surface normal and the normal force
            angle1 = math.acos(dot_product / (magnitude_normal * magnitude_force))
            contact1 = True

        except:
            angle1 = 2*np.pi
            contact1 = False

        try:
            contact_points2 = p.getContactPoints(self.RobotId, self.wall, 16, 1)
            contact_normal_b, normal_force = contact_points2[0][7], contact_points2[0][9]
            # Compute the dot product of the surface normal and the normal force
            dot_product = sum(a * normal_force for a in contact_normal_b)
            # Compute the magnitudes of the surface normal and the normal force
            magnitude_normal = math.sqrt(sum(a ** 2 for a in contact_normal_b))
            magnitude_force = abs(normal_force)
            # Compute the angle between the surface normal and the normal force
            angle2 = math.acos(dot_product / (magnitude_normal * magnitude_force))
            contact2 = True

        except:
            angle2 = 2*np.pi
            contact2 = False

        return contact1, contact2, angle1, angle2

    def reset(self, pose=None):
        if not pose:
            pose = self.HOME_POSE
        self.disable_collisions()
        self.maxForce = 1000
        self.movetoPose(pose)
        self.step()
        time.sleep(0.2)
        self.maxForce = 300
        self.enable_collisions()

    def enable_collisions(self, collision=1):
        # Get the number of links in the body
        numLinks_wall = p.getNumJoints(self.wall)
        numLinks_robot = p.getNumJoints(self.RobotId)

        link_id_wall = []
        link_id_robot = []

        # Iterate over all the links in the body
        for i in range(numLinks_wall):
            # Get information about the link
            link_id_wall.append(p.getJointInfo(self.wall, i)[0])

        for i in range(numLinks_robot):
            link_id_robot.append(p.getJointInfo(self.RobotId, i)[0])

        for i in range(numLinks_wall):
            for j in range(numLinks_robot):
                # Iterate over all pairs of bodies and turn off collisions
                p.setCollisionFilterPair(self.wall, self.RobotId, i, j, collision)

    def disable_collisions(self):
        self.enable_collisions(0)

    def getBodyState(self):
        #return the T matrix of body frame in world frame
        bodyDynamicsInfo = p.getDynamicsInfo(self.RobotId,-1)
        urdf_T_CoM = postion_quat_to_matrix([bodyDynamicsInfo[3], bodyDynamicsInfo[4]])
        bodyCoMinfo = p.getBasePositionAndOrientation(self.RobotId)
        bodyCoMvel = p.getBaseVelocity(self.RobotId)
        world_T_CoM = postion_quat_to_matrix(bodyCoMinfo)

        CoM_T_urdf = np.linalg.inv(urdf_T_CoM)

        return np.dot(world_T_CoM, CoM_T_urdf)

    def check_obstacle_goal(self, leg):
        # we now get our pose measurement from pybullet
        pos_link_meas = self.get_link_state(self.fk_ID_link[leg])[0]
        pose_meas = [pos_link_meas[0],pos_link_meas[1],pos_link_meas[2]]
        goal = self.bouldering_holds_list_pos[self.bouldering_hold_leg_index[leg]]
        goal[2] = self.bouldering_holds_list_height[self.bouldering_hold_leg_index[leg]]
        distance = self.distance_to_goal_pos(pose_meas, goal)
        if distance <= self.grasp_threshold:
            return True, distance
        else:
            return False, distance

    def distance_to_goal_pos(self, observation, goal):
        dx = observation[0] - goal[0]
        dy = observation[1] - goal[1]
        dz = observation[2] - goal[2]

        return math.sqrt(dx * dx + dy * dy + dz * dz)


    def getBodyVelocity(self):
        linear, angular = p.getBaseVelocity(self.RobotId)
        return [*linear, *angular]

    def fk_with_name(self, name):
        #return the T matrix of any link frame in body frame
        #For 3 DOF scaler, the end effector name is 'Toe_Link'+index number, eg. leg 0 end effector is 'Toe_Link0', the end effector is defined at the tip of the ball
        #For 6 DOF scaler, the end effector name is 'wrist3_Joint'+index number, eg. leg 0 end effector is 'wrist3_Joint0', the end effector is defined at the intersection of the wrist motors' frames
        return self.fk_with_index(self.linkNameToId[name])

    def fk_with_index(self, ind):
        trans = p.getLinkState(self.RobotId, ind)[4]
        orien = p.getLinkState(self.RobotId, ind)[5]
        world_T_Link = postion_quat_to_matrix([trans, orien])
        world_T_body = self.getBodyState()
        body_T_world = np.linalg.inv(world_T_body)


        return np.dot(body_T_world, world_T_Link)

    def step(self):
        p.stepSimulation()

    def setTimestep(self, delta_t):
        p.setTimeStep(delta_t)

    def getJointStates(self):
        JointAngleList = []
        JointVelocityList = []
        JointTorqueList = []
        for i in range(self.motorJointNum):
            JointAngleList.append(p.getJointState(self.RobotId, self.jointNameToId[self.motorJointNameList[i]])[0])
            JointVelocityList.append(p.getJointState(self.RobotId, self.jointNameToId[self.motorJointNameList[i]])[1])
            JointTorqueList.append(p.getJointState(self.RobotId, self.jointNameToId[self.motorJointNameList[i]])[3])
        for i in range(self.N_leg):
            JointAngleList[i * self.DoFnum + 2] = JointAngleList[i * self.DoFnum + 2] + np.pi / 2
        JointAngleList.insert(0, p.getJointState(self.RobotId, self.jointNameToId[self.bodyJointNameList[0]])[0])
        JointVelocityList.insert(0, p.getJointState(self.RobotId, self.jointNameToId[self.bodyJointNameList[0]])[1])
        JointTorqueList.insert(0, p.getJointState(self.RobotId, self.jointNameToId[self.bodyJointNameList[0]])[3])

        return [JointAngleList, JointVelocityList, JointTorqueList]

    def startRecordingVideo(self, video_path_name):
        # video_path_name: the path and the name of the generated video in mp4 format, for example: videos/round1.mp4
        self.recordId = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_path_name)

    def stopRecordingVideo(self):
        p.stopStateLogging(self.recordId)

    def get_link_state(self, link_id):
        link_state = p.getLinkState(self.RobotId, link_id)
        pos = link_state[0]
        orientation = link_state[1]

        return [pos, orientation]

    def create_constraint_init(self, bouldering_hold_pos, leg):
        # we first move the robot in the correct position and orientation before making the constraint
        pos = bouldering_hold_pos
        pos[2]= pos[2]+self.bouldering_holds_list_pos[self.bouldering_hold_leg_index[leg]][2]
        orn = self.bouldering_hold_rotation[leg]


        # Create a constraint that fixes the relative orientation between the two links
        constraint = p.createConstraint(self.RobotId, self.fk_ID_link[leg], -1, -1, p.JOINT_POINT2POINT,
                                        jointAxis=[0,0,0.0], parentFramePosition=[0,0,0],
                                        childFramePosition=pos, childFrameOrientation=orn)

        return  constraint

    def create_constraint(self, leg):
        # Create a constraint that fixes the relative orientation between the two links
        link_state = p.getLinkState(self.RobotId, self.fk_ID_link[leg])
        pos = link_state[0]
        orn = link_state[1]

        constraint = p.createConstraint(self.RobotId, self.fk_ID_link[leg], -1, -1, p.JOINT_POINT2POINT,
                                        jointAxis=[0, 0, 0.0], parentFramePosition=[0, 0, 0],
                                        childFramePosition=pos, childFrameOrientation=orn)

        return constraint

    def break_constraint(self, constraint_id):
        p.removeConstraint(constraint_id)

    def visualize_frames(self):
        # visualizing wrist3_joint3
        link_state = p.getLinkState(self.RobotId, 9)
        pos_T = self.fk_with_index(9)
        pos_T = self.body_to_world_frame(pos_T)
        pos = pos_T[0:3,3]
        orn = link_state[1]
        # Convert the quaternion to a rotation matrix
        R = p.getMatrixFromQuaternion(orn)
        R = np.array(R).reshape(3, 3)
        X_axis = np.array([1, 0, 0])
        Y_axis = np.array([0, 1, 0])
        Z_axis = np.array([0, 0, 1])
        X_end = pos + np.dot(R, X_axis)
        Y_end = pos + np.dot(R, Y_axis)
        Z_end = pos + np.dot(R, Z_axis)
        p.addUserDebugLine(pos, X_end, [1, 0, 0], lifeTime=0.2, lineWidth=3)
        p.addUserDebugLine(pos, Y_end, [0, 1, 0], lifeTime=0.2, lineWidth=3)
        p.addUserDebugLine(pos, Z_end, [0, 0, 1], lifeTime=0.2, lineWidth=3)

        # visualizing the fk of limb3
        self.leg = 3
        self.home_pose_T = self.kin.scalar_forward_kinematics(3, self.HOME_POSE[1 + self.leg * 6:1 + self.leg * 6 + 6], with_body=True)
        self.home_pose_T[0, 3] = self.home_pose_T[0, 3] / 1000
        self.home_pose_T[1, 3] = self.home_pose_T[1, 3] / 1000
        self.home_pose_T[2, 3] = self.home_pose_T[2, 3] / 1000
        self.home_pose_T = self.body_to_world_frame(self.home_pose_T)
        R = self.home_pose_T[0:3,0:3]
        R = np.array(R).reshape(3, 3)
        pos = np.array(self.home_pose_T[0:3,3]).reshape(3,1)
        pos = (pos[0][0],pos[1][0],pos[2][0])
        X_axis = np.array([1, 0, 0])
        Y_axis = np.array([0, 1, 0])
        Z_axis = np.array([0, 0, 1])
        X_end = pos + np.dot(R, X_axis)
        Y_end = pos + np.dot(R, Y_axis)
        Z_end = pos + np.dot(R, Z_axis)
        p.addUserDebugLine(pos, X_end, [1, 0, 0], lifeTime=0.2, lineWidth=3)
        p.addUserDebugLine(pos, Y_end, [0, 1, 0], lifeTime=0.2, lineWidth=3)
        p.addUserDebugLine(pos, Z_end, [0, 0, 1], lifeTime=0.2, lineWidth=3)

        link_state = p.getLinkState(self.wall, 1)
        pos = link_state[0]
        orn = link_state[1]

        # Convert the quaternion to a rotation matrix
        R = p.getMatrixFromQuaternion(orn)
        R = np.array(R).reshape(3, 3)
        X_axis = np.array([1, 0, 0])
        Y_axis = np.array([0, 1, 0])
        Z_axis = np.array([0, 0, 1])
        X_end = pos + np.dot(R, X_axis)
        Y_end = pos + np.dot(R, Y_axis)
        Z_end = pos + np.dot(R, Z_axis)
        p.addUserDebugLine(pos, X_end, [1, 0, 0], lifeTime=0.2, lineWidth=3)
        p.addUserDebugLine(pos, Y_end, [0, 1, 0], lifeTime=0.2, lineWidth=3)
        p.addUserDebugLine(pos, Z_end, [0, 0, 1], lifeTime=0.2, lineWidth=3)

    def getDepthOfImage(self, depth, index):
        # Get the camera link index
        camera_position = p.getJointInfo(self.wall, self.wallJointNameToId[self.cameraJointNameList[index]])[14]

        # Compute the view and projection matrices
        if (index == 1):
            view_matrix = p.computeViewMatrix(camera_position, (0.0, 0.43, 0.1), (0, 1, 0))
        else:
            view_matrix = p.computeViewMatrix(camera_position, (0, 0, 0), (1, 0, 0))

        projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.01, farVal=100.0)

        # Return depth image
        images = p.getCameraImage(width=224, height=224, viewMatrix=view_matrix, projectionMatrix=projection_matrix,
                                  physicsClientId=self.physicsClient)

        if depth == 1:
            return images[3]
        if depth == 0:
            return images[2]

        #plt.imshow(images[0])
        #plt.show()



