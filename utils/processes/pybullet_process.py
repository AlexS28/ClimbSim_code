from sim.pybullet.scalar_sim import pyb_sim
from utils.urdf_relative_path import urdf_filepath_resolver
from pathlib import Path
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from scaler_kin.SCALAR_kinematics import scalar_k as ScalerKinematics
import rospy
import time

class PYBULLET:
    def __init__(self):
        URDF_PATH = 'sim/pybullet/urdf_scalar_6DoF/urdf/SCALAR_6DoF_gripper_test.urdf'
        MESH_DIR = 'sim/pybullet/urdf_scalar_6DoF/meshes/'
        full_path = Path.cwd().joinpath(URDF_PATH)
        urdf_filename = urdf_filepath_resolver(full_path, MESH_DIR)
        urdf_file_path_wall = 'sim/pybullet/urdf_scalar_6DoF/urdf/temp_wall_camera_manyholds.urdf'
        urdf_file_path_wall = str(Path.cwd().joinpath(urdf_file_path_wall))
        # specify starting position
        self.robot = pyb_sim(urdf_filename, urdf_file_path_wall, bodyFixed=False, RobotStartPos=[0.0,0.0,0.55])
        self.home_pose = self.robot.HOME_POSE
        self.kin = ScalerKinematics()
        self.home_pose_T_list = []
        for leg in range(4):
            self.home_pose_T_list.append(self.kin.scalar_forward_kinematics(leg, self.home_pose[1+leg*6:1+leg*6+6], with_body=True))
        # initialize Joystick states
        self.joystick_foot_selection = Joy()
        self.joystick_leg_gripper = Joy()
        self.joystick_body_vel = Vector3()
        # receive footstep selection information
        self.sub_joy_footstep_selection = rospy.Subscriber('/cmd_foot_selection', Joy, self.receive_footstep_selection)
        # receive footstep leg and gripper information
        self.sub_joy_leg_gripper = rospy.Subscriber('/cmd_scaler', Joy, self.receive_joystick_leg_gripper)
        # receive body velocity information
        self.sub_joy_body_vel = rospy.Subscriber('/cmd_scaler_body', Vector3, self.receive_joystick_body_vel)
        self.current_footstep_selection = 0
        self.grasp_release_obstacle = 0
        self.contact_state = [1,1,1,0]

    def receive_joystick_body_vel(self, msg):
        self.joystick_body_vel = msg

    def receive_joystick_leg_gripper(self, msg):
        self.joystick_leg_gripper = msg

    def receive_joystick_gripper(self, msg):
        self.joystick_gripper = msg

    def receive_footstep_selection(self, msg):
        self.joystick_foot_selection = msg

    def get_velocity_cmd(self):
        axes_list = self.joystick_leg_gripper.axes
        body_vel = self.joystick_body_vel
        command_velocity_arm = np.zeros((3, ))
        command_velocity_arm[0] = axes_list[0]*1000
        command_velocity_arm[1] = axes_list[1]*1000
        command_velocity_arm[2] = axes_list[2]*1000
        command_velocity_gripper = axes_list[3]
        command_body_motor = axes_list[4]
        command_wrist_joints = np.zeros((3,))
        command_wrist_joints[0] = axes_list[5]*5.0
        command_wrist_joints[1] = axes_list[6]*5.0
        command_wrist_joints[2] = axes_list[7]*5.0
        command_body_vel = np.zeros((2,))
        command_body_vel[0] = body_vel.x*1000
        command_body_vel[1] = body_vel.y*1000

        return command_velocity_arm, command_velocity_gripper, command_body_motor, command_wrist_joints, command_body_vel

    def get_footstep_selection(self):
        footsteps_axes = self.joystick_foot_selection.axes
        footsteps_list = footsteps_axes[0:4]
        for i in range(4):
            if footsteps_list[i] == 1.0:
                self.current_footstep_selection = i
        self.grasp_release_obstacle = footsteps_axes[4]

        return self.current_footstep_selection

def activate(event_shutdown):
    pybullet_object = PYBULLET()
    time.sleep(1.0)
    # initialize ros node
    rospy.init_node('ros_teleoperation', anonymous=False)
    # initialize pose for each leg
    pose_T_list = []
    pose_gripper_list = []
    prev_ang_list = []
    wrist_joints_list = []
    for leg in range(4):
        pose_T = np.zeros((4,4))
        pose_T[:,:] = pybullet_object.home_pose_T_list[leg]
        pose_T_list.append(pose_T)
        pose_gripper_list.append(pybullet_object.home_pose[1+4*6+leg*2:1+4*6+leg*2+2])
        prev_ang = pybullet_object.home_pose[1+leg*6:1+leg*6+6]
        prev_ang_list.append(prev_ang)
        wrist_joints_list.append(prev_ang[3:6])
    pose_body_motor = pybullet_object.home_pose[0]

    while not event_shutdown.is_set() and not rospy.is_shutdown():
        # get leg number from joystick command
        leg = pybullet_object.get_footstep_selection()
        # get commanded joystick velocity
        cmd_vel_arm, cmd_vel_gripper, cmd_vel_body_motor, command_wrist_joints, command_body_vel = pybullet_object.get_velocity_cmd()
        # move body
        pose_T_list = pybullet_object.robot.correct_cmd_body_vel(pose_T_list,command_body_vel,leg)
        for k in range(4):
            if k != leg:
                pose_T_body = pose_T_list[k]
                prev_ang_body = prev_ang_list[k]
                ik_sol_body = pybullet_object.kin.scalar_inverse_kinematics(k,pose_T_body,is_first_ik=False,prev_angles=prev_ang_body,with_body=True)
                ik_sol_body[3:6] = prev_ang_body[3:6]
                prev_ang_list[k] = ik_sol_body
                pybullet_object.home_pose[1+k*6:1+k*6+6] = ik_sol_body

        # correct body motor
        pose_body_motor = pybullet_object.robot.correct_cmd_body(pose_body_motor, cmd_vel_body_motor)
        # get the correct pose of limb, pose gripper of fingers, and wrist joints for the limb to be commanded
        pose_T = pose_T_list[leg]
        pose = pose_T[0:3,3]
        pose_gripper = pose_gripper_list[leg]
        wrist_joints = wrist_joints_list[leg]
        prev_ang = prev_ang_list[leg]
        # update the pose, pose_gripper, and wrist_joints based on the commanded velocity
        pose, pose_gripper, wrist_joints = pybullet_object.robot.correct_cmd_limb(leg, pose, cmd_vel_arm, pose_gripper, cmd_vel_gripper, wrist_joints, command_wrist_joints)
        # build new pose_T matrix
        pose_T[0:3,3] = pose
        # use IK to get new joint angles
        ik_sol = pybullet_object.kin.scalar_inverse_kinematics(leg,pose_T,is_first_ik=False,prev_angles=prev_ang,with_body=True)
        prev_ang = ik_sol
        # command the ik solution to the robot
        prev_ang_updated = prev_ang.copy()
        prev_ang_updated[-3:] = wrist_joints
        pybullet_object.home_pose[1+leg*6:1+leg*6+6] = prev_ang_updated
        pybullet_object.home_pose[1+4*6+leg*2:1+4*6+leg*2+2] = pose_gripper
        pybullet_object.home_pose[0] = pose_body_motor
        # update the list with the newest values
        pose_T_list[leg] = pose_T
        pose_gripper_list[leg] = pose_gripper
        wrist_joints_list[leg] = wrist_joints
        prev_ang_list[leg] = prev_ang_updated
        # check if user wants to grasp or release object (note, object MUST be near gripper)
        if pybullet_object.grasp_release_obstacle:
            # check if gripper near object
            near_obstacle, _ = pybullet_object.robot.check_obstacle_goal(leg)
            if near_obstacle:
                # check if we want to release to grasp object
                if not pybullet_object.contact_state[leg]:
                    # if true, we make a constraint
                    pybullet_object.robot.constraint_list[leg] = pybullet_object.robot.create_constraint(leg)
                    cur_joints = pybullet_object.robot.getJointStates()
                    cur_joints_pos = cur_joints[0]
                    pybullet_object.home_pose = cur_joints_pos
                    pybullet_object.contact_state[leg] = 1
                    time.sleep(0.5)
                else:
                    pybullet_object.robot.break_constraint(pybullet_object.robot.constraint_list[leg])
                    pybullet_object.contact_state[leg] = 0
                    time.sleep(0.5)
        pybullet_object.robot.movetoPose(pybullet_object.home_pose)
        pybullet_object.robot.step()