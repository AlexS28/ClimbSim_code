import time
import sys
import os
dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(dir_path)
from scalar_sim import pyb_sim
from utils.urdf_relative_path import urdf_filepath_resolver
from pathlib import Path
import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import copy

URDF_PATH = dir_path + '/sim/pybullet/urdf_scalar_6DoF/urdf/SCALAR_6DoF_gripper_test.urdf'
MESH_DIR = dir_path + '/sim/pybullet/urdf_scalar_6DoF/meshes/'
urdf_filename = urdf_filepath_resolver(URDF_PATH, MESH_DIR)
urdf_file_path_wall = '/home/schperberg/ASCENT_workspace/src/ASCENT/sim/pybullet/urdf_scalar_6DoF/urdf/temp_wall_camera_manyholds.urdf'

robot = pyb_sim(urdf_filename,urdf_file_path_wall,DoFnum=6,RobotStartPos=[0.0,0.0,0.55],bodyFixed=False)
# Get the dynamics information for the robot
# Get the number of joints in the robot
numJoints = p.getNumJoints(robot.RobotId)
#numJoints_wall = p.getNumJoints(robot.wall)
# Set the joint motor control mode to POSITION_CONTROL
controlMode = p.POSITION_CONTROL
#next_pose = [0, -0.6, np.pi / 1.5, 0, 0, 0, -0.055,0.055]
#home_pose = [0.0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0.0, np.pi / 2, 0, 0, 0, -0.055,0.055, -0.055,0.055, -0.055,0.055, -0.055,0.055]

home_pose = copy.deepcopy(robot.HOME_POSE)

timestep = 0

while timestep < 100000:
    # Set the joint motor control for the joint
    for i in range(p.getNumJoints(robot.RobotId)):
        print(p.getJointInfo(robot.RobotId, i))
    timestep += 1


    #for i in range(p.getNumJoints(robot.wall)):
    #    print(p.getJointInfo(robot.wall, i))

    if timestep >= 50:
        robot.move_obstacle(robot.hold_1,[-0.12,0.8,0.07],[0.99,0.0,0.0,0.0])

    home_pose = robot.HOME_POSE
    #if timestep<10:
    #    robot.movetoPose(home_pose)
    robot.visualize_frames()

    leg = 3
    home_pose_T_body = robot.kin.scalar_forward_kinematics(3, home_pose[1 + leg * 6:1 + leg * 6 + 6], with_body=True)
    home_pose_T_body[0,3] = home_pose_T_body[0,3]/1000
    home_pose_T_body[1, 3] = home_pose_T_body[1, 3] / 1000
    home_pose_T_body[2, 3] = home_pose_T_body[2, 3] / 1000
    pose_T_world = robot.body_to_world_frame(home_pose_T_body)
    pos_T_link = robot.fk_with_index(17)
    print("world frame: {}".format(pose_T_world[0:3, 3]))
    print("body frame: {}".format(home_pose_T_body[0:3, 3]))
    pos_T_link = robot.body_to_world_frame(pos_T_link)
    pos = pos_T_link[0:3, 3]
    pos[0] = pos[0]
    pos[1] = pos[1]
    pos[2] = pos[2]
    time.sleep(0.01)
    robot.movetoPose(robot.HOME_POSE)
    print("pybullet gripper link: {}".format(pos))
    #bouldering_hold_location = [[-0.10844448228157065, 0.2570351620241008, 0.26388308983727704], [0.9860096546393706, -0.0433007239372018, 0.007014800296124778, -0.16081293741881825]]
    #bouldering_hold_location = [[-0.12, 0.3, 0.15], [0.999, 0.0, 0.0, 0.0]]
    #if timestep == 10:
    #    constraint = robot.create_constraint(bouldering_hold_location)
        #robot.reset_legacy()
    #robot.step()

    #if timestep == 200:
    #    robot.break_constraint(constraint)

    #if timestep == 10000:
    #    robot.reset_legacy()

    #next_pose = home_pose[19:27]
    #print('\nCurrent pose: {}'.format(home_pose[24]))
    #gripper_pos = robot.fk_with_name('base_link_gripper3')
    #gripper_pos = gripper_pos[0:3,3]
    #print(gripper_pos)
    #print(robot.get_finger_contact_normalOffset())
    #time.sleep(2)
    rgb_image = robot.getDepthOfImage(0,1)

    # Plot the RGB image
    #plt.imshow(rgb_image)
    #plt.show()
    # Save the RGB image to a folder
    #folder = "images"
    #if not os.path.exists(folder):
    #    os.makedirs(folder)
    #filename = "image.png"
    #filepath = os.path.join(folder, filename)
    #plt.imsave(filepath, rgb_image)



"""

# number of robots
num_robots = 1

# Create a thread pool with a specified number of threads
executor = ThreadPoolExecutor(max_workers=num_robots)

# create a list of pyb_sim objects using the thread pool
robot_list = []
for i in range(num_robots):
    # set the simulate argument to True for the first pyb_sim instance
    # and False for all other instances
    simulate = True if i == 0 else False
    robot_list.append(executor.submit(pyb_sim, urdf_filename=urdf_filename, DoFnum=6, delta_t=0.01, simulate=simulate))


for i in range(num_robots):
    # get the pyb_sim object from the future at the specified index
    cur_robot = robot_list[i].result()
    # access the values for the current pyb_sim instance
    cur_robot.dance2()
    cur_joints = cur_robot.getJointAngles()

# shut down the thread pool
executor.shutdown()
"""


