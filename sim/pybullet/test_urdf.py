import time

from sim.pybullet.scalar_sim import pyb_sim
from utils.urdf_relative_path import urdf_filepath_resolver
from pathlib import Path

import pybullet as p
URDF_PATH = 'urdf_scalar_6DoF/urdf/SCALAR_6DoF_gripper_test.urdf'
MESH_DIR = 'urdf_scalar_6DoF/meshes/'
full_path = Path.cwd().joinpath(URDF_PATH)
urdf_filename = urdf_filepath_resolver(full_path, MESH_DIR)
urdf_file_path_wall = 'urdf_scalar_6DoF/urdf/wall_camera_manyholds.urdf'
urdf_file_path_wall = str(Path.cwd().joinpath(urdf_file_path_wall))


robot = pyb_sim(urdf_filename, urdf_file_path_wall, bodyFixed=False, RobotStartPos=[0.0 ,0.0 ,0.55])

while True:
    #for i in range(p.getNumJoints(robot.RobotId)):
    #    print(p.getJointInfo(robot.RobotId, i))
    link_state = robot.get_link_state(robot.fk_ID_link[0])
    quat_meas = link_state[1]
    print('leg 0')
    print(quat_meas)
    link_state = robot.get_link_state(robot.fk_ID_link[1])
    quat_meas = link_state[1]
    print('leg 1')
    print(quat_meas)
    link_state = robot.get_link_state(robot.fk_ID_link[2])
    quat_meas = link_state[1]
    print('leg 2')
    print(quat_meas)
    link_state = robot.get_link_state(robot.fk_ID_link[3])
    quat_meas = link_state[1]
    print('leg 3')
    print(quat_meas)

    robot.movetoPose(robot.HOME_POSE)
    robot.step()
    time.sleep(0.01)

pass
