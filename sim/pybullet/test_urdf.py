from sim.pybullet.scalar_sim import pyb_sim
from utils.urdf_relative_path import urdf_filepath_resolver
from pathlib import Path

URDF_PATH = 'urdf_scalar_6DoF/urdf/SCALAR_6DoF_gripper_test.urdf'
MESH_DIR = 'urdf_scalar_6DoF/meshes/'
full_path = Path.cwd().joinpath(URDF_PATH)
urdf_filename = urdf_filepath_resolver(full_path, MESH_DIR)
urdf_file_path_wall = 'urdf_scalar_6DoF/urdf/wall_camera_manyholds.urdf'
urdf_file_path_wall = str(Path.cwd().joinpath(urdf_file_path_wall))


robot = pyb_sim(urdf_filename, urdf_file_path_wall, bodyFixed=True, RobotStartPos=[0.0 ,0.0 ,0.55])
robot.step()

pass
