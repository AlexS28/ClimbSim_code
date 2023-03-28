# ClimbSim
(WORK IN PROGRESS) 
Timeline for future repo updates:

1. **Add more features to the simulation (additional bouldering holds, reset simulation functionality, etc) (~April-May 2023)**
2. **Add Planner that uses Reinforcement Learning (~June 2023) along with Paper on Arxiv**

This repository creates a robot climbing simulator that runs in pybullet, and can be used with an Xbox controller for teleoperation. The simulator is based on the robot, SCALER (Spine-enhanced Climbing Autonomous Legged Exploration Robot), from the Robotics and Mechanisms Laboratory at UCLA - see reference [2] below. The paper on the simulator and use of reinforcement learning using this simulator will be written in citation [1] (currently work in progress)

# Please also cite [1] if you plan to use the simulator, for questions or if you'd like to contribute to the repo, email: aschperb@gmail.com
[1] Planner for Robotic Free-Climbing using Reinforcement Learning, by Alexander Schperberg (author list / title to be updated).

Special thanks to: Feng Xu for creating the initial urdf files and part of the simulation code, and the SCALER team: Yuki Shirai, and Yusuke Tanaka. 

# Installation:
0. System tested: Ubuntu 20.04, ROS Noetic, Python 3.8
1. Install ROS Noetic (potentially ROS Melodic may also work, but it has not been tested)
2. `sudo apt-get install ros-noetic-desktop-full`
3. `sudo apt-get install ros-noetic-joy`
4. mkdir -p ~/catkin_ws/src
5. git clone this repository inside the src folder
5. cd ~/catkin_ws/
6. catkin_make
7. cd ~/catkin_ws/src/ClimbSim/
7. `pip install -r requirements.txt`

# How to run:
0. Terminal 1: cd ~/catkin_ws/
1. `source devel/setup.bash`
2. `roslaunch joystick_package joystick.launch` 
3. Terminal 2: `python start_teleporation.py` 

[2] SCALER: A Tough Versatile Quadruped Free-Climber Robot by Yusuke Tanaka, Yuki Shirai, Xuan Lin, Alexander Schperberg, Hayato Kato, Alexander Swerdlow, Naoya Kumagai, and Dennis Hong.

[2] @INPROCEEDINGS{9981555,
  author={Tanaka, Yusuke and Shirai, Yuki and Lin, Xuan and Schperberg, Alexander and Kato, Hayato and Swerdlow, Alexander and Kumagai, Naoya and Hong, Dennis},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={SCALER: A Tough Versatile Quadruped Free-Climber Robot}, 
  year={2022},
  volume={},
  number={},
  pages={5632-5639},
  doi={10.1109/IROS47612.2022.9981555}}
  
TELEOPERATION CONTROLS:
![controller_pic](https://user-images.githubusercontent.com/45216484/228128498-df4d9325-ccb0-4a3b-a78c-03ba3c310ff3.png)

PYBULLET SIMULATION:
![Screenshot from 2023-02-01 12-59-01](https://user-images.githubusercontent.com/45216484/217660797-214e26ab-aa44-401f-a720-bfeecc94b546.png)

SCALER HARDWARE:
![fig1](https://user-images.githubusercontent.com/45216484/217659832-07cfde0b-ca75-406e-838b-7c108fecc7cc.jpg)
